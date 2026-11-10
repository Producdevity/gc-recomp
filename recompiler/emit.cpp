#include "emit.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>

namespace gcr {
namespace {

std::string fmt(const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof buffer, format, args);
    va_end(args);
    return buffer;
}

struct Regs {
    std::uint32_t gpr = 0, fpr = 0, ps1 = 0, cr = 0;
    bool lr = false, ctr = false, ca = false, so = false, ov = false;
};

// EABI volatile state: r0, r3-r12, f0-f13 (both halves), CR fields 0, 1, 5, 6, 7, CTR, XER, LR.
constexpr std::uint32_t kVolatileGpr = 0x00001FF9u;   // r0, r3..r12
constexpr std::uint32_t kSharedGpr = 0x00002006u;     // r1, r2, r13
constexpr std::uint32_t kVolatileFpr = 0x00003FFFu;   // f0..f13
constexpr std::uint32_t kVolatileCr = 0xFFF000FFu;    // cr0, cr1, cr5, cr6, cr7 (bit i = CR bit i)
// Calls and returns leave CR0, CR5-CR7 and XER[CA] alone: EABI scratch state that carries no
// argument or result. CR1 flags floating-point arguments to varargs calls.
constexpr std::uint32_t kAbiCr = 0x000000F0u;
// Returns write back only what the caller can read: r3/r4, f1/f2 and the shared r1, r2, r13.
constexpr std::uint32_t kRetGpr = 0x0000201Eu, kRetFpr = 0x00000006u;

class Emitter {
public:
    Emitter(const Program& program, const Function& function, EmitStats& stats, const Regs* used, const Regs* written)
        : p_(program), f_(function), stats_(stats), final_used_(used), final_written_(written) {}

    std::string body();
    Regs used, written;

private:
    // Register references; reading marks use, writing marks use and modification.
    std::string g(unsigned n) { used.gpr |= 1u << n; return fmt("r%u", n); }
    std::string wg(unsigned n) { used.gpr |= 1u << n; written.gpr |= 1u << n; return fmt("r%u", n); }
    std::string g0(unsigned n) { return n ? g(n) : std::string("0u"); }
    std::string fr(unsigned n) { used.fpr |= 1u << n; return fmt("f%u", n); }
    std::string wfr(unsigned n) { used.fpr |= 1u << n; written.fpr |= 1u << n; return fmt("f%u", n); }
    std::string ps(unsigned n) { used.ps1 |= 1u << n; return fmt("p%u", n); }
    std::string wps(unsigned n) { used.ps1 |= 1u << n; written.ps1 |= 1u << n; return fmt("p%u", n); }
    std::string crb(unsigned n) { used.cr |= 1u << n; return fmt("cr%u", n); }
    std::string wcrb(unsigned n) { used.cr |= 1u << n; written.cr |= 1u << n; return fmt("cr%u", n); }
    std::string so() { used.so = true; return "so"; }
    void wxer_ca() { used.ca = written.ca = true; }
    void wxer_ov() { used.ov = written.ov = used.so = written.so = true; }

    void line(const std::string& s) { out_ += "    " + s + "\n"; }

    void cr_compare(unsigned field, const std::string& a, const std::string& b) {
        line(fmt("{ %s; %s = a_ < b_; %s = a_ > b_; %s = a_ == b_; %s = %s; }", (a + "; " + b).c_str(),
                 wcrb(field * 4).c_str(), wcrb(field * 4 + 1).c_str(), wcrb(field * 4 + 2).c_str(),
                 wcrb(field * 4 + 3).c_str(), so().c_str()));
    }
    void record(const std::string& reg) {
        cr_compare(0, "int32_t a_ = (int32_t)" + reg, "int32_t b_ = 0");
    }
    void record_fp() {
        line(fmt("%s = (c->fpscr >> 31) & 1; %s = (c->fpscr >> 30) & 1; %s = (c->fpscr >> 29) & 1; %s = (c->fpscr >> 28) & 1;",
                 wcrb(4).c_str(), wcrb(5).c_str(), wcrb(6).c_str(), wcrb(7).c_str()));
    }

    std::string ea_d(unsigned ra, std::int32_t d) {
        if (!ra) return fmt("(uint32_t)%d", d);
        return fmt("(uint32_t)(%s + %d)", g(ra).c_str(), d);
    }
    std::string ea_x(unsigned ra, unsigned rb) {
        if (!ra) return g(rb);
        return fmt("(uint32_t)(%s + %s)", g(ra).c_str(), g(rb).c_str());
    }

    // Sync points between locals and the context.
    std::string flush_code(bool all, bool abi = false, bool ret = false);
    std::string reload_code(bool all, bool abi = false);
    void flush(bool all = false, bool abi = false, bool ret = false) {
        auto s = flush_code(all || f_.exact, abi, ret);
        if (!s.empty()) line(s);
    }
    void reload(bool all = false, bool abi = false) { auto s = reload_code(all || f_.exact, abi); if (!s.empty()) line(s); }
    void safepoint(std::uint32_t pc, std::uint32_t target);
    bool is_spin_loop(std::uint32_t target, std::uint32_t branch);

    std::string label(std::uint32_t a) const { return fmt("L_%08X", a); }
    bool inside(std::uint32_t a) const { return a >= f_.start && a < f_.end; }

    void emit_call(std::uint32_t pc, std::uint32_t target);
    void emit_tail(std::uint32_t pc, std::uint32_t target);
    void emit_return(std::uint32_t pc);
    void emit_branch(std::uint32_t pc, Insn i, Op op);
    void emit_insn(std::uint32_t pc, Insn i, Op op);
    void emit_load(Insn i, Op op);
    void emit_store(Insn i, Op op);
    void emit_fp(std::uint32_t pc, Insn i, Op op);
    void emit_ps(Insn i, Op op);
    void emit_arith(std::uint32_t pc, Insn i, Op op);
    void emit_switch(std::uint32_t pc);
    std::string condition(std::uint32_t bo, std::uint32_t bi, bool decrement);

    const Program& p_;
    const Function& f_;
    EmitStats& stats_;
    const Regs* final_used_;
    const Regs* final_written_;
    std::set<std::uint32_t> labels_;
    std::string out_;
};

std::string Emitter::flush_code(bool all, bool abi, bool ret) {
    if (!final_written_) return "/* flush */";
    const Regs& w = *final_written_;
    std::string s;
    if (ret && !all) {
        for (unsigned n = 0; n < 32; ++n)
            if ((w.gpr & kRetGpr) >> n & 1) s += fmt("c->r[%u] = r%u; ", n, n);
        for (unsigned n = 0; n < 32; ++n)
            if ((w.fpr & kRetFpr) >> n & 1) s += fmt("c->f[%u] = f%u; ", n, n);
        for (unsigned n = 0; n < 32; ++n)
            if ((w.ps1 & kRetFpr) >> n & 1) s += fmt("c->ps1[%u] = p%u; ", n, n);
        if (w.so) s += "c->so = so; ";
        if (w.ov) s += "c->ov = ov; ";
        return s;
    }
    std::uint32_t gmask = all ? w.gpr : (w.gpr & (kVolatileGpr | kSharedGpr));
    std::uint32_t fmask = all ? w.fpr : (w.fpr & kVolatileFpr);
    std::uint32_t pmask = all ? w.ps1 : (w.ps1 & kVolatileFpr);
    std::uint32_t cmask = all ? w.cr : (w.cr & (abi ? kAbiCr : kVolatileCr));
    for (unsigned n = 0; n < 32; ++n)
        if (gmask >> n & 1) s += fmt("c->r[%u] = r%u; ", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (fmask >> n & 1) s += fmt("c->f[%u] = f%u; ", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (pmask >> n & 1) s += fmt("c->ps1[%u] = p%u; ", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (cmask >> n & 1) s += fmt("c->cr[%u] = cr%u; ", n, n);
    if (w.lr) s += "c->lr = lr; ";
    if (w.ctr) s += "c->ctr = ctr; ";
    if (w.ca && (all || !abi)) s += "c->ca = ca; ";
    if (w.so) s += "c->so = so; ";
    if (w.ov) s += "c->ov = ov; ";
    return s;
}

std::string Emitter::reload_code(bool all, bool abi) {
    if (!final_used_) return "/* reload */";
    const Regs& u = *final_used_;
    std::string s;
    // r1, r2 and r13 come back too: startup code (__init_registers) sets them for its caller.
    std::uint32_t gmask = all ? u.gpr : (u.gpr & (kVolatileGpr | kSharedGpr));
    std::uint32_t fmask = all ? u.fpr : (u.fpr & kVolatileFpr);
    std::uint32_t pmask = all ? u.ps1 : (u.ps1 & kVolatileFpr);
    std::uint32_t cmask = all ? u.cr : (u.cr & (abi ? kAbiCr : kVolatileCr));
    for (unsigned n = 0; n < 32; ++n)
        if (gmask >> n & 1) s += fmt("r%u = c->r[%u]; ", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (fmask >> n & 1) s += fmt("f%u = c->f[%u]; ", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (pmask >> n & 1) s += fmt("p%u = c->ps1[%u]; ", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (cmask >> n & 1) s += fmt("cr%u = c->cr[%u]; ", n, n);
    if (u.lr) s += "lr = c->lr; ";
    if (u.ctr) s += "ctr = c->ctr; ";
    if (u.ca && (all || !abi)) s += "ca = c->ca; ";
    if (u.so) s += "so = c->so; ";
    if (u.ov) s += "ov = c->ov; ";
    return s;
}

// A loop that only rereads memory through unchanged address registers waits for an
// interrupt or device: the runtime blocks it until the next event instead of spinning.
bool Emitter::is_spin_loop(std::uint32_t target, std::uint32_t branch) {
    if (branch - target > 12 * 4) return false;
    std::uint32_t loaded = 0, addressing = 0;
    for (std::uint32_t a = target; a <= branch; a += 4) {
        Insn i{p_.image->read32(a)};
        Op op = decode(i);
        switch (op) {
        case Op::lwz: case Op::lhz: case Op::lha: case Op::lbz:
            if (i.ra()) addressing |= 1u << i.ra();
            loaded |= 1u << i.rd();
            break;
        case Op::cmpi: case Op::cmpli: case Op::cmp: case Op::cmpl:
            break;
        case Op::andi_rc: case Op::andis_rc:
            loaded |= 1u << i.ra();
            break;
        case Op::rlwinm:
            loaded |= 1u << i.ra();
            break;
        case Op::bc:
            if (!(i.bo() & 4)) return false;
            break;
        case Op::b:
            if (i.lk()) return false;
            break;
        default:
            return false;
        }
    }
    return (loaded & addressing) == 0;
}

void Emitter::safepoint(std::uint32_t pc, std::uint32_t target) {
    if (is_spin_loop(target, pc)) {
        ++stats_.spin_loops;
        line(fmt("c->r[1] = %s; gc_spin(c);", g(1).c_str()));
    } else {
        line(fmt("GC_BACKEDGE(%s);", g(1).c_str()));
    }
}

void Emitter::emit_call(std::uint32_t pc, std::uint32_t target) {
    used.lr = written.lr = true;
    line(fmt("lr = 0x%08Xu;", pc + 4));
    if (p_.save_context.count(target)) {
        // Returns once now and again when the saved context is loaded.
        flush(true);
        line(fmt("if (gcrt_setjmp(gc_resume_slot(c, %s))) { %s %s = 1u; } else { f_%08X(c); %s }",
                 g(3).c_str(), reload_code(true).c_str(), wg(3).c_str(), target, reload_code(true).c_str()));
        return;
    }
    flush(false, true);
    if (p_.function_at(target)) {
        line(fmt("f_%08X(c);", target));
    } else {
        ++stats_.unresolved;
        line(fmt("gc_call(c, 0x%08Xu);", target));
    }
    reload(false, true);
}

void Emitter::emit_tail(std::uint32_t, std::uint32_t target) {
    flush();
    if (p_.function_at(target)) {
        line(fmt("f_%08X(c); return;", target));
    } else {
        ++stats_.unresolved;
        line(fmt("gc_call(c, 0x%08Xu); return;", target));
    }
}

void Emitter::emit_return(std::uint32_t pc) {
    flush(false, false, true);
    if (p_.check_returns) line(fmt("GC_CHECK_RETURN(0x%08Xu, %s, entry_lr);", pc, "lr"));
    line("return;");
}

std::string Emitter::condition(std::uint32_t bo, std::uint32_t bi, bool decrement) {
    std::string c;
    if (decrement && !(bo & 4)) {
        used.ctr = written.ctr = true;
        line("ctr--;");
        c = (bo & 2) ? "ctr == 0" : "ctr != 0";
    }
    if (!(bo & 16)) {
        std::string t = (bo & 8) ? crb(bi) : "!" + crb(bi);
        c = c.empty() ? t : "(" + c + ") && " + t;
    }
    return c;
}

void Emitter::emit_branch(std::uint32_t pc, Insn i, Op op) {
    if (op == Op::b) {
        std::uint32_t target = i.aa() ? static_cast<std::uint32_t>(i.li()) : pc + i.li();
        if (i.lk()) {
            if (target == pc + 4) {
                used.lr = written.lr = true;
                line(fmt("lr = 0x%08Xu;", pc + 4));
            } else {
                emit_call(pc, target);
            }
        } else if (inside(target) && target != f_.start) {
            if (target <= pc) safepoint(pc, target);
            line("goto " + label(target) + ";");
        } else if (target == f_.start) {
            safepoint(pc, target);
            line("goto " + label(target) + ";");
        } else {
            emit_tail(pc, target);
        }
        return;
    }
    if (op == Op::bc) {
        std::uint32_t target = i.aa() ? static_cast<std::uint32_t>(i.bd()) : pc + i.bd();
        std::string c = condition(i.bo(), i.bi(), true);
        if (c.empty()) c = "1";
        line("if (" + c + ") {");
        if (i.lk()) {
            emit_call(pc, target);
        } else if (inside(target)) {
            if (target <= pc) safepoint(pc, target);
            line("goto " + label(target) + ";");
        } else {
            emit_tail(pc, target);
        }
        line("}");
        return;
    }
    if (op == Op::bclr) {
        std::string c = condition(i.bo(), i.bi(), true);
        used.lr = true;
        if (!c.empty()) line("if (" + c + ") {");
        if (i.lk()) {
            line("{ uint32_t t_ = lr;");
            written.lr = true;
            line(fmt("lr = 0x%08Xu;", pc + 4));
            flush(false, true);
            line("gc_call(c, t_); }");
            reload(false, true);
        } else {
            emit_return(pc);
        }
        if (!c.empty()) line("}");
        return;
    }
    if (op == Op::bcctr) {
        std::string c = condition(i.bo(), i.bi(), false);
        used.ctr = true;
        if (!c.empty()) line("if (" + c + ") {");
        if (i.lk()) {
            used.lr = written.lr = true;
            line(fmt("lr = 0x%08Xu;", pc + 4));
            flush(false, true);
            line("gc_call(c, ctr);");
            reload(false, true);
        } else {
            emit_switch(pc);
        }
        if (!c.empty()) line("}");
    }
}

void Emitter::emit_switch(std::uint32_t pc) {
    ++stats_.indirect_jumps;
    auto lo = std::lower_bound(p_.code_pointers.begin(), p_.code_pointers.end(), f_.start);
    std::string cases;
    for (auto it = lo; it != p_.code_pointers.end() && *it < f_.end; ++it) {
        if (*it == f_.start) continue;
        cases += fmt("case 0x%08Xu: goto %s; ", *it, label(*it).c_str());
    }
    if (!cases.empty()) line("switch (ctr) { " + cases + "default: break; }");
    flush();
    line(fmt("gc_jump(c, ctr, 0x%08Xu); return;", pc));
}

void Emitter::emit_load(Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb();
    const char* fn = nullptr;
    bool update = false, indexed = false;
    switch (op) {
    case Op::lwz: fn = "LD32"; break;
    case Op::lwzu: fn = "LD32"; update = true; break;
    case Op::lwzx: fn = "LD32"; indexed = true; break;
    case Op::lwzux: fn = "LD32"; indexed = update = true; break;
    case Op::lbz: fn = "LD8"; break;
    case Op::lbzu: fn = "LD8"; update = true; break;
    case Op::lbzx: fn = "LD8"; indexed = true; break;
    case Op::lbzux: fn = "LD8"; indexed = update = true; break;
    case Op::lhz: fn = "LD16"; break;
    case Op::lhzu: fn = "LD16"; update = true; break;
    case Op::lhzx: fn = "LD16"; indexed = true; break;
    case Op::lhzux: fn = "LD16"; indexed = update = true; break;
    case Op::lha: fn = "LDS16"; break;
    case Op::lhau: fn = "LDS16"; update = true; break;
    case Op::lhax: fn = "LDS16"; indexed = true; break;
    case Op::lhaux: fn = "LDS16"; indexed = update = true; break;
    case Op::lwbrx: fn = "LDBR32"; indexed = true; break;
    case Op::lhbrx: fn = "LDBR16"; indexed = true; break;
    case Op::lwarx: fn = "LD32"; indexed = true; break;
    default: break;
    }
    std::string ea = indexed ? ea_x(a, b) : ea_d(a, i.simm());
    if (update) {
        line(fmt("{ uint32_t ea_ = %s; %s = %s(ea_); %s = ea_; }", ea.c_str(), wg(d).c_str(), fn, wg(a).c_str()));
    } else {
        line(fmt("%s = %s(%s);", wg(d).c_str(), fn, ea.c_str()));
    }
}

void Emitter::emit_store(Insn i, Op op) {
    unsigned s = i.rs(), a = i.ra(), b = i.rb();
    const char* fn = nullptr;
    bool update = false, indexed = false;
    switch (op) {
    case Op::stw: fn = "ST32"; break;
    case Op::stwu: fn = "ST32"; update = true; break;
    case Op::stwx: fn = "ST32"; indexed = true; break;
    case Op::stwux: fn = "ST32"; indexed = update = true; break;
    case Op::stb: fn = "ST8"; break;
    case Op::stbu: fn = "ST8"; update = true; break;
    case Op::stbx: fn = "ST8"; indexed = true; break;
    case Op::stbux: fn = "ST8"; indexed = update = true; break;
    case Op::sth: fn = "ST16"; break;
    case Op::sthu: fn = "ST16"; update = true; break;
    case Op::sthx: fn = "ST16"; indexed = true; break;
    case Op::sthux: fn = "ST16"; indexed = update = true; break;
    case Op::stwbrx: fn = "STBR32"; indexed = true; break;
    case Op::sthbrx: fn = "STBR16"; indexed = true; break;
    case Op::stwcx: fn = "ST32"; indexed = true; break;
    default: break;
    }
    std::string ea = indexed ? ea_x(a, b) : ea_d(a, i.simm());
    if (update) {
        line(fmt("{ uint32_t ea_ = %s; %s(ea_, %s); %s = ea_; }", ea.c_str(), fn, g(s).c_str(), wg(a).c_str()));
    } else {
        line(fmt("%s(%s, %s);", fn, ea.c_str(), g(s).c_str()));
    }
    if (op == Op::stwcx)
        line(fmt("%s = 0; %s = 0; %s = 1; %s = %s;", wcrb(0).c_str(), wcrb(1).c_str(), wcrb(2).c_str(), wcrb(3).c_str(),
                 so().c_str()));
}

void Emitter::emit_arith(std::uint32_t, Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb();
    bool oe = i.oe();
    std::string ov;
    // 64-bit sum with carry out; x + y + cin.
    auto sum = [&](const std::string& x, const std::string& y, const std::string& cin, bool set_ca) {
        std::string s = fmt("{ uint32_t x_ = %s, y_ = %s; uint64_t t_ = (uint64_t)x_ + y_ + %s; %s = (uint32_t)t_;",
                            x.c_str(), y.c_str(), cin.c_str(), wg(d).c_str());
        if (set_ca) {
            wxer_ca();
            s += " ca = (uint8_t)(t_ >> 32);";
        }
        if (oe) {
            wxer_ov();
            s += fmt(" ov = (((x_ ^ (uint32_t)t_) & (y_ ^ (uint32_t)t_)) >> 31) & 1; so |= ov;");
        }
        line(s + " }");
    };
    switch (op) {
    case Op::add: sum(g(a), g(b), "0", false); break;
    case Op::addc: sum(g(a), g(b), "0", true); break;
    case Op::adde: used.ca = true; sum(g(a), g(b), "ca", true); break;
    case Op::addze: used.ca = true; sum(g(a), "0", "ca", true); break;
    case Op::addme: used.ca = true; sum(g(a), "0xFFFFFFFFu", "ca", true); break;
    case Op::subf: sum("~" + g(a), g(b), "1", false); break;
    case Op::subfc: sum("~" + g(a), g(b), "1", true); break;
    case Op::subfe: used.ca = true; sum("~" + g(a), g(b), "ca", true); break;
    case Op::subfze: used.ca = true; sum("~" + g(a), "0", "ca", true); break;
    case Op::subfme: used.ca = true; sum("~" + g(a), "0xFFFFFFFFu", "ca", true); break;
    case Op::neg:
        if (oe) {
            wxer_ov();
            line(fmt("ov = %s == 0x80000000u; so |= ov;", g(a).c_str()));
        }
        line(fmt("%s = 0u - %s;", wg(d).c_str(), g(a).c_str()));
        break;
    case Op::mullw:
        if (oe) {
            wxer_ov();
            line(fmt("{ int64_t t_ = (int64_t)(int32_t)%s * (int32_t)%s; ov = t_ != (int32_t)t_; so |= ov; }",
                     g(a).c_str(), g(b).c_str()));
        }
        line(fmt("%s = (uint32_t)((int32_t)%s * (int64_t)(int32_t)%s);", wg(d).c_str(), g(a).c_str(), g(b).c_str()));
        break;
    case Op::mulhw:
        line(fmt("%s = (uint32_t)(((int64_t)(int32_t)%s * (int32_t)%s) >> 32);", wg(d).c_str(), g(a).c_str(), g(b).c_str()));
        break;
    case Op::mulhwu:
        line(fmt("%s = (uint32_t)(((uint64_t)%s * %s) >> 32);", wg(d).c_str(), g(a).c_str(), g(b).c_str()));
        break;
    case Op::divw:
        if (oe) {
            wxer_ov();
            line(fmt("ov = %s == 0 || (%s == 0x80000000u && %s == 0xFFFFFFFFu); so |= ov;", g(b).c_str(), g(a).c_str(),
                     g(b).c_str()));
        }
        line(fmt("%s = gc_divw(%s, %s);", wg(d).c_str(), g(a).c_str(), g(b).c_str()));
        break;
    case Op::divwu:
        if (oe) {
            wxer_ov();
            line(fmt("ov = %s == 0; so |= ov;", g(b).c_str()));
        }
        line(fmt("%s = gc_divwu(%s, %s);", wg(d).c_str(), g(a).c_str(), g(b).c_str()));
        break;
    default: break;
    }
    if (i.rcbit()) record(g(d));
}

void Emitter::emit_fp(std::uint32_t pc, Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb(), cc = i.rc();
    auto single = [&](const std::string& expr) {
        line(fmt("%s = gc_rs(%s); %s = %s;", wfr(d).c_str(), expr.c_str(), wps(d).c_str(), fr(d).c_str()));
    };
    auto dbl = [&](const std::string& expr) { line(fmt("%s = %s;", wfr(d).c_str(), expr.c_str())); };
    std::string A = fr(a), B = fr(b), C = fr(cc);
    // Operands that are not read are still marked used by the calls above; harmless.
    switch (op) {
    case Op::fdivs: single(A + " / " + B); break;
    case Op::fsubs: single(A + " - " + B); break;
    case Op::fadds: single(A + " + " + B); break;
    case Op::fmuls: single(A + " * " + C); break;
    case Op::fres: single("1.0 / " + B); break;
    case Op::fmadds: single("fma(" + A + ", " + C + ", " + B + ")"); break;
    case Op::fmsubs: single("fma(" + A + ", " + C + ", -" + B + ")"); break;
    case Op::fnmadds: single("-fma(" + A + ", " + C + ", " + B + ")"); break;
    case Op::fnmsubs: single("-fma(" + A + ", " + C + ", -" + B + ")"); break;
    case Op::fdiv: dbl(A + " / " + B); break;
    case Op::fsub: dbl(A + " - " + B); break;
    case Op::fadd: dbl(A + " + " + B); break;
    case Op::fmul: dbl(A + " * " + C); break;
    case Op::fsel: dbl("(" + A + " >= 0.0) ? " + C + " : " + B); break;
    case Op::frsqrte: dbl("1.0 / sqrt(" + B + ")"); break;
    case Op::fmadd: dbl("fma(" + A + ", " + C + ", " + B + ")"); break;
    case Op::fmsub: dbl("fma(" + A + ", " + C + ", -" + B + ")"); break;
    case Op::fnmadd: dbl("-fma(" + A + ", " + C + ", " + B + ")"); break;
    case Op::fnmsub: dbl("-fma(" + A + ", " + C + ", -" + B + ")"); break;
    case Op::fneg: dbl("-" + B); break;
    case Op::fabs: dbl("fabs(" + B + ")"); break;
    case Op::fnabs: dbl("-fabs(" + B + ")"); break;
    case Op::fmr: dbl(B); break;
    case Op::frsp: single(B); break;
    case Op::fctiwz: dbl("gc_fctiwz(" + B + ")"); break;
    case Op::fctiw: dbl("gc_fctiw(c, " + B + ")"); break;
    case Op::fcmpu:
    case Op::fcmpo: {
        unsigned n = i.crfd();
        line(fmt("{ double a_ = %s, b_ = %s; %s = a_ < b_; %s = a_ > b_; %s = a_ == b_; %s = a_ != a_ || b_ != b_; }",
                 A.c_str(), B.c_str(), wcrb(n * 4).c_str(), wcrb(n * 4 + 1).c_str(), wcrb(n * 4 + 2).c_str(),
                 wcrb(n * 4 + 3).c_str()));
        return;
    }
    case Op::mffs: dbl("gc_u2d(0xFFF8000000000000ull | c->fpscr)"); break;
    case Op::mtfsf: line(fmt("gc_mtfsf(c, 0x%02Xu, (uint32_t)gc_d2u(%s));", i.fm(), B.c_str())); break;
    case Op::mtfsfi: {
        unsigned n = i.crfd(), imm = (i.w >> 12) & 15;
        line(fmt("c->fpscr = (c->fpscr & ~(0xFu << %u)) | (%uu << %u);", 28 - 4 * n, imm, 28 - 4 * n));
        break;
    }
    case Op::mtfsb0: line(fmt("c->fpscr &= ~(1u << %u);", 31 - d)); break;
    case Op::mtfsb1: line(fmt("c->fpscr |= 1u << %u;", 31 - d)); break;
    case Op::mcrfs: {
        unsigned n = i.crfd(), s = i.crfs();
        line(fmt("{ uint32_t f_ = (c->fpscr >> %u) & 15; %s = f_ >> 3; %s = (f_ >> 2) & 1; %s = (f_ >> 1) & 1; %s = f_ & 1; }",
                 28 - 4 * s, wcrb(n * 4).c_str(), wcrb(n * 4 + 1).c_str(), wcrb(n * 4 + 2).c_str(), wcrb(n * 4 + 3).c_str()));
        break;
    }
    default:
        line(fmt("gc_illegal(c, 0x%08Xu, 0x%08Xu);", pc, i.w));
        break;
    }
    if (i.rcbit()) record_fp();
}

void Emitter::emit_ps(Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb(), cc = i.rc();
    auto lanes = [&](const std::string& e0, const std::string& e1) {
        line(fmt("{ double a0 = %s, a1 = %s, b0 = %s, b1 = %s, c0 = %s, c1 = %s; (void)a0; (void)a1; (void)b0; (void)b1; (void)c0; "
                 "(void)c1; %s = gc_rs(%s); %s = gc_rs(%s); }",
                 fr(a).c_str(), ps(a).c_str(), fr(b).c_str(), ps(b).c_str(), fr(cc).c_str(), ps(cc).c_str(),
                 wfr(d).c_str(), e0.c_str(), wps(d).c_str(), e1.c_str()));
    };
    auto exact = [&](const std::string& e0, const std::string& e1) {
        line(fmt("{ double a0 = %s, a1 = %s, b0 = %s, b1 = %s; (void)a0; (void)a1; (void)b0; (void)b1; %s = %s; %s = %s; }",
                 fr(a).c_str(), ps(a).c_str(), fr(b).c_str(), ps(b).c_str(), wfr(d).c_str(), e0.c_str(), wps(d).c_str(),
                 e1.c_str()));
    };
    switch (op) {
    case Op::ps_add: lanes("a0 + b0", "a1 + b1"); break;
    case Op::ps_sub: lanes("a0 - b0", "a1 - b1"); break;
    case Op::ps_mul: lanes("a0 * c0", "a1 * c1"); break;
    case Op::ps_div: lanes("a0 / b0", "a1 / b1"); break;
    case Op::ps_madd: lanes("fma(a0, c0, b0)", "fma(a1, c1, b1)"); break;
    case Op::ps_msub: lanes("fma(a0, c0, -b0)", "fma(a1, c1, -b1)"); break;
    case Op::ps_nmadd: lanes("-fma(a0, c0, b0)", "-fma(a1, c1, b1)"); break;
    case Op::ps_nmsub: lanes("-fma(a0, c0, -b0)", "-fma(a1, c1, -b1)"); break;
    case Op::ps_madds0: lanes("fma(a0, c0, b0)", "fma(a1, c0, b1)"); break;
    case Op::ps_madds1: lanes("fma(a0, c1, b0)", "fma(a1, c1, b1)"); break;
    case Op::ps_muls0: lanes("a0 * c0", "a1 * c0"); break;
    case Op::ps_muls1: lanes("a0 * c1", "a1 * c1"); break;
    case Op::ps_sum0: lanes("a0 + b1", "c1"); break;
    case Op::ps_sum1: lanes("c0", "a0 + b1"); break;
    case Op::ps_res: lanes("1.0 / b0", "1.0 / b1"); break;
    case Op::ps_rsqrte: lanes("1.0 / sqrt(b0)", "1.0 / sqrt(b1)"); break;
    case Op::ps_sel: lanes("a0 >= 0.0 ? c0 : b0", "a1 >= 0.0 ? c1 : b1"); break;
    case Op::ps_neg: exact("-b0", "-b1"); break;
    case Op::ps_abs: exact("fabs(b0)", "fabs(b1)"); break;
    case Op::ps_nabs: exact("-fabs(b0)", "-fabs(b1)"); break;
    case Op::ps_mr: exact("b0", "b1"); break;
    case Op::ps_merge00: exact("a0", "b0"); break;
    case Op::ps_merge01: exact("a0", "b1"); break;
    case Op::ps_merge10: exact("a1", "b0"); break;
    case Op::ps_merge11: exact("a1", "b1"); break;
    case Op::ps_cmpu0: case Op::ps_cmpo0: case Op::ps_cmpu1: case Op::ps_cmpo1: {
        bool one = op == Op::ps_cmpu1 || op == Op::ps_cmpo1;
        unsigned n = i.crfd();
        line(fmt("{ double a_ = %s, b_ = %s; %s = a_ < b_; %s = a_ > b_; %s = a_ == b_; %s = a_ != a_ || b_ != b_; }",
                 (one ? ps(a) : fr(a)).c_str(), (one ? ps(b) : fr(b)).c_str(), wcrb(n * 4).c_str(),
                 wcrb(n * 4 + 1).c_str(), wcrb(n * 4 + 2).c_str(), wcrb(n * 4 + 3).c_str()));
        return;
    }
    default: break;
    }
    if (i.rcbit()) record_fp();
}

void Emitter::emit_insn(std::uint32_t pc, Insn i, Op op) {
    unsigned d = i.rd(), a = i.ra(), b = i.rb();
    switch (op) {
    case Op::invalid:
        ++stats_.invalid;
        line(fmt("gc_illegal(c, 0x%08Xu, 0x%08Xu);", pc, i.w));
        return;
    case Op::addi:
        line(fmt("%s = %s + (uint32_t)%d;", wg(d).c_str(), g0(a).c_str(), i.simm()));
        return;
    case Op::addis:
        line(fmt("%s = %s + 0x%08Xu;", wg(d).c_str(), g0(a).c_str(), static_cast<std::uint32_t>(i.simm()) << 16));
        return;
    case Op::addic:
    case Op::addic_rc:
        wxer_ca();
        line(fmt("{ uint64_t t_ = (uint64_t)%s + 0x%08Xu; %s = (uint32_t)t_; ca = (uint8_t)(t_ >> 32); }", g(a).c_str(),
                 static_cast<std::uint32_t>(i.simm()), wg(d).c_str()));
        if (op == Op::addic_rc) record(g(d));
        return;
    case Op::subfic:
        wxer_ca();
        line(fmt("{ uint64_t t_ = (uint64_t)(uint32_t)~%s + 0x%08Xu + 1u; %s = (uint32_t)t_; ca = (uint8_t)(t_ >> 32); }",
                 g(a).c_str(), static_cast<std::uint32_t>(i.simm()), wg(d).c_str()));
        return;
    case Op::mulli:
        line(fmt("%s = (uint32_t)((int64_t)(int32_t)%s * %d);", wg(d).c_str(), g(a).c_str(), i.simm()));
        return;
    case Op::cmpi:
        cr_compare(i.crfd(), "int32_t a_ = (int32_t)" + g(a), fmt("int32_t b_ = %d", i.simm()));
        return;
    case Op::cmpli:
        cr_compare(i.crfd(), "uint32_t a_ = " + g(a), fmt("uint32_t b_ = %uu", i.uimm()));
        return;
    case Op::cmp:
        cr_compare(i.crfd(), "int32_t a_ = (int32_t)" + g(a), "int32_t b_ = (int32_t)" + g(b));
        return;
    case Op::cmpl:
        cr_compare(i.crfd(), "uint32_t a_ = " + g(a), "uint32_t b_ = " + g(b));
        return;
    case Op::andi_rc:
        line(fmt("%s = %s & 0x%04Xu;", wg(a).c_str(), g(d).c_str(), i.uimm()));
        record(g(a));
        return;
    case Op::andis_rc:
        line(fmt("%s = %s & 0x%08Xu;", wg(a).c_str(), g(d).c_str(), i.uimm() << 16));
        record(g(a));
        return;
    case Op::ori:
        if (!d && !a && !i.uimm()) return;  // nop
        line(fmt("%s = %s | 0x%04Xu;", wg(a).c_str(), g(d).c_str(), i.uimm()));
        return;
    case Op::oris: line(fmt("%s = %s | 0x%08Xu;", wg(a).c_str(), g(d).c_str(), i.uimm() << 16)); return;
    case Op::xori: line(fmt("%s = %s ^ 0x%04Xu;", wg(a).c_str(), g(d).c_str(), i.uimm())); return;
    case Op::xoris: line(fmt("%s = %s ^ 0x%08Xu;", wg(a).c_str(), g(d).c_str(), i.uimm() << 16)); return;
    case Op::rlwinm: {
        std::uint32_t m = rotate_mask(i.mb(), i.me());
        line(fmt("%s = GC_ROTL(%s, %u) & 0x%08Xu;", wg(a).c_str(), g(d).c_str(), i.sh(), m));
        if (i.rcbit()) record(g(a));
        return;
    }
    case Op::rlwimi: {
        std::uint32_t m = rotate_mask(i.mb(), i.me());
        line(fmt("%s = (GC_ROTL(%s, %u) & 0x%08Xu) | (%s & 0x%08Xu);", wg(a).c_str(), g(d).c_str(), i.sh(), m,
                 g(a).c_str(), ~m));
        if (i.rcbit()) record(g(a));
        return;
    }
    case Op::rlwnm: {
        std::uint32_t m = rotate_mask(i.mb(), i.me());
        line(fmt("%s = GC_ROTL(%s, %s & 31) & 0x%08Xu;", wg(a).c_str(), g(d).c_str(), g(b).c_str(), m));
        if (i.rcbit()) record(g(a));
        return;
    }
    case Op::and_: case Op::andc: case Op::or_: case Op::orc: case Op::xor_: case Op::nand: case Op::nor: case Op::eqv: {
        const char* f = nullptr;
        switch (op) {
        case Op::and_: f = "%s & %s"; break;
        case Op::andc: f = "%s & ~%s"; break;
        case Op::or_: f = "%s | %s"; break;
        case Op::orc: f = "%s | ~%s"; break;
        case Op::xor_: f = "%s ^ %s"; break;
        case Op::nand: f = "~(%s & %s)"; break;
        case Op::nor: f = "~(%s | %s)"; break;
        default: f = "~(%s ^ %s)"; break;
        }
        line(wg(a) + " = " + fmt(f, g(d).c_str(), g(b).c_str()) + ";");
        if (i.rcbit()) record(g(a));
        return;
    }
    case Op::extsb: line(fmt("%s = (uint32_t)(int32_t)(int8_t)%s;", wg(a).c_str(), g(d).c_str())); if (i.rcbit()) record(g(a)); return;
    case Op::extsh: line(fmt("%s = (uint32_t)(int32_t)(int16_t)%s;", wg(a).c_str(), g(d).c_str())); if (i.rcbit()) record(g(a)); return;
    case Op::cntlzw: line(fmt("%s = %s ? (uint32_t)__builtin_clz(%s) : 32u;", wg(a).c_str(), g(d).c_str(), g(d).c_str())); if (i.rcbit()) record(g(a)); return;
    case Op::slw:
        line(fmt("%s = (%s & 0x20u) ? 0u : (%s << (%s & 31));", wg(a).c_str(), g(b).c_str(), g(d).c_str(), g(b).c_str()));
        if (i.rcbit()) record(g(a));
        return;
    case Op::srw:
        line(fmt("%s = (%s & 0x20u) ? 0u : (%s >> (%s & 31));", wg(a).c_str(), g(b).c_str(), g(d).c_str(), g(b).c_str()));
        if (i.rcbit()) record(g(a));
        return;
    case Op::srawi: {
        wxer_ca();
        unsigned sh = i.sh();
        if (sh == 0) {
            line(fmt("%s = %s; ca = 0;", wg(a).c_str(), g(d).c_str()));
        } else {
            line(fmt("{ uint32_t s_ = %s; ca = ((int32_t)s_ < 0) && (s_ & 0x%08Xu) != 0; %s = (uint32_t)((int32_t)s_ >> %u); }",
                     g(d).c_str(), (1u << sh) - 1, wg(a).c_str(), sh));
        }
        if (i.rcbit()) record(g(a));
        return;
    }
    case Op::sraw:
        wxer_ca();
        line(fmt("{ uint32_t s_ = %s, n_ = %s & 63; if (n_ & 32) { %s = (int32_t)s_ < 0 ? 0xFFFFFFFFu : 0u; ca = (int32_t)s_ < 0; } "
                 "else { ca = ((int32_t)s_ < 0) && (s_ & ((1u << n_) - 1)) != 0; %s = (uint32_t)((int32_t)s_ >> n_); } }",
                 g(d).c_str(), g(b).c_str(), wg(a).c_str(), wg(a).c_str()));
        if (i.rcbit()) record(g(a));
        return;
    case Op::add: case Op::addc: case Op::adde: case Op::addze: case Op::addme: case Op::subf: case Op::subfc:
    case Op::subfe: case Op::subfze: case Op::subfme: case Op::neg: case Op::mullw: case Op::mulhw: case Op::mulhwu:
    case Op::divw: case Op::divwu:
        emit_arith(pc, i, op);
        return;
    case Op::lwz: case Op::lwzu: case Op::lwzx: case Op::lwzux: case Op::lbz: case Op::lbzu: case Op::lbzx: case Op::lbzux:
    case Op::lhz: case Op::lhzu: case Op::lhzx: case Op::lhzux: case Op::lha: case Op::lhau: case Op::lhax: case Op::lhaux:
    case Op::lwbrx: case Op::lhbrx: case Op::lwarx:
        emit_load(i, op);
        return;
    case Op::stw: case Op::stwu: case Op::stwx: case Op::stwux: case Op::stb: case Op::stbu: case Op::stbx: case Op::stbux:
    case Op::sth: case Op::sthu: case Op::sthx: case Op::sthux: case Op::stwbrx: case Op::sthbrx: case Op::stwcx:
        emit_store(i, op);
        return;
    case Op::lmw: {
        std::string s = fmt("{ uint32_t ea_ = %s;", ea_d(a, i.simm()).c_str());
        for (unsigned r = d; r < 32; ++r) s += fmt(" %s = LD32(ea_ + %uu);", wg(r).c_str(), (r - d) * 4);
        line(s + " }");
        return;
    }
    case Op::stmw: {
        std::string s = fmt("{ uint32_t ea_ = %s;", ea_d(a, i.simm()).c_str());
        for (unsigned r = d; r < 32; ++r) s += fmt(" ST32(ea_ + %uu, %s);", (r - d) * 4, g(r).c_str());
        line(s + " }");
        return;
    }
    case Op::lswi: {
        unsigned n = b ? b : 32;
        std::string s = fmt("{ uint32_t ea_ = %s;", g0(a).c_str());
        for (unsigned k = 0; k < n; ++k) {
            unsigned r = (d + k / 4) % 32;
            if (k % 4 == 0) s += fmt(" %s = 0;", wg(r).c_str());
            s += fmt(" %s |= (uint32_t)LD8(ea_ + %uu) << %u;", wg(r).c_str(), k, 24 - 8 * (k % 4));
        }
        line(s + " }");
        return;
    }
    case Op::stswi: {
        unsigned n = b ? b : 32;
        std::string s = fmt("{ uint32_t ea_ = %s;", g0(a).c_str());
        for (unsigned k = 0; k < n; ++k) {
            unsigned r = (d + k / 4) % 32;
            s += fmt(" ST8(ea_ + %uu, (uint8_t)(%s >> %u));", k, g(r).c_str(), 24 - 8 * (k % 4));
        }
        line(s + " }");
        return;
    }
    case Op::lfs: case Op::lfsu: case Op::lfsx: case Op::lfsux: case Op::lfd: case Op::lfdu: case Op::lfdx: case Op::lfdux: {
        bool indexed = op == Op::lfsx || op == Op::lfsux || op == Op::lfdx || op == Op::lfdux;
        bool update = op == Op::lfsu || op == Op::lfsux || op == Op::lfdu || op == Op::lfdux;
        bool single = op == Op::lfs || op == Op::lfsu || op == Op::lfsx || op == Op::lfsux;
        std::string ea = indexed ? ea_x(a, b) : ea_d(a, i.simm());
        std::string s = fmt("{ uint32_t ea_ = %s; ", ea.c_str());
        if (single) s += fmt("%s = (double)LDF32(ea_); %s = %s;", wfr(d).c_str(), wps(d).c_str(), fr(d).c_str());
        else s += fmt("%s = LDF64(ea_);", wfr(d).c_str());
        if (update) s += fmt(" %s = ea_;", wg(a).c_str());
        line(s + " }");
        return;
    }
    case Op::stfs: case Op::stfsu: case Op::stfsx: case Op::stfsux: case Op::stfd: case Op::stfdu: case Op::stfdx:
    case Op::stfdux: case Op::stfiwx: {
        bool indexed = op == Op::stfsx || op == Op::stfsux || op == Op::stfdx || op == Op::stfdux || op == Op::stfiwx;
        bool update = op == Op::stfsu || op == Op::stfsux || op == Op::stfdu || op == Op::stfdux;
        std::string ea = indexed ? ea_x(a, b) : ea_d(a, i.simm());
        std::string s = fmt("{ uint32_t ea_ = %s; ", ea.c_str());
        if (op == Op::stfiwx) s += fmt("ST32(ea_, (uint32_t)gc_d2u(%s));", fr(d).c_str());
        else if (op == Op::stfs || op == Op::stfsu || op == Op::stfsx || op == Op::stfsux) s += fmt("STF32(ea_, (float)%s);", fr(d).c_str());
        else s += fmt("STF64(ea_, %s);", fr(d).c_str());
        if (update) s += fmt(" %s = ea_;", wg(a).c_str());
        line(s + " }");
        return;
    }
    case Op::psq_l: case Op::psq_lu: case Op::psq_lx: case Op::psq_lux: {
        bool indexed = op == Op::psq_lx || op == Op::psq_lux;
        bool update = op == Op::psq_lu || op == Op::psq_lux;
        std::string ea = indexed ? ea_x(a, b) : ea_d(a, i.psq_d());
        unsigned w = indexed ? i.psqx_w() : i.psq_w(), q = indexed ? i.psqx_i() : i.psq_i();
        std::string s = fmt("{ uint32_t ea_ = %s; gc_psq_l(c, M, ea_, %u, %u, &%s, &%s);", ea.c_str(), w, q,
                            wfr(d).c_str(), wps(d).c_str());
        if (update) s += fmt(" %s = ea_;", wg(a).c_str());
        line(s + " }");
        return;
    }
    case Op::psq_st: case Op::psq_stu: case Op::psq_stx: case Op::psq_stux: {
        bool indexed = op == Op::psq_stx || op == Op::psq_stux;
        bool update = op == Op::psq_stu || op == Op::psq_stux;
        std::string ea = indexed ? ea_x(a, b) : ea_d(a, i.psq_d());
        unsigned w = indexed ? i.psqx_w() : i.psq_w(), q = indexed ? i.psqx_i() : i.psq_i();
        std::string s = fmt("{ uint32_t ea_ = %s; gc_psq_st(c, M, ea_, %u, %u, %s, %s);", ea.c_str(), w, q,
                            fr(d).c_str(), ps(d).c_str());
        if (update) s += fmt(" %s = ea_;", wg(a).c_str());
        line(s + " }");
        return;
    }
    case Op::b: case Op::bc: case Op::bclr: case Op::bcctr:
        emit_branch(pc, i, op);
        return;
    case Op::mcrf: {
        unsigned n = i.crfd(), s = i.crfs();
        line(fmt("{ uint8_t t0 = %s, t1 = %s, t2 = %s, t3 = %s; %s = t0; %s = t1; %s = t2; %s = t3; }", crb(s * 4).c_str(),
                 crb(s * 4 + 1).c_str(), crb(s * 4 + 2).c_str(), crb(s * 4 + 3).c_str(), wcrb(n * 4).c_str(),
                 wcrb(n * 4 + 1).c_str(), wcrb(n * 4 + 2).c_str(), wcrb(n * 4 + 3).c_str()));
        return;
    }
    case Op::crand: line(fmt("%s = %s & %s;", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str())); return;
    case Op::crandc: line(fmt("%s = %s & !%s;", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str())); return;
    case Op::cror: line(fmt("%s = %s | %s;", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str())); return;
    case Op::crorc: line(fmt("%s = %s | !%s;", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str())); return;
    case Op::crxor:
        if (a == b && a == d) { line(fmt("%s = 0;", wcrb(d).c_str())); return; }
        line(fmt("%s = %s ^ %s;", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str()));
        return;
    case Op::crnand: line(fmt("%s = !(%s & %s);", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str())); return;
    case Op::crnor: line(fmt("%s = !(%s | %s);", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str())); return;
    case Op::creqv:
        if (a == b && a == d) { line(fmt("%s = 1;", wcrb(d).c_str())); return; }
        line(fmt("%s = !(%s ^ %s);", wcrb(d).c_str(), crb(a).c_str(), crb(b).c_str()));
        return;
    case Op::mfcr: {
        std::string s = wg(d) + " = 0";
        for (unsigned n = 0; n < 32; ++n) s += fmt(" | ((uint32_t)%s << %u)", crb(n).c_str(), 31 - n);
        line(s + ";");
        return;
    }
    case Op::mtcrf: {
        std::string s;
        for (unsigned f = 0; f < 8; ++f) {
            if (!(i.crm() >> (7 - f) & 1)) continue;
            for (unsigned k = 0; k < 4; ++k) {
                unsigned n = f * 4 + k;
                s += fmt("%s = (%s >> %u) & 1; ", wcrb(n).c_str(), g(d).c_str(), 31 - n);
            }
        }
        line(s);
        return;
    }
    case Op::mcrxr: {
        unsigned n = i.crfd();
        wxer_ca();
        wxer_ov();
        line(fmt("%s = so; %s = ov; %s = ca; %s = 0; so = ov = ca = 0;", wcrb(n * 4).c_str(), wcrb(n * 4 + 1).c_str(),
                 wcrb(n * 4 + 2).c_str(), wcrb(n * 4 + 3).c_str()));
        return;
    }
    case Op::mfspr: {
        unsigned spr = i.spr();
        if (spr == 1) {
            used.ca = used.so = used.ov = true;
            line(fmt("%s = ((uint32_t)so << 31) | ((uint32_t)ov << 30) | ((uint32_t)ca << 29) | c->bc;", wg(d).c_str()));
        } else if (spr == 8) {
            used.lr = true;
            line(wg(d) + " = lr;");
        } else if (spr == 9) {
            used.ctr = true;
            line(wg(d) + " = ctr;");
        } else if (spr >= 912 && spr <= 919) {
            line(fmt("%s = c->gqr[%u];", wg(d).c_str(), spr - 912));
        } else {
            line(fmt("%s = gc_mfspr(c, %uu);", wg(d).c_str(), spr));
        }
        return;
    }
    case Op::mtspr: {
        unsigned spr = i.spr();
        if (spr == 1) {
            wxer_ca();
            wxer_ov();
            line(fmt("so = (%s >> 31) & 1; ov = (%s >> 30) & 1; ca = (%s >> 29) & 1; c->bc = %s & 0x7F;", g(d).c_str(),
                     g(d).c_str(), g(d).c_str(), g(d).c_str()));
        } else if (spr == 8) {
            used.lr = written.lr = true;
            line("lr = " + g(d) + ";");
        } else if (spr == 9) {
            used.ctr = written.ctr = true;
            line("ctr = " + g(d) + ";");
        } else if (spr >= 912 && spr <= 919) {
            line(fmt("c->gqr[%u] = %s;", spr - 912, g(d).c_str()));
        } else {
            line(fmt("c->r[1] = %s; gc_mtspr(c, %uu, %s);", g(1).c_str(), spr, g(d).c_str()));
        }
        return;
    }
    case Op::mftb: line(fmt("%s = gc_mftb(%uu);", wg(d).c_str(), i.spr())); return;
    case Op::mfmsr: line(wg(d) + " = gc_msr;"); return;
    case Op::mtmsr: line(fmt("c->r[1] = %s; gc_mtmsr(c, %s);", g(1).c_str(), g(d).c_str())); return;
    case Op::mfsr: line(fmt("%s = gc_sr[%u];", wg(d).c_str(), i.sr())); return;
    case Op::mtsr: line(fmt("gc_sr[%u] = %s;", i.sr(), g(d).c_str())); return;
    case Op::mfsrin: line(fmt("%s = gc_sr[%s >> 28];", wg(d).c_str(), g(b).c_str())); return;
    case Op::mtsrin: line(fmt("gc_sr[%s >> 28] = %s;", g(b).c_str(), g(d).c_str())); return;
    case Op::sync: case Op::isync: case Op::eieio: case Op::dcbst: case Op::dcbf: case Op::dcbt: case Op::dcbtst:
    case Op::dcbi: case Op::icbi: case Op::tlbie: case Op::tlbsync:
        return;
    case Op::dcbz: line(fmt("gc_dcbz(M, %s);", ea_x(a, b).c_str())); return;
    case Op::dcbz_l: line(fmt("gc_dcbz_l(M, %s);", ea_x(a, b).c_str())); return;
    case Op::sc: line("gc_sc(c);"); return;
    case Op::rfi:
        flush(true);
        line(fmt("gc_rfi(c, 0x%08Xu); return;", pc));
        return;
    case Op::tw: case Op::twi: {
        unsigned to = d;
        std::string bv = op == Op::tw ? g(b) : fmt("(uint32_t)%d", i.simm());
        std::string av = g(a);
        std::string cond;
        auto add = [&](const std::string& e) { cond += (cond.empty() ? "" : " || ") + e; };
        if (to & 16) add(fmt("(int32_t)%s < (int32_t)%s", av.c_str(), bv.c_str()));
        if (to & 8) add(fmt("(int32_t)%s > (int32_t)%s", av.c_str(), bv.c_str()));
        if (to & 4) add(fmt("%s == %s", av.c_str(), bv.c_str()));
        if (to & 2) add(fmt("%s < %s", av.c_str(), bv.c_str()));
        if (to & 1) add(fmt("%s > %s", av.c_str(), bv.c_str()));
        if (!cond.empty()) {
            line("if (" + cond + ") {");
            flush(true);
            line(fmt("gc_trap(c, 0x%08Xu); }", pc));
        }
        return;
    }
    case Op::eciwx: case Op::ecowx: case Op::lswx: case Op::stswx:
        ++stats_.invalid;
        line(fmt("gc_illegal(c, 0x%08Xu, 0x%08Xu);", pc, i.w));
        return;
    default:
        break;
    }
    // Floating point and paired singles.
    if (op >= Op::fdivs && op <= Op::mtfsf) {
        emit_fp(pc, i, op);
        return;
    }
    if (op >= Op::ps_sum0 && op <= Op::ps_merge11) {
        emit_ps(i, op);
        return;
    }
    ++stats_.invalid;
    line(fmt("gc_illegal(c, 0x%08Xu, 0x%08Xu);", pc, i.w));
}

std::string Emitter::body() {
    // Labels: branch targets inside the function and indirect-jump candidates.
    for (std::uint32_t pc = f_.start; pc < f_.end; pc += 4) {
        Insn i{p_.image->read32(pc)};
        Op op = decode(i);
        if (op == Op::b && !i.lk()) {
            std::uint32_t t = i.aa() ? static_cast<std::uint32_t>(i.li()) : pc + i.li();
            if (inside(t)) labels_.insert(t);
        } else if (op == Op::bc && !i.lk()) {
            std::uint32_t t = i.aa() ? static_cast<std::uint32_t>(i.bd()) : pc + i.bd();
            if (inside(t)) labels_.insert(t);
        } else if (op == Op::bcctr && !i.lk()) {
            auto lo = std::lower_bound(p_.code_pointers.begin(), p_.code_pointers.end(), f_.start);
            for (auto it = lo; it != p_.code_pointers.end() && *it < f_.end; ++it)
                if (*it != f_.start) labels_.insert(*it);
        }
    }
    labels_.insert(f_.start);
    bool ended = false;
    for (std::uint32_t pc = f_.start; pc < f_.end; pc += 4) {
        Insn i{p_.image->read32(pc)};
        Op op = decode(i);
        if (labels_.count(pc)) out_ += label(pc) + ":;\n";
        out_ += fmt("    /* %08X %08X %s */\n", pc, i.w, op_name(op));
        ++stats_.instructions;
        emit_insn(pc, i, op);
        // Unconditional transfers end straight-line code.
        ended = (op == Op::b && !i.lk()) ||
                ((op == Op::bc || op == Op::bclr || op == Op::bcctr) && !i.lk() && (i.bo() & 0x14) == 0x14) || op == Op::rfi;
    }
    if (!ended) {
        // Falls into the next function.
        emit_tail(f_.end - 4, f_.end);
    }
    return out_;
}

std::string declarations(const Regs& u) {
    std::string s = "    uint8_t* const M = gc_mem; (void)M;\n";
    for (unsigned n = 0; n < 32; ++n)
        if (u.gpr >> n & 1) s += fmt("    uint32_t r%u = c->r[%u];\n", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (u.fpr >> n & 1) s += fmt("    double f%u = c->f[%u];\n", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (u.ps1 >> n & 1) s += fmt("    double p%u = c->ps1[%u];\n", n, n);
    for (unsigned n = 0; n < 32; ++n)
        if (u.cr >> n & 1) s += fmt("    uint8_t cr%u = c->cr[%u];\n", n, n);
    if (u.lr) s += "    uint32_t lr = c->lr;\n";
    if (u.ctr) s += "    uint32_t ctr = c->ctr;\n";
    if (u.ca) s += "    uint8_t ca = c->ca;\n";
    if (u.so) s += "    uint8_t so = c->so;\n";
    if (u.ov) s += "    uint8_t ov = c->ov;\n";
    return s;
}

} // namespace

std::string emit_function(const Program& program, const Function& function, EmitStats& stats) {
    if (!function.hle.empty())
        return fmt("void f_%08X(gc_ctx* c) { hle_%s(c); }\n", function.start, function.hle.c_str());
    if (!function.wrap.empty()) {
        Function inner = function;
        inner.wrap.clear();
        std::string body = emit_function(program, inner, stats);
        std::string name = fmt("f_%08X", function.start);
        body.replace(body.find(name + "(gc_ctx* c)"), name.size(), name + "_body");
        return "static void " + name + "_body(gc_ctx* c);\n" + body +
               fmt("void f_%08X(gc_ctx* c) { wrap_%s(c, %s_body); }\n", function.start, function.wrap.c_str(), name.c_str());
    }
    EmitStats scratch;
    Emitter first(program, function, scratch, nullptr, nullptr);
    first.body();
    Regs used = first.used, written = first.written;
    // Values flushed or reloaded around calls must exist as locals.
    used.lr = used.lr || written.lr;
    Emitter second(program, function, stats, &used, &written);
    std::string body = second.body();
    std::string s = fmt("void f_%08X(gc_ctx* c) {\n", function.start);
    if (!function.name.empty()) s = "/* " + function.name + " */\n" + s;
    s += declarations(used);
    // Functions with OSSaveContext call sites resume through setjmp: keep their locals in memory.
    bool resumes = body.find("gcrt_setjmp") != std::string::npos;
    if (resumes) {
        std::string d = declarations(used), v;
        std::size_t at = 0;
        while (at < d.size()) {
            auto e = d.find('\n', at);
            std::string l = d.substr(at, e - at);
            if (l.find("uint8_t* const M") == std::string::npos) l.insert(4, "volatile ");
            v += l + "\n";
            at = e + 1;
        }
        s = fmt("void f_%08X(gc_ctx* c) {\n", function.start) + v;
    }
    if (program.check_returns) s += "    const uint32_t entry_lr = c->lr; (void)entry_lr;\n";
    return s + body + "}\n";
}

} // namespace gcr
