// Loader for translated code images: maps the code executable, binds runtime imports and
// builds the guest function table. Also provides the out-of-line helpers the translated
// code calls for rare paths.
#include "gcbe/loader.h"

#include "gcbe/blob.h"

#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#ifdef __APPLE__
#include <libkern/OSCacheControl.h>
#include <pthread.h>
#endif

extern "C" {

uint64_t gcbe_io_read64(uint32_t ea) { return (uint64_t(gc_io_read32(ea)) << 32) | gc_io_read32(ea + 4); }

struct gcbe_pair { double ps0, ps1; };

gcbe_pair gcbe_psq_l(gc_ctx* c, uint32_t ea, uint32_t w, uint32_t i) {
    gcbe_pair r;
    gc_psq_l(c, gc_mem, ea, w, i, &r.ps0, &r.ps1);
    return r;
}

void gcbe_psq_st(gc_ctx* c, uint32_t ea, uint32_t w, uint32_t i, double ps0, double ps1) {
    gc_psq_st(c, gc_mem, ea, w, i, ps0, ps1);
}

} // extern "C"

struct gcbe_program {
    std::uint8_t dol_sha1[20] = {};
    std::uint8_t* code = nullptr;
    std::size_t mapped = 0;
    std::uint32_t flags = 0;
    std::uint64_t dol_hash = 0;
    std::vector<gc_func_entry> functions;
    std::vector<std::uint32_t> sizes;
};

namespace {

struct Symbol {
    const char* name;
    const void* address;
};

const Symbol kRuntime[] = {
    {"gc_mem", &gc_mem}, {"gc_attn", (const void*)&gc_attn}, {"gc_steps", &gc_steps},
    {"gc_step_deadline", &gc_step_deadline}, {"gc_msr", &gc_msr}, {"gc_sr", gc_sr},
    {"gc_safepoint", (const void*)&gc_safepoint}, {"gc_spin", (const void*)&gc_spin}, {"gc_call", (const void*)&gc_call},
    {"gc_jump", (const void*)&gc_jump}, {"gc_mtmsr", (const void*)&gc_mtmsr}, {"gc_mfspr", (const void*)&gc_mfspr},
    {"gc_mtspr", (const void*)&gc_mtspr}, {"gc_mftb", (const void*)&gc_mftb}, {"gc_sc", (const void*)&gc_sc},
    {"gc_rfi", (const void*)&gc_rfi}, {"gc_trap", (const void*)&gc_trap}, {"gc_illegal", (const void*)&gc_illegal},
    {"gcrt_setjmp", (const void*)&gcrt_setjmp}, {"gc_resume_slot", (const void*)&gc_resume_slot},
    {"gc_mtfsf", (const void*)&gc_mtfsf}, {"gc_fctiw", (const void*)&gc_fctiw},
    {"gc_io_read8", (const void*)&gc_io_read8}, {"gc_io_read16", (const void*)&gc_io_read16},
    {"gc_io_read32", (const void*)&gc_io_read32}, {"gcbe_io_read64", (const void*)&gcbe_io_read64},
    {"gc_io_write8", (const void*)&gc_io_write8}, {"gc_io_write16", (const void*)&gc_io_write16},
    {"gc_io_write32", (const void*)&gc_io_write32}, {"gc_io_write64", (const void*)&gc_io_write64},
    {"gcbe_psq_l", (const void*)&gcbe_psq_l}, {"gcbe_psq_st", (const void*)&gcbe_psq_st},
    {"gc_gp", (const void*)&gc_gp}, {"gc_gather_burst", (const void*)&gc_gather_burst},
    {"hle_Nop", (const void*)&hle_Nop}, {"hle_PSMTXConcat", (const void*)&hle_PSMTXConcat},
    {"hle_PSVECAdd", (const void*)&hle_PSVECAdd}, {"hle_PSVECSubtract", (const void*)&hle_PSVECSubtract}, {"hle_PSVECScale", (const void*)&hle_PSVECScale}, {"hle_PSVECNormalize", (const void*)&hle_PSVECNormalize}, {"hle_PSVECSquareMag", (const void*)&hle_PSVECSquareMag}, {"hle_PSVECCrossProduct", (const void*)&hle_PSVECCrossProduct},
    {"hle_PSVECMag", (const void*)&hle_PSVECMag}, {"hle_WriteMTXPS4x3", (const void*)&hle_WriteMTXPS4x3},
    {"hle_WriteMTXPS3x3from3x4", (const void*)&hle_WriteMTXPS3x3from3x4}, {"hle_VecNormalizeSafe", (const void*)&hle_VecNormalizeSafe},
    {"hle_NameHashFind", (const void*)&hle_NameHashFind}, {"hle_NameParamGet", (const void*)&hle_NameParamGet}, {"hle_FindEntryByKey", (const void*)&hle_FindEntryByKey},
};

const void* resolve(const std::string& name) {
    for (const auto& s : kRuntime)
        if (name == s.name) return s.address;
    // hle_ and wrap_ replacements live in the runtime or the port's own sources.
    return dlsym(RTLD_DEFAULT, name.c_str());
}

void fail(char* error, std::size_t size, const std::string& message) {
    if (error && size) std::snprintf(error, size, "%s", message.c_str());
}

} // namespace

extern "C" {

gcbe_program* gcbe_load(const char* path, char* error, size_t error_size) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) {
        fail(error, error_size, std::string("cannot open ") + path);
        return nullptr;
    }
    std::vector<std::uint8_t> file;
    std::uint8_t chunk[1 << 16];
    for (std::size_t n; (n = std::fread(chunk, 1, sizeof chunk, f)) > 0;) file.insert(file.end(), chunk, chunk + n);
    std::fclose(f);
    gcbe_header h;
    if (file.size() < sizeof h) {
        fail(error, error_size, "translated code: file too small");
        return nullptr;
    }
    std::memcpy(&h, file.data(), sizeof h);
    if (h.magic != GCBE_MAGIC || h.version != GCBE_VERSION || h.translator != GCBE_TRANSLATOR || h.file_size != file.size() ||
        std::uint64_t(h.code_offset) + h.code_size > file.size()) {
        fail(error, error_size, "translated code: bad header or version");
        return nullptr;
    }
    std::vector<const void*> imports;
    const char* names = reinterpret_cast<const char*>(file.data() + h.imports_offset);
    for (std::uint32_t i = 0; i < h.import_count; ++i) {
        std::string name = names;
        names += name.size() + 1;
        const void* a = resolve(name);
        if (!a) {
            fail(error, error_size, "translated code needs " + name + ", which this executable does not export");
            return nullptr;
        }
        imports.push_back(a);
    }
    std::size_t page = static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
    std::size_t mapped = (h.code_size + page - 1) / page * page;
#ifdef __APPLE__
    void* mem = mmap(nullptr, mapped, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);
#else
    void* mem = mmap(nullptr, mapped, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
#endif
    if (mem == MAP_FAILED) {
        fail(error, error_size, "cannot map memory for translated code");
        return nullptr;
    }
    auto* code = static_cast<std::uint8_t*>(mem);
#ifdef __APPLE__
    pthread_jit_write_protect_np(0);
#endif
    std::memcpy(code, file.data() + h.code_offset, h.code_size);
    const auto* relocs = reinterpret_cast<const gcbe_reloc*>(file.data() + h.relocs_offset);
    for (std::uint32_t r = 0; r < h.reloc_count; ++r) {
        if (relocs[r].import >= imports.size() || relocs[r].offset + 8 > h.code_size) {
            fail(error, error_size, "translated code: bad relocation");
            munmap(mem, mapped);
            return nullptr;
        }
        auto value = reinterpret_cast<std::uint64_t>(imports[relocs[r].import]);
        std::memcpy(code + relocs[r].offset, &value, 8);
    }
#ifdef __APPLE__
    pthread_jit_write_protect_np(1);
    sys_icache_invalidate(code, h.code_size);
#else
    if (mprotect(mem, mapped, PROT_READ | PROT_EXEC) != 0) {
        fail(error, error_size, "cannot make translated code executable");
        munmap(mem, mapped);
        return nullptr;
    }
    __builtin___clear_cache(reinterpret_cast<char*>(code), reinterpret_cast<char*>(code + h.code_size));
#endif
    auto* p = new gcbe_program;
    p->code = code;
    p->mapped = mapped;
    p->flags = h.flags;
    p->dol_hash = h.dol_hash;
    std::memcpy(p->dol_sha1, h.dol_sha1, 20);
    const auto* fns = reinterpret_cast<const gcbe_function*>(file.data() + h.functions_offset);
    p->functions.reserve(h.function_count);
    for (std::uint32_t n = 0; n < h.function_count; ++n) {
        gc_func_entry e;
        e.start = fns[n].start;
        e.end = fns[n].end;
        e.func = reinterpret_cast<gc_func>(code + fns[n].code_offset);
        p->functions.push_back(e);
        p->sizes.push_back(fns[n].code_size);
    }
    return p;
}

unsigned gcbe_function_count(const gcbe_program* p) { return static_cast<unsigned>(p->functions.size()); }
const gc_func_entry* gcbe_functions(const gcbe_program* p) { return p->functions.data(); }
uint64_t gcbe_dol_hash(const gcbe_program* p) { return p->dol_hash; }
uint32_t gcbe_flags(const gcbe_program* p) { return p->flags; }
void gcbe_dol_sha1(const gcbe_program* p, uint8_t out[20]) { std::memcpy(out, p->dol_sha1, 20); }

int gcbe_cache_valid(const char* path, const uint8_t dol_sha1[20], uint32_t flags) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) return 0;
    gcbe_header h;
    bool ok = std::fread(&h, sizeof h, 1, f) == 1;
    std::fseek(f, 0, SEEK_END);
    long size = std::ftell(f);
    std::fclose(f);
    return ok && h.magic == GCBE_MAGIC && h.version == GCBE_VERSION && h.translator == GCBE_TRANSLATOR &&
           h.flags == flags && size == static_cast<long>(h.file_size) && std::memcmp(h.dol_sha1, dol_sha1, 20) == 0;
}

void gcbe_write_perf_map(const gcbe_program* p) {
    char path[64];
    std::snprintf(path, sizeof path, "/tmp/perf-%d.map", static_cast<int>(getpid()));
    std::FILE* f = std::fopen(path, "w");
    if (!f) return;
    for (std::size_t n = 0; n < p->functions.size(); ++n)
        std::fprintf(f, "%llx %x f_%08X\n", static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(p->functions[n].func)),
                     p->sizes[n], p->functions[n].start);
    std::fclose(f);
}

} // extern "C"
