#!/usr/bin/env python3
"""Names unnamed functions of one DOL from the named functions of another.

Functions are compared by their instructions with relocated fields masked (branch
displacements, address immediates of addi/addis/ori/oris/loads/stores). A name is
taken only when the masked body is unique on both sides.

Usage: python3 -I match_symbols.py TARGET.dol TARGET_SYMBOLS REF.dol REF_SYMBOLS > names.txt
Output: "ADDR NAME" lines for target functions named fn_... in TARGET_SYMBOLS.
"""
import re
import struct
import sys
from collections import defaultdict

SYM = re.compile(r"^(\S+) = \.(?:text|init):0x([0-9A-Fa-f]+); // type:function size:0x([0-9A-Fa-f]+)")


def load_dol(path):
    d = open(path, "rb").read()
    secs = []
    for i in range(18):
        o, a, s = (struct.unpack_from(">I", d, i * 4)[0], struct.unpack_from(">I", d, 0x48 + i * 4)[0],
                   struct.unpack_from(">I", d, 0x90 + i * 4)[0])
        if s:
            secs.append((a, s, o))

    def read(addr, n):
        for a, s, o in secs:
            if a <= addr and addr + n <= a + s:
                return d[o + addr - a:o + addr - a + n]
        return None
    return read


def mask(w):
    op = w >> 26
    if op in (16, 18):
        return w & (0xFFFF0003 if op == 16 else 0xFC000003)
    if op in (14, 15, 24, 25) or 32 <= op <= 55:
        return w & 0xFFFF0000
    if op in (56, 57, 60, 61):
        return w & 0xFFFFF000
    return w


def functions(read, path):
    out = []
    for line in open(path):
        m = SYM.match(line)
        if not m:
            continue
        name, addr, size = m.group(1), int(m.group(2), 16), int(m.group(3), 16)
        body = read(addr, size)
        if body is None or size < 12:
            continue
        key = tuple(mask(w) for w in struct.unpack(">%dI" % (size // 4), body))
        out.append((addr, name, key))
    return out


def main():
    tread, tsyms, rread, rsyms = load_dol(sys.argv[1]), sys.argv[2], load_dol(sys.argv[3]), sys.argv[4]
    target = functions(tread, tsyms)
    ref = functions(rread, rsyms)
    ref_by = defaultdict(list)
    for a, n, k in ref:
        ref_by[k].append(n)
    tgt_by = defaultdict(list)
    for a, n, k in target:
        tgt_by[k].append((a, n))
    named = 0
    for k, items in tgt_by.items():
        names = ref_by.get(k)
        if not names or len(names) != 1 or len(items) != 1:
            continue
        a, n = items[0]
        if not n.startswith("fn_") or names[0].startswith(("fn_", "lbl_")):
            continue
        print("%08X %s" % (a, names[0]))
        named += 1
    print("# named %d of %d target functions" % (named, len(target)), file=sys.stderr)


if __name__ == "__main__":
    main()
