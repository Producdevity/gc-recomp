#!/usr/bin/env python3
"""Inspect a GameCube ISO: header, DOL sections and file system.

Usage:
  python3 -I gcdisc.py info IMAGE.iso
  python3 -I gcdisc.py dol IMAGE.iso OUT.dol
  python3 -I gcdisc.py ls IMAGE.iso
  python3 -I gcdisc.py extract IMAGE.iso DISC_PATH OUT
"""
import struct
import sys


def read(f, off, n):
    f.seek(off)
    return f.read(n)


def dol_size(hdr: bytes) -> int:
    offs = struct.unpack_from(">18I", hdr, 0)
    sizes = struct.unpack_from(">18I", hdr, 0x90)
    return max(o + s for o, s in zip(offs, sizes) if s)


def dol_info(hdr: bytes):
    offs = struct.unpack_from(">18I", hdr, 0)
    addrs = struct.unpack_from(">18I", hdr, 0x48)
    sizes = struct.unpack_from(">18I", hdr, 0x90)
    bss_addr, bss_size, entry = struct.unpack_from(">III", hdr, 0xD8)
    secs = []
    for i in range(18):
        if sizes[i]:
            secs.append(("T%d" % i if i < 7 else "D%d" % (i - 7), offs[i], addrs[i], sizes[i]))
    return secs, bss_addr, bss_size, entry


def fst_entries(f):
    fst_off, fst_size = struct.unpack(">II", read(f, 0x424, 8))
    fst = read(f, fst_off, fst_size)
    n = struct.unpack_from(">I", fst, 8)[0]
    names = fst[n * 12:]

    def name(o):
        e = names.index(b"\0", o)
        return names[o:e].decode("shift_jis", "replace")

    out = []
    stack = [(n, "")]
    for i in range(1, n):
        while stack and i >= stack[-1][0]:
            stack.pop()
        w0, a, b = struct.unpack_from(">III", fst, i * 12)
        is_dir = w0 >> 24
        path = stack[-1][1] + "/" + name(w0 & 0xFFFFFF) if stack else name(w0 & 0xFFFFFF)
        if is_dir:
            stack.append((b, path))
            out.append((path + "/", 0, 0))
        else:
            out.append((path, a, b))
    return out


def main():
    cmd, image = sys.argv[1], sys.argv[2]
    with open(image, "rb") as f:
        hdr = read(f, 0, 0x440)
        dol_off, fst_off, fst_size = struct.unpack_from(">III", hdr, 0x420)
        if cmd == "info":
            print("id", hdr[0:6].decode(), "rev", hdr[7], "name", hdr[0x20:0x60].split(b"\0")[0].decode())
            print("dol 0x%x fst 0x%x size 0x%x" % (dol_off, fst_off, fst_size))
            secs, ba, bs, ep = dol_info(read(f, dol_off, 0x100))
            for s in secs:
                print("  %-4s file 0x%06x addr 0x%08x size 0x%06x" % s)
            print("  bss 0x%08x size 0x%x entry 0x%08x" % (ba, bs, ep))
            app = read(f, 0x2440, 0x20)
            print("apploader", app[0:10].decode(errors="replace"), "entry 0x%08x size 0x%x" % struct.unpack_from(">II", app, 0x10))
        elif cmd == "dol":
            h = read(f, dol_off, 0x100)
            data = read(f, dol_off, dol_size(h))
            with open(sys.argv[3], "wb") as o:
                o.write(data)
        elif cmd == "ls":
            for p, o, s in fst_entries(f):
                print("%10d 0x%08x %s" % (s, o, p))
        elif cmd == "extract":
            want = sys.argv[3]
            for p, o, s in fst_entries(f):
                if p == want:
                    with open(sys.argv[4], "wb") as out:
                        out.write(read(f, o, s))
                    return
            raise SystemExit("not found")


if __name__ == "__main__":
    main()
