#!/usr/bin/env python3
"""Convert a GameCube RVZ image to a plain ISO.

Implements the RVZ container as described in Dolphin's docs/WiaAndRvz.md:
header 1, header 2 (disc), raw-data entries, RVZ group entries, optional zstd
group compression and RVZ packing (raw runs and lagged-Fibonacci junk runs).
GameCube images only (no partitions).

Usage: python3 -I rvz2iso.py IMAGE.rvz OUT.iso
"""
import hashlib
import struct
import sys

try:
    from compression import zstd
except ImportError:  # pragma: no cover
    zstd = None

LFG_K = 521
LFG_J = 32
SEED_WORDS = 17
BLOCK = 0x8000


class Junk:
    """Lagged Fibonacci generator used for GameCube/Wii disc padding."""

    def __init__(self, seed_be: bytes):
        words = list(struct.unpack(">17I", seed_be))
        buf = words + [0] * (LFG_K - SEED_WORDS)
        for i in range(SEED_WORDS, LFG_K):
            buf[i] = ((buf[i - 17] << 23) & 0xFFFFFFFF) ^ (buf[i - 16] >> 9) ^ buf[i - 1]
        # Output bytes are each word's big-endian bytes with bits 16..23 taken
        # from bits 18..25 of the state word.
        self.buf = [(x & 0xFF00FFFF) | ((x >> 2) & 0x00FF0000) for x in buf]
        self.pos = 0
        for _ in range(4):
            self._step()
        self._bytes = None

    def _step(self):
        b = self.buf
        for i in range(LFG_J):
            b[i] ^= b[i + LFG_K - LFG_J]
        for i in range(LFG_J, LFG_K):
            b[i] ^= b[i - LFG_J]
        self._bytes = None

    def _view(self):
        if self._bytes is None:
            self._bytes = struct.pack(">521I", *self.buf)
        return self._bytes

    def skip(self, n):
        self.pos += n
        while self.pos >= LFG_K * 4:
            self._step()
            self.pos -= LFG_K * 4

    def take(self, n):
        out = bytearray()
        while n:
            chunk = min(n, LFG_K * 4 - self.pos)
            out += self._view()[self.pos:self.pos + chunk]
            self.pos += chunk
            n -= chunk
            if self.pos == LFG_K * 4:
                self._step()
                self.pos = 0
        return bytes(out)


def unpack_rvz(data: bytes, disc_offset: int, out_len: int) -> bytes:
    out = bytearray()
    i = 0
    off = disc_offset
    while len(out) < out_len:
        (size,) = struct.unpack_from(">I", data, i)
        i += 4
        junk = bool(size & 0x80000000)
        size &= 0x7FFFFFFF
        if junk:
            gen = Junk(data[i:i + SEED_WORDS * 4])
            i += SEED_WORDS * 4
            gen.skip(off % BLOCK)
            out += gen.take(size)
        else:
            out += data[i:i + size]
            i += size
        off += size
    return bytes(out[:out_len])


def convert(src: str, dst: str) -> str:
    with open(src, "rb") as f:
        img = f.read()
    magic, version, compat, disc_size = struct.unpack_from(">4sIII", img, 0)
    if magic != b"RVZ\x01":
        raise SystemExit("not an RVZ image")
    iso_size, file_size = struct.unpack_from(">QQ", img, 0x24)
    d = 0x48
    disc_type, compression, level, chunk = struct.unpack_from(">IIiI", img, d)
    if disc_type != 1:
        raise SystemExit("only GameCube images are supported")
    dhead = img[d + 0x10:d + 0x90]
    p = d + 0x90
    n_part, part_size, part_off = struct.unpack_from(">IIQ", img, p)
    p += 16 + 20
    n_raw, raw_off, raw_size, n_groups, group_off, group_size = struct.unpack_from(">IQIIQI", img, p)

    def table(off, csize):
        blob = img[off:off + csize]
        if compression == 5:
            return zstd.decompress(blob)
        if compression in (0, 1):
            return blob
        raise SystemExit(f"unsupported compression {compression}")

    raws = table(raw_off, raw_size)
    groups = table(group_off, group_size)
    out = bytearray(iso_size)
    out[0:0x80] = dhead
    for r in range(n_raw):
        doff, dsize, gidx, gcount = struct.unpack_from(">QQII", raws, r * 24)
        skipped = doff % BLOCK
        base = doff - skipped
        total = dsize + skipped
        for g in range(gcount):
            goff, gsize, packed = struct.unpack_from(">III", groups, (gidx + g) * 12)
            start = base + g * chunk
            length = min(chunk, base + total - start)
            compressed = bool(gsize & 0x80000000)
            gsize &= 0x7FFFFFFF
            if gsize == 0:
                data = bytes(length)
            else:
                data = img[goff << 2:(goff << 2) + gsize]
                if compressed:
                    data = zstd.decompress(data)
                if packed:
                    data = unpack_rvz(data, start, length)
            # The first group overlaps the 0x80-byte header stored separately.
            lo = max(start, doff)
            out[lo:start + length] = data[lo - start:length]
    with open(dst, "wb") as f:
        f.write(out)
    return hashlib.sha1(out).hexdigest()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    print(convert(sys.argv[1], sys.argv[2]))
