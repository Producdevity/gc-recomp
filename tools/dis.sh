#!/bin/sh
# Disassembles ADDR..ADDR+SIZE of a DOL with LLVM (paired singles show as raw words).
# Usage: tools/dis.sh DOL ADDR SIZE
set -eu
# LLVM_BIN selects an LLVM install; otherwise the tools come from PATH.
bin=${LLVM_BIN:+$LLVM_BIN/}
dol=$1 addr=$2 size=$3
off=$(python3 -I - "$dol" "$addr" <<'EOF'
import struct, sys
d = open(sys.argv[1], 'rb').read(0x100)
a = int(sys.argv[2], 16)
for i in range(18):
    o, va, s = struct.unpack_from('>I', d, i*4)[0], struct.unpack_from('>I', d, 0x48+i*4)[0], struct.unpack_from('>I', d, 0x90+i*4)[0]
    if s and va <= a < va + s:
        print(o + a - va); break
EOF
)
tmp=$(mktemp)
dd if="$dol" of="$tmp" bs=1 skip="$off" count=$(($3)) 2>/dev/null
"${bin}llvm-objcopy" -I binary -O elf32-powerpc --rename-section=.data=.text,code "$tmp" "$tmp.elf"
"${bin}llvm-objdump" -d --no-show-raw-insn --adjust-vma="$addr" "$tmp.elf" | tail -n +7
rm -f "$tmp" "$tmp.elf"
