// AArch64 fiber switch for guest threads.
// void gcrt_fiber_switch(void** save_sp, void* load_sp)
// void gcrt_fiber_start(void)   first frame of a new fiber: calls gcrt_fiber_main(x19)

#ifdef __APPLE__
#define SYM(name) _##name
#else
#define SYM(name) name
#endif

    .text
    .p2align 4
    .globl SYM(gcrt_fiber_switch)
SYM(gcrt_fiber_switch):
    sub sp, sp, #160
    stp x19, x20, [sp, #0]
    stp x21, x22, [sp, #16]
    stp x23, x24, [sp, #32]
    stp x25, x26, [sp, #48]
    stp x27, x28, [sp, #64]
    stp x29, x30, [sp, #80]
    stp d8, d9, [sp, #96]
    stp d10, d11, [sp, #112]
    stp d12, d13, [sp, #128]
    stp d14, d15, [sp, #144]
    mov x2, sp
    str x2, [x0]
    mov sp, x1
    ldp x19, x20, [sp, #0]
    ldp x21, x22, [sp, #16]
    ldp x23, x24, [sp, #32]
    ldp x25, x26, [sp, #48]
    ldp x27, x28, [sp, #64]
    ldp x29, x30, [sp, #80]
    ldp d8, d9, [sp, #96]
    ldp d10, d11, [sp, #112]
    ldp d12, d13, [sp, #128]
    ldp d14, d15, [sp, #144]
    add sp, sp, #160
    ret

    .p2align 4
    .globl SYM(gcrt_fiber_start)
SYM(gcrt_fiber_start):
    mov x0, x19
    mov x29, #0
    bl SYM(gcrt_fiber_main)
    brk #0

// int gcrt_setjmp(void* buffer)   (22 words) -- returns 0 now, 1 from gcrt_longjmp
// void gcrt_longjmp(void* buffer)
    .p2align 4
    .globl SYM(gcrt_setjmp)
SYM(gcrt_setjmp):
    stp x19, x20, [x0, #0]
    stp x21, x22, [x0, #16]
    stp x23, x24, [x0, #32]
    stp x25, x26, [x0, #48]
    stp x27, x28, [x0, #64]
    stp x29, x30, [x0, #80]
    mov x1, sp
    str x1, [x0, #96]
    stp d8, d9, [x0, #104]
    stp d10, d11, [x0, #120]
    stp d12, d13, [x0, #136]
    stp d14, d15, [x0, #152]
    mov w0, #0
    ret

    .p2align 4
    .globl SYM(gcrt_longjmp)
SYM(gcrt_longjmp):
    ldp x19, x20, [x0, #0]
    ldp x21, x22, [x0, #16]
    ldp x23, x24, [x0, #32]
    ldp x25, x26, [x0, #48]
    ldp x27, x28, [x0, #64]
    ldp x29, x30, [x0, #80]
    ldr x1, [x0, #96]
    mov sp, x1
    ldp d8, d9, [x0, #104]
    ldp d10, d11, [x0, #120]
    ldp d12, d13, [x0, #136]
    ldp d14, d15, [x0, #152]
    mov w0, #1
    ret

#ifndef __APPLE__
    .section .note.GNU-stack,"",%progbits
#endif
