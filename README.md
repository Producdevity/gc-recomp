# gc-recomp

Static recompiler and runtime for GameCube games on Linux AArch64 handhelds. Shipped
ports contain no code from the game: at first launch the frontend translates the
executable on the player's own disc straight to AArch64 (the gcbe backend) and caches the
result; a C path (DOL -> C -> clang at build time) is the reference for differential
testing. Both run over a model of the console's memory and hardware. Graphics go through
[Starlet](https://github.com/Producdevity/starlet), which decodes the GX command stream the game writes.

## Layout

```
recompiler/   gcrecomp: DOL -> C (decoder, emitter, image and config loading);
              gcbe: DOL -> AArch64 code image (a64_backend, a64_translate)
backend/      loader that maps gcbe images into the process
runtime/      gcrt: guest memory, CPU state, OS context bridge, devices, disc images
frontend/     SDL2 + OpenGL ES 3.1 frontend shared by the ports
cmake/        AArch64 cross toolchain file
tests/        instruction tests, disc reader check
tools/        headless runner, disc helpers (rvz2iso.py, gcdisc.py, dis.sh)
```

## Recompiler

Input: the DOL (or the disc image it is read from), a function list (dtk
`symbols.txt` from a decompilation project) and a per-game configuration. Bl targets
missing from the list become extra entries. Each guest function becomes one C function
`void f_XXXXXXXX(gc_ctx*)`:

- Guest registers live in C locals, loaded at entry. At calls and returns the code
  writes back only what the EABI makes visible to the other side: volatile registers
  (r0, r3-r12, f0-f13 and their paired-single halves, CR fields 0, 1, 5, 6, 7, CTR, XER,
  LR) plus r1, r2 and r13. Callee-saved registers stay in locals; an ABI-conforming
  callee restores them. Functions can opt into exact write-back (`exact`).
- Branches inside a function are `goto`s. `bctr` dispatches through a `switch` over
  every data word pointing into the function (jump tables), then through the global
  function table (indirect tail calls).
- Loop back-edges are safepoints: a single flag test, and interrupt delivery when set.
  Loops that only reread memory through unchanged address registers are spin loops:
  they let the runtime sleep until the next event.
- Memory accesses byte-swap through a 4 GiB host reservation where guest address A is at
  `gc_mem + A`. Accesses to 0xC8000000-0xCFFFFFFF go to the runtime (hardware registers,
  write-gather pipe, EFB).
- Gekko floating point: FPRs are doubles with a separate paired-single half; single
  operations round to single and fill both halves; quantized loads and stores follow the
  GQRs.

Configuration directives (`hle`, `save_context`, `function`, `exact`, `skip`) are
described in `recompiler/image.hpp`. Some SDK and game functions are replaced by runtime
code wherever a function has the same length and hash as the one replaced
(`recompiler/natives.hpp`); the source holds no game code.

The AArch64 backend (gcbe) translates the same function list with the same guest
semantics and write-back rules straight to machine code, writing back only registers
whose values changed since they were loaded. Its image is checked against the DOL's
SHA-1 and the translator version. The runtime receives either kind of code as a
`gcrt::Program` (`gcrt/program.hpp`). Tests run random functions through both paths and
compare state and memory (`tests/gcbe_fuzz_*`).

## Runtime

- **Threads.** The game's own Dolphin OS scheduler runs recompiled. Only
  `OSSaveContext` and `OSLoadContext` are replaced: every guest thread runs on its own
  host fiber, a call site of `OSSaveContext` gets a resume point, and `OSLoadContext`
  unwinds to it, switches to the fiber that owns it, or starts a new fiber for a context
  set up by `OSCreateThread`. See `runtime/src/os.cpp`.
- **Interrupts and exceptions.** Delivered at safepoints while MSR[EE] is set: the state
  is saved into the current `OSContext` as the exception vector would, and the handler
  from the OS exception table runs on a copy of the registers. External interrupts come
  from the processor interface's cause and mask; the decrementer drives `OSAlarm`.
- **Time.** The host monotonic clock drives the time base (40.5 MHz), the decrementer,
  the VI beam and an event queue; a timer thread raises the safepoint flag when an event
  falls due.
- **Devices.** PI, MI, VI (display interrupts by beam line), SI (four controller ports,
  transfers and polling), DI (ISO/GCM and RVZ through Starlet's disc module), EXI (IPL chip: RTC, SRAM, UART;
  memory cards), DSP (boot ROM handshake, ARAM DMA, audio DMA), AI (streaming sample
  counter), CP/PE and the write-gather pipe.
- **Memory cards** are raw card images (slot A: `SAVES/memcard-a.raw`, created as an
  unformatted 59-block card; 4-64 Mbit images from elsewhere work). The card answers the
  SDK's commands, reports itself unlocked, and signals program and erase completion with
  its EXI interrupt; SRAM's flash IDs are derived from each card's header so that any
  formatted card mounts. Images are written shortly after the last change and on exit.
- **DSP microcode** is identified by a hash of the uploaded IRAM image and served by a
  host model (`runtime/src/ucode.cpp`). Both games' JAudio microcodes run on one engine
  (`runtime/src/jaudio_dsp.cpp`) that keeps the DSP's data memory and follows the
  microcode's mixing arithmetic: AFC/PCM sources and oscillators, resampling, filters,
  sends with ramps, surround folds, reverb/echo and chorus lines. The per-game mail
  protocols are `jaudio.cpp` (Luigi's Mansion's early JAudio) and `jasystem.cpp`
  (Sunshine's JASystem).
- **Graphics.** Gather-pipe bursts land in the CPU FIFO in guest memory like the
  hardware's; while the command processor is linked to that FIFO they also go to
  Starlet's decoder. Draw-done and token BP writes raise pixel-engine interrupts; copy
  executes become Starlet display or texture copies; VI retraces present the XFB.
  Texture copies stay on the GPU; their RAM is stamped so that memory the game reuses
  keeps the game's bytes (Starlet `stamp_resident_copies`). The frontend runs GL on
  Starlet's submission thread; the guest thread decodes and records.

## Building

Host (macOS or Linux, AArch64): the recompiler, runtime, tests and a headless runner.
Starlet is a submodule in `third_party/starlet`; `-DGC_STARLET_DIR=...` uses another checkout.

```sh
git clone --recurse-submodules https://github.com/Producdevity/gc-recomp
```

```sh
cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/host && ctest --test-dir build/host
```

`-DGC_NATIVES_DOL=main.dol`, with Luigi's Mansion's (GLME01) executable, adds the test that
compares those replacements with the translated functions they replace.

`-DGC_GAME_DOL=main.dol -DGC_GAME_SYMBOLS=symbols.txt -DGC_GAME_CONFIG=game.cfg` adds
`gc-headless DISC [SECONDS]`, which runs a game without video or audio output. With
`GCRT_SAMPLE=1` it samples the guest thread and names recompiled functions.

Ports add this directory with `add_subdirectory`, set `GC_FRONTEND=ON` and
`STARLET_GLES=ON`, and call `gc_recompile(target DISC game.iso SYMBOLS ... CONFIG ...)`.
Cross builds pass a host-built `gcrecomp` as `RECOMPILER`.

Profile-guided optimization is opt-in. Build an instrumented frontend with
`-DGC_PROFILE_GENERATE=ON -DGC_PROFILE_RUNTIME=.../libclang_rt.profile-aarch64.a`, run it on
a device with `LLVM_PROFILE_FILE=/tmp/NAME.profraw`, merge the `.profraw` files with
`llvm-profdata merge -o FILE.profdata`, then build with `-DGC_PROFILE_USE=FILE.profdata`. The
frontend writes its profile when it exits (`--seconds` or a frontend quit). Code a frontend
translates at run time is not part of the profile.

## Sources

Independent implementation. Instruction semantics follow the PowerPC architecture and
the 750CL (Gekko) paired-single and quantization definitions; hardware registers follow
YAGCD and the Dolphin SDK headers and decompiled SDK sources of the GameCube
decompilation projects (doldecomp/sms, Moddimation/Yasiki), which also document the
DSP task and JAudio mail protocols. The test RVZ writer follows the format described in
Dolphin's `docs/WiaAndRvz.md`. No code from Dolphin, DolRecomp, XenonRecomp, N64Recomp,
WiiCompiled, Aurora or other ports is used.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

gc-recomp is licensed under the GNU General Public License v3.0; see [LICENSE](LICENSE).
It links Starlet (GPL-3.0) and, through Starlet, the Zstandard decoder (BSD). Code that
gc-recomp translates from a game disc remains the game's and is not covered by this license.
