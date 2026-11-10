# Disc reader check: compares an RVZ against its ISO byte for byte (needs both images).
add_executable(gc-disc-compare disc_compare.cpp)
target_link_libraries(gc-disc-compare PRIVATE gcrt)

# MEM1 write generations for the GX caches.
add_executable(gc-ram-tracking-test ram_tracking_run.cpp)
target_link_libraries(gc-ram-tracking-test PRIVATE gcrt)
add_test(NAME ram_tracking COMMAND gc-ram-tracking-test)

# Draw-sync tokens with EFB snapshots.
add_executable(gc-gx-sync-test gx_sync_run.cpp)
target_link_libraries(gc-gx-sync-test PRIVATE gcrt)
add_test(NAME gx_sync COMMAND gc-gx-sync-test)

# DSP mailbox polls run due events.
add_executable(gc-dsp-poll-test dsp_poll_run.cpp)
target_link_libraries(gc-dsp-poll-test PRIVATE gcrt)
add_test(NAME dsp_poll COMMAND gc-dsp-poll-test)

# Input scripts.
add_executable(gc-input-test input_run.cpp)
target_link_libraries(gc-input-test PRIVATE gcrt)
add_test(NAME input_script COMMAND gc-input-test)

# Instruction tests: generated image -> gcrecomp -> C -> runner.
if(TARGET gcrecomp)
    add_executable(gc-ppc-gen ppc_gen.cpp)
    set(ppc_dir "${CMAKE_CURRENT_BINARY_DIR}/ppc")
    file(MAKE_DIRECTORY "${ppc_dir}")
    add_custom_command(OUTPUT "${ppc_dir}/tests.dol" "${ppc_dir}/symbols.txt"
        COMMAND gc-ppc-gen "${ppc_dir}/tests.dol" "${ppc_dir}/symbols.txt"
        DEPENDS gc-ppc-gen ppc_tests.hpp)
    gc_recompile(gc_ppc_code DOL "${ppc_dir}/tests.dol" SYMBOLS "${ppc_dir}/symbols.txt" FILES 1 FLAGS --exact)
    add_executable(gc-ppc-tests ppc_run.cpp)
    target_link_libraries(gc-ppc-tests PRIVATE gc_ppc_code)
    add_test(NAME ppc_instructions COMMAND gc-ppc-tests)
endif()
