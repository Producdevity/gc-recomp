# Direct AArch64 backend: the on-device translator (gcbe), the loader that maps its output,
# and runners and tests that use translated code instead of compiled C.
option(GC_BACKEND "Build the AArch64 backend (gcbe), its loader and runners" ${PROJECT_IS_TOP_LEVEL})
if(NOT GC_BACKEND)
    return()
endif()
set(gcbe_root "${CMAKE_CURRENT_LIST_DIR}/..")

add_library(gcbe_translator STATIC
    ${gcbe_root}/recompiler/a64_backend.cpp
    ${gcbe_root}/recompiler/a64_sched.cpp
    ${gcbe_root}/recompiler/a64_translate.cpp
    ${gcbe_root}/recompiler/program.cpp
    ${gcbe_root}/recompiler/image.cpp
    ${gcbe_root}/recompiler/ppc.cpp)
target_include_directories(gcbe_translator PUBLIC ${gcbe_root}/recompiler ${gcbe_root}/backend/include
    ${gcbe_root}/runtime/include "${GC_STARLET_DIR}/include")
target_compile_features(gcbe_translator PUBLIC cxx_std_17)

add_executable(gcbe ${gcbe_root}/recompiler/gcbe_main.cpp)
target_link_libraries(gcbe PRIVATE gcbe_translator)

add_library(gcbe_loader STATIC ${gcbe_root}/backend/loader.cpp)
target_include_directories(gcbe_loader PUBLIC ${gcbe_root}/backend/include)
target_link_libraries(gcbe_loader PUBLIC gcrt ${CMAKE_DL_LIBS})

add_executable(gc-headless-gcbe ${gcbe_root}/tools/headless.cpp)
target_link_libraries(gc-headless-gcbe PRIVATE gcbe_loader)
target_compile_definitions(gc-headless-gcbe PRIVATE GCRT_TRANSLATED)
set_target_properties(gc-headless-gcbe PROPERTIES ENABLE_EXPORTS ON)

if(GC_TESTS AND TARGET gc-ppc-gen)
    set(gcbe_ppc "${CMAKE_BINARY_DIR}/gcbe-ppc")
    file(MAKE_DIRECTORY "${gcbe_ppc}")
    add_custom_command(OUTPUT "${gcbe_ppc}/tests.dol" "${gcbe_ppc}/symbols.txt"
        COMMAND gc-ppc-gen "${gcbe_ppc}/tests.dol" "${gcbe_ppc}/symbols.txt"
        DEPENDS gc-ppc-gen)
    add_custom_command(OUTPUT "${gcbe_ppc}/tests.gcbe"
        COMMAND gcbe --dol "${gcbe_ppc}/tests.dol" --symbols "${gcbe_ppc}/symbols.txt" --exact --out "${gcbe_ppc}/tests.gcbe"
        DEPENDS gcbe "${gcbe_ppc}/tests.dol" "${gcbe_ppc}/symbols.txt")
    add_custom_target(gcbe_ppc_image ALL DEPENDS "${gcbe_ppc}/tests.gcbe")
    add_executable(gc-ppc-tests-gcbe ${gcbe_root}/tests/ppc_run.cpp)
    target_link_libraries(gc-ppc-tests-gcbe PRIVATE gcbe_loader)
    target_compile_definitions(gc-ppc-tests-gcbe PRIVATE GCRT_TRANSLATED GCBE_TEST_BLOB="${gcbe_ppc}/tests.gcbe")
    add_dependencies(gc-ppc-tests-gcbe gcbe_ppc_image)
    add_test(NAME ppc_instructions_gcbe COMMAND gc-ppc-tests-gcbe)
endif()

# Runtime replacements of SDK functions against their translated guest code, which the test
# copies from Luigi's Mansion's main.dol.
set(GC_NATIVES_DOL "" CACHE FILEPATH "Luigi's Mansion (GLME01) main.dol for the natives test")
set(GC_NATIVES_SMS_DOL "" CACHE FILEPATH "Super Mario Sunshine (GMSE01) main.dol for its natives in that test")
if(GC_TESTS AND GC_NATIVES_DOL)
    set(gcbe_nat "${CMAKE_BINARY_DIR}/gcbe-natives")
    file(MAKE_DIRECTORY "${gcbe_nat}")
    add_executable(gc-natives-gen ${gcbe_root}/tests/natives_gen.cpp)
    target_link_libraries(gc-natives-gen PRIVATE gcbe_translator)
    add_custom_command(OUTPUT "${gcbe_nat}/natives.dol" "${gcbe_nat}/symbols.txt"
        COMMAND gc-natives-gen "${GC_NATIVES_DOL}" "${gcbe_nat}/natives.dol" "${gcbe_nat}/symbols.txt" ${GC_NATIVES_SMS_DOL}
        DEPENDS gc-natives-gen "${GC_NATIVES_DOL}" ${GC_NATIVES_SMS_DOL})
    add_custom_command(OUTPUT "${gcbe_nat}/natives.gcbe"
        COMMAND gcbe --dol "${gcbe_nat}/natives.dol" --symbols "${gcbe_nat}/symbols.txt" --no-natives
                --out "${gcbe_nat}/natives.gcbe"
        DEPENDS gcbe "${gcbe_nat}/natives.dol" "${gcbe_nat}/symbols.txt")
    add_custom_target(gcbe_natives_image ALL DEPENDS "${gcbe_nat}/natives.gcbe")
    add_executable(gc-natives-test ${gcbe_root}/tests/natives_run.cpp)
    target_link_libraries(gc-natives-test PRIVATE gcbe_loader)
    target_compile_definitions(gc-natives-test PRIVATE GCBE_NATIVES_BLOB="${gcbe_nat}/natives.gcbe")
    if(GC_NATIVES_SMS_DOL)
        target_compile_definitions(gc-natives-test PRIVATE GCBE_NATIVES_SMS_DOL="${GC_NATIVES_SMS_DOL}")
    endif()
    add_dependencies(gc-natives-test gcbe_natives_image)
    add_test(NAME natives_exact COMMAND gc-natives-test)
endif()
foreach(t gcbe_translator gcbe gcbe_loader)
    target_compile_options(${t} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>)
endforeach()

# Differential test: random Gekko functions through the C path and the backend.
if(GC_TESTS)
    set(gcbe_fuzz "${CMAKE_BINARY_DIR}/gcbe-fuzz")
    set(GCBE_FUZZ_SEED 1 CACHE STRING "Seed of the backend differential test program")
    file(MAKE_DIRECTORY "${gcbe_fuzz}")
    add_executable(gc-gcbe-fuzz-gen ${gcbe_root}/tests/gcbe_fuzz_gen.cpp)
    add_custom_command(OUTPUT "${gcbe_fuzz}/fuzz.dol" "${gcbe_fuzz}/symbols.txt"
        COMMAND gc-gcbe-fuzz-gen "${gcbe_fuzz}/fuzz.dol" "${gcbe_fuzz}/symbols.txt" ${GCBE_FUZZ_SEED}
        DEPENDS gc-gcbe-fuzz-gen ${gcbe_root}/tests/gcbe_fuzz.hpp)
    gc_recompile(gc_gcbe_fuzz_code DOL "${gcbe_fuzz}/fuzz.dol" SYMBOLS "${gcbe_fuzz}/symbols.txt" FILES 4)
    add_custom_command(OUTPUT "${gcbe_fuzz}/fuzz.gcbe"
        COMMAND gcbe --dol "${gcbe_fuzz}/fuzz.dol" --symbols "${gcbe_fuzz}/symbols.txt" --out "${gcbe_fuzz}/fuzz.gcbe"
        DEPENDS gcbe "${gcbe_fuzz}/fuzz.dol" "${gcbe_fuzz}/symbols.txt")
    add_custom_target(gcbe_fuzz_image ALL DEPENDS "${gcbe_fuzz}/fuzz.gcbe")
    add_executable(gc-gcbe-fuzz ${gcbe_root}/tests/gcbe_fuzz_run.cpp)
    target_link_libraries(gc-gcbe-fuzz PRIVATE gc_gcbe_fuzz_code gcbe_loader)
    add_dependencies(gc-gcbe-fuzz gcbe_fuzz_image)
    add_test(NAME gcbe_differential COMMAND gc-gcbe-fuzz "${gcbe_fuzz}/fuzz.gcbe" ${GCBE_FUZZ_SEED})

    # The same program with every function in exact mode (all registers synchronised).
    gc_recompile(gc_gcbe_fuzz_exact DOL "${gcbe_fuzz}/fuzz.dol" SYMBOLS "${gcbe_fuzz}/symbols.txt" FILES 4 FLAGS --exact)
    add_custom_command(OUTPUT "${gcbe_fuzz}/fuzz-exact.gcbe"
        COMMAND gcbe --dol "${gcbe_fuzz}/fuzz.dol" --symbols "${gcbe_fuzz}/symbols.txt" --exact
                --out "${gcbe_fuzz}/fuzz-exact.gcbe"
        DEPENDS gcbe "${gcbe_fuzz}/fuzz.dol" "${gcbe_fuzz}/symbols.txt")
    add_custom_target(gcbe_fuzz_exact_image ALL DEPENDS "${gcbe_fuzz}/fuzz-exact.gcbe")
    add_executable(gc-gcbe-fuzz-exact ${gcbe_root}/tests/gcbe_fuzz_run.cpp)
    target_link_libraries(gc-gcbe-fuzz-exact PRIVATE gc_gcbe_fuzz_exact gcbe_loader)
    target_compile_definitions(gc-gcbe-fuzz-exact PRIVATE GCBE_FUZZ_EXACT)
    add_dependencies(gc-gcbe-fuzz-exact gcbe_fuzz_exact_image)
    add_test(NAME gcbe_differential_exact COMMAND gc-gcbe-fuzz-exact "${gcbe_fuzz}/fuzz-exact.gcbe" ${GCBE_FUZZ_SEED})
endif()
