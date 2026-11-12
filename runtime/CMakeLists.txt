set(GC_STARLET_DIR "${GC_RECOMP_ROOT}/third_party/starlet" CACHE PATH "Starlet checkout")
if(NOT TARGET starlet_gx)
    add_subdirectory("${GC_STARLET_DIR}" "${CMAKE_BINARY_DIR}/starlet" EXCLUDE_FROM_ALL)
endif()

add_library(gcrt STATIC
    src/boot.cpp
    src/cpu.cpp
    src/debug_wraps.cpp
    src/di.cpp
    src/dsp.cpp
    src/events.cpp
    src/exi.cpp
    src/fiber_aarch64.S
    src/gx.cpp
    src/gx_starlet.cpp
    src/input.cpp
    src/io.cpp
    src/jaudio.cpp
    src/jaudio_dsp.cpp
    src/jasystem.cpp
    src/memory.cpp
    src/natives.cpp
    src/os.cpp
    src/sampler.cpp
    src/si.cpp
    src/ucode.cpp
    src/vi.cpp)
target_include_directories(gcrt PUBLIC include)
target_compile_features(gcrt PUBLIC cxx_std_17)
find_package(Threads REQUIRED)
target_link_libraries(gcrt PUBLIC starlet_gx starlet_disc starlet_card Threads::Threads ${CMAKE_DL_LIBS})
target_compile_options(gcrt PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>)

# gc_recompile(<target> (DOL <dol> | DISC <iso>) SYMBOLS <symbols.txt> [CONFIG <cfg>] [FILES <n>]
#              [RECOMPILER <path>] [FLAGS <gcrecomp flags>...])
# Recompiles a DOL at build time into a static library of C sources.
function(gc_recompile target)
    cmake_parse_arguments(GC "" "DOL;DISC;SYMBOLS;CONFIG;FILES;RECOMPILER" "FLAGS" ${ARGN})
    if(GC_DISC)
        set(input --disc "${GC_DISC}")
        set(GC_DOL "${GC_DISC}")
    else()
        set(input --dol "${GC_DOL}")
    endif()
    if(NOT GC_FILES)
        set(GC_FILES 64)
    endif()
    set(depends "${GC_DOL}" "${GC_SYMBOLS}")
    if(NOT GC_RECOMPILER)
        set(GC_RECOMPILER $<TARGET_FILE:gcrecomp>)
        list(APPEND depends gcrecomp)
    else()
        list(APPEND depends "${GC_RECOMPILER}")
    endif()
    set(out "${CMAKE_CURRENT_BINARY_DIR}/${target}-generated")
    file(MAKE_DIRECTORY "${out}")
    set(sources "${out}/recomp_table.c")
    math(EXPR last "${GC_FILES} - 1")
    foreach(n RANGE ${last})
        string(LENGTH "${n}" len)
        if(len EQUAL 1)
            set(n "00${n}")
        elseif(len EQUAL 2)
            set(n "0${n}")
        endif()
        list(APPEND sources "${out}/recomp_${n}.c")
    endforeach()
    set(config_args)
    if(GC_CONFIG)
        list(APPEND config_args --config "${GC_CONFIG}")
        list(APPEND depends "${GC_CONFIG}")
    endif()
    add_custom_command(
        OUTPUT ${sources} "${out}/recomp_funcs.h"
        COMMAND ${GC_RECOMPILER} ${input} --symbols "${GC_SYMBOLS}" ${config_args}
                --out "${out}" --files ${GC_FILES} ${GC_FLAGS}
        DEPENDS ${depends}
        COMMENT "Recompiling ${GC_DOL}"
        VERBATIM)
    add_library(${target} STATIC ${sources} "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/src/linked_program.cpp")
    target_include_directories(${target} PRIVATE "${out}")
    target_link_libraries(${target} PUBLIC gcrt)
    # Guest code keeps exact IEEE semantics: no contraction or reassociation.
    target_compile_options(${target} PRIVATE -O2 -ffp-contract=off -fno-strict-aliasing -w)
endfunction()
