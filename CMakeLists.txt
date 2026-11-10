cmake_minimum_required(VERSION 3.20)
project(gc_recomp C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# The recompiler runs on the build machine. Cross builds take a prebuilt one
# through GC_RECOMPILER instead.
option(GC_BUILD_RECOMPILER "Build the gcrecomp tool" ON)
if(GC_BUILD_RECOMPILER)
    add_executable(gcrecomp
        recompiler/main.cpp
        recompiler/emit.cpp
        recompiler/image.cpp
        recompiler/ppc.cpp)
endif()

add_subdirectory(runtime)

# The SDL2 + OpenGL ES frontend, for targets with SDL2 and Starlet's GLES renderer.
option(GC_FRONTEND "Build the SDL2/GLES frontend" OFF)
if(GC_FRONTEND)
    find_path(SDL2_INCLUDE_DIR SDL.h PATH_SUFFIXES SDL2 REQUIRED)
    find_library(SDL2_LIBRARY NAMES SDL2 SDL2-2.0 REQUIRED)
    add_library(gcrt_frontend STATIC frontend/src/frontend.cpp)
    target_include_directories(gcrt_frontend PUBLIC frontend/include PRIVATE "${SDL2_INCLUDE_DIR}")
    target_link_libraries(gcrt_frontend PUBLIC gcrt starlet_gx_gles "${SDL2_LIBRARY}")
endif()

option(GC_TESTS "Build gc-recomp tests" ${PROJECT_IS_TOP_LEVEL})
if(GC_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Development: a headless runner for one game's DOL.
set(GC_GAME_DOL "" CACHE FILEPATH "DOL to recompile into gc-headless")
set(GC_GAME_SYMBOLS "" CACHE FILEPATH "dtk symbols.txt for GC_GAME_DOL")
set(GC_GAME_CONFIG "" CACHE FILEPATH "gcrecomp configuration for GC_GAME_DOL")
if(GC_GAME_DOL)
    gc_recompile(gc_game DOL "${GC_GAME_DOL}" SYMBOLS "${GC_GAME_SYMBOLS}" CONFIG "${GC_GAME_CONFIG}")
    option(GC_GAME_WATCH "Compile store watchpoints into the headless game code" OFF)
    if(GC_GAME_WATCH)
        target_compile_definitions(gc_game PRIVATE GC_WATCH)
    endif()
    option(GC_GAME_VIRTUAL_TIME "Count back-edges for deterministic runs (GCRT_VIRTUAL_TIME=1)" OFF)
    if(GC_GAME_VIRTUAL_TIME)
        target_compile_definitions(gc_game PRIVATE GC_VIRTUAL_TIME)
    endif()
    add_executable(gc-headless tools/headless.cpp)
    target_link_libraries(gc-headless PRIVATE gc_game)
endif()
include(cmake/gcbe.cmake)
# With the backend, the frontend can translate the player's disc at first launch.
if(GC_FRONTEND AND TARGET gcbe_loader)
    target_link_libraries(gcrt_frontend PUBLIC gcbe_loader gcbe_translator)
    target_compile_definitions(gcrt_frontend PUBLIC GCRT_WITH_GCBE)
endif()
