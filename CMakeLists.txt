cmake_minimum_required(VERSION 3.16)

# The prebuilt raylib libraries are built against the release DLL CRT.
# Use the same CRT in every configuration so no linker mismatch
# (LNK4098) appears in Debug builds. Has to be set before project()
# to influence the compiler defaults.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

project(Chase_in_the_City LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The game layer calls setbuf(); silence MSVC's "unsafe CRT function"
# deprecation noise on every platform toolchain that flags it.
if (MSVC)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

set(SOURCES
    src/main.cpp
    src/grid.cpp
    src/chase.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE src)

# ---------------------------------------------------------------------------
# raylib
# Dependency management:
# 1) Prefer a standard raylib CMake package (find_package) — an imported
#    target that brings its own include dirs and transitive platform
#    dependencies (GLFW, X11, ...). Nothing is ever downloaded: the
#    package config must already be installed on the machine.
# 2) Otherwise use a prebuilt raylib directory (include/ + raylib.lib,
#    optionally glfw.lib) via -DRAYLIB_ROOT=<dir> or the RAYLIB_ROOT
#    environment variable.
# FetchContent is deliberately NOT used: it downloads source trees, which
# is against this project's no-download dependency policy.
# ---------------------------------------------------------------------------
find_package(raylib QUIET)

function(link_raylib target)
    if (TARGET raylib)
        # The imported target already carries its transitive dependencies
        # (GLFW et al.); linking it explicitly again would be redundant.
        target_link_libraries(${target} PRIVATE raylib)
    else()
        if (NOT RAYLIB_ROOT)
            set(RAYLIB_ROOT "$ENV{RAYLIB_ROOT}")
        endif()
        if (NOT EXISTS "${RAYLIB_ROOT}/include/raylib.h")
            message(FATAL_ERROR
                "raylib not found. Either install raylib via CMake (find_package(raylib)) "
                "or point -DRAYLIB_ROOT=<dir> at a prebuilt raylib directory "
                "containing include/raylib.h and raylib.lib / libraylib.*")
        endif()
        target_include_directories(${target} SYSTEM PRIVATE "${RAYLIB_ROOT}/include")
        target_link_directories(${target} PRIVATE "${RAYLIB_ROOT}")
        target_link_libraries(${target} PRIVATE raylib)
        # Static raylib pulls in GLFW. Link it only when the lib is actually
        # present next to raylib — never twice, never on speculation.
        if (WIN32 AND EXISTS "${RAYLIB_ROOT}/glfw.lib")
            target_link_libraries(${target} PRIVATE glfw)
        elseif (NOT WIN32 AND EXISTS "${RAYLIB_ROOT}/libglfw3.a")
            target_link_libraries(${target} PRIVATE glfw3)
        endif()
        if (WIN32)
            target_link_libraries(${target} PRIVATE winmm gdi32 opengl32)
        endif()
    endif()
endfunction()

link_raylib(${PROJECT_NAME})

# Warnings: MSVC vs GCC/Clang
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Optional sanitizers: AddressSanitizer + UndefinedBehavior
# Sanitizer on the test target. Off by default; enable per build dir:
#   cmake -S . -B build-asan -DCHASE_SANITIZE=ON
# ---------------------------------------------------------------------------
# ASan on MSVC; ASan + UBSan on GCC/Clang (UBSan is unavailable for MSVC).
option(CHASE_SANITIZE "Build chase_tests with sanitizers" OFF)

# ---------------------------------------------------------------------------
# Headless logic tests.
# Built with CHASE_TESTING so the test harness can reach private members.
# ---------------------------------------------------------------------------
option(CHASE_BUILD_TESTS "Build the headless logic tests" ON)
if (CHASE_BUILD_TESTS)
    enable_testing()

    add_executable(chase_tests
        tests/test_main.cpp
        src/grid.cpp
        src/chase.cpp
    )
    target_include_directories(chase_tests PRIVATE src)
    target_compile_definitions(chase_tests PRIVATE CHASE_TESTING)
    link_raylib(chase_tests)
    if (MSVC)
        target_compile_options(chase_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(chase_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    if (CHASE_SANITIZE)
        if (MSVC)
            # MSVC supports /fsanitize=address (the compile step imports the
            # clang_rt.asan_dynamic-x86_64.dll runtime, which the MSVC 14.4x
            # toolset ships in <VC>\Tools\MSVC\<ver>\bin\Hostx64\x64 — put
            # that directory on PATH at test-run time).
            # /Zi + /DEBUG emit the info ASan needs for reports. UBSan is
            # NOT available for MSVC: cl silently ignores
            # /fsanitize=undefined (warning D9002), so it is deliberately
            # not passed.
            target_compile_options(chase_tests PRIVATE /fsanitize=address /Zi)
            target_link_options(chase_tests PRIVATE /DEBUG)
        else()
            # GCC/Clang: the standard -fsanitize= form for both the compile
            # and the link step.
            target_compile_options(chase_tests PRIVATE -fsanitize=address,undefined -O1)
            target_link_options(chase_tests PRIVATE -fsanitize=address,undefined)
        endif()
    endif()
    add_test(NAME chase_tests COMMAND chase_tests)
endif()
