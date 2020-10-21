cmake_minimum_required(VERSION 3.18)
project("Particle") # Hier Namen ergänzen

set(CMAKE_CXX_STANDARD 20)

set(general_flags "-std=c++20 -fverbose-asm -save-temps -masm=intel")
set(debug_flags "-g -fasynchronous-unwind-tables -grecord-gcc-switches -Wall -Wextra -Wpedantic -Wfloat-equal -Wundef -Wpointer-arith -Wcast-align -Wswitch-default -Wswitch-enum -Wunreachable-code -Wformat=2 -Werror=format-security")
set(hardening_flags "-D_GLIBCXX_ASSERTIONS -fexceptions -fstack-clash-protection -fstack-protector -fstack-protector-strong -fcf-protection -Wl,-z,-defs -Wl,-z,now -Wl,-z,relro -fvisibility=hidden")
set(aslr_executable "-fpie -Wl,-pie")
set(aslr_shared_lib "-fpic -shared")
set(optimization_flags "-O3")

set(CMAKE_CXX_FLAGS_DEBUG "${general_flags} ${hardening_flags} ${debug_flags}")
set(CMAKE_CXX_FLAGS_RELEASE "${general_flags} ${optimization_flags} ${aslr_executable} -DNDEBUG -D_FORTIFY_SOURCE=2")

# -D_FORTIFY_SOURCE=2: Runtime buffer overflow detection, requires -O2

# -D_GLIBCXX_ASSERTIONS: C++ standard library hardening (string + container bound checking)
# -fexceptions: Hardening of multi-threaded C++ code
# -fstack-clash-protection: Prevents attacks caused by overlapping heap-/stack-memory
# -fstack-protector(-strong): Stack smashing protector
# -fcf-protection: Enable Control-Flow Enforcement Technology
# -Wl,-z,defs: Reject underlinking
# -Wl,-z,now: Reject lazy-binding
# -Wl,-z,relro: Rad-only segments after relocation

# -fpie -Wl,-pie: Enables address space layout randomization (ASLR) for executables
# -fpic -shared: Needed by shared libraries for ASLR

# -fasynchronous-unwind-tables: Required for many debugging- and performance-tools
# -g: Add debug information
# -grecord-gcc-switches: Stores compiler-flags in debug info
# -Wall: All warnings shown
# -Werror=format-security: Reject unsafe Strings
# -pedantic: Show errors with the C++-Standard
# -fverbose-asm: Informative assembly comments

# -O2 Optimizations without additional inlining or loop-unrolling

message("DEBUG-Flags: ${CMAKE_CXX_FLAGS_DEBUG}")
message("RELEASE-Flags: ${CMAKE_CXX_FLAGS_RELEASE}")

# Executables
file(GLOB source_files "${PROJECT_SOURCE_DIR}/src/*.cpp")
add_executable(${PROJECT_NAME} ${source_files})

# Libraries
# find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
# target_link_libraries(${PROJECT_NAME} sfml-graphics sfml-window sfml-system)

# Custom Targets for Build Types
ADD_CUSTOM_TARGET(debug
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Debug"
  )

ADD_CUSTOM_TARGET(release
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Release"
  )

# Target to run app
ADD_CUSTOM_TARGET(run
  COMMAND ${PROJECT_NAME}
  COMMENT "Run with ${CMAKE_BUILD_TYPE} configuration"
  )
