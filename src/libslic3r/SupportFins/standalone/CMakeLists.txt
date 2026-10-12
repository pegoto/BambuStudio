# Standalone build of the Support Fins engine + parity runner. Needs only Eigen
# (bundled in src/eigen) and nlohmann/json (bundled in src/nlohmann); no boost, no
# wx. Used to develop the port against the upstream JS oracle without building
# BambuStudio. The engine sources are the same files libslic3r compiles.
#
#   cmake -S standalone -B build-sf -G Ninja -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-sf
#   ./build-sf/sf_parity refs/*.json
cmake_minimum_required(VERSION 3.16)
project(SupportFinsStandalone CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

get_filename_component(SF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
get_filename_component(SRC_ROOT "${SF_ROOT}/../.." ABSOLUTE)

file(GLOB_RECURSE SF_SOURCES CONFIGURE_DEPENDS "${SF_ROOT}/Engine/*.cpp")
add_library(supportfins_engine STATIC ${SF_SOURCES})
target_include_directories(supportfins_engine PUBLIC "${SRC_ROOT}/eigen" "${SF_ROOT}")
# -ffp-contract=off: no fused multiply-add. aarch64 (and any -march=native x86) would
# otherwise fuse a*b+c into one rounding and break bit parity with the JS engine.
target_compile_options(supportfins_engine PRIVATE -ffp-contract=off -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-misleading-indentation)
if(SF_SANITIZE)
  target_compile_options(supportfins_engine PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer -g)
  target_link_options(supportfins_engine PUBLIC -fsanitize=address,undefined)
endif()

add_executable(sf_parity "${SF_ROOT}/tools/parity.cpp")
target_include_directories(sf_parity PRIVATE "${SRC_ROOT}")
target_link_libraries(sf_parity PRIVATE supportfins_engine)

# Per-module unit tests (added by the module owners): tests/test_*.cpp, one executable each.
file(GLOB SF_TESTS CONFIGURE_DEPENDS "${SF_ROOT}/tests/test_*.cpp")
enable_testing()
foreach(t ${SF_TESTS})
  get_filename_component(name "${t}" NAME_WE)
  add_executable(${name} "${t}")
  target_include_directories(${name} PRIVATE "${SRC_ROOT}")
  target_link_libraries(${name} PRIVATE supportfins_engine)
  add_test(NAME ${name} COMMAND ${name})
endforeach()
