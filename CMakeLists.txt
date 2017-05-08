cmake_minimum_required(VERSION 2.8.7)

project(TmGen)

option(TM_GEN_DISABLE_TESTS "If tests should be compiled or not" ON)
option(TM_GEN_DEBUG "A debug build" OFF)
option(TM_GEN_ASAN "Compile with ASAN on" OFF)
option(TM_GEN_TSAN "Compile with TSAN on" OFF)

set(TM_GEN_BASE_FLAGS "-g -std=c++11 -pedantic-errors -Winit-self -Woverloaded-virtual -Wuninitialized -Wall -Wextra -fno-exceptions")
set(TM_GEN_BASE_LD_FLAGS "")
if (TM_GEN_ASAN)
   set(TM_GEN_BASE_FLAGS "${TM_GEN_BASE_FLAGS} -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(TM_GEN_BASE_LD_FLAGS "${TM_GEN_BASE_LD_FLAGS} -fsanitize=address")
endif()
if (TM_GEN_TSAN)
   set(TM_GEN_BASE_FLAGS "${TM_GEN_BASE_FLAGS} -fsanitize=thread -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(TM_GEN_BASE_LD_FLAGS "${TM_GEN_BASE_LD_FLAGS} -fsanitize=thread")
endif()

if(APPLE)
elseif(UNIX)
  set(TM_GEN_BASE_FLAGS "${TM_GEN_BASE_FLAGS} -pthread")
endif()

if (TM_GEN_DEBUG)
  set(TM_GEN_BASE_FLAGS "${TM_GEN_BASE_FLAGS} -O0 -fno-omit-frame-pointer --coverage")
else()
  set(TM_GEN_BASE_FLAGS "${TM_GEN_BASE_FLAGS} -O3 -march=native -DNDEBUG")   
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TM_GEN_BASE_FLAGS}")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${TM_GEN_BASE_LD_FLAGS}")

if (NOT TM_GEN_DISABLE_TESTS)
   include(CTest)
   add_subdirectory(external/googletest)
   macro(add_test_exec name src_file deps)
     add_executable(${name} ${src_file})
     target_link_libraries(${name} gtest gmock_main ${deps} ${ARGN})
     add_test(NAME ${name} COMMAND ${name})
   endmacro(add_test_exec)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_extensions)

add_subdirectory(external/ncode_common)
add_subdirectory(external/ncode_net)
add_subdirectory(external/ncode_lp)
include_directories(${CMAKE_SOURCE_DIR}/external)

set(TM_GEN_HEADER_FILES src/tm_gen.h)
add_library(tm_gen SHARED src/tm_gen.cc ${TM_GEN_HEADER_FILES})
target_link_libraries(tm_gen_lib ncode_lp)

if (NOT TM_GEN_DISABLE_TESTS)
  add_test_exec(tm_gen_test src/tm_gen_test.cc tm_gen)
endif()

add_executable(tm_generator src/tm_gen_main.cc)
target_link_libraries(tm_generator tm_gen)


INSTALL(TARGETS tm_gen
  DESTINATION "lib")

INSTALL (
  FILES ${NET_HEADER_FILES}
  DESTINATION include/tm_gen)
