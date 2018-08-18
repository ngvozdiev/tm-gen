cmake_minimum_required(VERSION 2.8.7)

project(TM_GEN)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_extensions)
option(TM_GEN_BASE_DISABLE_TESTS "If tests should be compiled or not" ON)
option(TM_GEN_BASE_DEBUG "A debug build" OFF)
option(TM_GEN_BASE_ASAN "Compile with ASAN on" OFF)
option(TM_GEN_BASE_TSAN "Compile with TSAN on" OFF)

set(TM_GEN_BASE_BASE_FLAGS "-g -std=c++11 -pedantic-errors -Winit-self -Woverloaded-virtual -Wuninitialized -Wall -Wextra -fno-exceptions")
set(TM_GEN_BASE_BASE_LD_FLAGS "")
if (TM_GEN_BASE_ASAN)
   set(TM_GEN_BASE_BASE_FLAGS "${TM_GEN_BASE_BASE_FLAGS} -fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(TM_GEN_BASE_BASE_LD_FLAGS "${TM_GEN_BASE_BASE_LD_FLAGS} -fsanitize=address")
endif()
if (TM_GEN_BASE_TSAN)
   set(TM_GEN_BASE_BASE_FLAGS "${TM_GEN_BASE_BASE_FLAGS} -fsanitize=thread -fno-omit-frame-pointer -fno-optimize-sibling-calls")
   set(TM_GEN_BASE_BASE_LD_FLAGS "${TM_GEN_BASE_BASE_LD_FLAGS} -fsanitize=thread")
endif()

if (TM_GEN_BASE_DEBUG)
  set(TM_GEN_BASE_BASE_FLAGS "${TM_GEN_BASE_BASE_FLAGS} -O0 -fno-omit-frame-pointer")
else()
  set(TM_GEN_BASE_BASE_FLAGS "${TM_GEN_BASE_BASE_FLAGS} -O3 -march=native -DNDEBUG")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TM_GEN_BASE_BASE_FLAGS}")
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} ${TM_GEN_BASE_BASE_LD_FLAGS}")

if (NOT TM_GEN_BASE_DISABLE_TESTS)
   include(CTest)
   add_subdirectory(external/googletest)
   macro(add_test_exec name src_file deps)
     add_executable(${name} ${src_file})
     target_link_libraries(${name} gtest gmock_main ${deps} ${ARGN})
     add_test(NAME ${name} COMMAND ${name})
   endmacro(add_test_exec)
endif()

SET(OLD_BUILD_TESTING ${BUILD_TESTING})
SET(BUILD_TESTING False)
add_subdirectory(external/gflags)
SET(BUILD_TESTING ${OLD_BUILD_TESTING})

add_library(topology_input src/topology_input.cc)
target_link_libraries(topology_input ncode gflags)

add_library(tm_gen_lib STATIC src/tm_gen.cc)
target_link_libraries(tm_gen_lib ncode)

add_executable(tm-gen src/tm_gen_main.cc)
target_link_libraries(tm-gen topology_input tm_gen_lib gflags pthread)

install(TARGETS tm-gen DESTINATION bin)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/topologies 
        DESTINATION /usr/local/tm-gen)
