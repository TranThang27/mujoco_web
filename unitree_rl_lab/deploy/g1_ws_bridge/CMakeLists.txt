cmake_minimum_required(VERSION 3.12)
project(g1_ws_bridge)

set(CMAKE_CXX_STANDARD 17)

set(UNITREE_SDK2_DIR ${PROJECT_SOURCE_DIR}/../../unitree_sdk2)
set(DDS_LIB_DIR     ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cppzmq QUIET)

include_directories(
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  ${UNITREE_SDK2_DIR}/thirdparty/include
  /usr/local/include
)

link_directories(${DDS_LIB_DIR})

add_executable(g1_ws_bridge main.cpp)

target_link_libraries(g1_ws_bridge
  unitree_sdk2
  # Force bundled CycloneDDS 0.10.2 (system has 0.11.0 → segfault)
  ${DDS_LIB_DIR}/libddscxx.so
  ${DDS_LIB_DIR}/libddsc.so
  rt pthread
  boost_system
  fmt
  zmq
)

# RPATH so the binary finds bundled .so at runtime without LD_LIBRARY_PATH
set_target_properties(g1_ws_bridge PROPERTIES
  BUILD_RPATH   "${DDS_LIB_DIR}"
  INSTALL_RPATH "${DDS_LIB_DIR}"
)
