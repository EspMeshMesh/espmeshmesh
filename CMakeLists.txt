cmake_minimum_required(VERSION 3.16)
project(espmeshmesh VERSION 1.5.2 LANGUAGES C CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/nanopb/extra/)

include(FetchContent)
include(GNUInstallDirs)

FetchContent_Declare(
  nanopb
  GIT_REPOSITORY https://github.com/nanopb/nanopb.git
  GIT_TAG "nanopb-0.4.9.1"
  GIT_SHALLOW TRUE
  GIT_PROGRESS ON
  SYSTEM
)

FetchContent_Declare(
  tiny-aes
  GIT_REPOSITORY https://github.com/kokke/tiny-AES-c.git
  GIT_TAG "v1.0.0"
  GIT_SHALLOW TRUE
  GIT_PROGRESS ON
  SYSTEM
)

FetchContent_MakeAvailable(tiny-aes nanopb)

list(APPEND CMAKE_MODULE_PATH ${nanopb_SOURCE_DIR}/extra)
find_package(Nanopb REQUIRED)

message(STATUS "nanopb: ${nanopb_SOURCE_DIR}")
message(STATUS "tiny-aes: ${tiny-aes_SOURCE_DIR}")

set(TARGET_PLATFORM "linux" CACHE STRING "Target platform: linux")
# Directory sorgenti
set(ESPMESH_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(ESPMESH_PROTOC_DIR ${ESPMESH_SRC_DIR}/protoc)
set(ESPMESH_PROTO_DIR ${ESPMESH_SRC_DIR}/proto)

message(STATUS "ESPMESH_PROTO_DIR: ${ESPMESH_PROTO_DIR}")

# Sorgenti C++ comuni
set(ESPMESH_CPP_SOURCES
  ${ESPMESH_SRC_DIR}/broadcast.cpp
  ${ESPMESH_SRC_DIR}/broadcast2.cpp
  ${ESPMESH_SRC_DIR}/connectedpath.cpp
  ${ESPMESH_SRC_DIR}/crc16.cpp
  ${ESPMESH_SRC_DIR}/defines.cpp
  ${ESPMESH_SRC_DIR}/discovery.cpp
  ${ESPMESH_SRC_DIR}/espmeshmesh.cpp
  ${ESPMESH_SRC_DIR}/espmeshmesh_impl.cpp
  ${ESPMESH_SRC_DIR}/log.cpp
  ${ESPMESH_SRC_DIR}/memringbuffer.cpp
  ${ESPMESH_SRC_DIR}/meshaddress.cpp
  ${ESPMESH_SRC_DIR}/meshsocket.cpp
  ${ESPMESH_SRC_DIR}/multipath.cpp
  ${ESPMESH_SRC_DIR}/packetbuf.cpp
  ${ESPMESH_SRC_DIR}/packetbufprotocol.cpp
  ${ESPMESH_SRC_DIR}/radiopacket.cpp
  ${ESPMESH_SRC_DIR}/polite.cpp
  ${ESPMESH_SRC_DIR}/recvdups.cpp
  ${ESPMESH_SRC_DIR}/starpath.cpp
  ${ESPMESH_SRC_DIR}/uart.cpp
  ${ESPMESH_SRC_DIR}/unicast.cpp
  ${ESPMESH_SRC_DIR}/wifidrv.cpp
  ${ESPMESH_SRC_DIR}/encryption.cpp
)

set(ESPMESH_WIFIDRV_SOURCES
  ${ESPMESH_SRC_DIR}/wifidrvesp32.cpp
  ${ESPMESH_SRC_DIR}/wifidrvesp8266.cpp
  ${ESPMESH_SRC_DIR}/wifidrvlinux.cpp
)

# Sorgenti C (nanopb + encryption)
set(ESPMESH_C_SOURCES
  ${ESPMESH_PROTOC_DIR}/commands.pb.c
  ${ESPMESH_PROTOC_DIR}/disoverybeaconreply.pb.c
  ${ESPMESH_PROTOC_DIR}/nodepresentation.pb.c
  ${ESPMESH_PROTOC_DIR}/nodepresentationrx.pb.c
  ${ESPMESH_PROTOC_DIR}/notificationbeacon.pb.c
  ${ESPMESH_PROTOC_DIR}/pathrouting.pb.c
)

set(ESPMESH_PROTO_SOURCES
  ${ESPMESH_PROTO_DIR}/dummy.proto
)

set(ESPMESH_PUBLIC_HEADERS
  ${ESPMESH_SRC_DIR}/broadcast.h
  ${ESPMESH_SRC_DIR}/broadcast2.h
  ${ESPMESH_SRC_DIR}/connectedpath.h
  ${ESPMESH_SRC_DIR}/crc16.h
  ${ESPMESH_SRC_DIR}/defines.h
  ${ESPMESH_SRC_DIR}/discovery.h
  ${ESPMESH_SRC_DIR}/espmeshmesh.h
  ${ESPMESH_SRC_DIR}/encryption.h
  ${ESPMESH_SRC_DIR}/meshaddress.h
  ${ESPMESH_SRC_DIR}/meshsocket.h
  ${ESPMESH_SRC_DIR}/multipath.h
  ${ESPMESH_SRC_DIR}/packetbuf.h
  ${ESPMESH_SRC_DIR}/radiopacket.h
  ${ESPMESH_SRC_DIR}/memringbuffer.h
  ${ESPMESH_SRC_DIR}/polite.h
  ${ESPMESH_SRC_DIR}/recvdups.h
  ${ESPMESH_SRC_DIR}/starpath.h
  ${ESPMESH_SRC_DIR}/uart.h
  ${ESPMESH_SRC_DIR}/unicast.h
  ${ESPMESH_SRC_DIR}/wifi.h
  ${ESPMESH_SRC_DIR}/log.h
)

set(ESPMESH_WIFIDRV_HEADERS
  ${ESPMESH_SRC_DIR}/wifidrv.h
  ${ESPMESH_SRC_DIR}/wifidrvesp32.h
  ${ESPMESH_SRC_DIR}/wifidrvesp8266.h
  ${ESPMESH_SRC_DIR}/wifidrvlinux.h
)

# Sorgenti per piattaforma
if(TARGET_PLATFORM STREQUAL "linux")
  list(APPEND ESPMESH_CPP_SOURCES
    ${ESPMESH_SRC_DIR}/uart_linux.cpp
    ${ESPMESH_SRC_DIR}/wifidrvlinux.cpp
  )
endif()

set(ESPMESH_DEFINITIONS USE_LINUX ESPHOME_LOG_LEVEL=ESPHOME_LOG_LEVEL_VERBOSE)

# Opzioni facoltative (stesso comportamento dei #ifdef nel codice)
option(USE_POLITE_BROADCAST_PROTOCOL "Abilita protocollo Polite Broadcast" OFF)
option(ESPMESH_STARPATH_ENABLED "Abilita StarPath" ON)

if(USE_POLITE_BROADCAST_PROTOCOL)
  list(APPEND ESPMESH_DEFINITIONS USE_POLITE_BROADCAST_PROTOCOL)
endif()
if(ESPMESH_STARPATH_ENABLED)
  list(APPEND ESPMESH_DEFINITIONS ESPMESH_STARPATH_ENABLED)
endif()

list(APPEND ESPMESH_DEFINITIONS ESPMESHMESH_VERSION="${PROJECT_VERSION}")

if(NOT DEFINED ESPMESH_LIB_TYPE)
  set(ESPMESH_LIB_TYPE SHARED CACHE STRING "Library type: SHARED or STATIC")
endif()
if(NOT ESPMESH_LIB_TYPE MATCHES "^(SHARED|STATIC)$")
  set(ESPMESH_LIB_TYPE SHARED)
endif()

configure_file(espmeshmesh.pc.in ${CMAKE_CURRENT_BINARY_DIR}/espmeshmesh.pc @ONLY)

nanopb_generate_cpp(TARGET proto ${ESPMESH_PROTO_SOURCES})

add_library(espmeshmesh ${ESPMESH_LIB_TYPE}
  ${ESPMESH_CPP_SOURCES}
  ${ESPMESH_C_SOURCES}
)

target_link_libraries(espmeshmesh PRIVATE nanopb tiny-aes)

target_include_directories(espmeshmesh PUBLIC
  ${ESPMESH_SRC_DIR}
  ${ESPMESH_PROTOC_DIR}
  ${tiny-aes_SOURCE_DIR}
  nanopb
)

target_compile_definitions(espmeshmesh PRIVATE ${ESPMESH_DEFINITIONS})

set_target_properties(espmeshmesh PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})
set_target_properties(espmeshmesh PROPERTIES PUBLIC_HEADER "${ESPMESH_PUBLIC_HEADERS}")
set_target_properties(espmeshmesh PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON C_STANDARD 99)

install(TARGETS espmeshmesh DESTINATION ${CMAKE_INSTALL_LIBDIR} PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/espmeshmesh.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)