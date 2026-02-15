cmake_minimum_required(VERSION 3.16)
project(espmeshmesh VERSION 1.5.2 LANGUAGES C CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/nanopb/extra/)

find_package(Nanopb REQUIRED)

set(TARGET_PLATFORM "linux" CACHE STRING "Target platform: linux")
# Directory sorgenti
set(ESPMESH_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(ESPMESH_PROTOC_DIR ${ESPMESH_SRC_DIR}/protoc)

# Sorgenti C++ comuni
set(ESPMESH_CPP_SOURCES
  ${ESPMESH_SRC_DIR}/broadcast.cpp
  ${ESPMESH_SRC_DIR}/broadcast2.cpp
  ${ESPMESH_SRC_DIR}/connectedpath.cpp
  ${ESPMESH_SRC_DIR}/crc16.cpp
  ${ESPMESH_SRC_DIR}/defines.cpp
  ${ESPMESH_SRC_DIR}/discovery.cpp
  ${ESPMESH_SRC_DIR}/espmeshmesh.cpp
  ${ESPMESH_SRC_DIR}/log.cpp
  ${ESPMESH_SRC_DIR}/memringbuffer.cpp
  ${ESPMESH_SRC_DIR}/meshaddress.cpp
  ${ESPMESH_SRC_DIR}/meshsocket.cpp
  ${ESPMESH_SRC_DIR}/multipath.cpp
  ${ESPMESH_SRC_DIR}/packetbuf.cpp
  ${ESPMESH_SRC_DIR}/polite.cpp
  ${ESPMESH_SRC_DIR}/recvdups.cpp
  ${ESPMESH_SRC_DIR}/starpath.cpp
  ${ESPMESH_SRC_DIR}/uart.cpp
  ${ESPMESH_SRC_DIR}/unicast.cpp
  ${ESPMESH_SRC_DIR}/wifi.cpp
  ${ESPMESH_SRC_DIR}/encryption.cpp
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

# Sorgenti per piattaforma
if(TARGET_PLATFORM STREQUAL "linux")
  list(APPEND ESPMESH_CPP_SOURCES
    ${ESPMESH_SRC_DIR}/uart_linux.cpp
    ${ESPMESH_SRC_DIR}/wifi_linux.cpp
  )
endif()

set(ESPMESH_DEFINITIONS LINUX USE_LINUX)

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

# Tipo libreria: SHARED (.so) o STATIC (.a)
if(NOT DEFINED ESPMESH_LIB_TYPE)
  set(ESPMESH_LIB_TYPE SHARED CACHE STRING "Library type: SHARED or STATIC")
endif()
if(NOT ESPMESH_LIB_TYPE MATCHES "^(SHARED|STATIC)$")
  set(ESPMESH_LIB_TYPE SHARED)
endif()

# Libreria condivisa (o statica)
add_library(espmeshmesh ${ESPMESH_LIB_TYPE}
  ${ESPMESH_CPP_SOURCES}
  ${ESPMESH_C_SOURCES}
)

target_include_directories(espmeshmesh PUBLIC
  ${ESPMESH_SRC_DIR}
  ${ESPMESH_PROTOC_DIR}
  ${NANOPB_INCLUDE_DIRS}
)

target_compile_definitions(espmeshmesh PRIVATE ${ESPMESH_DEFINITIONS})

# C++ standard
set_target_properties(espmeshmesh PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  C_STANDARD 99
)

# Per uso come componente sotto ESP-IDF o ESP8266 RTOS SDK:
# aggiungi questo progetto con add_subdirectory(espmeshmesh) e
# assicurati che il toolchain/ambiente fornisca include e link per
# ESP (esp_wifi, FreeRTOS, nanopb, ecc.). In alternativa configura
# la build con il toolchain file della piattaforma scelta.
#
# Esempio ESP-IDF (dalla root di un progetto IDF):
#   add_subdirectory(path/to/espmeshmesh)
#   set(TARGET_PLATFORM esp32)  # prima di add_subdirectory se serve
#
# Esempio ESP8266 RTOS:
#   set(TARGET_PLATFORM esp8266)
#   add_subdirectory(path/to/espmeshmesh)
#
# Opzioni da riga di comando:
#   cmake -DTARGET_PLATFORM=esp8266 -DESPMESH_LIB_TYPE=STATIC ...
#   cmake -DUSE_POLITE_BROADCAST_PROTOCOL=ON -DESPMESH_STARPATH_ENABLED=ON ...
