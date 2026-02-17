cmake_minimum_required(VERSION 3.16)
project(espmeshmesh-app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)


# Includi espmeshmesh come sottoprogetto (include nanopb, tiny-aes e tutte le dipendenze)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/espmeshmesh EXCLUDE_FROM_ALL)

add_executable(espmeshmesh-app src/main.cpp)

# Include e link da target espmeshmesh (propagati automaticamente)
target_link_libraries(espmeshmesh-app PRIVATE espmeshmesh)
