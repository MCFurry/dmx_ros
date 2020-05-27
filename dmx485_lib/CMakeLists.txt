cmake_minimum_required(VERSION 2.6)
project(DMX485 C)

add_library(dmx485 SHARED dmx485.c)
target_link_libraries(dmx485 rt)

install(FILES dmx485.h DESTINATION include)
install(TARGETS dmx485 LIBRARY DESTINATION lib)
