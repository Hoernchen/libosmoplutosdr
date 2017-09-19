# Copyright 2017 OSMOCOM Project
#
# This file is part of osmoplutosdr
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

MACRO(RTLSDR_APPEND_SRCS)
    LIST(APPEND osmoplutosdr_srcs ${ARGV})
ENDMACRO(RTLSDR_APPEND_SRCS)

RTLSDR_APPEND_SRCS(
	libosmoplutosdr.c
)

########################################################################
# Setup shared library variant
########################################################################
add_library(osmoplutosdr_shared SHARED ${osmoplutosdr_srcs})
target_link_libraries(osmoplutosdr_shared ${LIBUSB_LIBRARIES})
set_target_properties(osmoplutosdr_shared PROPERTIES DEFINE_SYMBOL "osmoplutosdr_EXPORTS")
set_target_properties(osmoplutosdr_shared PROPERTIES OUTPUT_NAME osmoplutosdr)
set_target_properties(osmoplutosdr_shared PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties(osmoplutosdr_shared PROPERTIES VERSION ${LIBVER})

########################################################################
# Setup static library variant
########################################################################
add_library(osmoplutosdr_static STATIC ${osmoplutosdr_srcs})
target_link_libraries(osmoplutosdr_static ${LIBUSB_LIBRARIES})
set_property(TARGET osmoplutosdr_static APPEND PROPERTY COMPILE_DEFINITIONS "osmoplutosdr_STATIC" )
if(NOT WIN32)
# Force same library filename for static and shared variants of the library
set_target_properties(osmoplutosdr_static PROPERTIES OUTPUT_NAME osmoplutosdr)
endif()

########################################################################
# Setup libraries used in executables
########################################################################

if(WIN32)
add_library(libgetopt_static STATIC
    getopt/getopt.c
)

endif()

########################################################################
# Build utility
########################################################################
add_executable(osmo_pluto osmo_pluto.c)
set(INSTALL_TARGETS osmoplutosdr_shared osmoplutosdr_static osmo_pluto)

target_link_libraries(osmo_pluto osmoplutosdr_shared
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

if(WIN32)
target_link_libraries(osmo_pluto libgetopt_static)
set_property(TARGET osmo_pluto APPEND PROPERTY COMPILE_DEFINITIONS "osmoplutosdr_STATIC" )
endif()
########################################################################
# Install built library files & utilities
########################################################################
install(TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} # .so/.dylib file
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
