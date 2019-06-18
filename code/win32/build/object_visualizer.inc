#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_object_visualizer = \
	../arduino/pa_driver/pixart_object.cpp \
	src/serial/serial_port.cpp \
	src/util/format.cpp \
	src/util/config.cpp \
	src/util/command_line.cpp \
	src/apps/object_visualizer.cpp

PROGRAMS += object_visualizer
