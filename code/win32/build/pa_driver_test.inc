#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_pa_driver_test = \
	src/serial/serial_port.cpp \
	src/util/format.cpp \
	src/util/config.cpp \
	src/util/command_line.cpp \
	src/apps/tests/pa_driver_test.cpp

PROGRAMS += pa_driver_test
