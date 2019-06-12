#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_ping_client = \
	src/serial/serial_port.cpp \
	src/util/format.cpp \
	src/apps/ping_client.cpp

PROGRAMS += ping_client
