#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_ping_client = \
	serial_port.cpp \
	util/format.cpp \
	ping_client.cpp

PROGRAMS += ping_client
