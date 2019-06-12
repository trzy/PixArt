#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_test_client = \
	src/serial/serial_port.cpp \
	src/apps/test_client.cpp

PROGRAMS += test_client
