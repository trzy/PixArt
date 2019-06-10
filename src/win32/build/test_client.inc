#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_test_client = \
	serial_port.cpp \
	test_client.cpp

PROGRAMS += test_client
