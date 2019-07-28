#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_pnp_test = \
	src/apps/tests/pnp_test.cpp

LDFLAGS_pnp_test = $(addprefix -l,$(LIBS_OPENCV))

PROGRAMS += pnp_test
