#
# Makefile
#
# Project Makefile. Edit libraries section as needed.
#

###############################################################################
# Search Paths
###############################################################################

PROJECT_INCLUDE_DIRS = ..\arduino

BOOST_INCLUDE_DIR = c:\boost\boost_1_69_0
BOOST_LIB_DIR = c:\boost\boost_1_69_0\stage\lib

OPENCV_INCLUDE_DIR = c:\opencv\builds\opencv-4.0.1\mingw\install\include
OPENCV_LIB_DIR = c:\opencv\builds\opencv-4.0.1\mingw\install\x64\mingw\staticlib

SDL2_INCLUDE_DIR = c:/mingw-w64/x86_64-8.1.0-posix-seh-rt_v6-rev0/mingw64/lib/gcc/x86_64-w64-mingw32/8.1.0/include
SDL2_LIB_DIR = c:/mingw-w64/x86_64-8.1.0-posix-seh-rt_v6-rev0/mingw64/lib/gcc/x86_64-w64-mingw32/8.1.0/lib

INCLUDE_DIRS = $(PROJECT_INCLUDE_DIRS) $(BOOST_INCLUDE_DIR) $(OPENCV_INCLUDE_DIR) $(SDL2_INCLUDE_DIR)

LIB_DIRS = $(BOOST_LIB_DIR) $(OPENCV_LIB_DIR) $(SDL2_LIB_DIR)

###############################################################################
# Libraries
###############################################################################

LIBS_WIN32API = gdi32 comdlg32
LIBS_NETWORKING = ws2_32 wsock32
LIBS_OPENGL = opengl32 glu32
LIBS_OPENCV = opencv_calib3d401 opencv_objdetect401 opencv_imgcodecs401 opencv_imgproc401 opencv_highgui401 opencv_core401 libtiff libjpeg-turbo libwebp libpng libjasper IlmImf zlib
LIBS_BOOST = boost_filesystem-mgw81-mt-s-x64-1_69
LIBS_UNIT_TEST = boost_unit_test_framework-mgw81-mt-s-x64-1_69
LIBS_COMMON = $(LIBS_WIN32API) $(LIBS_BOOST)

###############################################################################
# Build Options
###############################################################################

#
# Verbose progression
#
VERBOSE =
ifneq ($(filter $(strip $(VERBOSE)),0 1),$(strip $(VERBOSE)))
	override VERBOSE =
endif
SILENT = @
ifeq ($(strip $(VERBOSE)),1)
	SILENT =
endif

###############################################################################
# Source Files
###############################################################################

#
# Each program definition will add itself to this variable
#
PROGRAMS =

#
# Program definitions
#
include build/pa_driver_test.inc
include build/object_visualizer.inc

#
# Header file location
#
INCLUDE_DIRS += src/include/

###############################################################################
# Output Locations
###############################################################################

OBJ_DIR = obj
BIN_DIR = bin

###############################################################################
# Toolchain
###############################################################################

CXX = g++
LD = g++

CXXFLAGS = -c -std=c++17 -O3 $(addprefix -I,$(sort $(INCLUDE_DIRS)))
LDFLAGS = $(addprefix -L,$(sort $(LIB_DIRS))) $(addprefix -l,$(LIBS_COMMON))

DELETE = rmdir /s /q

###############################################################################
# Target Generator Function
###############################################################################

define MakeTarget =

#
# Program targets can add custom linker options
#
LDFLAGS_RESOLVED_$(1) := $(LDFLAGS_$(1))

#
# Object files in sorted-path compile order
#
OBJ_FILES_$(1) = $(foreach file,$(SRC_FILES_$(1)),$(OBJ_DIR)/$(basename $(notdir $(file))).o)

#
# Build the executable
#
$(BIN_DIR)/$(1).exe: $$(OBJ_FILES_$(1))
	$$(info -----------------------------------------------------------------------------)
	$$(info Linking                : $(BIN_DIR)/$(1).exe)
	$$(info -----------------------------------------------------------------------------)
	$(SILENT)$(LD) -o $(BIN_DIR)/$(1).exe $$(OBJ_FILES_$(1)) $(LDFLAGS) $$(LDFLAGS_RESOLVED_$(1))

#
# Create list of auto-generated dependency files (which contain rules that make
# understands) and include them all.
#
AUTODEPS := $$(patsubst %.o,%.d,$$(OBJ_FILES_$(1)))
-include $$(AUTODEPS)

#
# VPATH is the search path for files. This trick allows a single %.cpp rule to
# be matched against files in any subdirectory. This function strips off file
# names, saving the directory prefix, and sorts to remove duplicates and
# construct a set.
#
VPATH = $(sort $(VPATH) $(foreach file,$(SRC_FILES_$(1)),$(dir	$(file))))

endef

###############################################################################
# Targets
###############################################################################

.PHONY: all clean

#
# Default target: builds the binary. Must be specified *before* the auto-
# generated dependencies because otherwise, make gets confused for some reason
# and thinks the default target is just one of the object files.
#
all:	$(BIN_DIR) $(OBJ_DIR) $(foreach program,$(PROGRAMS),$(BIN_DIR)/$(program).exe)

clean:
	$(SILENT)echo Cleaning up $(BIN_DIR) and $(OBJ_DIR)...
	$(SILENT)$(DELETE) $(BIN_DIR)
	$(SILENT)$(DELETE) $(OBJ_DIR)

$(BIN_DIR):
	$(info Creating directory     : $(BIN_DIR))
	$(SILENT)mkdir $(BIN_DIR)

$(OBJ_DIR):
	$(info Creating directory     : $(OBJ_DIR))
	$(SILENT)mkdir $(OBJ_DIR)

#
# Generate targets for each program
#
$(foreach program,$(PROGRAMS),$(eval $(call MakeTarget,$(program))))

###############################################################################
# Rules
###############################################################################

#
# Compilation rules that both auto-generate the dependency files and compile
# the source code. This technique is described in the reply by user "rr-" at:
# https://stackoverflow.com/questions/8025766/makefile-auto-dependency-generation
#
$(OBJ_DIR)/%.o:	%.cpp
	$(info Generating dependencies: $< -> $(OBJ_DIR)/$(*F).d)
	$(SILENT)$(CXX) -MM -MP -MT $(OBJ_DIR)/$(*F).o -MT $(OBJ_DIR)/$(*F).d $(CXXFLAGS) $< > $(OBJ_DIR)/$(*F).d
	$(info Compiling              : $< -> $@)
	$(SILENT)$(CXX) $(CXXFLAGS) $< -o $@