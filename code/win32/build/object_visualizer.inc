#
# This file defines the source files necessary to produce a single binary. It
# is included from the main Makefile.
#

SRC_FILES_object_visualizer = \
	src/util/format.cpp \
	src/util/config.cpp \
	src/util/command_line.cpp \
	src/serial/serial_replay_device.cpp \
	src/apps/object_visualizer/print_objects.cpp \
	src/apps/object_visualizer/sensor_settings.cpp \
	src/apps/object_visualizer/window.cpp \
	src/apps/object_visualizer/render.cpp \
	src/apps/object_visualizer/object_window.cpp \
	src/apps/object_visualizer/perspective_window.cpp \
	../arduino/pa_driver/pixart_object.cpp \
	src/serial/serial_port.cpp \
	src/apps/object_visualizer/main.cpp

LDFLAGS_object_visualizer = -lmingw32 -lSDL2main -lSDL2 $(addprefix -l,$(LIBS_OPENGL)) $(addprefix -l,$(LIBS_OPENCV))

PROGRAMS += object_visualizer
