# Project Name
TARGET = marigold

USE_DAISYSP_LGPL = 1

# Sources
CPP_SOURCES = Marigold.cpp

OPT = -O1
#OPT ?= -O3

# Library Locations
LIBDAISY_DIR = ../../../../Dependencies/libDaisy
DAISYSP_DIR = ../../../../Dependencies/DaisySP


# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile


# Include funbox.h
C_INCLUDES += -I../../include