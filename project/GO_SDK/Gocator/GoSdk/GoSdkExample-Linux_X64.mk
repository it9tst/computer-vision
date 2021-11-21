
ifeq ($(OS)$(os), Windows_NT)
	CROSS_PREFIX := C:/tools/GccX64_4.9.4-p23/x86_64-linux-gnu/bin/x86_64-linux-gnu-
	CROSS_SUFFIX := .exe
	PYTHON := python
	MKDIR_P := $(PYTHON) ../../../Platform/scripts/Utils/kUtil.py mkdir_p
	RM_F := $(PYTHON) ../../../Platform/scripts/Utils/kUtil.py rm_f
	RM_RF := $(PYTHON) ../../../Platform/scripts/Utils/kUtil.py rm_rf
	CP := $(PYTHON) ../../../Platform/scripts/Utils/kUtil.py cp
else
	BUILD_MACHINE := $(shell uname -m)
	ifneq ($(BUILD_MACHINE), x86_64)
		CROSS_PREFIX := /tools/GccX64_4.9.4-p23/x86_64-linux-gnu/bin/x86_64-linux-gnu-
		CROSS_SUFFIX := 
	endif
	PYTHON := python3
	MKDIR_P := mkdir -p
	RM_F := rm -f
	RM_RF := rm -rf
	CP := cp
endif

C_COMPILER := $(CROSS_PREFIX)gcc$(CROSS_SUFFIX)
CXX_COMPILER := $(CROSS_PREFIX)g++$(CROSS_SUFFIX)
LINKER := $(CROSS_PREFIX)g++$(CROSS_SUFFIX)
ARCHIVER := $(CROSS_PREFIX)ar$(CROSS_SUFFIX)
GNU_READELF := $(CROSS_PREFIX)readelf$(CROSS_SUFFIX)
APP_GEN := $(PYTHON) ../../../Platform/scripts/Utils/kAppGen.py

ifndef verbose
	SILENT := @
endif

ifndef config
	config := Debug
endif

# We require tools to be installed according to specific conventions (see manuals).
# Tool prerequisites may change between major releases; check and report.
ifeq ($(shell $(C_COMPILER) --version),)
.PHONY: tc_err
tc_err:
	$(error Cannot build because of missing prerequisite; please install)
endif

ifeq ($(config),Debug)
	TARGET := ../../bin/linux_x64d/GoSdkExample
	INTERMEDIATES := 
	OBJ_DIR := ../../build/GoSdkExample-gnumk_linux_x64-Debug
	PREBUILD := 
	POSTBUILD := 
	COMPILER_FLAGS := -g -march=x86-64 -fpic
	C_FLAGS := -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value -Wno-missing-braces
	CXX_FLAGS := -std=c++14 -Wall -Wfloat-conversion
	INCLUDE_DIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES :=
	LINKER_FLAGS := -Wl,-no-undefined -Wl,--allow-shlib-undefined -Wl,-rpath,'$$ORIGIN/../../lib/linux_x64d' -Wl,-rpath-link,../../lib/linux_x64d
	LIB_DIRS := -L../../lib/linux_x64d
	LIBS := -Wl,--start-group -lkApi -lGoSdk -Wl,--end-group
	LDFLAGS := $(LINKER_FLAGS) $(LIBS) $(LIB_DIRS)

	OBJECTS := ../../build/GoSdkExample-gnumk_linux_x64-Debug/GoSdkExample.c.o
	DEP_FILES = ../../build/GoSdkExample-gnumk_linux_x64-Debug/GoSdkExample.c.d
	TARGET_DEPS = ./../../lib/linux_x64d/libGoSdk.so

endif

ifeq ($(config),Release)
	TARGET := ../../bin/linux_x64/GoSdkExample
	INTERMEDIATES := 
	OBJ_DIR := ../../build/GoSdkExample-gnumk_linux_x64-Release
	PREBUILD := 
	POSTBUILD := 
	COMPILER_FLAGS := -O2 -march=x86-64 -fpic
	C_FLAGS := -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value -Wno-missing-braces
	CXX_FLAGS := -std=c++14 -Wall -Wfloat-conversion
	INCLUDE_DIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES :=
	LINKER_FLAGS := -Wl,-no-undefined -Wl,--allow-shlib-undefined -Wl,-rpath,'$$ORIGIN/../../lib/linux_x64' -Wl,-rpath-link,../../lib/linux_x64
	LIB_DIRS := -L../../lib/linux_x64
	LIBS := -Wl,--start-group -lkApi -lGoSdk -Wl,--end-group
	LDFLAGS := $(LINKER_FLAGS) $(LIBS) $(LIB_DIRS)

	OBJECTS := ../../build/GoSdkExample-gnumk_linux_x64-Release/GoSdkExample.c.o
	DEP_FILES = ../../build/GoSdkExample-gnumk_linux_x64-Release/GoSdkExample.c.d
	TARGET_DEPS = ./../../lib/linux_x64/libGoSdk.so

endif

ifdef profile
	COMPILER_FLAGS += -pg
	LDFLAGS += -pg
endif

ifdef coverage
	COMPILER_FLAGS += --coverage -fprofile-arcs -ftest-coverage
	LDFLAGS += --coverage
	LIBS += -lgcov
endif

ifdef sanitize
	COMPILER_FLAGS += -fsanitize=$(sanitize)
	LDFLAGS += -fsanitize=$(sanitize)
endif

.PHONY: all all-obj all-dep clean

all: $(OBJ_DIR)
	$(PREBUILD)
	$(SILENT) $(MAKE) -f GoSdkExample-Linux_X64.mk all-dep
	$(SILENT) $(MAKE) -f GoSdkExample-Linux_X64.mk all-obj

clean:
	$(SILENT) $(info Cleaning $(OBJ_DIR))
	$(SILENT) $(RM_RF) $(OBJ_DIR)
	$(SILENT) $(info Cleaning $(TARGET) $(INTERMEDIATES))
	$(SILENT) $(RM_F) $(TARGET) $(INTERMEDIATES)

all-obj: $(OBJ_DIR) $(TARGET)
all-dep: $(OBJ_DIR) $(DEP_FILES)

$(OBJ_DIR):
	$(SILENT) $(MKDIR_P) $@

ifeq ($(config),Debug)

$(TARGET): $(OBJECTS) $(TARGET_DEPS)
	$(SILENT) $(info LdX64 $(TARGET))
	$(SILENT) $(LINKER) $(OBJECTS) $(LDFLAGS) -o$(TARGET)

endif

ifeq ($(config),Release)

$(TARGET): $(OBJECTS) $(TARGET_DEPS)
	$(SILENT) $(info LdX64 $(TARGET))
	$(SILENT) $(LINKER) $(OBJECTS) $(LDFLAGS) -o$(TARGET)

endif

ifeq ($(config),Debug)

../../build/GoSdkExample-gnumk_linux_x64-Debug/GoSdkExample.c.o ../../build/GoSdkExample-gnumk_linux_x64-Debug/GoSdkExample.c.d: GoSdkExample/GoSdkExample.c
	$(SILENT) $(info GccX64 GoSdkExample/GoSdkExample.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdkExample-gnumk_linux_x64-Debug/GoSdkExample.c.o -c GoSdkExample/GoSdkExample.c -MMD -MP

endif

ifeq ($(config),Release)

../../build/GoSdkExample-gnumk_linux_x64-Release/GoSdkExample.c.o ../../build/GoSdkExample-gnumk_linux_x64-Release/GoSdkExample.c.d: GoSdkExample/GoSdkExample.c
	$(SILENT) $(info GccX64 GoSdkExample/GoSdkExample.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdkExample-gnumk_linux_x64-Release/GoSdkExample.c.o -c GoSdkExample/GoSdkExample.c -MMD -MP

endif

ifeq ($(MAKECMDGOALS),all-obj)

ifeq ($(config),Debug)

include ../../build/GoSdkExample-gnumk_linux_x64-Debug/GoSdkExample.c.d

endif

ifeq ($(config),Release)

include ../../build/GoSdkExample-gnumk_linux_x64-Release/GoSdkExample.c.d

endif

endif

