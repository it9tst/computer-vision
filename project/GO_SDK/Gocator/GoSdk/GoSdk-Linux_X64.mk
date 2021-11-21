
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
	TARGET := ../../lib/linux_x64d/libGoSdk.so
	INTERMEDIATES := 
	OBJ_DIR := ../../build/GoSdk-gnumk_linux_x64-Debug
	PREBUILD := 
	POSTBUILD := 
	COMPILER_FLAGS := -g -march=x86-64 -fpic
	C_FLAGS := -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value -Wno-missing-braces
	CXX_FLAGS := -std=c++14 -Wall -Wfloat-conversion
	INCLUDE_DIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES := -DK_DEBUG -DGO_EMIT -DEXPERIMENTAL_FEATURES_ENABLED
	LINKER_FLAGS := -shared -Wl,-no-undefined -Wl,-rpath,'$$ORIGIN'
	LIB_DIRS := -L../../lib/linux_x64d
	LIBS := -Wl,--start-group -lc -lpthread -lrt -lm -lkApi -Wl,--end-group
	LDFLAGS := $(LINKER_FLAGS) $(LIBS) $(LIB_DIRS)

	OBJECTS := ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkLib.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkDef.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAcceleratorMgr.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelerator.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoLayout.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAdvanced.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMaterial.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMultiplexBank.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoPartDetection.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoPartMatching.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoPartModel.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileGeneration.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoRecordingFilter.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoReplay.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoReplayCondition.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSection.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSections.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSensor.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSensorInfo.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSetup.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceGeneration.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSystem.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTransform.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTracheid.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoGeoCal.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoUtils.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAlgorithm.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelSensorPortAlloc.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoControl.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscovery.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoReceiver.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSerializer.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDataSet.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDataTypes.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscoveryExtInfo.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoHealth.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoOutput.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAnalog.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDigital.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoEthernet.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSerial.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurement.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurements.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtMeasurement.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoFeature.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoFeatures.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTool.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParam.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParams.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtTool.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtToolDataOutput.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileToolUtils.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoRangeTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceToolUtils.c.o
	DEP_FILES = ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkLib.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkDef.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAcceleratorMgr.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelerator.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoLayout.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAdvanced.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMaterial.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMultiplexBank.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoPartDetection.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoPartMatching.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoPartModel.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileGeneration.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoRecordingFilter.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoReplay.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoReplayCondition.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSection.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSections.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSensor.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSensorInfo.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSetup.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceGeneration.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSystem.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTransform.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTracheid.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoGeoCal.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoUtils.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAlgorithm.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelSensorPortAlloc.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoControl.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscovery.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoReceiver.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSerializer.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDataSet.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDataTypes.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscoveryExtInfo.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoHealth.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoOutput.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoAnalog.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoDigital.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoEthernet.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSerial.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurement.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurements.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtMeasurement.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoFeature.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoFeatures.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTool.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParam.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParams.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtTool.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoExtToolDataOutput.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileToolUtils.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoRangeTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceToolUtils.c.d
	TARGET_DEPS = ../../Platform/kApi/../../lib/linux_x64d/libkApi.so

endif

ifeq ($(config),Release)
	TARGET := ../../lib/linux_x64/libGoSdk.so
	INTERMEDIATES := 
	OBJ_DIR := ../../build/GoSdk-gnumk_linux_x64-Release
	PREBUILD := 
	POSTBUILD := 
	COMPILER_FLAGS := -O2 -march=x86-64 -fpic
	C_FLAGS := -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value -Wno-missing-braces
	CXX_FLAGS := -std=c++14 -Wall -Wfloat-conversion
	INCLUDE_DIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES := -DGO_EMIT -DEXPERIMENTAL_FEATURES_ENABLED
	LINKER_FLAGS := -shared -Wl,-no-undefined -Wl,-rpath,'$$ORIGIN'
	LIB_DIRS := -L../../lib/linux_x64
	LIBS := -Wl,--start-group -lc -lpthread -lrt -lm -lkApi -Wl,--end-group
	LDFLAGS := $(LINKER_FLAGS) $(LIBS) $(LIB_DIRS)

	OBJECTS := ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkLib.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSdkDef.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAcceleratorMgr.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAccelerator.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoLayout.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAdvanced.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMaterial.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMultiplexBank.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoPartDetection.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoPartMatching.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoPartModel.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoProfileGeneration.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoRecordingFilter.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoReplay.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoReplayCondition.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSection.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSections.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSensor.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSensorInfo.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSetup.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceGeneration.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSystem.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTransform.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTracheid.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoGeoCal.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoUtils.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAlgorithm.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAccelSensorPortAlloc.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoControl.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDiscovery.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoReceiver.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSerializer.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDataSet.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDataTypes.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDiscoveryExtInfo.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoHealth.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoOutput.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAnalog.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDigital.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoEthernet.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSerial.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurement.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurements.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtMeasurement.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoFeature.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoFeatures.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTool.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtParam.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtParams.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtTool.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtToolDataOutput.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoProfileTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoProfileToolUtils.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoRangeTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceTools.c.o \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceToolUtils.c.o
	DEP_FILES = ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkLib.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSdkDef.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAcceleratorMgr.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAccelerator.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoLayout.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAdvanced.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMaterial.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMultiplexBank.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoPartDetection.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoPartMatching.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoPartModel.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoProfileGeneration.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoRecordingFilter.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoReplay.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoReplayCondition.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSection.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSections.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSensor.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSensorInfo.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSetup.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceGeneration.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSystem.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTransform.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTracheid.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoGeoCal.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoUtils.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAlgorithm.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAccelSensorPortAlloc.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoControl.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDiscovery.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoReceiver.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSerializer.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDataSet.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDataTypes.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDiscoveryExtInfo.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoHealth.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoOutput.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoAnalog.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoDigital.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoEthernet.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSerial.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurement.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurements.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtMeasurement.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoFeature.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoFeatures.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTool.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtParam.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtParams.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtTool.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoExtToolDataOutput.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoProfileTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoProfileToolUtils.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoRangeTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceTools.c.d \
	../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceToolUtils.c.d
	TARGET_DEPS = ../../Platform/kApi/../../lib/linux_x64/libkApi.so

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
	$(SILENT) $(MAKE) -f GoSdk-Linux_X64.mk all-dep
	$(SILENT) $(MAKE) -f GoSdk-Linux_X64.mk all-obj

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

../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkLib.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkLib.c.d: GoSdk/GoSdkLib.c
	$(SILENT) $(info GccX64 GoSdk/GoSdkLib.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkLib.c.o -c GoSdk/GoSdkLib.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkDef.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkDef.c.d: GoSdk/GoSdkDef.c
	$(SILENT) $(info GccX64 GoSdk/GoSdkDef.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkDef.c.o -c GoSdk/GoSdkDef.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoAcceleratorMgr.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAcceleratorMgr.c.d: GoSdk/GoAcceleratorMgr.c
	$(SILENT) $(info GccX64 GoSdk/GoAcceleratorMgr.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAcceleratorMgr.c.o -c GoSdk/GoAcceleratorMgr.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelerator.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelerator.c.d: GoSdk/GoAccelerator.c
	$(SILENT) $(info GccX64 GoSdk/GoAccelerator.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelerator.c.o -c GoSdk/GoAccelerator.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoLayout.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoLayout.c.d: GoSdk/GoLayout.c
	$(SILENT) $(info GccX64 GoSdk/GoLayout.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoLayout.c.o -c GoSdk/GoLayout.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoAdvanced.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAdvanced.c.d: GoSdk/GoAdvanced.c
	$(SILENT) $(info GccX64 GoSdk/GoAdvanced.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAdvanced.c.o -c GoSdk/GoAdvanced.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoMaterial.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMaterial.c.d: GoSdk/GoMaterial.c
	$(SILENT) $(info GccX64 GoSdk/GoMaterial.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMaterial.c.o -c GoSdk/GoMaterial.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoMultiplexBank.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMultiplexBank.c.d: GoSdk/GoMultiplexBank.c
	$(SILENT) $(info GccX64 GoSdk/GoMultiplexBank.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMultiplexBank.c.o -c GoSdk/GoMultiplexBank.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoPartDetection.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartDetection.c.d: GoSdk/GoPartDetection.c
	$(SILENT) $(info GccX64 GoSdk/GoPartDetection.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartDetection.c.o -c GoSdk/GoPartDetection.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoPartMatching.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartMatching.c.d: GoSdk/GoPartMatching.c
	$(SILENT) $(info GccX64 GoSdk/GoPartMatching.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartMatching.c.o -c GoSdk/GoPartMatching.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoPartModel.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartModel.c.d: GoSdk/GoPartModel.c
	$(SILENT) $(info GccX64 GoSdk/GoPartModel.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartModel.c.o -c GoSdk/GoPartModel.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileGeneration.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileGeneration.c.d: GoSdk/GoProfileGeneration.c
	$(SILENT) $(info GccX64 GoSdk/GoProfileGeneration.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileGeneration.c.o -c GoSdk/GoProfileGeneration.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoRecordingFilter.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoRecordingFilter.c.d: GoSdk/GoRecordingFilter.c
	$(SILENT) $(info GccX64 GoSdk/GoRecordingFilter.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoRecordingFilter.c.o -c GoSdk/GoRecordingFilter.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoReplay.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoReplay.c.d: GoSdk/GoReplay.c
	$(SILENT) $(info GccX64 GoSdk/GoReplay.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoReplay.c.o -c GoSdk/GoReplay.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoReplayCondition.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoReplayCondition.c.d: GoSdk/GoReplayCondition.c
	$(SILENT) $(info GccX64 GoSdk/GoReplayCondition.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoReplayCondition.c.o -c GoSdk/GoReplayCondition.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSection.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSection.c.d: GoSdk/GoSection.c
	$(SILENT) $(info GccX64 GoSdk/GoSection.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSection.c.o -c GoSdk/GoSection.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSections.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSections.c.d: GoSdk/GoSections.c
	$(SILENT) $(info GccX64 GoSdk/GoSections.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSections.c.o -c GoSdk/GoSections.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSensor.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSensor.c.d: GoSdk/GoSensor.c
	$(SILENT) $(info GccX64 GoSdk/GoSensor.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSensor.c.o -c GoSdk/GoSensor.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSensorInfo.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSensorInfo.c.d: GoSdk/GoSensorInfo.c
	$(SILENT) $(info GccX64 GoSdk/GoSensorInfo.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSensorInfo.c.o -c GoSdk/GoSensorInfo.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSetup.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSetup.c.d: GoSdk/GoSetup.c
	$(SILENT) $(info GccX64 GoSdk/GoSetup.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSetup.c.o -c GoSdk/GoSetup.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceGeneration.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceGeneration.c.d: GoSdk/GoSurfaceGeneration.c
	$(SILENT) $(info GccX64 GoSdk/GoSurfaceGeneration.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceGeneration.c.o -c GoSdk/GoSurfaceGeneration.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSystem.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSystem.c.d: GoSdk/GoSystem.c
	$(SILENT) $(info GccX64 GoSdk/GoSystem.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSystem.c.o -c GoSdk/GoSystem.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoTransform.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTransform.c.d: GoSdk/GoTransform.c
	$(SILENT) $(info GccX64 GoSdk/GoTransform.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTransform.c.o -c GoSdk/GoTransform.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoTracheid.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTracheid.c.d: GoSdk/GoTracheid.c
	$(SILENT) $(info GccX64 GoSdk/GoTracheid.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTracheid.c.o -c GoSdk/GoTracheid.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoGeoCal.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoGeoCal.c.d: GoSdk/GoGeoCal.c
	$(SILENT) $(info GccX64 GoSdk/GoGeoCal.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoGeoCal.c.o -c GoSdk/GoGeoCal.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoUtils.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoUtils.c.d: GoSdk/GoUtils.c
	$(SILENT) $(info GccX64 GoSdk/GoUtils.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoUtils.c.o -c GoSdk/GoUtils.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoAlgorithm.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAlgorithm.c.d: GoSdk/GoAlgorithm.c
	$(SILENT) $(info GccX64 GoSdk/GoAlgorithm.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAlgorithm.c.o -c GoSdk/GoAlgorithm.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelSensorPortAlloc.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelSensorPortAlloc.c.d: GoSdk/Internal/GoAccelSensorPortAlloc.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoAccelSensorPortAlloc.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelSensorPortAlloc.c.o -c GoSdk/Internal/GoAccelSensorPortAlloc.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoControl.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoControl.c.d: GoSdk/Internal/GoControl.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoControl.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoControl.c.o -c GoSdk/Internal/GoControl.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscovery.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscovery.c.d: GoSdk/Internal/GoDiscovery.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoDiscovery.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscovery.c.o -c GoSdk/Internal/GoDiscovery.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoReceiver.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoReceiver.c.d: GoSdk/Internal/GoReceiver.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoReceiver.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoReceiver.c.o -c GoSdk/Internal/GoReceiver.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSerializer.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSerializer.c.d: GoSdk/Internal/GoSerializer.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoSerializer.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSerializer.c.o -c GoSdk/Internal/GoSerializer.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoDataSet.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDataSet.c.d: GoSdk/Messages/GoDataSet.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoDataSet.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDataSet.c.o -c GoSdk/Messages/GoDataSet.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoDataTypes.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDataTypes.c.d: GoSdk/Messages/GoDataTypes.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoDataTypes.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDataTypes.c.o -c GoSdk/Messages/GoDataTypes.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscoveryExtInfo.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscoveryExtInfo.c.d: GoSdk/Messages/GoDiscoveryExtInfo.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoDiscoveryExtInfo.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscoveryExtInfo.c.o -c GoSdk/Messages/GoDiscoveryExtInfo.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoHealth.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoHealth.c.d: GoSdk/Messages/GoHealth.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoHealth.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoHealth.c.o -c GoSdk/Messages/GoHealth.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoOutput.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoOutput.c.d: GoSdk/Outputs/GoOutput.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoOutput.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoOutput.c.o -c GoSdk/Outputs/GoOutput.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoAnalog.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAnalog.c.d: GoSdk/Outputs/GoAnalog.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoAnalog.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoAnalog.c.o -c GoSdk/Outputs/GoAnalog.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoDigital.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDigital.c.d: GoSdk/Outputs/GoDigital.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoDigital.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoDigital.c.o -c GoSdk/Outputs/GoDigital.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoEthernet.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoEthernet.c.d: GoSdk/Outputs/GoEthernet.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoEthernet.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoEthernet.c.o -c GoSdk/Outputs/GoEthernet.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSerial.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSerial.c.d: GoSdk/Outputs/GoSerial.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoSerial.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSerial.c.o -c GoSdk/Outputs/GoSerial.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurement.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurement.c.d: GoSdk/Tools/GoMeasurement.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoMeasurement.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurement.c.o -c GoSdk/Tools/GoMeasurement.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurements.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurements.c.d: GoSdk/Tools/GoMeasurements.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoMeasurements.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurements.c.o -c GoSdk/Tools/GoMeasurements.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoExtMeasurement.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtMeasurement.c.d: GoSdk/Tools/GoExtMeasurement.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtMeasurement.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtMeasurement.c.o -c GoSdk/Tools/GoExtMeasurement.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoFeature.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoFeature.c.d: GoSdk/Tools/GoFeature.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoFeature.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoFeature.c.o -c GoSdk/Tools/GoFeature.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoFeatures.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoFeatures.c.d: GoSdk/Tools/GoFeatures.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoFeatures.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoFeatures.c.o -c GoSdk/Tools/GoFeatures.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoTool.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTool.c.d: GoSdk/Tools/GoTool.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoTool.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTool.c.o -c GoSdk/Tools/GoTool.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParam.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParam.c.d: GoSdk/Tools/GoExtParam.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtParam.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParam.c.o -c GoSdk/Tools/GoExtParam.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParams.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParams.c.d: GoSdk/Tools/GoExtParams.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtParams.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParams.c.o -c GoSdk/Tools/GoExtParams.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoExtTool.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtTool.c.d: GoSdk/Tools/GoExtTool.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtTool.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtTool.c.o -c GoSdk/Tools/GoExtTool.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoExtToolDataOutput.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtToolDataOutput.c.d: GoSdk/Tools/GoExtToolDataOutput.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtToolDataOutput.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtToolDataOutput.c.o -c GoSdk/Tools/GoExtToolDataOutput.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoTools.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTools.c.d: GoSdk/Tools/GoTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoTools.c.o -c GoSdk/Tools/GoTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileTools.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileTools.c.d: GoSdk/Tools/GoProfileTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoProfileTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileTools.c.o -c GoSdk/Tools/GoProfileTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileToolUtils.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileToolUtils.c.d: GoSdk/Tools/GoProfileToolUtils.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoProfileToolUtils.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileToolUtils.c.o -c GoSdk/Tools/GoProfileToolUtils.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoRangeTools.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoRangeTools.c.d: GoSdk/Tools/GoRangeTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoRangeTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoRangeTools.c.o -c GoSdk/Tools/GoRangeTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceTools.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceTools.c.d: GoSdk/Tools/GoSurfaceTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoSurfaceTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceTools.c.o -c GoSdk/Tools/GoSurfaceTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceToolUtils.c.o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceToolUtils.c.d: GoSdk/Tools/GoSurfaceToolUtils.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoSurfaceToolUtils.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceToolUtils.c.o -c GoSdk/Tools/GoSurfaceToolUtils.c -MMD -MP

endif

ifeq ($(config),Release)

../../build/GoSdk-gnumk_linux_x64-Release/GoSdkLib.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkLib.c.d: GoSdk/GoSdkLib.c
	$(SILENT) $(info GccX64 GoSdk/GoSdkLib.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkLib.c.o -c GoSdk/GoSdkLib.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSdkDef.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkDef.c.d: GoSdk/GoSdkDef.c
	$(SILENT) $(info GccX64 GoSdk/GoSdkDef.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkDef.c.o -c GoSdk/GoSdkDef.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoAcceleratorMgr.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoAcceleratorMgr.c.d: GoSdk/GoAcceleratorMgr.c
	$(SILENT) $(info GccX64 GoSdk/GoAcceleratorMgr.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoAcceleratorMgr.c.o -c GoSdk/GoAcceleratorMgr.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoAccelerator.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoAccelerator.c.d: GoSdk/GoAccelerator.c
	$(SILENT) $(info GccX64 GoSdk/GoAccelerator.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoAccelerator.c.o -c GoSdk/GoAccelerator.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoLayout.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoLayout.c.d: GoSdk/GoLayout.c
	$(SILENT) $(info GccX64 GoSdk/GoLayout.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoLayout.c.o -c GoSdk/GoLayout.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoAdvanced.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoAdvanced.c.d: GoSdk/GoAdvanced.c
	$(SILENT) $(info GccX64 GoSdk/GoAdvanced.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoAdvanced.c.o -c GoSdk/GoAdvanced.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoMaterial.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoMaterial.c.d: GoSdk/GoMaterial.c
	$(SILENT) $(info GccX64 GoSdk/GoMaterial.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoMaterial.c.o -c GoSdk/GoMaterial.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoMultiplexBank.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoMultiplexBank.c.d: GoSdk/GoMultiplexBank.c
	$(SILENT) $(info GccX64 GoSdk/GoMultiplexBank.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoMultiplexBank.c.o -c GoSdk/GoMultiplexBank.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoPartDetection.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoPartDetection.c.d: GoSdk/GoPartDetection.c
	$(SILENT) $(info GccX64 GoSdk/GoPartDetection.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoPartDetection.c.o -c GoSdk/GoPartDetection.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoPartMatching.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoPartMatching.c.d: GoSdk/GoPartMatching.c
	$(SILENT) $(info GccX64 GoSdk/GoPartMatching.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoPartMatching.c.o -c GoSdk/GoPartMatching.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoPartModel.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoPartModel.c.d: GoSdk/GoPartModel.c
	$(SILENT) $(info GccX64 GoSdk/GoPartModel.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoPartModel.c.o -c GoSdk/GoPartModel.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoProfileGeneration.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileGeneration.c.d: GoSdk/GoProfileGeneration.c
	$(SILENT) $(info GccX64 GoSdk/GoProfileGeneration.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileGeneration.c.o -c GoSdk/GoProfileGeneration.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoRecordingFilter.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoRecordingFilter.c.d: GoSdk/GoRecordingFilter.c
	$(SILENT) $(info GccX64 GoSdk/GoRecordingFilter.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoRecordingFilter.c.o -c GoSdk/GoRecordingFilter.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoReplay.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoReplay.c.d: GoSdk/GoReplay.c
	$(SILENT) $(info GccX64 GoSdk/GoReplay.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoReplay.c.o -c GoSdk/GoReplay.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoReplayCondition.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoReplayCondition.c.d: GoSdk/GoReplayCondition.c
	$(SILENT) $(info GccX64 GoSdk/GoReplayCondition.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoReplayCondition.c.o -c GoSdk/GoReplayCondition.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSection.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSection.c.d: GoSdk/GoSection.c
	$(SILENT) $(info GccX64 GoSdk/GoSection.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSection.c.o -c GoSdk/GoSection.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSections.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSections.c.d: GoSdk/GoSections.c
	$(SILENT) $(info GccX64 GoSdk/GoSections.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSections.c.o -c GoSdk/GoSections.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSensor.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSensor.c.d: GoSdk/GoSensor.c
	$(SILENT) $(info GccX64 GoSdk/GoSensor.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSensor.c.o -c GoSdk/GoSensor.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSensorInfo.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSensorInfo.c.d: GoSdk/GoSensorInfo.c
	$(SILENT) $(info GccX64 GoSdk/GoSensorInfo.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSensorInfo.c.o -c GoSdk/GoSensorInfo.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSetup.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSetup.c.d: GoSdk/GoSetup.c
	$(SILENT) $(info GccX64 GoSdk/GoSetup.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSetup.c.o -c GoSdk/GoSetup.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceGeneration.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceGeneration.c.d: GoSdk/GoSurfaceGeneration.c
	$(SILENT) $(info GccX64 GoSdk/GoSurfaceGeneration.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceGeneration.c.o -c GoSdk/GoSurfaceGeneration.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSystem.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSystem.c.d: GoSdk/GoSystem.c
	$(SILENT) $(info GccX64 GoSdk/GoSystem.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSystem.c.o -c GoSdk/GoSystem.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoTransform.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoTransform.c.d: GoSdk/GoTransform.c
	$(SILENT) $(info GccX64 GoSdk/GoTransform.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoTransform.c.o -c GoSdk/GoTransform.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoTracheid.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoTracheid.c.d: GoSdk/GoTracheid.c
	$(SILENT) $(info GccX64 GoSdk/GoTracheid.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoTracheid.c.o -c GoSdk/GoTracheid.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoGeoCal.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoGeoCal.c.d: GoSdk/GoGeoCal.c
	$(SILENT) $(info GccX64 GoSdk/GoGeoCal.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoGeoCal.c.o -c GoSdk/GoGeoCal.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoUtils.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoUtils.c.d: GoSdk/GoUtils.c
	$(SILENT) $(info GccX64 GoSdk/GoUtils.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoUtils.c.o -c GoSdk/GoUtils.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoAlgorithm.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoAlgorithm.c.d: GoSdk/GoAlgorithm.c
	$(SILENT) $(info GccX64 GoSdk/GoAlgorithm.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoAlgorithm.c.o -c GoSdk/GoAlgorithm.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoAccelSensorPortAlloc.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoAccelSensorPortAlloc.c.d: GoSdk/Internal/GoAccelSensorPortAlloc.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoAccelSensorPortAlloc.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoAccelSensorPortAlloc.c.o -c GoSdk/Internal/GoAccelSensorPortAlloc.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoControl.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoControl.c.d: GoSdk/Internal/GoControl.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoControl.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoControl.c.o -c GoSdk/Internal/GoControl.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoDiscovery.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoDiscovery.c.d: GoSdk/Internal/GoDiscovery.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoDiscovery.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoDiscovery.c.o -c GoSdk/Internal/GoDiscovery.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoReceiver.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoReceiver.c.d: GoSdk/Internal/GoReceiver.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoReceiver.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoReceiver.c.o -c GoSdk/Internal/GoReceiver.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSerializer.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSerializer.c.d: GoSdk/Internal/GoSerializer.c
	$(SILENT) $(info GccX64 GoSdk/Internal/GoSerializer.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSerializer.c.o -c GoSdk/Internal/GoSerializer.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoDataSet.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoDataSet.c.d: GoSdk/Messages/GoDataSet.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoDataSet.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoDataSet.c.o -c GoSdk/Messages/GoDataSet.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoDataTypes.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoDataTypes.c.d: GoSdk/Messages/GoDataTypes.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoDataTypes.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoDataTypes.c.o -c GoSdk/Messages/GoDataTypes.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoDiscoveryExtInfo.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoDiscoveryExtInfo.c.d: GoSdk/Messages/GoDiscoveryExtInfo.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoDiscoveryExtInfo.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoDiscoveryExtInfo.c.o -c GoSdk/Messages/GoDiscoveryExtInfo.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoHealth.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoHealth.c.d: GoSdk/Messages/GoHealth.c
	$(SILENT) $(info GccX64 GoSdk/Messages/GoHealth.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoHealth.c.o -c GoSdk/Messages/GoHealth.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoOutput.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoOutput.c.d: GoSdk/Outputs/GoOutput.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoOutput.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoOutput.c.o -c GoSdk/Outputs/GoOutput.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoAnalog.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoAnalog.c.d: GoSdk/Outputs/GoAnalog.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoAnalog.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoAnalog.c.o -c GoSdk/Outputs/GoAnalog.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoDigital.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoDigital.c.d: GoSdk/Outputs/GoDigital.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoDigital.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoDigital.c.o -c GoSdk/Outputs/GoDigital.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoEthernet.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoEthernet.c.d: GoSdk/Outputs/GoEthernet.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoEthernet.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoEthernet.c.o -c GoSdk/Outputs/GoEthernet.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSerial.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSerial.c.d: GoSdk/Outputs/GoSerial.c
	$(SILENT) $(info GccX64 GoSdk/Outputs/GoSerial.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSerial.c.o -c GoSdk/Outputs/GoSerial.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurement.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurement.c.d: GoSdk/Tools/GoMeasurement.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoMeasurement.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurement.c.o -c GoSdk/Tools/GoMeasurement.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurements.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurements.c.d: GoSdk/Tools/GoMeasurements.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoMeasurements.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurements.c.o -c GoSdk/Tools/GoMeasurements.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoExtMeasurement.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtMeasurement.c.d: GoSdk/Tools/GoExtMeasurement.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtMeasurement.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtMeasurement.c.o -c GoSdk/Tools/GoExtMeasurement.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoFeature.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoFeature.c.d: GoSdk/Tools/GoFeature.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoFeature.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoFeature.c.o -c GoSdk/Tools/GoFeature.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoFeatures.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoFeatures.c.d: GoSdk/Tools/GoFeatures.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoFeatures.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoFeatures.c.o -c GoSdk/Tools/GoFeatures.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoTool.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoTool.c.d: GoSdk/Tools/GoTool.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoTool.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoTool.c.o -c GoSdk/Tools/GoTool.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoExtParam.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtParam.c.d: GoSdk/Tools/GoExtParam.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtParam.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtParam.c.o -c GoSdk/Tools/GoExtParam.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoExtParams.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtParams.c.d: GoSdk/Tools/GoExtParams.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtParams.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtParams.c.o -c GoSdk/Tools/GoExtParams.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoExtTool.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtTool.c.d: GoSdk/Tools/GoExtTool.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtTool.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtTool.c.o -c GoSdk/Tools/GoExtTool.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoExtToolDataOutput.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtToolDataOutput.c.d: GoSdk/Tools/GoExtToolDataOutput.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoExtToolDataOutput.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoExtToolDataOutput.c.o -c GoSdk/Tools/GoExtToolDataOutput.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoTools.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoTools.c.d: GoSdk/Tools/GoTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoTools.c.o -c GoSdk/Tools/GoTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoProfileTools.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileTools.c.d: GoSdk/Tools/GoProfileTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoProfileTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileTools.c.o -c GoSdk/Tools/GoProfileTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoProfileToolUtils.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileToolUtils.c.d: GoSdk/Tools/GoProfileToolUtils.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoProfileToolUtils.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileToolUtils.c.o -c GoSdk/Tools/GoProfileToolUtils.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoRangeTools.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoRangeTools.c.d: GoSdk/Tools/GoRangeTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoRangeTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoRangeTools.c.o -c GoSdk/Tools/GoRangeTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceTools.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceTools.c.d: GoSdk/Tools/GoSurfaceTools.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoSurfaceTools.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceTools.c.o -c GoSdk/Tools/GoSurfaceTools.c -MMD -MP

../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceToolUtils.c.o ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceToolUtils.c.d: GoSdk/Tools/GoSurfaceToolUtils.c
	$(SILENT) $(info GccX64 GoSdk/Tools/GoSurfaceToolUtils.c)
	$(SILENT) $(C_COMPILER) $(COMPILER_FLAGS) $(C_FLAGS) $(DEFINES) $(INCLUDE_DIRS) -o ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceToolUtils.c.o -c GoSdk/Tools/GoSurfaceToolUtils.c -MMD -MP

endif

ifeq ($(MAKECMDGOALS),all-obj)

ifeq ($(config),Debug)

include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkLib.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSdkDef.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoAcceleratorMgr.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelerator.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoLayout.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoAdvanced.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoMaterial.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoMultiplexBank.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartDetection.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartMatching.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoPartModel.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileGeneration.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoRecordingFilter.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoReplay.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoReplayCondition.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSection.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSections.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSensor.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSensorInfo.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSetup.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceGeneration.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSystem.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoTransform.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoTracheid.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoGeoCal.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoUtils.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoAlgorithm.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoAccelSensorPortAlloc.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoControl.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscovery.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoReceiver.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSerializer.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoDataSet.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoDataTypes.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoDiscoveryExtInfo.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoHealth.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoOutput.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoAnalog.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoDigital.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoEthernet.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSerial.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurement.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoMeasurements.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtMeasurement.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoFeature.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoFeatures.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoTool.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParam.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtParams.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtTool.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoExtToolDataOutput.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoProfileToolUtils.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoRangeTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Debug/GoSurfaceToolUtils.c.d

endif

ifeq ($(config),Release)

include ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkLib.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSdkDef.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoAcceleratorMgr.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoAccelerator.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoLayout.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoAdvanced.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoMaterial.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoMultiplexBank.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoPartDetection.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoPartMatching.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoPartModel.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileGeneration.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoRecordingFilter.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoReplay.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoReplayCondition.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSection.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSections.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSensor.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSensorInfo.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSetup.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceGeneration.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSystem.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoTransform.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoTracheid.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoGeoCal.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoUtils.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoAlgorithm.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoAccelSensorPortAlloc.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoControl.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoDiscovery.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoReceiver.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSerializer.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoDataSet.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoDataTypes.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoDiscoveryExtInfo.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoHealth.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoOutput.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoAnalog.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoDigital.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoEthernet.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSerial.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurement.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoMeasurements.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoExtMeasurement.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoFeature.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoFeatures.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoTool.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoExtParam.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoExtParams.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoExtTool.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoExtToolDataOutput.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoProfileToolUtils.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoRangeTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceTools.c.d
include ../../build/GoSdk-gnumk_linux_x64-Release/GoSurfaceToolUtils.c.d

endif

endif

