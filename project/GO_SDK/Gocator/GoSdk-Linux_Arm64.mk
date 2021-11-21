ifndef verbose
	SILENT = @
endif

.PHONY: all
all: kApi GoSdk GoSdkExample 

.PHONY: kApi
kApi: 
	$(SILENT) $(MAKE) -C ../Platform/kApi -f kApi-Linux_Arm64.mk

.PHONY: GoSdk
GoSdk: kApi 
	$(SILENT) $(MAKE) -C GoSdk -f GoSdk-Linux_Arm64.mk

.PHONY: GoSdkExample
GoSdkExample: GoSdk 
	$(SILENT) $(MAKE) -C GoSdk -f GoSdkExample-Linux_Arm64.mk

.PHONY: clean
clean: kApi-clean GoSdk-clean GoSdkExample-clean 

.PHONY: kApi-clean
kApi-clean:
	$(SILENT) $(MAKE) -C ../Platform/kApi -f kApi-Linux_Arm64.mk clean

.PHONY: GoSdk-clean
GoSdk-clean:
	$(SILENT) $(MAKE) -C GoSdk -f GoSdk-Linux_Arm64.mk clean

.PHONY: GoSdkExample-clean
GoSdkExample-clean:
	$(SILENT) $(MAKE) -C GoSdk -f GoSdkExample-Linux_Arm64.mk clean


