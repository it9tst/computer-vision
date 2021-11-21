ifndef verbose
	SILENT = @
endif

.PHONY: all
all: kApi GoSdk 

.PHONY: kApi
kApi: 
	$(SILENT) $(MAKE) -C ../Platform/kApi -f kApi-Linux_X86.mk

.PHONY: GoSdk
GoSdk: kApi 
	$(SILENT) $(MAKE) -C GoSdk -f GoSdk-Linux_X86.mk

.PHONY: clean
clean: kApi-clean GoSdk-clean 

.PHONY: kApi-clean
kApi-clean:
	$(SILENT) $(MAKE) -C ../Platform/kApi -f kApi-Linux_X86.mk clean

.PHONY: GoSdk-clean
GoSdk-clean:
	$(SILENT) $(MAKE) -C GoSdk -f GoSdk-Linux_X86.mk clean


