include ../BuildSystem/CommonDefs.mak

BIN_DIR = Bin

INC_DIRS += $(OPENNI2_INCLUDE) $(NITE2_INCLUDE)

SRC_FILES = *.cpp

ifeq ("$(OSTYPE)","Darwin")
	CFLAGS += MACOS
	LDFLAGS += -framework OpenGL -framework GLUT
else
	CFLAGS += -DUNIX -DGLX_GLXEXT_LEGACY
	USED_LIBS += glut GL
endif

LIB_DIRS += $(OPENNI2_REDIST) $(NITE2_REDIST64)

USED_LIBS += OpenNI2 NiTE2

EXE_NAME = UserViewer

CFLAGS += -Wall

include ../BuildSystem/CommonCppMakefile

.PHONY: copy-redist
copy-redist:
	cp -R $(NITE2_REDIST64)/* $(OUT_DIR)

$(OUTPUT_FILE): copy-redist
