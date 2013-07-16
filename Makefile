include $(BUILDER_HOME)/builder.mk

CFLAGS += -O2 -Wall
LDFLAGS += -lplsdk
out := plhwtools
libs := libplsdk.so

include $(BUILDER_HOME)/app.mk
