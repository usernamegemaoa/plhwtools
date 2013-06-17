include $(BUILDER_HOME)/builder.mk

CFLAGS += -O2 -Wall
out := plhwtools
libs := libplhw.a

include $(BUILDER_HOME)/app.mk
