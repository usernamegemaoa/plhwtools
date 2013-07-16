LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := plhwtools
LOCAL_MODULE_TAGS := eng
LOCAL_CFLAGS += -Wall -O2
LOCAL_SRC_FILES := plhwtools.c
LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/../libplhw \
	$(LOCAL_PATH)/../libplepaper
LOCAL_STATIC_LIBRARIES := libplhw libplepaper
include $(BUILD_EXECUTABLE)
