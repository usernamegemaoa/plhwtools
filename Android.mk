LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := plhwtools
LOCAL_MODULE_TAGS := eng
LOCAL_CFLAGS += -Wall -O2
LOCAL_SRC_FILES := $(call all-subdir-c-files)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../libplhw
LOCAL_STATIC_LIBRARIES := libplhw
include $(BUILD_EXECUTABLE)
