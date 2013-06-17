/*
 * Copyright (C) 2011, 2012 Plastic Logic Limited
 * All rights reserved.
 */

#ifndef INCLUDE_LOG_H
#define INCLUDE_LOG_H 1

#ifndef LOG_FILE
# define LOG_FILE stderr
#endif

#ifdef LOG_TAG
#include <stdio.h>
# define LOG_N(msg, ...) \
	fprintf(LOG_FILE, "[%-8s %4i] "msg, LOG_TAG, __LINE__, ##__VA_ARGS__)
# define LOG_PRINT(msg, ...) \
	fprintf(LOG_FILE, msg, ##__VA_ARGS__)
#else
# define LOG_N(msg, ...)
# define LOG_PRINT(msg, ...)
#endif

#define LOG(msg, ...) LOG_N(msg"\n", ##__VA_ARGS__)

#endif /* INCLUDE_LOG_H */
