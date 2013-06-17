/*
  E-Paper test - plhwtools

  Copyright (C) 2011, 2012, 2013 Plastic Logic Limited

      Guillaume Tucker <guillaume.tucker@plasticlogic.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
