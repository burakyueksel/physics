#ifndef CONFIG_H_
#define CONFIG_H_

/**
 * @author  Burak Yueksel
 * @date    13 May 2023
 * @brief   Config libraries
 * @addtogroup CONFIG
 **/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include "json-c/json.h"

#define MAX_LINE_LENGTH 1024

void generate_header_from_json(const char* input_file, const char* output_file, const char* header_guard);

#endif // CONFIG_H_