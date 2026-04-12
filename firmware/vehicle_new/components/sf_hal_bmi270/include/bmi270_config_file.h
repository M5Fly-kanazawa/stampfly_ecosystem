/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Kouhei Ito
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file bmi270_config_file.h
 * @brief BMI270 Configuration File (8KB)
 *
 * This file contains the official Bosch Sensortec BMI270 configuration data.
 * This configuration must be uploaded to the BMI270 during initialization
 * to enable advanced features.
 *
 * Source: Bosch Sensortec BMI270 Configuration File
 * Size: 8192 bytes (0x2000)
 *
 * Usage:
 *   During BMI270 initialization, this data must be written to register
 *   INIT_DATA (0x5E) in 16-byte bursts.
 */

#ifndef BMI270_CONFIG_FILE_H
#define BMI270_CONFIG_FILE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Size of BMI270 configuration file in bytes */
#define BMI270_CONFIG_FILE_SIZE 8192

/** BMI270 configuration data array */
extern const uint8_t bmi270_config_file[BMI270_CONFIG_FILE_SIZE];

#ifdef __cplusplus
}
#endif

#endif // BMI270_CONFIG_FILE_H
