/****************************************************************************
 * apps/boot/nxboot/loader/crc32.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOOT_NXBOOT_LOADER_CRC32_H
#define __BOOT_NXBOOT_LOADER_CRC32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

uint32_t crc32_update(uint32_t crc, uint8_t *data, size_t len);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t crc32_init(void)
{
  return 0xffffffff;
}

static inline uint32_t crc32_finalize(const uint32_t crc)
{
  return ~crc;
}

#endif /* __BOOT_NXBOOT_LOADER_CRC32_H */
