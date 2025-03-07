/****************************************************************************
 * apps/boot/nxboot/loader/boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <errno.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/crc32.h>
#include <nxboot.h>

#include "flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NXBOOT_PREVENT_DOWNGRADE
  # warning "Downgrade prevention currently ignores prerelease."
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum nxboot_operation
{
  CREATE_RECOVERY = 0,
  DO_UPDATE,
  DO_REVERT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int get_image_last_wp(int fd)
{
  uint8_t val;
  struct flash_partition_info info;

  if (flash_partition_info(fd, &info) < 0)
    {
      return ERROR;
    }

  if (flash_partition_read(fd, &val, 1, info.size - info.blocksize) < 0)
    {
      return ERROR;
    }

  return val;
}

static inline int set_image_last_wp(int fd, uint8_t value)
{
  uint8_t val;
  struct flash_partition_info info;

  if (flash_partition_info(fd, &info) < 0)
    {
      return ERROR;
    }

  return flash_partition_write(fd, &val, 1, info.size - info.blocksize);
}

static inline void get_image_header(int fd, struct nxboot_img_header *header)
{
  int ret;
  ret = flash_partition_read(fd, header, sizeof *header, 0);
  if (ret < 0)
    {
      /* Something went wrong, treat the partition as empty. */

      memset(header, 0, sizeof *header);
    }
}

static inline bool validate_image_header(struct nxboot_img_header *header)
{
  return header->magic == NXBOOT_HEADER_MAGIC ||
         header->magic == NXBOOT_HEADER_MAGIC_INV;
}

static uint32_t calculate_crc(int fd, struct nxboot_img_header *header)
{
  char *buf;
  int remain;
  int readsiz;
  off_t off;
  uint32_t crc;
  struct flash_partition_info info;

  if (flash_partition_info(fd, &info) < 0)
    {
      return false;
    }

  buf = malloc(info.blocksize);
  if (!buf)
    {
      return false;
    }

  crc = 0xffffffff;
  off = offsetof(struct nxboot_img_header, crc) + sizeof crc;
  remain = header->size + header->header_size - off;
  while (remain > 0)
    {
      readsiz = remain > info.blocksize ? info.blocksize : remain;
      if (flash_partition_read(fd, buf, readsiz, off) != 0)
        {
          free(buf);
          return 0xffffffff;
        }

      off += readsiz;
      remain -= readsiz;
      crc = crc32part((uint8_t *)buf, readsiz, crc);
    }

  free(buf);
  return ~crc;
}

static int copy_partition(int from, int where,
                          enum nxboot_operation operation)
{
  struct nxboot_img_header header;
  struct flash_partition_info info_from;
  struct flash_partition_info info_where;
  uint32_t magic;
  int readsiz;
  int remain;
  int blocksize;
  off_t off;
  char *buf;

  get_image_header(from, &header);

  if (flash_partition_info(from, &info_from) < 0)
    {
      return ERROR;
    }

  if (flash_partition_info(where, &info_where) < 0)
    {
      return ERROR;
    }

  if ((header.size + CONFIG_NXBOOT_HEADER_SIZE) > info_where.size)
    {
      return ERROR;
    }

  blocksize = MAX(info_from.blocksize, info_where.blocksize);

  buf = malloc(blocksize);
  if (!buf)
    {
      return ERROR;
    }

  if (flash_partition_erase_last_sector(where) < 0)
    {
      return ERROR;
    }

  remain = header.size + CONFIG_NXBOOT_HEADER_SIZE;
  off = 0;
  if (operation != DO_REVERT)
    {
      magic = header.magic == NXBOOT_HEADER_MAGIC_INV ?
        NXBOOT_HEADER_MAGIC : NXBOOT_HEADER_MAGIC_INV;

      if (flash_partition_read(from, buf, blocksize, 0) < 0)
        {
          free(buf);
          return ERROR;
        }
        
      syslog(LOG_INFO, "Old magic %lx, new %lx\n", header.magic, magic);
      memcpy(buf + offsetof(struct nxboot_img_header, magic), &magic,
             sizeof magic);
      if (flash_partition_write(where, buf, blocksize, 0) < 0)
        {
          free(buf);
          return ERROR;
        }

      off += blocksize;
      remain -= blocksize;
    }

  while (remain > 0)
    {
      readsiz = remain > blocksize ? blocksize : remain;
      if (flash_partition_read(from, buf, readsiz, off) < 0)
        {
          free(buf);
          return ERROR;
        }

      if (flash_partition_write(where, buf, readsiz, off) < 0)
        {
          free(buf);
          return ERROR;
        }

      off += readsiz;
      remain -= readsiz;
    }

  free(buf);
  return OK;
}

static bool validate_image(int fd)
{
  struct nxboot_img_header header;

  get_image_header(fd, &header);
  if (!validate_image_header(&header))
    {
      return false;
    }

  return calculate_crc(fd, &header) == header.crc;
}

static bool compare_versions(struct nxboot_img_version *v1,
                             struct nxboot_img_version *v2)
{
#ifndef CONFIG_NXBOOT_PREVENT_DOWNGRADE
  int i;
  if (v1->major != v2->major ||
      v1->minor != v2->minor ||
      v1->patch != v2->patch)
    {
      return false;
    }

  for (i = 0; i < NXBOOT_HEADER_PRERELEASE_MAXLEN; i++)
    {
      if (v1->pre_release[i] != v2->pre_release[i])
        {
          return false;
        }
    }

  return true;
#else
  if (v1->major > v2->major ||
      v1->minor > v2->minor ||
      v1->patch > v2->patch)
    {
      return true;
    }

  if (v1->major == v2->major &&
      v1->minor == v2->minor &&
      v1->patch == v2->patch)
    {
      /* TODO: compare prerelease */
    }

  return false;
#endif
}

static enum nxboot_update_type
  get_update_type(struct nxboot_state *state,
                  int primary, int update, int recovery,
                  struct nxboot_img_header *primary_header,
                  struct nxboot_img_header *update_header,
                  struct nxboot_img_header *recovery_header)
{
  bool ready_to_upd;
  
  ready_to_upd = false;
  if (update_header->magic == NXBOOT_HEADER_MAGIC && validate_image(update))
    {
      ready_to_upd = true;
    }

  syslog(LOG_INFO, "recovery magic %lx, primary magic %lx, primary confirmed %d, recovery valid %d\n",
    recovery_header->magic, primary_header->magic, state->primary_confirmed, state->recovery_valid);

  if ((recovery_header->magic == NXBOOT_HEADER_MAGIC_INV) &&
       !ready_to_upd &&
      ((primary_header->magic == NXBOOT_HEADER_MAGIC_INV &&
      !state->primary_confirmed) || !validate_image(primary))
      && validate_image(recovery))
    {
      return NXBOOT_UPDATE_TYPE_REVERT;
    }

  if (ready_to_upd)
    {
      if (compare_versions(&primary_header->img_version,
          &update_header->img_version) &&
          validate_image(primary))
        {
          return NXBOOT_UPDATE_TYPE_NONE;
        }

      return NXBOOT_UPDATE_TYPE_UPDATE;
    }

  return NXBOOT_UPDATE_TYPE_NONE;
}

static int perform_update(struct nxboot_state *state, bool check_only)
{
  int update;
  int recovery;
  int primary;
  int secondary;
  int tertiary;
  bool primary_valid;

  primary = flash_partition_open(CONFIG_NXBOOT_PRIMARY_SLOT_PATH);
  assert(primary >= 0);

  secondary = flash_partition_open(CONFIG_NXBOOT_SECONDARY_SLOT_PATH);
  assert(secondary >= 0);

  tertiary = flash_partition_open(CONFIG_NXBOOT_TERTIARY_SLOT_PATH);
  assert(tertiary >= 0);

  if (state->update == NXBOOT_SECONDARY_SLOT_NUM)
    {
      update = secondary;
      recovery = tertiary;
    }
  else
    {
      update = tertiary;
      recovery = secondary;
    }

  if (state->next_boot == NXBOOT_UPDATE_TYPE_REVERT &&
      (!check_only || !validate_image(primary)))
    {
      if (validate_image(recovery))
        {
          syslog(LOG_INFO, "Reverting image to recovery.\n");
          copy_partition(recovery, primary, DO_REVERT);
        }
    }
  else
    {
      primary_valid = validate_image(primary);
      if (primary_valid && check_only)
        {
          /* Skip if primary image is valid (does not mather whether
           * confirmed or not) and check_only option is set.
           */

          goto perform_update_done;
        }

      if (!state->recovery_valid && state->primary_confirmed &&
          primary_valid)
        {
          /* Save current image as recovery only if it is valid and
           * confirmed. We have to check this in case of restart
           * during update process.
           * If board is restarted during update, primary slot contains
           * non-valid image and we do not want to copy this image to
           * recovery slot.
           * There also might be a case where primary image is valid
           * but not confirmed (timing of board reset right after
           * update is uploaded to secondary). Still we do not want
           * to upload this to recovery.
           */

          syslog(LOG_INFO, "Creating recovery image.\n");
          copy_partition(primary, recovery, CREATE_RECOVERY);
          if (!validate_image(recovery))
            {
              syslog(LOG_INFO, "New recovery is not valid, stop update\n");
              goto perform_update_done;
            }

          syslog(LOG_INFO, "Recovery image created.\n");
        }

      if (validate_image(update))
        {
          /* Perform update only if update slot contains valid image. */

          syslog(LOG_INFO, "Updating from update image.\n");
          if (copy_partition(update, primary, DO_UPDATE) >= 0)
            {
              syslog(LOG_INFO, "Update done, set flag and erase header\n");
              set_image_last_wp(primary, state->recovery);

              /* Mark update slot as updated. This is to prevent repeated
               * updates.
               */
        
              flash_partition_erase_first_sector(update);
            }
        }
    }

perform_update_done:
  flash_partition_close(primary);
  flash_partition_close(secondary);
  flash_partition_close(tertiary);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxboot_get_state
 *
 * Description:
 *   Gets the current bootloader state and stores it in the nxboot_state
 *   structure passed as an argument. This function may be used to determine
 *   which slot is update slot and where should application save incoming
 *   firmware.
 *
 * Input parameters:
 *   state: The pointer to nxboot_state structure. The state is stored here.
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int nxboot_get_state(struct nxboot_state *state)
{
  int primary;
  int secondary;
  int tertiary;
  int update;
  int recovery;
  int recovery_pointer;
  struct nxboot_img_header primary_header;
  struct nxboot_img_header secondary_header;
  struct nxboot_img_header tertiary_header;
  struct nxboot_img_header *update_header;
  struct nxboot_img_header *recovery_header;

  memset(state, 0, sizeof *state);

  primary = flash_partition_open(CONFIG_NXBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  secondary = flash_partition_open(CONFIG_NXBOOT_SECONDARY_SLOT_PATH);
  if (secondary < 0)
    {
      flash_partition_close(primary);
      return ERROR;
    }

  tertiary = flash_partition_open(CONFIG_NXBOOT_TERTIARY_SLOT_PATH);
  if (tertiary < 0)
    {
      flash_partition_close(primary);
      flash_partition_close(secondary);
      return ERROR;
    }

  get_image_header(primary, &primary_header);
  get_image_header(secondary, &secondary_header);
  get_image_header(tertiary, &tertiary_header);

  update = secondary;
  recovery = tertiary;
  update_header = &secondary_header;
  recovery_header = &tertiary_header;
  state->update = NXBOOT_SECONDARY_SLOT_NUM;
  state->recovery = NXBOOT_TERTIARY_SLOT_NUM;

  if (tertiary_header.magic == NXBOOT_HEADER_MAGIC)
    {
      update = tertiary;
      recovery = secondary;
      update_header = &tertiary_header;
      recovery_header = &secondary_header;
      state->recovery = NXBOOT_SECONDARY_SLOT_NUM;
      state->update = NXBOOT_TERTIARY_SLOT_NUM;      
    }
  else if (secondary_header.magic == NXBOOT_HEADER_MAGIC_INV &&
           tertiary_header.magic == NXBOOT_HEADER_MAGIC_INV)
    {
      recovery_pointer = get_image_last_wp(primary);
      if (recovery_pointer != 0xff)
        {
          recovery_pointer &= 0x3;
          if (recovery_pointer == NXBOOT_SECONDARY_SLOT_NUM)
            {
              update = tertiary;
              recovery = secondary;
              update_header = &tertiary_header;
              recovery_header = &secondary_header;
              state->recovery = NXBOOT_SECONDARY_SLOT_NUM;
              state->update = NXBOOT_TERTIARY_SLOT_NUM;     
            }
        }
      else if (primary_header.crc == secondary_header.crc)
        {
          update = tertiary;
          recovery = secondary;
          update_header = &tertiary_header;
          recovery_header = &secondary_header;
          state->recovery = NXBOOT_SECONDARY_SLOT_NUM;
          state->update = NXBOOT_TERTIARY_SLOT_NUM;     
        }
    }
  else if (secondary_header.magic == NXBOOT_HEADER_MAGIC_INV)
    {
      update = tertiary;
      recovery = secondary;
      update_header = &tertiary_header;
      recovery_header = &secondary_header;
      state->recovery = NXBOOT_SECONDARY_SLOT_NUM;
      state->update = NXBOOT_TERTIARY_SLOT_NUM;      
    }

  if (primary_header.crc == recovery_header->crc && validate_image(recovery))
    {
      state->recovery_valid = true;
    }

  if (primary_header.magic == NXBOOT_HEADER_MAGIC ||
      (primary_header.magic == NXBOOT_HEADER_MAGIC_INV &&
      primary_header.crc == recovery_header->crc))
    {
      state->primary_confirmed = true;
    }

  state->next_boot = get_update_type(state, primary, update, recovery,
                                     &primary_header, update_header,
                                     recovery_header);


  syslog(LOG_INFO, "Primary header %lx\n", primary_header.magic);
  syslog(LOG_INFO, "Update = %d, Recovery = %d, confirmed %d, valid %d, do %d\n", state->update, state->recovery,
         state->primary_confirmed, state->recovery_valid, state->next_boot);
  flash_partition_close(primary);
  flash_partition_close(secondary);
  flash_partition_close(tertiary);
  return OK;
}

/****************************************************************************
 * Name: nxboot_open_update_partition
 *
 * Description:
 *   Gets the current bootloader state and opens the partition to which an
 *   update image should be stored. It returns the valid file descriptor to
 *   this partition, the user is responsible for writing to it and closing
 *   if afterwards.
 *
 * Returned Value:
 *   Valid file descriptor on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int nxboot_open_update_partition(void)
{
  char *path;
  struct nxboot_state state;

  nxboot_get_state(&state);

  path = state.update == NXBOOT_SECONDARY_SLOT_NUM ?
    CONFIG_NXBOOT_SECONDARY_SLOT_PATH : CONFIG_NXBOOT_TERTIARY_SLOT_PATH;

  return flash_partition_open(path);
}

/****************************************************************************
 * Name: nxboot_get_confirm
 *
 * Description:
 *   This function can be used to determine whether primary image is
 *   confirmed or not. This provides more direct access to confirm
 *   state compared to nxboot_get_state function that returns the full
 *   state of the bootloader.
 *
 * Returned Value:
 *   1 means confirmed, 0 not confirmed, -1 and sets errno on failure.
 *
 ****************************************************************************/

int nxboot_get_confirm(void)
{
  int primary;
  int recovery;
  int recovery_pointer;
  char *path;
  int ret = 0;
  struct nxboot_img_header primary_header;
  struct nxboot_img_header recovery_header;

  primary = flash_partition_open(CONFIG_NXBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  get_image_header(primary, &primary_header);
  
  if (primary_header.magic == NXBOOT_HEADER_MAGIC)
    {
      close(primary);
      return 1;
    }
  else if (primary_header.magic == NXBOOT_HEADER_MAGIC_INV)
    {
      recovery_pointer = get_image_last_wp(primary);
      if (recovery_pointer != 0xff)
        {
          recovery_pointer &= 0x3;
          path = recovery_pointer == NXBOOT_SECONDARY_SLOT_NUM ?
            CONFIG_NXBOOT_SECONDARY_SLOT_PATH :
            CONFIG_NXBOOT_TERTIARY_SLOT_PATH;

          recovery = flash_partition_open(path);
          if (recovery < 0)
            {
              close(primary);
              return ERROR;
            }
          
          get_image_header(recovery, &recovery_header);
          if (primary_header.crc == recovery_header.crc)
            {
              ret = 1;
            }

          close(recovery);
        }
    }

  close(primary);
  return ret;
}

/****************************************************************************
 * Name: nxboot_confirm
 *
 * Description:
 *   Confirms the image currently located in primary partition and marks
 *   its copy in update partition as a recovery.
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int nxboot_confirm(void)
{
  int ret;
  int update;
  int primary;
  int remain;
  int readsiz;
  char *path;
  char *buf;
  off_t off;
  struct nxboot_state state;
  struct flash_partition_info info_update;

  ret = OK;
  nxboot_get_state(&state);
  if (state.primary_confirmed)
    {
      return OK;
    }

  path = state.update == NXBOOT_SECONDARY_SLOT_NUM ?
    CONFIG_NXBOOT_SECONDARY_SLOT_PATH : CONFIG_NXBOOT_TERTIARY_SLOT_PATH;

  primary = flash_partition_open(CONFIG_NXBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  update = flash_partition_open(path);
  if (update < 0)
    {
      flash_partition_close(primary);
      return ERROR;
    }

  /* We need to mark both primary and update partitions as confirmed
   * (update partition will become recovery once confirmed) and
   * we have to remove confirmed flag from old recovery and set updated
   * flag there. This is to prevent old recovery still identify as
   * recovery and not to look as possible update. Therefore remove the
   * entire last sector (clears confirmed flag) and write updated
   * flag.
   */

  if (flash_partition_info(update, &info_update) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

  buf = malloc(info_update.blocksize);
  remain = info_update.erasesize;
  off = 0;

  while (remain > 0)
    {
      readsiz = remain > info_update.blocksize ?
        info_update.blocksize : remain;
      if (flash_partition_read(primary, buf, readsiz, off) < 0)
        {
          free(buf);
          ret = ERROR;
          goto confirm_done;
        }

      if (flash_partition_write(update, buf, readsiz, off) < 0)
        {
          free(buf);
          ret = ERROR;
          goto confirm_done;
        }

      off += readsiz;
      remain -= readsiz;
    }

  free(buf);

confirm_done:
  flash_partition_close(primary);
  flash_partition_close(update);

  return ret;
}

/****************************************************************************
 * Name: nxboot_perform_update
 *
 * Description:
 *   Checks for the possible firmware update and performs it by copying
 *   update image to primary slot or recovery image to primary slot in case
 *   of the revert. In any situation, this function ends with the valid
 *   image in primary slot.
 *
 *   This is an entry point function that should be called from the
 *   bootloader application.
 *
 * Input parameters:
 *   check_only: Only repairs corrupted update, but do not start another one
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

int nxboot_perform_update(bool check_only)
{
  int ret;
  int primary;
  struct nxboot_state state;
  struct nxboot_img_header header;

  ret = nxboot_get_state(&state);
  if (ret < 0)
    {
      return ERROR;
    }

  if (state.next_boot != NXBOOT_UPDATE_TYPE_NONE)
    {
      /* We either want to update or revert. */

      ret = perform_update(&state, check_only);
      if (ret < 0)
        {
          syslog(LOG_ERR, "Update process failed. %s\n",
             strerror(errno));
          return ERROR;
        }
    }

  /* Check whether there is a valid image in the primary slot. This just
   * checks whether the header is valid, but does not calculate the CRC
   * of the image as this would prolong the boot process.
   */

  primary = flash_partition_open(CONFIG_NXBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  get_image_header(primary, &header);
  if (!validate_image_header(&header))
    {
      ret = ERROR;
    }

  flash_partition_close(primary);

  return ret;
}
