/****************************************************************************
 * apps/boot/nxboot/loader/boot.c
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

#include <nxboot.h>

#include "crc32.h"
#include "flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NXBOOT_PREVENT_DOWNGRADE
  # warning "Downgrade prevention currently ignores prerelease."
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int set_image_header_state(int fd, uint8_t state)
{
  return flash_partition_write(fd, &state, sizeof state,
                               offsetof(struct nxboot_img_header, state));
}

static void get_image_header(int fd, struct nxboot_img_header *header)
{
  int ret;
  ret = flash_partition_read(fd, header, sizeof *header, 0);
  if (ret < 0)
    {
      /* Something went wrong, treat the partition as empty. */

      memset(header, 0, sizeof *header);
    }
}

static int copy_partition(int from, int where)
{
  struct nxboot_img_header header;
  int blocksiz;
  int readsiz;
  int destsiz;
  int remain;
  int ret;
  off_t off;
  char *buf;

  get_image_header(from, &header);

  destsiz = flash_partition_size(where);
  if (destsiz < 0)
    {
      return ERROR;
    }

  if ((header.size + CONFIG_NXBOOT_HEADER_SIZE) > destsiz)
    {
      return ERROR;
    }

  blocksiz = flash_partition_blocksize(from);
  if (blocksiz < 0)
    {
      return ERROR;
    }

  buf = malloc(blocksiz);
  if (!buf)
    {
      return ERROR;
    }

  remain = header.size + CONFIG_NXBOOT_HEADER_SIZE;
  off = 0;
  ret = OK;
  while (remain > 0 && ret >= 0)
    {
      readsiz = remain > blocksiz ? blocksiz : remain;
      ret = flash_partition_read(from, buf, readsiz, off);
      ret = flash_partition_write(where, buf, readsiz, off);
      off += readsiz;
      remain -= readsiz;
    }

  return ret;
}

static bool validate_image_header(struct nxboot_img_header *header)
{
  return header->magic == NXBOOT_HEADER_MAGIC;
}

static bool validate_image(int fd)
{
  int ret;
  char *buf;
  int remain;
  int readsiz;
  int blocksiz;
  uint32_t crc;
  off_t off;
  struct nxboot_img_header header;

  get_image_header(fd, &header);
  if (!validate_image_header(&header))
    {
      return false;
    }

  blocksiz = flash_partition_blocksize(fd);
  if (blocksiz < 0)
    {
      return false;
    }

  buf = malloc(blocksiz);
  if (!buf)
    {
      return false;
    }

  crc = crc32_init();
  remain = header.size;
  off = CONFIG_NXBOOT_HEADER_SIZE;
  ret = OK;
  while (remain > 0 && ret >= 0)
    {
      readsiz = remain > blocksiz ? blocksiz : remain;
      ret = flash_partition_read(fd, buf, readsiz, off);
      off += readsiz;
      remain -= readsiz;
      crc = crc32_update(crc, (uint8_t *)buf, readsiz);
    }

  crc = crc32_finalize(crc);

  free(buf);
  return crc == header.crc32;
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
  get_update_type(int primary, int update, int recovery,
                  struct nxboot_img_header *primary_header,
                  struct nxboot_img_header *update_header,
                  struct nxboot_img_header *recovery_header)
{
  if (recovery_header->state == NXBOOT_IMAGE_STATE_CONFIRM &&
      update_header->state == NXBOOT_IMAGE_STATE_UPDATED &&
      (primary_header->state != NXBOOT_IMAGE_STATE_CONFIRM ||
      !validate_image(primary)) && validate_image(recovery))
    {
      return NXBOOT_UPDATE_TYPE_REVERT;
    }

  if (update_header->state == NXBOOT_IMAGE_STATE_UNSET &&
      validate_image(update))
    {
      if (compare_versions(&primary_header->img_version,
                            &update_header->img_version))
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
          flash_partition_erase(primary);
          copy_partition(recovery, primary);
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
          copy_partition(primary, recovery);
          if (!validate_image(recovery))
            {
              syslog(LOG_INFO, "New recovery is not valid, stop update\n");
              goto perform_update_done;
            }
        }

      if (validate_image(update))
        {
          /* Perform update only if update slot contains valid image. */

          syslog(LOG_INFO, "Updating from update image.\n");
          flash_partition_erase(primary);
          copy_partition(update, primary);

          /* Mark update slot as updated. This is to prevent repeated
           * updates.
           */

          set_image_header_state(update, NXBOOT_IMAGE_STATE_UPDATED);
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

int nxboot_get_update_partition(void)
{
  int ret;
  int secondary;
  int tertiary;
  struct nxboot_img_header secondary_header;
  struct nxboot_img_header tertiary_header;

  secondary = flash_partition_open(CONFIG_NXBOOT_SECONDARY_SLOT_PATH);
  if (secondary < 0)
    {
      return ERROR;
    }

  tertiary = flash_partition_open(CONFIG_NXBOOT_TERTIARY_SLOT_PATH);
  if (tertiary < 0)
    {
      flash_partition_close(secondary);
      return ERROR;
    }

  get_image_header(secondary, &secondary_header);
  get_image_header(tertiary, &tertiary_header);

  if (secondary_header.magic != NXBOOT_HEADER_MAGIC)
    {
      /* Secondary is either empty or corrupted. In any case, we want
       * this to mark as a partition for update.
       */

      ret = NXBOOT_SECONDARY_SLOT_NUM;
    }
  else if (tertiary_header.magic != NXBOOT_HEADER_MAGIC)
    {
      /* Tertiary is either empty or corrupted. In any case, we want
       * this to mark as a partition for update.
       */

      ret = NXBOOT_TERTIARY_SLOT_NUM;
    }
  else if (tertiary_header.state == NXBOOT_IMAGE_STATE_AVAIL ||
           tertiary_header.state == NXBOOT_IMAGE_STATE_UPDATED)
    {
      /* Tertiary image is marked as available, use it for update */

      ret = NXBOOT_TERTIARY_SLOT_NUM;
    }
  else
    {
      /* Any other case. */

      ret = NXBOOT_SECONDARY_SLOT_NUM;
    }

  flash_partition_close(secondary);
  flash_partition_close(tertiary);
  return ret;
}

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
  if (secondary_header.state == NXBOOT_IMAGE_STATE_CONFIRM &&
      validate_image(secondary))
    {
      update = tertiary;
      recovery = secondary;
      state->recovery = NXBOOT_SECONDARY_SLOT_NUM;
      state->update = NXBOOT_TERTIARY_SLOT_NUM;
      recovery_header = &secondary_header;
      update_header = &tertiary_header;
      if (secondary_header.crc32 == primary_header.crc32)
        {
          state->recovery_valid = true;
        }
    }
  else if (tertiary_header.state == NXBOOT_IMAGE_STATE_CONFIRM &&
           validate_image(tertiary))
    {
      if (tertiary_header.crc32 == primary_header.crc32)
        {
          state->recovery_valid = true;
        }
    }

  if (primary_header.state == NXBOOT_IMAGE_STATE_CONFIRM)
    {
      state->primary_confirmed = true;
    }

  state->next_boot = get_update_type(primary, update, recovery,
                                     &primary_header, update_header,
                                     recovery_header);

  flash_partition_close(primary);
  flash_partition_close(secondary);
  flash_partition_close(tertiary);
  return OK;
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
  struct nxboot_img_header primary_header;

  primary = flash_partition_open(CONFIG_NXBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  get_image_header(primary, &primary_header);
  flash_partition_close(primary);
  if (validate_image_header(&primary_header) &&
      primary_header.state == NXBOOT_IMAGE_STATE_CONFIRM)
    {
      return 1;
    }

  return 0;
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
  int recovery;
  int primary;
  int secondary;
  int tertiary;
  struct nxboot_state state;

  nxboot_get_state(&state);
  if (state.primary_confirmed)
    {
      return OK;
    }

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

  if (state.update == NXBOOT_SECONDARY_SLOT_NUM)
    {
      syslog(LOG_INFO, "update is secondary\n");
      update = secondary;
      recovery = tertiary;
    }
  else
    {
      syslog(LOG_INFO, "update is tertiary\n");
      update = tertiary;
      recovery = secondary;
    }

  ret = OK;
  if (set_image_header_state(primary, NXBOOT_IMAGE_STATE_CONFIRM) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

  if (set_image_header_state(update, NXBOOT_IMAGE_STATE_CONFIRM) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

  if (set_image_header_state(recovery, NXBOOT_IMAGE_STATE_AVAIL) < 0)
    {
      ret = ERROR;
      goto confirm_done;
    }

confirm_done:
  flash_partition_close(primary);
  flash_partition_close(secondary);
  flash_partition_close(tertiary);

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

  ret = nxboot_get_state(&state);
  if (ret < 0)
    {
      return ERROR;
    }

  if (state.next_boot != NXBOOT_UPDATE_TYPE_NONE)
    {
      ret = perform_update(&state, check_only);
      if (ret < 0)
        {
          syslog(LOG_ERR, "Update process failed. %s\n",
             strerror(errno));
        }
    }

  primary = flash_partition_open(CONFIG_NXBOOT_PRIMARY_SLOT_PATH);
  if (primary < 0)
    {
      return ERROR;
    }

  ret = OK;
  if (!validate_image(primary))
    {
      ret = ERROR;
    }

  flash_partition_close(primary);
  return ret;
}
