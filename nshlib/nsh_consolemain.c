/****************************************************************************
 * apps/nshlib/nsh_consolemain.c
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

#include <assert.h>

#include "nsh.h"
#include "nsh_console.h"

#ifdef HAVE_USB_CONSOLE
#  include <stdlib.h>
#  include <unistd.h>
#  include <fcntl.h>
#  include <assert.h>
#  include <sys/boardctl.h>
#  include <nuttx/debug.h>
#  include "netutils/netinit.h"
#  ifdef CONFIG_CDCACM
#    include <nuttx/usb/cdcacm.h>
#  endif

#  ifdef CONFIG_PL2303
#    include <nuttx/usb/pl2303.h>
#  endif
#endif

#ifdef HAVE_USB_CONSOLE
struct console_thread_args
{
  FAR struct console_stdio_s *pstate;
  int argc;
  FAR char **argv;
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_configstdio
 *
 * Description:
 *   Configure standard I/O
 *
 ****************************************************************************/

#ifdef HAVE_USB_CONSOLE
static void nsh_configstdio(int fd)
{
  /* Dup the fd to create standard fd 0-2 */

  dup2(fd, 0);
  dup2(fd, 1);
  dup2(fd, 2);
}

/****************************************************************************
 * Name: nsh_nullstdio
 *
 * Description:
 *   Use /dev/null for standard I/O
 *
 ****************************************************************************/

static int nsh_nullstdio(void)
{
  int fd;

  /* Open /dev/null for read/write access */

  fd = open("/dev/console", O_RDWR);
  if (fd >= 0)
    {
      /* Configure standard I/O to use /dev/null */

      nsh_configstdio(fd);

      /* We can close the original file descriptor now (unless it was one of
       * 0-2)
       */

      if (fd > 2)
        {
          close(fd);
        }

      return OK;
    }

  return fd;
}

/****************************************************************************
 * Name: nsh_waitusbready
 *
 * Description:
 *   Wait for the USB console device to be ready
 *
 ****************************************************************************/

static int nsh_waitusbready(void)
{
  char inch;
  ssize_t nbytes;
  int nlc;
  int fd;

  /* Don't start the NSH console until the console device is ready.  Chances
   * are, we get here with no functional console.  The USB console will not
   * be available until the device is connected to the host and until the
   * host-side application opens the connection.
   */

restart:

  /* Open the USB serial device for read/write access */

  do
    {
      /* Try to open the console */

	  printf("condev = %s\n", CONFIG_NSH_USBCONDEV);
      fd = open(CONFIG_NSH_USBCONDEV, O_RDWR);
      if (fd < 0)
        {
          /* ENOTCONN means that the USB device is not yet connected.
           * Anything else is bad.
           */

          DEBUGASSERT(errno == ENOTCONN);

          /* Sleep a bit and try again */

          sleep(2);
        }
    }
  while (fd < 0);

  /* Configure standard I/O */

  nsh_configstdio(fd);

  /* Now wait until we successfully read a carriage return a few times.
   * That is a sure way of know that there is something at the other end of
   * the USB serial connection that is ready to talk with us.  The user needs
   * to hit ENTER a few times to get things started.
   */

 // nlc = 0;
 // do
 //   {
 //     /* Read one byte */

 //     inch = 0;
 //     nbytes = read(fd, &inch, 1);

 //     /* Is it a carriage return (or maybe a newline)? */

 //     if (nbytes == 1 && (inch == '\n' || inch == '\r'))
 //       {
 //         /* Yes.. increment the count */

 //         nlc++;
 //       }
 //     else
 //       {
 //         /* No.. Reset the count.  We need to see 3 in a row to continue. */

 //         nlc = 0;

 //         /* If a read error occurred (nbytes < 0) or an end-of-file was
 //          * encountered (nbytes == 0), then close the driver and start
 //          * over.
 //          */

 //         if (nbytes <= 0)
 //           {
 //             close(fd);
 //             goto restart;
 //           }
 //       }
 //   }
 // while (nlc < 3);

 // /* We can close the original file descriptor (unless it was one of 0-2) */

  if (fd > 2)
    {
      close(fd);
    }

  //close(fd);
  return OK;
}

static FAR void *nsh_usbconsole_thread(FAR void *arg)
{
  int ret;
  FAR struct console_thread_args *args =
    (FAR struct console_thread_args *)arg;

  /* Now loop, executing creating a session for each USB connection */

  for (; ; )
    {
      /* Wait for the USB to be connected to the host and switch
       * standard I/O to the USB serial device.
       */

      ret = nsh_waitusbready();
      UNUSED(ret); /* Eliminate warning if not used */
      DEBUGASSERT(ret == OK);

      /* Execute the session */

	  FAR struct console_stdio_s *pstate = nsh_newconsole(true);
      nsh_session(pstate, NSH_LOGIN_LOCAL, args->argc, args->argv);

      /* Switch to /dev/null because we probably no longer have a
       * valid console device.
       */

	  nsh_release(&pstate->cn_vtbl);
      //nsh_nullstdio();
    }

  return NULL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_consolemain (Normal character device version)
 *
 * Description:
 *   This interface may be to called or started with task_start to start a
 *   single an NSH instance that operates on stdin and stdout.  This
 *   function does not normally return (see below).
 *
 *   This version of nsh_consolemain handles generic /dev/console character
 *   devices (see nsh_usbconsole.c and usb_usbkeyboard for other versions
 *   for special USB console devices).
 *
 * Input Parameters:
 *   Standard task start-up arguments.  These are not used.  argc may be
 *   zero and argv may be NULL.
 *
 * Returned Values:
 *   This function does not normally return.  exit() is usually called to
 *   terminate the NSH session.  This function will return in the event of
 *   an error.  In that case, a non-zero value is returned (EXIT_FAILURE=1).
 *
 ****************************************************************************/

int nsh_consolemain(int argc, FAR char *argv[])
{
  int ret;
#ifdef HAVE_USB_CONSOLE
  struct boardioc_usbdev_ctrl_s ctrl;
  FAR void *handle;
  pthread_t usb_tid;

  /* Initialize the USB serial driver */

#if defined(CONFIG_PL2303) || defined(CONFIG_CDCACM)
#if defined(CONFIG_USBDEV_COMPOSITE)

  ctrl.usbdev   = BOARDIOC_USBDEV_COMPOSITE;
  ctrl.action   = BOARDIOC_USBDEV_INITIALIZE;
  ctrl.instance = 0;
  ctrl.config   = 0;
  ctrl.handle   = NULL;
  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);

  ctrl.usbdev   = BOARDIOC_USBDEV_COMPOSITE;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.config   = 0;
  ctrl.handle   = &handle;

#elif defined(CONFIG_CDCACM)

  printf("register cdc acm\n");
  ctrl.usbdev   = BOARDIOC_USBDEV_CDCACM;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = CONFIG_NSH_USBDEV_MINOR;
  ctrl.handle   = &handle;

#else

  ctrl.usbdev   = BOARDIOC_USBDEV_PL2303;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = CONFIG_NSH_USBDEV_MINOR;
  ctrl.handle   = &handle;

#endif

  printf("register %d\n", ctrl.instance);
  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  printf("done %d %d\n", ret, errno);
  UNUSED(ret); /* Eliminate warning if not used */
  DEBUGASSERT(ret == OK);
#endif

  /* Configure to use /dev/null if we do not have a valid console. */

#ifndef CONFIG_DEV_CONSOLE
  nsh_nullstdio();
#endif

  struct console_thread_args args =
    {
	  .argc = argc,
	  .argv = argv,
    };

  ret = pthread_create(&usb_tid, NULL, nsh_usbconsole_thread, &args);
  if (ret != OK)
    {
      // TODO
	}
#endif

#ifdef CONFIG_NSH_CONSOLE
  FAR struct console_stdio_s *pstate = nsh_newconsole(true);

  DEBUGASSERT(pstate != NULL);
  if (pstate == NULL)
    {
      return -ENOMEM;
    }

  /* Execute the session */

  ret = nsh_session(pstate, NSH_LOGIN_LOCAL, argc, argv);

  /* Exit upon return */

  nsh_exit(&pstate->cn_vtbl, ret);
  return ret;
#endif
}
