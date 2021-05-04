/****************************************************************************
 * examples/lpe/lpe_main.c
 *
 *   Copyright (C) 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <pthread.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/rgbcolors.h>

#include "lpe.h"
#include "KISS_FFT/kiss_fft.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define NCOLORS 6

int tmp = 0;

static const char g_default_fbdev[] = CONFIG_EXAMPLES_FB_DEFAULTFB;

struct fb_state_s
{
  int fd;
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
#ifdef CONFIG_FB_OVERLAY
  struct fb_overlayinfo_s oinfo;
#endif
  FAR void *fbmem;
};

struct fb_state_s my_state;

typedef struct data {
    int stop_flag; //indicates if the program is running or not;
    int to_sent; //last received letter
    pthread_cond_t output_condvar;
    pthread_mutex_t mtx;
} data_t;

  int nsteps;
  int xstep;
  int ystep;
  int width;
  int height;
  size_t readsize;
  ssize_t nbytes;
  int fd;
  int ret;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/

static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (adc->devpath)
    {
      free(adc->devpath);
    }

  /* Then set-up the new device path by copying the string */

  adc->devpath = strdup(devpath);
}

static void draw_rect16(FAR struct fb_state_s *state2,
                        FAR struct fb_area_s *area, uint32_t color)
{
  FAR uint16_t *dest;
  FAR uint8_t *row;

  row = (FAR uint8_t *)state2->fbmem + state2->pinfo.stride * area->y;
  for (int y = 0; y < area->h; y++)
    {
      dest = ((FAR uint16_t *)row) + area->x;
      for (int x = 0; x < area->w; x++)
        {
          *dest++ = color;
        }

      row += state2->pinfo.stride;
    }
  #ifdef CONFIG_FB_UPDATE
  ret = ioctl(state2->fd, FBIO_UPDATE,
              (unsigned long)((uintptr_t)area));
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: ioctl(FBIO_UPDATE) failed: %d\n",
              errcode);
    }
  #endif
}

void *adc_thread(void* arg)
{
  data_t *data = (data_t*)arg;
  int errval;
  struct fb_area_s area;
  struct adc_msg_s sample[CONFIG_EXAMPLES_LPE_GROUPSIZE];
  kiss_fft_cpx cin[210];
  kiss_fft_cpx cout[210];
  kiss_fft_cfg cfg = kiss_fft_alloc( 210 ,0,0,0);
  while (data->stop_flag == 0)
  {
    //fflush(stdout);

#ifdef CONFIG_EXAMPLES_LPE_SWTRIG
    /* Issue the software trigger to start ADC conversion */

    ret = ioctl(fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
      {
        int errcode = errno;
        printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
      }
#endif

    /* Read up to CONFIG_EXAMPLES_LPE_GROUPSIZE samples */

    readsize = CONFIG_EXAMPLES_LPE_GROUPSIZE * sizeof(struct adc_msg_s);
    nbytes = read(fd, sample, readsize);

    /* Handle unexpected return values */
    
    if (nbytes < 0)
      {
        errval = errno;
        if (errval != EINTR)
          {
            printf("adc_main: read failed: %d\n", errval);
            errval = 3;
          }

        printf("adc_main: Interrupted read...\n");
      }
    else if (nbytes == 0)
      {
        printf("adc_main: No data read, Ignoring\n");
      }

    /* Print the sample data on successful return */

    else
      {
        int nsamples = nbytes / sizeof(struct adc_msg_s);
        if (nsamples * sizeof(struct adc_msg_s) != nbytes)
          {
            printf("adc_main: read size=%ld is not a multiple of "
                   "sample size=%d, Ignoring\n",
                   (long)nbytes, sizeof(struct adc_msg_s));
          }
        else
          {
           for (int i = 0; i < nsamples; i++)
            {
              cin[tmp].r = tmp;
              cin[tmp].i = sample[i].am_data;
              tmp +=1;
            }
            if (tmp == 210)
              {
                //data->to_sent = 1;
                area.x = 0;
                area.y = 0;
                area.w = 176;
                area.h = 220;
                draw_rect16(&my_state, &area, RGB16_BLACK);
                kiss_fft(cfg,cin,cout);
                for (int j = 0;j < tmp;j++)
                  {
                    area.x = cin[j].i/(23.27);
                    area.y = j;
                    //printf("(%d , %d )\n", area.x, area.y);
                    area.w = 10;
                    area.h = 5;
                    //printf("cout r %f, cout i %f\n", cout[j].r, cout[j].i);
                    draw_rect16(&my_state, &area, RGB16_GREEN);
                  }
                tmp = 0;
                //free(cfg);
              }
          }
        } 
  }
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{

  /* Check if we have initialized */
  static struct adc_state_s g_adcstate;
  FAR const char *fbdev = g_default_fbdev;
  if (!g_adcstate.initialized)
    {
      /* Initialization of the ADC hardware must be performed by
       * board-specific logic prior to running this test.
       */

      /* Set the default values */

      adc_devpath(&g_adcstate, CONFIG_EXAMPLES_ADC_DEVPATH);

      g_adcstate.initialized = true;
    }

  g_adcstate.count = CONFIG_EXAMPLES_LPE_NSAMPLES;

  /* Open the framebuffer driver */

  my_state.fd = open(fbdev, O_RDWR);
  if (my_state.fd < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", fbdev, errcode);
      return EXIT_FAILURE;
    }

  /* Get the characteristics of the framebuffer */

  ret = ioctl(my_state.fd, FBIOGET_VIDEOINFO,
              (unsigned long)((uintptr_t)&my_state.vinfo));
  if (ret < 0)
    {
      int errcode = errno;
      printf("ERROR: ioctl(FBIOGET_VIDEOINFO) failed: %d\n",
              errcode);
      close(my_state.fd);
      return EXIT_FAILURE;
    }

  printf("VideoInfo:\n");
  printf("      fmt: %u\n", my_state.vinfo.fmt);
  printf("     xres: %u\n", my_state.vinfo.xres);
  printf("     yres: %u\n", my_state.vinfo.yres);
  printf("  nplanes: %u\n", my_state.vinfo.nplanes);

  ret = ioctl(my_state.fd, FBIOGET_PLANEINFO,
              (unsigned long)((uintptr_t)&my_state.pinfo));
  if (ret < 0)
    {
      int errcode = errno;
      printf("ERROR: ioctl(FBIOGET_PLANEINFO) failed: %d\n",
              errcode);
      close(my_state.fd);
      return EXIT_FAILURE;
    }

  printf("PlaneInfo (plane 0):\n");
  printf("    fbmem: %p\n", my_state.pinfo.fbmem);
  printf("    fblen: %lu\n", (unsigned long)my_state.pinfo.fblen);
  printf("   stride: %u\n", my_state.pinfo.stride);
  printf("  display: %u\n", my_state.pinfo.display);
  printf("      bpp: %u\n", my_state.pinfo.bpp);

  /* mmap() the framebuffer.
   *
   * NOTE: In the FLAT build the frame buffer address returned by the
   * FBIOGET_PLANEINFO IOCTL command will be the same as the framebuffer
   * address.  mmap(), however, is the preferred way to get the framebuffer
   * address because in the KERNEL build, it will perform the necessary
   * address mapping to make the memory accessible to the application.
   */

  my_state.fbmem = mmap(NULL, my_state.pinfo.fblen, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_FILE, my_state.fd, 0);
  if (my_state.fbmem == MAP_FAILED)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: ioctl(FBIOGET_PLANEINFO) failed: %d\n",
              errcode);
      close(my_state.fd);
      return EXIT_FAILURE;
    }

  printf("Mapped FB: %p\n", my_state.fbmem);

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

  printf("adc_main: g_adcstate.count: %d\n", g_adcstate.count);

  /* Open the ADC device for reading */

  printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
         g_adcstate.devpath);

  fd = open(g_adcstate.devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("adc_main: open %s failed: %d\n", g_adcstate.devpath, errno);
    }

  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */

  data_t shared_data;
  shared_data.stop_flag = 0;
  shared_data.to_sent = 0;
  if (pthread_mutex_init(&shared_data.mtx, NULL)) {
        fprintf(stderr, "ERROR: Could not initialize mutex.\n");
  }
  if (pthread_cond_init(&shared_data.output_condvar, NULL)) {
      fprintf(stderr, "ERROR: Could not initialize condvar.\n");
  }

  enum {
    ADC_THREAD,
    NUM_THREADS
  };
  const char* thread_names[] = {"ADC"};
  void* (*thd_functions[])(void*) = {
      adc_thread,
     };
  pthread_t threads[NUM_THREADS]; //array of threads

  for(int i = 0; i < NUM_THREADS; ++i)
    { //start the threads
      int check = pthread_create(&threads[i], NULL, thd_functions[i], &shared_data);
      fprintf(stderr, "INFO: %s thread start: %s\r\n",
          thread_names[i],
          check ? "FAIL" : "ok");
    }
  for (int i = 0; i < NUM_THREADS; ++i)
    { //join the threads
      int check = pthread_join(threads[i], NULL);
      fprintf(stderr, "INFO: %s thread joined: %s\r\n",
          thread_names[i],
          check ? "FAIL" : "ok");
    }

  close(fd);
  return OK;

}
