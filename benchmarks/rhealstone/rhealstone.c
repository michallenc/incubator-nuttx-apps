/****************************************************************************
 * apps/benchmarks/rhealstone/rhealstone.c
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

#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <sys/param.h>
#include <sys/poll.h>

#include <nuttx/sched.h>

#include "rhealstone.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rhealstone_entry
{
  const char name[NAME_MAX];
  CODE size_t (*entry)(size_t count);
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rhealstone_entry g_entry_list[] =
{
  {"Semaphore Shuffle", semaphore_shuffle},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rhealstone_help(void)
{
  printf("Usage: rhealstone [OPTIONS]\n\n");
  printf("OPTIONS:\n");
  printf("\t-c, \tNumber of times to run each test (default 20000)\n");
  printf("\t-h, \tShow this help message\n");
}

static void rhealstone_run(const FAR struct rhealstone_entry *item,
                            size_t count, bool detail)
{
  irqstate_t flags = enter_critical_section();
  size_t time = item->entry(count);
  leave_critical_section(flags);

  printf("%-*s %5zu\n", NAME_MAX, item->name, time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int performance_thread_create(FAR void *(*entry)(FAR void *), FAR void *arg,
                              int priority)
{
  struct sched_param param;
  pthread_attr_t attr;
  pthread_t tid;

  param.sched_priority = priority;
  pthread_attr_init(&attr);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  pthread_attr_setschedparam(&attr, &param);
  pthread_create(&tid, &attr, entry, arg);
  DEBUGASSERT(tid > 0);
  return tid;
}

void performance_start(FAR struct performance_time_s *result)
{
  result->start = perf_gettime();
}

void performance_end(FAR struct performance_time_s *result)
{
  result->end = perf_gettime();
}

size_t performance_gettime(FAR struct performance_time_s *result)
{
  struct timespec ts;
  perf_convert(result->end - result->start, &ts);
  return ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

int main(int argc, FAR char *argv[])
{
  const FAR struct rhealstone_entry *item = NULL;
  bool detail = false;
  size_t count = 20000;
  size_t i;
  int opt;

  while ((opt = getopt(argc, argv, "c:h")) != -1)
    {
      switch (opt)
        {
          case 'c':
            count = strtoul(optarg, NULL, 0);
            break;
          case 'h':
            rhealstone_help();
            return EXIT_FAILURE;
        }
    }

  for (i = 0; i < nitems(g_entry_list); i++)
    {
      item = &g_entry_list[i];
      rhealstone_run(item, count, detail);
    }

  return EXIT_SUCCESS;
}
