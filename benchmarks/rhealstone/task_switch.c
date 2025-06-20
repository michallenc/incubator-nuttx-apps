/****************************************************************************
 * apps/benchmarks/rhealstone/task_switch.c
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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/poll.h>

#include <nuttx/sched.h>

#include "rhealstone.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct task_switch_s
{
  int ntests;

  size_t loop_overhead;
  size_t measured_time;
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *task2(FAR void *arg)
{
  struct task_switch_s *priv = (struct task_switch_s *)arg;
  struct performance_time_s result;
  ssize_t elapsed;

  performance_start(&result);
  for (int i = 0; i < priv->ntests - 1; i++)
    {
      sched_yield();
    }

  performance_end(&result);
  elapsed = performance_gettime(&result);
  priv->measured_time = (elapsed - priv->loop_overhead) /
                        ((priv->ntests * 2) - 1);

  return NULL;
}

static FAR void *task1(FAR void *arg)
{
  struct task_switch_s *priv = (struct task_switch_s *)arg;
  pthread_t task;

  task = rhealstone_thread_create(task2, arg,
          CONFIG_BENCHMARK_RHEALSTONE_PRIORITY + 1);

  sched_yield();

  for (int i = 0; i < priv->ntests; i++)
    {
      sched_yield();
    }

  pthread_join(task, NULL);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t task_switching(size_t count)
{
  struct performance_time_s result;
  pthread_t task;

  struct task_switch_s priv =
    {
      .ntests = count,
    };

  performance_start(&result);
  for (int i = 0; i < priv.ntests - 1; i++);
  for (int i = 0; i < priv.ntests; i++);
  performance_end(&result);
  priv.loop_overhead = performance_gettime(&result);

  task = rhealstone_thread_create(task1, &priv,
                                  CONFIG_BENCHMARK_RHEALSTONE_PRIORITY + 1);
  pthread_join(task, NULL);
  return priv.measured_time;
}
