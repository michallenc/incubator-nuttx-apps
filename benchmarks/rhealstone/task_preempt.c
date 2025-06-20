/****************************************************************************
 * apps/benchmarks/rhealstone/task_preempt.c
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

struct task_preempt_s
{
  int ntests;
  pthread_cond_t cond;
  pthread_mutex_t mutex;
  struct performance_time_s result;

  size_t loop_overhead;
  size_t switch_overhead;
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
  struct task_preempt_s *priv = (struct task_preempt_s *)arg;
  struct performance_time_s result;
  ssize_t elapsed;

  performance_start(&priv->result);
  pthread_mutex_lock(&priv->mutex);
  pthread_cond_wait(&priv->cond, &priv->mutex);
  pthread_mutex_unlock(&priv->mutex);

  for (int i = 0; i < priv->ntests - 1; i++)
    {
      pthread_mutex_lock(&priv->mutex);
      pthread_cond_wait(&priv->cond, &priv->mutex);
      pthread_mutex_unlock(&priv->mutex);
    }

  performance_end(&result);
  elapsed = performance_gettime(&result);
  printf("elapes %ld, loop %ld, switch %ld\n", elapsed, priv->loop_overhead, priv->switch_overhead);
  priv->measured_time = ((elapsed - priv->loop_overhead)
                         / (priv->ntests - 1)) - priv->switch_overhead;

  return NULL;
}

static FAR void *task1(FAR void *arg)
{
  struct task_preempt_s *priv = (struct task_preempt_s *)arg;
  pthread_t task;

  task = rhealstone_thread_create(task2, arg,
          CONFIG_BENCHMARK_RHEALSTONE_PRIORITY + 2);

  performance_end(&priv->result);
  priv->switch_overhead = performance_gettime(&priv->result);

  performance_start(&priv->result);

  for (int i = 0; i < priv->ntests; i++)
    {
      pthread_mutex_lock(&priv->mutex);
      pthread_cond_signal(&priv->cond);
      pthread_mutex_unlock(&priv->mutex);
    }

  pthread_join(task, NULL);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t task_preempt(size_t count)
{
  struct performance_time_s result;
  pthread_t task;

  struct task_preempt_s priv =
    {
      .ntests = count,
    };

  pthread_cond_init(&priv.cond, NULL);
  pthread_mutex_init(&priv.mutex, NULL);

  performance_start(&result);
  for (int i = 0; i < (priv.ntests * 2) - 1; i++);
  performance_end(&result);
  priv.loop_overhead = performance_gettime(&result);
  
  task = rhealstone_thread_create(task1, &priv,
                                  CONFIG_BENCHMARK_RHEALSTONE_PRIORITY + 1);
  pthread_join(task, NULL);
  pthread_cond_destroy(&priv.cond);
  pthread_mutex_destroy(&priv.mutex);
  return priv.measured_time;
}
