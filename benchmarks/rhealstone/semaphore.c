/****************************************************************************
 * apps/benchmarks/rhealstone/semaphore.c
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

#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stddef.h>
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

struct semaphore_shuffle_s
{
  sem_t semaphore;
  int ntests;
  atomic_uint count;
  bool measure_semaphores;

  size_t context_switch_overhead;
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

static FAR void *sem_task2(FAR void *arg)
{
  struct semaphore_shuffle_s *priv = (struct semaphore_shuffle_s *)arg;
  struct performance_time_s result;
  ssize_t elapsed;

  performance_start(&result);
  for (priv->count = 0; priv->count < priv->ntests; priv->count++)
    {
      if (priv->measure_semaphores)
        {
          sem_wait(&priv->semaphore);
        }

      sched_yield();
      if (priv->measure_semaphores)
        {
          sem_post(&priv->semaphore);
        }

      sched_yield();
    }

  performance_end(&result);
  elapsed = performance_gettime(&result);
  if (!priv->measure_semaphores)
    {
      priv->context_switch_overhead = elapsed;
    }
  else
    {
      priv->measured_time = (elapsed - priv->context_switch_overhead) /
                            (priv->ntests * 2);
    }

  return NULL;
}

static FAR void *sem_task1(FAR void *arg)
{
  struct semaphore_shuffle_s *priv = (struct semaphore_shuffle_s *)arg;
  pthread_t task;

  task = rhealstone_thread_create(sem_task2, arg,
                                   CONFIG_BENCHMARK_RHEALSTONE_PRIORITY + 1);

  sched_yield();

  while (priv->count < priv->ntests)
    {
      if (priv->measure_semaphores)
        {
          sem_wait(&priv->semaphore);
        }

      sched_yield();
      if (priv->measure_semaphores)
        {
          sem_post(&priv->semaphore);
        }

      sched_yield();
    }

  pthread_join(task, NULL);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t semaphore_shuffle(size_t count)
{
  pthread_t task;

  struct semaphore_shuffle_s priv =
    {
      .measure_semaphores = false,
      .ntests = count,
    };

  sem_init(&priv.semaphore, 0, 1);

  task = rhealstone_thread_create(sem_task1, &priv,
                                   CONFIG_BENCHMARK_RHEALSTONE_PRIORITY + 1);
  pthread_join(task, NULL);

  priv.measure_semaphores = true;
  priv.count = 0;
  task = rhealstone_thread_create(sem_task1, &priv,
                                   CONFIG_BENCHMARK_RHEALSTONE_PRIORITY + 1);
  pthread_join(task, NULL);
  sem_destroy(&priv.semaphore);
  return priv.measured_time;
}
