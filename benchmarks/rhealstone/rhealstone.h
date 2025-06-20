/****************************************************************************
 * apps/benchmarks/rhealstone/rhealstone.h
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct performance_time_s
{
  clock_t start;
  clock_t end;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rhealstone_thread_create
 *
 * Description:
 *   Creates the thread with SCHED_FIFO policy.
 *
 * Input Parameters:
 *   entry - routine to be started
 *   arg - optional argument passed to the routine
 *   priority - priority of the routine
 *
 * Returned Value:
 *   ID of the newly created thread.
 *
 ****************************************************************************/

int rhealstone_thread_create(FAR void *(*entry)(FAR void *), FAR void *arg,
                              int priority);

/****************************************************************************
 * Name: performance_start
 *
 * Description:
 *   Starts the performance measurement.
 *
 * Input Parameters:
 *   result - pointer to performance_time_s structure
 *
 ****************************************************************************/

void performance_start(FAR struct performance_time_s *result);

/****************************************************************************
 * Name: performance_end
 *
 * Description:
 *   Stops the performance measurement.
 *
 * Input Parameters:
 *   result - pointer to performance_time_s structure
 *
 ****************************************************************************/

void performance_end(FAR struct performance_time_s *result);

/****************************************************************************
 * Name: performance_gettime
 *
 * Description:
 *   Calculates the time between performance_start and performance_end
 *   calls.
 *
 * Input Parameters:
 *   result - pointer to performance_time_s structure
 *
 ****************************************************************************/

size_t performance_gettime(FAR struct performance_time_s *result);

/****************************************************************************
 * Name: semaphore_shuffle
 *
 * Description:
 *
 * Input Parameters:
 *   count - number of times the test is performed
 *
 ****************************************************************************/

size_t semaphore_shuffle(size_t count);

/****************************************************************************
 * Name: task_switching
 *
 * Description:
 *
 * Input Parameters:
 *   count - number of times the test is performed
 *
 ****************************************************************************/

size_t task_switching(size_t count);

/****************************************************************************
 * Name: task_preempt
 *
 * Description:
 *
 * Input Parameters:
 *   count - number of times the test is performed
 *
 ****************************************************************************/

size_t task_preempt(size_t count);
