/****************************************************************************
 * apps/testing/libc/stdbit/stdbit.c
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

#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbit.h>
#include <limits.h>
#include <sys/param.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHAR_SIZ (8)
#define NBITS(x) (sizeof(x) * CHAR_SIZ)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct test_entry_s
{
  const char name[NAME_MAX];
  CODE int (*entry)(void);
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int leading_zeros(void);
static int leading_ones(void);
static int trailing_zeros(void);
static int trailing_ones(void);
static int first_leading_zero(void);
static int first_leading_one(void);
static int first_trailing_zero(void);
static int first_trailing_one(void);

/****************************************************************************
 * Private data
 ****************************************************************************/

static const struct test_entry_s g_entry_list[] =
{
  {"leading_zeros", leading_zeros},
  {"leading_ones", leading_ones},
  {"trailing_zeros", trailing_zeros},
  {"trailing_ones", trailing_ones},
  {"first_leading_zero", first_leading_zero},
  {"first_leading_one", first_leading_one},
  {"first_trailing_zero", first_trailing_zero},
  {"first_trailing_one", first_trailing_one},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: leading_zeros
 ****************************************************************************/

static int leading_zeros(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = (1 << (nbits - 1));
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_zeros_uc(uc);
	  if (ret != i)
	    {
		  return -1;
		}

	  uc = uc >> 1;
	}

  nbits = NBITS(us);
  us = (1 << (nbits - 1));
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_zeros_us(us);
	  if (ret != i)
	    {
		  return -1;
		}

	  us = us >> 1;
	}

  nbits = NBITS(ui);
  ui = (1U << (nbits - 1));
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_zeros_ui(ui);
	  if (ret != i)
	    {
		  return -1;
		}

	  ui = ui >> 1;
	}

  nbits = NBITS(ul);
  ul = (1UL << (nbits - 1));
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_zeros_ul(ul);
	  if (ret != i)
	    {
		  return -1;
		}

	  ul = ul >> 1;
	}

  nbits = NBITS(ull);
  ull = (1ULL << (nbits - 1));
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_zeros_ul(ull);
	  if (ret != i)
	    {
		  return -1;
		}

	  ull = ull >> 1;
	}

  return 0;
}

/****************************************************************************
 * Name: leading_ones
 ****************************************************************************/

static int leading_ones(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_ones_uc(uc);
	  if (ret != i)
	    {
		  return -1;
		}

	  uc |= (1 << (nbits - (i + 1)));
	}

  nbits = NBITS(us);
  us = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_ones_us(us);
	  if (ret != i)
	    {
		  return -1;
		}

	  us |= (1 << (nbits - (i + 1)));
	}

  nbits = NBITS(ui);
  ui = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_ones_ui(ui);
	  if (ret != i)
	    {
		  return -1;
		}

	  ui |= (1U << (nbits - (i + 1)));
	}

  nbits = NBITS(ul);
  ul = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_ones_ul(ul);
	  if (ret != i)
	    {
		  return -1;
		}

	  ul |= (1UL << (nbits - (i + 1)));
	}

  nbits = NBITS(ull);
  ull = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_leading_ones_ul(ull);
	  if (ret != i)
	    {
		  return -1;
		}

	  ull |= (1ULL << (nbits - (i + 1)));
	}

  return 0;
}

/****************************************************************************
 * Name: trailing_zeros
 ****************************************************************************/

static int trailing_zeros(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = 1;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_zeros_uc(uc);
	  if (ret != i)
	    {
		  return -1;
		}

	  uc = uc << 1;
	}

  nbits = NBITS(us);
  us = 1;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_zeros_us(us);
	  if (ret != i)
	    {
		  return -1;
		}

	  us = us << 1;
	}

  nbits = NBITS(ui);
  ui = 1;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_zeros_ui(ui);
	  if (ret != i)
	    {
		  return -1;
		}

	  ui = ui << 1;
	}

  nbits = NBITS(ul);
  ul = 1;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_zeros_ul(ul);
	  if (ret != i)
	    {
		  return -1;
		}

	  ul = ul << 1;
	}

  nbits = NBITS(ull);
  ull = 1;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_zeros_ul(ull);
	  if (ret != i)
	    {
		  return -1;
		}

	  ull = ull << 1;
	}

  return 0;
}

/****************************************************************************
 * Name: traling_ones
 ****************************************************************************/

static int trailing_ones(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_ones_uc(uc);
	  if (ret != i)
	    {
		  return -1;
		}

	  uc |= (1 << i);
	}

  nbits = NBITS(us);
  us = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_ones_us(us);
	  if (ret != i)
	    {
		  return -1;
		}

	  us |= (1 << i);
	}

  nbits = NBITS(ui);
  ui = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_ones_ui(ui);
	  if (ret != i)
	    {
		  return -1;
		}

	  ui |= (1U << i);
	}

  nbits = NBITS(ul);
  ul = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_ones_ul(ul);
	  if (ret != i)
	    {
		  return -1;
		}

	  ul |= (1UL << i);
	}

  nbits = NBITS(ull);
  ull = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_trailing_ones_ul(ull);
	  if (ret != i)
	    {
		  return -1;
		}

	  ull |= (1ULL << i);
	}

  return 0;
}

/****************************************************************************
 * Name: first_leading_zero
 ****************************************************************************/

static int first_leading_zero(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = 0;
  for (i = 1; i <= nbits + 1; i++)
    {
	  ret = stdc_first_leading_zero_uc(uc);
	  if (ret != (i % (nbits + 1)))
	    {
		  return -1;
		}

	  uc |= (1 << (nbits - i));
	}

  nbits = NBITS(us);
  us = 0;
  for (i = 1; i <= nbits + 1; i++)
    {
	  ret = stdc_first_leading_zero_us(us);
	  if (ret != (i % (nbits + 1)))
	    {
		  return -1;
		}

	  us |= (1 << (nbits - i));
	}

  nbits = NBITS(ui);
  ui = 0;
  for (i = 1; i <= nbits + 1; i++)
    {
	  ret = stdc_first_leading_zero_ui(ui);
	  if (ret != (i % (nbits + 1)))
	    {
		  return -1;
		}

	  ui |= (1U << (nbits - i));
	}

  nbits = NBITS(ul);
  ul = 0;
  for (i = 1; i <= (nbits + 1); i++)
    {
	  ret = stdc_first_leading_zero_ul(ul);
	  if (ret != (i % (nbits + 1)))
	    {
		  return -1;
		}

	  ul |= (1UL << (nbits - i));
	}

  nbits = NBITS(ull);
  ull = 0;
  for (i = 1; i <= nbits + 1; i++)
    {
	  ret = stdc_first_leading_zero_ul(ull);
	  if (ret != (i % (nbits + 1)))
	    {
		  return -1;
		}

	  ull |= (1ULL << (nbits - i));
	}

  return 0;
}

/****************************************************************************
 * Name: first_leading_zero
 ****************************************************************************/

static int first_leading_one(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = UCHAR_MAX;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_leading_one_uc(uc);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  uc &= ~(1 << (nbits - (i + 1)));
	}

  nbits = NBITS(us);
  us = USHRT_MAX;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_leading_one_us(us);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  us &= ~(1 << (nbits - (i + 1)));
	}

  nbits = NBITS(ui);
  ui = UINT_MAX;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_leading_one_ui(ui);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  ui &= ~(1U << (nbits - (i + 1)));
	}

  nbits = NBITS(ul);
  ul = ULONG_MAX;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_leading_one_ul(ul);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  ul &= ~(1UL << (nbits - (i + 1)));
	}

  nbits = NBITS(ull);
  ull = ULLONG_MAX;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_leading_one_ull(ull);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  ull &= ~(1ULL << (nbits - (i + 1)));
	}


  return 0;
}

/****************************************************************************
 * Name: first_trailing_zero
 ****************************************************************************/

static int first_trailing_zero(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_zero_uc(uc);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  uc |= (1 << i);
	}

  nbits = NBITS(us);
  us = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_zero_us(us);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  us |= (1 << i);
	}

  nbits = NBITS(ui);
  ui = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_zero_ui(ui);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  ui |= (1U << i);
	}

  nbits = NBITS(ul);
  ul = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_zero_ul(ul);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  ul |= (1UL << i);
	}

  nbits = NBITS(ull);
  ull = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_zero_ull(ull);
	  if (ret != ((i + 1) % (nbits + 1)))
	    {
		  return -1;
		}

	  ull |= (1ULL << i);
	}

  return 0;
}

/****************************************************************************
 * Name: first_trailing_one
 ****************************************************************************/

static int first_trailing_one(void)
{
  int i;
  int nbits;
  int ret;
  unsigned char uc;
  unsigned short us;
  unsigned int ui;
  unsigned long int ul;
  unsigned long long int ull;

  nbits = NBITS(uc);
  uc = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_one_uc(uc);
	  if (ret != i)
	    {
		  return -1;
		}

	  uc = (1 << i);
	}

  nbits = NBITS(us);
  us = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_one_us(us);
	  if (ret != i)
	    {
		  return -1;
		}

	  us = (1 << i);
	}

  nbits = NBITS(ui);
  ui = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_one_ui(ui);
	  if (ret != i)
	    {
		  return -1;
		}

	  ui = (1U << i);
	}

  nbits = NBITS(ul);
  ul = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_one_ul(ul);
	  if (ret != i)
	    {
		  return -1;
		}

	  ul = (1UL << i);
	}

  nbits = NBITS(ull);
  ull = 0;
  for (i = 0; i <= nbits; i++)
    {
	  ret = stdc_first_trailing_one_ull(ull);
	  if (ret != i)
	    {
		  return -1;
		}

	  ull = (1ULL << i);
	}

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  const FAR struct test_entry_s *item = NULL;
  int i;
  int ret;
  int tests_ok;
  int tests_err;

  tests_ok = tests_err = 0;

  for (i = 0; i < sizeof(g_entry_list)/sizeof(g_entry_list[0]); i++)
    {
	  item = &g_entry_list[i];
	  ret = item->entry();
	  if (ret < 0)
	    {
		  tests_err++;
		  printf("stdbit_tests: %s test failed.\n", item->name);
		}
	  else
	    {
		  tests_ok++;
		}
	}

  /* Run tests. They should return 0 on success, -1 otherwise. */

  printf("stdbit tests: SUCCESSFUL: %d; FAILED: %d\n", tests_ok,
          tests_err);

  return 0;
}
