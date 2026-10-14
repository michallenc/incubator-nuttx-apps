/****************************************************************************
 * apps/examples/hello/hello_main.c
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
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <net/if.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/net/mii.h>
#include <nuttx/net/netconfig.h>

#include <netutils/netlib.h>

#define MAC_STR_MAXLEN 18
#define IPADDR_MAXLEN INET6_ADDRSTRLEN

struct net_ctx_s {
  bool active;
  bool error;
  bool dhcp;
};

bool network_bringup(bool bringup) {
  const char *name = "eth0";
  if (bringup)
    return netlib_ifup(name) >= 0;
  else
    return netlib_ifdown(name) >= 0;
}

static bool str2mac(const char *mac, uint8_t *parsed) {
  return sscanf(mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &parsed[0], &parsed[1],
                &parsed[2], &parsed[3], &parsed[4], &parsed[5]) == 6;
}

static bool set_addresses(const char *name, struct in_addr addrs[3]) {
  if (netlib_set_ipv4addr(name, &addrs[0]) < 0) {
    syslog(LOG_ERR, "Failed to set IP address: %d", errno);
    return false;
  }
  /* Setup default router address */
  if (netlib_set_dripv4addr(name, &addrs[1]) < 0) {
    syslog(LOG_ERR, "Failed to set default router address: %d", errno);
    return false;
  }
  /* Setup default subnest mask */
  if (netlib_set_ipv4netmask(name, &addrs[2]) < 0) {
    syslog(LOG_ERR, "Failed to set subnet_mask: %d", errno);
    return false;
  }

  return true;
}

bool sernum2mac(const char *serial_number, char mac[MAC_STR_MAXLEN]) {
  if (strlen(serial_number) < 10 || strncmp(serial_number, "SEL", 3) != 0) {
    syslog(LOG_ERR, "Invalid serial number format: \"%s\", expected SELXYYNNNN",
           serial_number);
    return false;
  }

  uint8_t family = serial_number[3];
  uint8_t year = ((serial_number[4] - '0') * 10) + (serial_number[5] - '0');
  uint16_t heg = (serial_number[6] - '0') * 1000 +
                 (serial_number[7] - '0') * 100 +
                 (serial_number[8] - '0') * 10 + (serial_number[9] - '0');
  snprintf(mac, MAC_STR_MAXLEN, "e2:00:%02x:%02x:%02x:%02x", family, year,
           heg >> 8, heg & 0xff);
  return true;
}

bool network_initialize(void) {
  // TODO: support IPv6

  const char *serial_number = "SELV260001";

  const char *name = "eth0";
  struct in_addr addr[3];
  /* Set MAC address of the device */
  uint8_t parsed[6];
  char mac[MAC_STR_MAXLEN];
  if (!sernum2mac(serial_number, mac) || !str2mac(mac, parsed)) {
    syslog(LOG_ERR, "Invalid MAC address format");
    return false;
  }
  if (netlib_setmacaddr(name, parsed) < 0) {
    syslog(LOG_ERR, "Failed to set MAC address: %s: %d", mac, errno);
    return false;
  }

  syslog(LOG_INFO, "Use DHCP to obtain IP address");
  addr[0].s_addr = 0;
  addr[1].s_addr = HTONL(0x0a000001);
  addr[2].s_addr = HTONL(0xffffff00);
  if (!set_addresses(name, addr))
    return false;
  /* Bring up the network */
  if (netlib_ifup(name) < 0) {
    syslog(LOG_ERR, "Failed to bring up network %s: %d", name, errno);
    return false;
  }
  /* Obtain IP address with DHCP */
  netlib_obtain_ipv4addr(name);
  /* Get back the obtained addresses and write it to configuration */
  int fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
  struct ifreq ifr;
  strncpy(ifr.ifr_name, name, IFNAMSIZ);
  ioctl(fd, SIOCGIFADDR, &ifr);
  const char *ipaddr =
      inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
  ioctl(fd, SIOCGIFNETMASK, &ifr);
  const char *subnet_mask =
      inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
  ioctl(fd, SIOCGIFDSTADDR, &ifr);
  const char *router_addr =
      inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
  syslog(LOG_INFO, "IP %s, router %s, subnet %s", ipaddr, router_addr,
         subnet_mask);

  close(fd);
  return true;
}

bool net_haslink(const char *name, bool ipv6) {
  bool res = false;
  int sock = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);
  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, name, strlen(name));
  ifr.ifr_mii_reg_num = MII_MSR;

  int ret = ioctl(sock, SIOCGMIIPHY, (unsigned long)&ifr);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to get PHY address: %d", errno);
    close(sock);
    return false;
  }

  ret = ioctl(sock, SIOCGMIIREG, (unsigned long)&ifr);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to get PHY status: %d", errno);
    close(sock);
    return false;
  }

  res = ifr.ifr_mii_val_out & MII_MSR_LINKSTATUS;
  close(sock);
  return res;
}

bool net_isup(const char *name, bool ipv6) {
  struct ifreq ifr;
  int sock = socket(ipv6 ? PF_INET6 : PF_INET, SOCK_DGRAM, IPPROTO_IP);
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, name, strlen(name));
  if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
    syslog(LOG_ERR, "Failed to get network status: %d", errno);
    return false;
  }

  close(sock);
  return ifr.ifr_flags & IFF_UP && ifr.ifr_flags & IFF_RUNNING;
}

static void eloop_net(void) {
  bool activate = true;
  struct net_ctx_s ctx = {.dhcp = true};
  while (true) {
    bool link = net_haslink("eth0", false);
    bool renew = false;
    if (ctx.active && !link) {
      syslog(LOG_ERR, "Network should be active but there is no link");
      /* We should be activate but there is no link */
      /* Bring it down */
      network_bringup(false);
      ctx.active = false;
      ctx.error = true;
    } else if (ctx.error && link) {
      /* The connection is back. Just set error flag to false and
       * call the loop again, the first if statement will take care
       * of the ifup
       */
      if (ctx.dhcp) {
        renew = true;
      } else {
        ctx.error = false;
      }
    }

    if (ctx.dhcp && !ctx.error && activate && !ctx.active) {
      renew = true;
    }

    if (renew) {
      struct sched_param param;
      param.sched_priority = 80;
      sched_setscheduler(pthread_self(), SCHED_RR, &param);
      ctx.error = !network_initialize();
      /* Get back to standard priority */
      param.sched_priority = 100;
      sched_setscheduler(pthread_self(), SCHED_RR, &param);
    }

    if (!ctx.error && activate != ctx.active) {
      syslog(LOG_INFO, "Try to %s network",
             activate ? "activate" : "deactivate");

      /* We need to set low priority here otherwise the loop in
       * bringup would lag the system.
       */
      struct sched_param param;
      param.sched_priority = 80;
      sched_setscheduler(pthread_self(), SCHED_RR, &param);
      bool res = network_bringup(activate);
      if (!res) {
        syslog(LOG_ERR, "Failed to %s network",
               activate ? "activate" : "deactivate");
      } else {
        syslog(LOG_INFO, "Network was %s",
               activate ? "activated" : "deactivated");
      }

      /* Get back to standard priority */
      param.sched_priority = 100;
      sched_setscheduler(pthread_self(), SCHED_RR, &param);

      ctx.active = activate;
      ctx.error = false;
    }

    if (ctx.error && !activate) {
      ctx.error = false;
    }

    clock_nanosleep(CLOCK_MONOTONIC, 0,
                    &(struct timespec){.tv_nsec = 500000000}, NULL);
  }
}

int main(int argc, FAR char *argv[]) {
  openlog("net", LOG_PERROR, LOG_USER);
  eloop_net();
  return 0;
}
