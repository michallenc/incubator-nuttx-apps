#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>

#include <netpacket/can.h>
#include <nuttx/can.h>

#include <fcntl.h>

#include <poll.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int can_init(FAR const char *can_iface, int nonblock)
{
	int sd;
	struct sockaddr_can addr;
	struct ifreq ifr;
	int addrlen = sizeof(addr);

	sd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
	if (sd < 0)
	{
		nerr("ERROR: failed to create CAN socket");
		return -1;
	}
	printf("I am here\n");
	if (nonblock) {
		int flags = fcntl(sd, F_GETFL, 0);
		if (flags < 0) {
			nerr("fcntl (GETFL)");
			return -1;
		}
		flags |= O_NONBLOCK;
		if (fcntl(sd, F_SETFL, flags) < 0) {
			nerr("fcntl (SETFL, O_NONBLOCK)");
			return -1;
		}
	}
	printf("I am here2\n");
	strncpy(ifr.ifr_name, can_iface, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		nerr("if_nametoindex");
		return -1;
	}
	printf("I am here3\n");
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sd, (const struct sockaddr *)&addr, addrlen) < 0) {
		nerr("ERROR: binding socket failed");
		close(sd);
		return -1;
	}
	printf("I am here4\n");
	return sd;
}

static const char *can_interface = "can0";

int main(int argc, FAR char *argv[])
{
	(void) argc;
	(void) argv;

	const int can_nonblock = 0;
	int can_socket;

	printf("canser-ng startup halo\n");

	can_socket = can_init(can_interface, can_nonblock);
	printf("socket %d\n", can_socket);
	if (can_socket < 0)	
		{
			printf("no socket\n");
			return 1;
		}


	// if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0) < 0)
	// 	perror("setsockopt, clear filter");

	struct can_frame can_frame;
	while (1) {
		int rd = read(can_socket, &can_frame, sizeof(can_frame));
		if (rd == 0) {
			printf("spurious wakeup on CAN");
			continue;
		} else if (rd < 0 && errno == EINTR) {
			printf("interrupted read on CAN");
			continue;
		} else if (rd != CAN_MTU) {
			printf("read from CAN");
			return 1;
		}

		//printf("after read\n");

		//printf("\nReceived CAN frame, id: 0x%lX\n", can_frame.can_id);
		//print_buf(can_frame.data, can_frame.can_dlc);

		can_frame.can_id -= 1;
		int wr = write(can_socket, &can_frame, sizeof(can_frame));
		if (wr != CAN_MTU) {
			printf("write to CAN");
		}

		//printf("after write\n");
	}

	return 0;
}
