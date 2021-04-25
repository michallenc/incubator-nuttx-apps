#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>

#include <netpacket/can.h>
#include <nuttx/can.h>

#include <fcntl.h>


#define SHV_CAN_DEV_ADDR		0x100

#define SHV_CAN_OPERATOR_PDO	(0x0 << 9)
#define SHV_CAN_OPERATOR_RQ		(0x1 << 9)
#define SHV_CAN_OPERATOR_RSP	(0x2 << 9)
#define SHV_CAN_OPERATOR_DATA	(0x3 << 9)

#define SHV_CAN_TOPIC			0x80

static const char *can_interface = "can0";
// static const char *serial_device = "/dev/ttyS1";

static const int can_nonblock = 0;
static int can_socket;


static int can_init(FAR const char *can_iface, int nonblock)
{
	int sd;
	struct sockaddr_can addr;
	struct ifreq ifr;
	int addrlen = sizeof(addr);

	sd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
	if (sd < 0)
	{
		perror("ERROR: failed to create CAN socket");
		return -1;
	}

	if (nonblock) {
		int flags = fcntl(sd, F_GETFL, 0);
		if (flags < 0) {
			perror("fcntl (GETFL)");
			return -1;
		}
		flags |= O_NONBLOCK;
		if (fcntl(sd, F_SETFL, flags) < 0) {
			perror("fcntl (SETFL, O_NONBLOCK)");
			return -1;
		}
	}

	strncpy(ifr.ifr_name, can_iface, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sd, (const struct sockaddr *)&addr, addrlen) < 0) {
		perror("ERROR: binding socket failed");
		close(sd);
		return -1;
	}

	return sd;
}

static int serial_init(FAR const char *dev_path, int nonblock)
{
	int flags = O_RDWR;
	if (nonblock)
		flags |= O_NONBLOCK;

	int fd = open(dev_path, flags);
	if (fd < 0) {
		perror(dev_path);
	}

	return fd;
}


static uint16_t count_crc16(FAR const uint8_t *data, int data_len)
{
	uint16_t crc = 0xFFFF;

	for (int pos = 0; pos < data_len; pos++) {
		crc ^= (unsigned int) data[pos];

		for (int i = 8; i != 0; i--) {
			if ((crc & 0x0001) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	return crc;
}


static void canser_test(void)
{
	struct can_frame frame;

	int i = 0;

	if(can_socket < 0) {
		return;
	}

	memset(&frame, 0, sizeof(struct can_frame));
	frame.can_id = SHV_CAN_DEV_ADDR | SHV_CAN_OPERATOR_PDO;
	frame.can_dlc = 8;
	frame.data[0] = SHV_CAN_TOPIC + 1;
	frame.data[1] = 0x11;
	frame.data[2] = 0x22;
	frame.data[3] = 0x33;
	frame.data[4] = 0x44;
	frame.data[5] = 0x55;
	frame.data[6] = 0x66;
	frame.data[7] = 0x77;

	while (1) {
		int wr = write(can_socket, &frame, sizeof(frame));
		if (wr != CAN_MTU) {
			perror("Write to CAN");
		}

		if (i > 100) {
			i = 0;
			usleep(100);
		} else
			++i;

		uint8_t tmp = frame.data[7];
		frame.data[7] = frame.data[6];
		frame.data[6] = frame.data[5];
		frame.data[5] = frame.data[4];
		frame.data[4] = frame.data[3];
		frame.data[3] = frame.data[2];
		frame.data[2] = frame.data[1];
		frame.data[1] = tmp;
	}
}

int main(int argc, FAR char *argv[])
{
	(void) argc;
	(void) argv;

	printf("cantest startup\n");

	can_socket = can_init(can_interface, can_nonblock);
	if (can_socket < 0)
		return 1;

	// if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0) < 0)
	// 	perror("setsockopt, clear filter");

	// serial_fd = serial_init(serial_device, serial_nonblock);
	// if (serial_fd < 0)
	// 	return 1;

	canser_test();

	return 0;
}
