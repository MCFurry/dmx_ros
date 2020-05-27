/*
 * DMX-485 - Library for using DMX through RS-485 USB adapters, like the Enttec
 * Open DMX USB.  Supports generic RS-485 adapters as long as they can be be
 * set to 250kbaud.
 *
 * (C)2009-2010 Mike Bourgeous of Nitrogen Logic - Licensed under LGPL 2.1 or later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>
#include <time.h>

#include "dmx_ros/dmx485.h"

#define DEFAULT_CHANNELS_TO_SEND	32	// Number of channels to send per frame
#define DEFAULT_BRK			176	// Default break time
#define DEFAULT_MAB			12	// Default mark after break time
#define DEFAULT_PERIOD			20000	// Default frame period (50Hz)

/*
 * Compares two struct timespec values.  Returns -1 if a is less than b, 0 if they
 * are equal, or 1 if a is greater than b.
 */
static inline int compare_timespec(struct timespec *a, struct timespec *b)
{
	if(a->tv_sec < b->tv_sec) {
		return -1;
	}
	if(a->tv_sec == b->tv_sec) {
		if(a->tv_nsec < b->tv_nsec) {
			return -1;
		}
		if(a->tv_nsec == b->tv_nsec) {
			return 0;
		}
	}

	return 1;
}

/*
 * Opens the specified RS-485 device (i.e. /dev/ttyUSB0), initializes it for
 * DMX operation, and returns its file descriptor.  Returns -1 on error.
 */
dmx_state *dmx_open(char *filename)
{
	return dmx_open_ex(filename, 0);
}

/*
 * Opens the specified RS-485 device (i.e. /dev/ttyUSB0), initializes it for
 * DMX operation, and returns its file descriptor.  Returns -1 on error.
 * Displays no device open error messages if quiet is nonzero (other errors and
 * warnings will still be displayed).
 */
dmx_state *dmx_open_ex(char *filename, int quiet)
{
	struct serial_struct serinfo;	// Linux-specific serial port settings
	struct termios terminfo;	// Terminal settings
	int actual_rate;		// Actual baud rate achieved
	int mctrl;			// Modem control lines
	int fd;				// File descriptor
	struct timespec res;		// Monotonic clock resolution

	dmx_state *status;		// DMX status

	// Open device
	if((fd = open(filename, O_WRONLY)) < 0) {
		if(!quiet) {
			fprintf(stderr, "Error opening %s: %s\n", filename, strerror(errno));
		}
		return NULL;
	}

	// Set the custom baud rate
	if(ioctl(fd, TIOCGSERIAL, &serinfo)) {
		perror("Error getting serial port settings");
		close(fd);
		return NULL;
	}

	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / 250000;
	actual_rate = serinfo.baud_base / serinfo.custom_divisor;

	if(actual_rate != 250000) {
		// Rate is inexact, but within DMX specs
		if(actual_rate > 248756 && actual_rate < 255102) {
			fprintf(stderr, "Warning: actual baud rate is not exactly 250000 (%d)\n", actual_rate);
		} else {
			fprintf(stderr, "Error: actual baud rate is too far from 250000 (%d)\n", actual_rate);
			close(fd);
			return NULL;
		}
	}

	if(ioctl(fd, TIOCSSERIAL, &serinfo)) {
		perror("Error setting serial port settings");
		close(fd);
		return NULL;
	}

	// Set terminal and line settings (echo, stop bits, etc.)
	if(tcgetattr(fd, &terminfo)) {
		perror("Error getting terminal settings");
		close(fd);
		return NULL;
	}

	cfmakeraw(&terminfo);
	terminfo.c_cflag = CS8 | CSTOPB;
	cfsetspeed(&terminfo, B38400); // custom speed will be used instead of 38400

	if(tcsetattr(fd, TCSAFLUSH, &terminfo)) {
		perror("Error setting terminal settings");
		close(fd);
		return NULL;
	}

	// Set control lines low
	mctrl = 0;
	if(ioctl(fd, TIOCMSET, &mctrl)) {
		perror("Error setting handshaking lines");
		close(fd);
		return NULL;
	}

	// Allocate status structure
	if((status = calloc(1, sizeof(dmx_state))) == NULL) {
		perror("Error allocating status memory");
		close(fd);
		return NULL;
	}
	status->fd = fd;
	status->channels_to_send = DEFAULT_CHANNELS_TO_SEND;
	status->break_time = DEFAULT_BRK;
	status->mark_time = DEFAULT_MAB;
	status->frame_period = DEFAULT_PERIOD;

	// Initialize rate control
	if(clock_getres(CLOCK_MONOTONIC, &res)) {
		perror("Error: no monotonic clock?");
		close(fd);
		free(status);
		return NULL;
	}
	if(res.tv_sec != 0 || res.tv_nsec > 100000) {
		fprintf(stderr, "Warning: monotonic clock resolution is worse than 100us (%ld.%09lds)\n",
				(long)res.tv_sec, res.tv_nsec);
	}

	status->next_time = malloc(sizeof(struct timespec));
	if(status->next_time == NULL) {
		perror("Error allocating timer memory");
		close(fd);
		free(status);
		return NULL;
	}

	// Make sure the first frame gets sent immediately
	clock_gettime(CLOCK_MONOTONIC, status->next_time);

	return status;
}

/*
 * Closes the previously-opened DMX device and frees associated state.
 */
void dmx_close(dmx_state *status)
{
	if(status != NULL) {
		if(close(status->fd) < 0) {
			perror("Error closing DMX device");
		}
		if(status->next_time != NULL) {
			free(status->next_time);
		}
		free(status);
	}
}

/*
 * Sends the stored DMX state to its associated DMX port.  If fewer than
 * frame_period microseconds have elapsed since the start of the last frame,
 * then this function will delay until that time, unless status->period_mode is
 * nonzero, in which case calls to send_state will be ignored until the
 * specified frame period has elapsed.  This function should be called at least
 * once per second.  Returns zero on success, nonzero on error.
 */
int send_state(dmx_state *status)
{
	struct timespec this_time;

	if(status == NULL) {
		fprintf(stderr, "NULL status parameter");
		return -1;
	}

	// Mark before break + rate control
	fsync(status->fd); // fsync on a tty probably doesn't do anything...
	if(status->period_mode) {
		// Skip frame rate control mode
		clock_gettime(CLOCK_MONOTONIC, &this_time);
		if(compare_timespec(&this_time, status->next_time) < 0) {
			return 0;
		}
	} else {
		// Sleep rate control mode
		while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, status->next_time, NULL) == -1 && errno==EINTR) {
			// Repeatedly do nothing as long as clock_nanosleep() gets
			// interrupted (this space intentionally left blank)
		}
	}
	clock_gettime(CLOCK_MONOTONIC, status->next_time);
	status->next_time->tv_nsec += status->frame_period * 1000;
	if(status->next_time->tv_nsec >= 1000000000) {
		status->next_time->tv_nsec -= 1000000000;
		status->next_time->tv_sec += 1;
	}

	// Break
	if(ioctl(status->fd, TIOCSBRK, 0)) {
		perror("Error setting break");
		return -1;
	}
	usleep(status->break_time);
	if(ioctl(status->fd, TIOCCBRK, 0)) {
		perror("Error clearing break");
		return -1;
	}

	// Mark after break
	usleep(status->mark_time);

	// Data
	if(write(status->fd, &status->start_code, 1) < 0) {
		perror("Error sending start code");
		return -1;
	}
	if(write(status->fd, status->dmx_values, status->channels_to_send) < 0) {
		perror("Error sending dmx data");
		return -1;
	}

	return 0;
}

/*
 * Sets the given channel in the given DMX state to the given value.  An out of
 * range channel will be ignored.  An out of range value will be clamped.
 */
void set_channel(dmx_state *status, int channel, int value)
{
	if(status == NULL) {
		fprintf(stderr, "NULL DMX status parameter\n");
		return;
	}
	if(channel < 0 || channel >= 512) {
		fprintf(stderr, "Invalid DMX channel %d\n", channel);
		return;
	}
	if(value < 0) {
		value = 0;
	}
	if(value > 255) {
		value = 255;
	}

	status->dmx_values[channel] = value;
}

/*
 * Sets all 512 DMX channels in the given DMX state to the given value.
 * Out-of-range values will be clamped.
 */
void set_all_channels(dmx_state *status, int value)
{
	if(status == NULL) {
		fprintf(stderr, "NULL DMX status parameter\n");
		return;
	}

	if(value < 0) {
		value = 0;
	}
	if(value > 255) {
		value = 255;
	}

	memset(status->dmx_values, value, 512);
}
