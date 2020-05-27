/*
 * DMX-485 - Library for using DMX through RS-485 USB adapters, like the Enttec
 * Open DMX USB.  Supports generic RS-485 adapters as long as they can be be
 * set to 250kbaud.
 *
 * (C)2009-2010 Mike Bourgeous of Nitrogen Logic - Licensed under LGPL 2.1 or later
 */

#ifndef _DMX485_H_
#define _DMX485_H_

struct timespec;

/*
 * DMX output state
 */
typedef struct {
	int fd;				// File descriptor of open DMX device
	int channels_to_send;		// Number of DMX channels to transmit (1-512)
	unsigned char start_code;	// The first byte of each DMX frame (usually 0)
	unsigned char dmx_values[512];	// DMX values in the range 0-255

	struct timespec *next_time;	// The time the next outgoing DMX frame should start
	int break_time;			// The approximate duration of the start-of-frame break, in us (92-256)
	int mark_time;			// The approximate duration of the mark after break, in us (12-1000)
	int frame_period;		// The minimum amount of time between frame starts, in us (10000-1000000)
	int period_mode;		// How to limit the frame rate: zero=sleep (default), nonzero=skip
} dmx_state;

/*
 * Opens the specified RS-485 device (i.e. /dev/ttyUSB0), initializes it for
 * DMX operation, and returns its file descriptor.  Returns -1 on error.
 */
dmx_state *dmx_open(char *filename);

/*
 * Opens the specified RS-485 device (i.e. /dev/ttyUSB0), initializes it for
 * DMX operation, and returns its file descriptor.  Returns -1 on error.
 * Displays no device open error messages if quiet is nonzero (other errors and
 * warnings will still be displayed).
 */
dmx_state *dmx_open_ex(char *filename, int quiet);

/*
 * Closes the previously-opened DMX device and frees associated state.
 */
void dmx_close(dmx_state *status);

/*
 * Sends the stored DMX state to its associated DMX port.  If fewer than
 * frame_period microseconds have elapsed since the start of the last frame,
 * then this function will delay until that time, unless status->period_mode is
 * nonzero, in which case calls to send_state will be ignored until the
 * specified frame period has elapsed.  This function should be called at least
 * once per second.  Returns zero on success, nonzero on error.
 */
int send_state(dmx_state *status);

/*
 * Sets the given channel in the given DMX state to the given value.  An out of
 * range channel will be ignored.  An out of range value will be clamped.
 */
void set_channel(dmx_state *status, int channel, int value);

/*
 * Sets all 512 DMX channels in the given DMX state to the given value.
 * Out-of-range values will be clamped.
 */
void set_all_channels(dmx_state *status, int value);


#endif /* _DMX485_H_ */
