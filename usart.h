//
//	usart.h
//
//	Library for the AVR USART present on the ATMEGA168 and AT90CAN128.
//	Provides buffering for both incoming and outgoing data.
//
//	Michael Jean <michael.jean@shaw.ca>
//

//
//	PLEASE TAKE NOTE:
//
//	The USART library allows the user to supply `callback functions' to handle
//	various events like a new character being received or a transmission
//	completing.
//
//	It is important to note that these functions will be called from within
//	an ISR. You should avoid doing any heavy-duty processing in the callback.
//	Use them to set flags for later processing that is required. Otherwise,
//	it's entirely likely you will start getting data overruns, or you will
//	miss other interrupt events you have scheduled.
//
//	The callbacks occur after the ISR is finished with the buffer, so it's ok
//	to modify the buffer states within the callbacks.
//


#ifndef _USART_H
#define _USART_H

#include <inttypes.h>

//
// 	N.B. The circular buffers use `read' and `write' counters to keep track
//	of data in the buffers. These are 16-bit numbers and must be able to count
//	higher than the length of the buffers. Therefore, maximum buffer length is
//	limited to 2^16 - 2. However, given the amount of work memory available on
//	the AVR processors, this likely won't be an issue.
//

#define	RX_BUF_LEN 64
#define TX_BUF_LEN 64

//
//	Set the USART baud rate and frame options. The default is to use 8 data
//	bits, no parity bit, and 1 stop bit.
//

void
usart0_init
(
	uint32_t	baud_rate
);

//
//	Enable the receiver portion of the USART. The RX buffer is not touched by
//	this operation.
//

void
usart0_enable_rx ();

//
//	Disable the receiver portion of the USART. The RX buffer is not touched by
//	this operation.
//

void
usart0_disable_rx ();

//
//	Enable the transmitter portion of the USART. The TX buffer is not touched by
//	this operation.
//

void
usart0_enable_tx ();

//
//	Disable the transmitter portion of the USART. The TX buffer is not touched by
//	this operation.
//

void
usart0_disable_tx ();

//
//	Read at most `n' bytes from the RX buffer into the buffer pointed to by
//	`dst'. Return the number of bytes actually read.
//
//	This function advances the start-of-data pointer in the RX buffer, so the
//	read data becomes free. If `n' is larger than the number of bytes actually
//	available to be read, then everything in the buffer is copied and the
//	buffer becomes empty.
//
//	If the `append_null' parameter is non-zero, then a null terminator will be
//	appended to the end of the destination buffer. The terminator does not
//	count towards the `n' characters copied, so the buffer must be at least
//	n + 1 bytes long if this option is chosen.
//

uint16_t
usart0_read_from_rx_buf
(
	uint8_t		*dst,
	uint16_t	n,
	int8_t		append_null
);

//
//	Write `n' bytes from the buffer pointed to by `src' into the TX buffer.
//	Return the number of bytes actually written.
//
//	It is the responsibility of the caller to make sure there is enough free
//	space in the buffer before writing.
//
//	If the buffer is initially empty and at least 1 byte is written,
//	data transmission begins immediately and does not cease until the
//	transmitter is disabled or the buffer is again empty.
//

uint16_t
usart0_write_to_tx_buf
(
	uint8_t		*src,
	uint16_t	n
);

//
//	Return the number of bytes free in the RX buffer.
//

uint16_t
usart0_rx_bytes_free ();

//
//	Return the number of bytes free in the TX buffer.
//

uint16_t
usart0_tx_bytes_free ();

//
//	Flush the RX buffer. All data is lost.
//

void
usart0_flush_rx_buf ();

//
//	Flush the TX buffer. All data is lost.
//

void
usart0_flush_tx_buf ();

//
//	Set the function that will called whenever a new byte is received. The
//	function will be passed the byte that was received.
//
//	If this function is passed a null pointer, the callback is disabled.
//

void
usart0_set_rx_byte_callback
(
	void (*callback_func)(uint8_t byte)
);

//
//	Set the function that will called whenever a newline is received.
//
//	Different platforms send different characters when enter/return is pressed,
//	e.g., UNIX sends `\n', MacOS X sends `\r', and Windows sends `\r\n'.
//
//	We consider `\n' or `\r' to be newlines. We don't care about Windows.
//
//	If this function is passed a null pointer, the callback is disabled.
//

void
usart0_set_rx_newline_callback
(
	void (*callback_func)(void)
);

//
//	Set the function that will called whenever a receiver error occurs.
//
//	The three types of receiver errors are:
//
//	`Data Overrun': A new byte was received before the old byte was read. If
//					this error occurs, you are probably spending way too much
//					time in an ISR that is blocking the receiver ISR.
//
//	`Frame Error':	The frame received had an invalid format. Likely, there is
//					a configuration problem on either the sender or receiver.
//
//	`Parity Error':	The frame failed a parity check. There is likey noise on
//					the channel, or a configuration issue. By default we
//					don't use parity, so this will likely never occur. But,
//					it doesn't hurt to include it, since it could happen if
//					the init routine is changed to use parity.
//
//	If this function is passed a null pointer, the callback is disabled.
//

void
usart0_set_rx_error_callback
(
	void (*callback_func)(void)
);

//
//	Set the function that will called whenever the rx buffer is full.
//
//	If this function is passed a null pointer, the callback is disabled.
//

void
usart0_set_rx_full_callback
(
	void (*callback_func)(void)
);

//
//	Set the function that will called whenever the rx buffer is overrun.
//	This callback will occur before the buffer is overrun, so you can
//	actually rescue the buffer before it gets overwrote.
//
//	If this function is passed a null pointer, the callback is disabled.
//

void
usart0_set_rx_overrun_callback
(
	void (*callback_func)(void)
);

//
//	Set the function that will called whenever the transmission has completed
//	and the buffer is now empty.
//
//	If this function is passed a null pointer, the callback is disabled.
//

void
usart0_set_tx_complete_callback
(
	void 		(*callback_func)(void)
);

#endif /* _USART_H */

