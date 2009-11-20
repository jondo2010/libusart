//
//	usart.c
//
//	Library for the AVR USART present on the ATMEGA168 and AT90CAN128.
//	Provides buffering for both incoming and outgoing data.
//
//	Michael Jean <michael.jean@shaw.ca>
//

#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "usart.h"

// RX buffer (circular)

static 			uint8_t		rx_buf[RX_BUF_LEN];		// buffer ptr
static volatile	uint8_t		*rx_start = rx_buf;		// start of valid data ptr
static volatile	uint8_t 	*rx_end = rx_buf;		// end of valid data ptr

static volatile uint16_t 	rx_read_count = 0;		// total elements read
static volatile uint16_t	rx_write_count = 0;		// total elements written

// TX buffer (circular)

static 			uint8_t		tx_buf[TX_BUF_LEN];		// buffer ptr
static volatile	uint8_t		*tx_start = tx_buf;		// start of valid data ptr
static volatile	uint8_t 	*tx_end = tx_buf;		// end of valid data ptr

static volatile uint16_t 	tx_read_count = 0;		// total elements read
static volatile uint16_t	tx_write_count = 0;		// total elements written

// Callback function pointers

static void (*rx_byte_callback)		(uint8_t byte)	= 0;
static void	(*rx_newline_callback)	(void)			= 0;
static void	(*rx_error_callback)	(void)			= 0;
static void	(*rx_full_callback)		(void)			= 0;
static void	(*rx_overrun_callback)	(void)			= 0;
static void	(*tx_complete_callback)	(void)			= 0;

//
//	This interrupt is fired when a new byte is received.
//

ISR (USART_RX_vect)
{
	uint8_t status 		= UCSR0A; /* 1 */
	uint8_t data		= UDR0;
	int8_t	buf_full 	= rx_write_count - rx_read_count == RX_BUF_LEN - 1;
	int8_t 	buf_overrun = rx_write_count - rx_read_count == RX_BUF_LEN;

	if (status & (_BV (DOR0) | _BV (FE0) | _BV (UPE0)))
	{
		if (rx_error_callback)
			rx_error_callback ();
		reti ();
	}

	if (buf_overrun)
	{
		rx_start++;
		rx_read_count++;

		if (rx_start == rx_buf + RX_BUF_LEN)
			rx_start = rx_buf;
	}

	*(rx_end++) = data;
	rx_write_count++;

	if (rx_end == rx_buf + RX_BUF_LEN)
		rx_end = rx_buf;

	if (rx_byte_callback)
		rx_byte_callback (data);

	if ((data == '\r' || data == '\n') && rx_newline_callback)
		rx_newline_callback ();

	if (buf_full && rx_full_callback)
	{
		rx_full_callback ();
	}
	else if (buf_overrun && rx_overrun_callback)
	{
		rx_overrun_callback ();
	}
}

//
//	1.	Reading the data register blows away the status register, so we have
//		to read it into a temporary register before reading the data register.
//

//
//	This interrupt is fired when the transmit register is empty and
//	new data may be transferred.
//

ISR (USART_UDRE_vect)
{
	UDR0 = *(tx_start++);
	if (tx_start == tx_buf + TX_BUF_LEN)
		tx_start = tx_buf;

	tx_read_count++;

	if (!(tx_write_count - tx_read_count)) /* 1 */
	{
		UCSR0B &= ~_BV (UDRIE0);

		if (tx_complete_callback)
			tx_complete_callback();
	}
}

//
//	1.	As soon as the write and read counter match up, we have run out of data
//		to read and can turn off the data-ready interrupt.
//

//
// 	Below here is the implementation of the public interface.
//

void
usart0_init
(
	uint32_t baud_rate
)
{
	UBRR0 = (uint16_t)(F_CPU / 16 / baud_rate) - 1; /* 1 */
}

//
//	1.	The bit-rate prescaler formula is given in the datasheet.
//

void
usart0_enable_rx ()
{
	UCSR0B |= _BV (RXEN0) | _BV (RXCIE0);
}

void
usart0_disable_rx ()
{
	UCSR0B &= ~_BV (RXEN0) & ~_BV (RXCIE0);
}

void
usart0_enable_tx ()
{
	UCSR0B |= _BV (TXEN0);

	if (tx_write_count - tx_read_count)	/* 1 */
		UCSR0B |= _BV (UDRIE0);
}

//
//	1.	We don't neccessarily want to re-enable the data-ready interrupt
//		because it will keep firing until we either disable it or put
//		something in the tx buffer.
//

void
usart0_disable_tx ()
{
	UCSR0B &= ~_BV (TXEN0) & ~_BV (UDRIE0);
}

uint16_t
usart0_read_from_rx_buf
(
	uint8_t		*dst,
	uint16_t	n,
	int8_t		append_null
)
{
	uint16_t	bytes_read 	= 0;

	while ((rx_write_count - rx_read_count) && bytes_read < n)
	{
		*(dst++) = *(rx_start++);
		if (rx_start == rx_buf + RX_BUF_LEN)
			rx_start = rx_buf;

		rx_read_count++;
		bytes_read++;
	}

	if (append_null)
		*(dst) = '\0';

	return bytes_read;
}

uint16_t
usart0_write_to_tx_buf
(
	uint8_t		*src,
	uint16_t	n
)
{
	uint16_t bytes_written = 0;

	while (bytes_written < n)
	{
		if (tx_write_count - tx_read_count == TX_BUF_LEN)
		{
			tx_start++;
			if (tx_start == tx_buf + TX_BUF_LEN)
				tx_start = tx_buf;

			tx_read_count++;
		}

		*(tx_end++) = *(src++);
		if (tx_end == tx_buf + TX_BUF_LEN)
			tx_end = tx_buf;

		tx_write_count++;
		bytes_written++;
	}

	if (tx_write_count - tx_read_count) /* 1 */
		UCSR0B |= _BV (UDRIE0);

	return bytes_written;
}

//
//	1.	As soon as the write and read counter match up, we have run out of data
//		to read and can turn off the data-ready interrupt.
//

uint16_t
usart0_rx_bytes_free ()
{
	return RX_BUF_LEN - rx_write_count - rx_read_count;
}

uint16_t
usart0_tx_bytes_free ()
{
	return TX_BUF_LEN - tx_write_count - tx_read_count;
}

void
usart0_flush_rx_buf ()
{
	rx_start = rx_end = rx_buf;
	rx_read_count = rx_write_count = 0;
}

void
usart0_flush_tx_buf ()
{
	tx_start = tx_end = tx_buf;
	tx_read_count = tx_write_count = 0;

	UCSR0B &= ~_BV (UDRIE0); /* 1 */
}

//
//	1. 	The data-ready interrupt will keep firing until we put something new
//		into the transmit buffer, so we have to keep it disabled when the
//		buffer is empty.
//

void
usart0_set_rx_byte_callback
(
	void (*callback_func)(uint8_t byte)
)
{
	rx_byte_callback = callback_func;
}

void
usart0_set_rx_newline_callback
(
	void (*callback_func)(void)
)
{
	rx_newline_callback = callback_func;
}

void
usart0_set_rx_error_callback
(
	void (*callback_func)(void)
)
{
	rx_error_callback = callback_func;
}

void
usart0_set_rx_full_callback
(
	void (*callback_func)(void)
)
{
	rx_full_callback = callback_func;
}

void
usart0_set_rx_overrun_callback
(
	void (*callback_func)(void)
)
{
	rx_overrun_callback = callback_func;
}

void
usart0_set_tx_complete_callback
(
	void 		(*callback_func)(void)
)
{
	tx_complete_callback = callback_func;
}

