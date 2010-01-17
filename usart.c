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

static 			uint8_t		rx0_buf[RX_BUF_LEN];		// buffer ptr
static 			uint8_t		rx1_buf[RX_BUF_LEN];
static volatile	uint8_t		*rx0_start = rx0_buf;		// start of valid data ptr
static volatile	uint8_t		*rx1_start = rx1_buf;
static volatile	uint8_t 	*rx0_end = rx0_buf;		// end of valid data ptr
static volatile	uint8_t 	*rx1_end = rx1_buf;

static volatile uint16_t 	rx0_read_count = 0;		// total elements read
static volatile uint16_t 	rx1_read_count = 0;
static volatile uint16_t	rx0_write_count = 0;		// total elements written
static volatile uint16_t	rx1_write_count = 0;

// TX buffer (circular)

static 			uint8_t		tx0_buf[TX_BUF_LEN];		// buffer ptr
static 			uint8_t		tx1_buf[TX_BUF_LEN];
static volatile	uint8_t		*tx0_start = tx0_buf;		// start of valid data ptr
static volatile	uint8_t		*tx1_start = tx1_buf;
static volatile	uint8_t 	*tx0_end = tx0_buf;		// end of valid data ptr
static volatile	uint8_t 	*tx1_end = tx1_buf;

static volatile uint16_t 	tx0_read_count = 0;		// total elements read
static volatile uint16_t 	tx1_read_count = 0;
static volatile uint16_t	tx0_write_count = 0;		// total elements written
static volatile uint16_t	tx1_write_count = 0;

// Callback function pointers

static void (*rx0_byte_callback)		(uint8_t byte)	= 0;
static void	(*rx0_newline_callback)		(void)			= 0;
static void	(*rx0_error_callback)		(void)			= 0;
static void	(*rx0_full_callback)		(void)			= 0;
static void	(*rx0_overrun_callback)		(void)			= 0;
static void	(*tx0_complete_callback)	(void)			= 0;

static void (*rx1_byte_callback)		(uint8_t byte)	= 0;
static void	(*rx1_newline_callback)		(void)			= 0;
static void	(*rx1_error_callback)		(void)			= 0;
static void	(*rx1_full_callback)		(void)			= 0;
static void	(*rx1_overrun_callback)		(void)			= 0;
static void	(*tx1_complete_callback)	(void)			= 0;

//
//	This interrupt is fired when a new byte is received.
//

ISR (USART0_RX_vect)
{
	uint8_t status 		= UCSR0A; /* 1 */
	uint8_t data		= UDR0;
	int8_t	buf_full 	= rx0_write_count - rx0_read_count == RX_BUF_LEN - 1;
	int8_t 	buf_overrun = rx0_write_count - rx0_read_count == RX_BUF_LEN;

	if (status & (_BV (DOR0) | _BV (FE0) | _BV (UPE0)))
	{
		if (rx0_error_callback)
			rx0_error_callback ();
		reti ();
	}

	if (buf_overrun)
	{
		rx0_start++;
		rx0_read_count++;

		if (rx0_start == rx0_buf + RX_BUF_LEN)
			rx0_start = rx0_buf;
	}

	*(rx0_end++) = data;
	rx0_write_count++;

	if (rx0_end == rx0_buf + RX_BUF_LEN)
		rx0_end = rx0_buf;

	if (rx0_byte_callback)
		rx0_byte_callback (data);

	if ((data == '\r' || data == '\n') && rx0_newline_callback)
		rx0_newline_callback ();

	if (buf_full && rx0_full_callback)
	{
		rx0_full_callback ();
	}
	else if (buf_overrun && rx0_overrun_callback)
	{
		rx0_overrun_callback ();
	}
}

ISR (USART1_RX_vect)
{
	uint8_t status 		= UCSR1A; /* 1 */
	uint8_t data		= UDR1;
	int8_t	buf_full 	= rx1_write_count - rx1_read_count == RX_BUF_LEN - 1;
	int8_t 	buf_overrun = rx1_write_count - rx1_read_count == RX_BUF_LEN;

	if (status & (_BV (DOR1) | _BV (FE1) | _BV (UPE1)))
	{
		if (rx1_error_callback)
			rx1_error_callback ();
		reti ();
	}

	if (buf_overrun)
	{
		rx1_start++;
		rx1_read_count++;

		if (rx1_start == rx1_buf + RX_BUF_LEN)
			rx1_start = rx1_buf;
	}

	*(rx1_end++) = data;
	rx1_write_count++;

	if (rx1_end == rx1_buf + RX_BUF_LEN)
		rx1_end = rx1_buf;

	if (rx1_byte_callback)
		rx1_byte_callback (data);

	if ((data == '\r' || data == '\n') && rx1_newline_callback)
		rx1_newline_callback ();

	if (buf_full && rx1_full_callback)
	{
		rx1_full_callback ();
	}
	else if (buf_overrun && rx1_overrun_callback)
	{
		rx1_overrun_callback ();
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

ISR (USART0_UDRE_vect)
{
	UDR0 = *(tx0_start++);
	if (tx0_start == tx0_buf + TX_BUF_LEN)
		tx0_start = tx0_buf;

	tx0_read_count++;

	if (!(tx0_write_count - tx0_read_count)) /* 1 */
	{
		UCSR0B &= ~_BV (UDRIE0);

		if (tx0_complete_callback)
			tx0_complete_callback();
	}
}

ISR (USART1_UDRE_vect)
{
	UDR1 = *(tx1_start++);
	if (tx1_start == tx1_buf + TX_BUF_LEN)
		tx1_start = tx1_buf;

	tx1_read_count++;

	if (!(tx1_write_count - tx1_read_count)) /* 1 */
	{
		UCSR1B &= ~_BV (UDRIE1);

		if (tx1_complete_callback)
			tx1_complete_callback();
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
	UBRR0 = (uint16_t)(F_CPU / (16 * baud_rate)) - 1; /* 1 */
	UCSR0C = 0b00000110;    //set frame format (8 bits, 1 stop bit)
}

void
usart1_init
(
	uint32_t baud_rate
)
{
	UBRR1 = (uint16_t)(F_CPU / (16 * baud_rate)) - 1; /* 1 */
	UCSR1C = 0b00000110;    //set frame format (8 bits, 1 stop bit)
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
usart1_enable_rx ()
{
	UCSR1B |= _BV (RXEN1) | _BV (RXCIE1);
}

void
usart0_disable_rx ()
{
	UCSR0B &= ~_BV (RXEN0) & ~_BV (RXCIE0);
}

void
usart1_disable_rx ()
{
	UCSR1B &= ~_BV (RXEN1) & ~_BV (RXCIE1);
}

void
usart0_enable_tx ()
{
	UCSR0B |= _BV (TXEN0);

	if (tx0_write_count - tx0_read_count)	/* 1 */
		UCSR0B |= _BV (UDRIE0);
}

void
usart1_enable_tx ()
{
	UCSR1B |= _BV (TXEN1);

	if (tx1_write_count - tx1_read_count)	/* 1 */
		UCSR1B |= _BV (UDRIE1);
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

void
usart1_disable_tx ()
{
	UCSR1B &= ~_BV (TXEN1) & ~_BV (UDRIE1);
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

	while ((rx0_write_count - rx0_read_count) && bytes_read < n)
	{
		*(dst++) = *(rx0_start++);
		if (rx0_start == rx0_buf + RX_BUF_LEN)
			rx0_start = rx0_buf;

		rx0_read_count++;
		bytes_read++;
	}

	if (append_null)
		*(dst) = '\0';

	return bytes_read;
}

uint16_t
usart1_read_from_rx_buf
(
	uint8_t		*dst,
	uint16_t	n,
	int8_t		append_null
)
{
	uint16_t	bytes_read 	= 0;

	while ((rx1_write_count - rx1_read_count) && bytes_read < n)
	{
		*(dst++) = *(rx1_start++);
		if (rx1_start == rx1_buf + RX_BUF_LEN)
			rx1_start = rx1_buf;

		rx1_read_count++;
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
		if (tx0_write_count - tx0_read_count == TX_BUF_LEN)
		{
			tx0_start++;
			if (tx0_start == tx0_buf + TX_BUF_LEN)
				tx0_start = tx0_buf;

			tx0_read_count++;
		}

		*(tx0_end++) = *(src++);
		if (tx0_end == tx0_buf + TX_BUF_LEN)
			tx0_end = tx0_buf;

		tx0_write_count++;
		bytes_written++;
	}

	if (tx0_write_count - tx0_read_count) /* 1 */
		UCSR0B |= _BV (UDRIE0);

	return bytes_written;
}

uint16_t
usart1_write_to_tx_buf
(
	uint8_t		*src,
	uint16_t	n
)
{
	uint16_t bytes_written = 0;

	while (bytes_written < n)
	{
		if (tx1_write_count - tx1_read_count == TX_BUF_LEN)
		{
			tx1_start++;
			if (tx1_start == tx1_buf + TX_BUF_LEN)
				tx1_start = tx1_buf;

			tx1_read_count++;
		}

		*(tx1_end++) = *(src++);
		if (tx1_end == tx1_buf + TX_BUF_LEN)
			tx1_end = tx1_buf;

		tx1_write_count++;
		bytes_written++;
	}

	if (tx1_write_count - tx1_read_count) /* 1 */
		UCSR1B |= _BV (UDRIE1);

	return bytes_written;
}

//
//	1.	As soon as the write and read counter match up, we have run out of data
//		to read and can turn off the data-ready interrupt.
//

uint16_t
usart0_rx_bytes_free ()
{
	return RX_BUF_LEN - rx0_write_count - rx0_read_count;
}

uint16_t
usart1_rx_bytes_free ()
{
	return RX_BUF_LEN - rx1_write_count - rx1_read_count;
}

uint16_t
usart0_tx_bytes_free ()
{
	return TX_BUF_LEN - tx0_write_count - tx0_read_count;
}

uint16_t
usart1_tx_bytes_free ()
{
	return TX_BUF_LEN - tx1_write_count - tx1_read_count;
}

void
usart0_flush_rx_buf ()
{
	rx0_start = rx0_end = rx0_buf;
	rx0_read_count = rx0_write_count = 0;
}

void
usart1_flush_rx_buf ()
{
	rx1_start = rx1_end = rx1_buf;
	rx1_read_count = rx1_write_count = 0;
}

void
usart0_flush_tx_buf ()
{
	tx0_start = tx0_end = tx0_buf;
	tx0_read_count = tx0_write_count = 0;

	UCSR0B &= ~_BV (UDRIE0); /* 1 */
}

void
usart1_flush_tx_buf ()
{
	tx1_start = tx1_end = tx1_buf;
	tx1_read_count = tx1_write_count = 0;

	UCSR1B &= ~_BV (UDRIE1); /* 1 */
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
	rx0_byte_callback = callback_func;
}

void
usart1_set_rx_byte_callback
(
	void (*callback_func)(uint8_t byte)
)
{
	rx1_byte_callback = callback_func;
}

void
usart0_set_rx_newline_callback
(
	void (*callback_func)(void)
)
{
	rx0_newline_callback = callback_func;
}

void
usart1_set_rx_newline_callback
(
	void (*callback_func)(void)
)
{
	rx1_newline_callback = callback_func;
}

void
usart0_set_rx_error_callback
(
	void (*callback_func)(void)
)
{
	rx0_error_callback = callback_func;
}

void
usart1_set_rx_error_callback
(
	void (*callback_func)(void)
)
{
	rx1_error_callback = callback_func;
}

void
usart0_set_rx_full_callback
(
	void (*callback_func)(void)
)
{
	rx0_full_callback = callback_func;
}

void
usart1_set_rx_full_callback
(
	void (*callback_func)(void)
)
{
	rx1_full_callback = callback_func;
}

void
usart0_set_rx_overrun_callback
(
	void (*callback_func)(void)
)
{
	rx0_overrun_callback = callback_func;
}

void
usart1_set_rx_overrun_callback
(
	void (*callback_func)(void)
)
{
	rx1_overrun_callback = callback_func;
}

void
usart0_set_tx_complete_callback
(
	void 		(*callback_func)(void)
)
{
	tx0_complete_callback = callback_func;
}

void
usart1_set_tx_complete_callback
(
	void 		(*callback_func)(void)
)
{
	tx1_complete_callback = callback_func;
}

