#ifndef RETRO_UART_H
#define RETRO_UART_H

// The wire protocol is stop-and-wait. Sending directly through USART0 avoids
// a second 64-byte transmit queue; receive buffering remains 64 bytes.
#if defined(__AVR_ATmega328P__)
#include <avr/io.h>
#include <avr/interrupt.h>

class RetroUart
{
  public:
    volatile uint8_t received[64];
    volatile uint8_t head;
    volatile uint8_t tail;
    bool written;

    void begin(unsigned long baud)
    {
        uint16_t divider = (uint16_t)((F_CPU / 4UL / baud - 1UL) / 2UL);
        head = tail = 0;
        written = false;
        UCSR0A = _BV(U2X0);
        UBRR0H = (uint8_t)(divider >> 8);
        UBRR0L = (uint8_t)divider;
        UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
        UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
    }

    int available() const
    {
        return (uint8_t)(64 + head - tail) & 63;
    }

    int read()
    {
        if (head == tail)
        {
            return -1;
        }
        uint8_t value = received[tail];
        tail = (uint8_t)(tail + 1) & 63;
        return value;
    }

    size_t write(uint8_t value)
    {
        while (!(UCSR0A & _BV(UDRE0)))
        {
        }
        UCSR0A |= _BV(TXC0);
        UDR0 = value;
        written = true;
        return 1;
    }

    void flush()
    {
        if (written)
        {
            while (!(UCSR0A & _BV(TXC0)))
            {
            }
        }
    }

    void end()
    {
        flush();
        UCSR0B = 0;
        written = false;
        head = tail = 0;
    }

    void receive(uint8_t value)
    {
        uint8_t next = (uint8_t)(head + 1) & 63;
        if (next != tail)
        {
            received[head] = value;
            head = next;
        }
    }
};

static RetroUart retroUart;

ISR(USART_RX_vect)
{
    uint8_t status = UCSR0A;
    uint8_t value = UDR0;
    if (!(status & _BV(UPE0)))
    {
        retroUart.receive(value);
    }
}

#define Serial retroUart
#endif
#endif
