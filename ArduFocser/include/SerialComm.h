#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <Arduino.h>  // Enable the use of the Arduino String class

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#ifndef F_CPU
#define F_CPU 16000000UL  // Adjust if your board runs at a different clock speed.
#endif

// Change the callback function signature so that it takes an Arduino String


#define RX_BUFFER_SIZE 128
#define LINE_BUFFER_SIZE 64  // Adjust based on expected input length

// Global variables for ring buffer management.
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
volatile uint8_t bufferOverflow = 0;

// Global variables for line buffering.
volatile bool lineCompleteFlag = false;
char lineBuffer[LINE_BUFFER_SIZE];
uint8_t lineIndex = 0;

// -----------------------
// USART RX Interrupt Service Routine
// -----------------------
// This ISR reads each incoming byte and stores it in our ring buffer.
ISR(USART_RX_vect) {
  char data = UDR0;  // Read the received byte from hardware register.
  uint8_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;

  // Check for available space in the ring buffer.
  if (next_head != rx_tail) {
    rx_buffer[rx_head] = data;
    rx_head = next_head;
  } else {
    // Optionally flag an overflow error.
    bufferOverflow = 1;
  }
}

// -----------------------
// UART Initialization (with Double-Speed Mode Enabled)
// -----------------------
void setupUART(uint32_t baud) {
  // Enable double-speed mode for improved timing at higher baud rates.
  UCSR0A |= (1 << U2X0);

  // Calculate the UBRR value (with proper rounding) for double-speed mode.
  uint16_t ubrr_value = ((F_CPU + (baud * 4UL)) / (8UL * baud)) - 1;
  UBRR0H = (unsigned char)(ubrr_value >> 8);
  UBRR0L = (unsigned char)ubrr_value;

  // Enable receiver (RX), transmitter (TX) and RX complete interrupt.
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

  // Set frame format: 8 data bits, no parity, 1 stop bit.
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// -----------------------
// Transmission Functions
// -----------------------

// Transmit a single character.
void tx(char c) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;  // Wait until the transmit buffer is empty.
  UDR0 = c;
}

// Send a null-terminated string.
void sendString(const char *s) {
  while (*s) {
    tx(*s++);
  }
}

// -----------------------
// Ring Buffer Access Functions
// -----------------------

// Returns the number of bytes available in the ring buffer.
uint8_t available() {
  uint8_t count;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (rx_head >= rx_tail)
      count = rx_head - rx_tail;
    else
      count = RX_BUFFER_SIZE - rx_tail + rx_head;
  }
  return count;
}

// Reads a single character from the ring buffer.
// Returns the character or -1 if the buffer is empty.
int readChar() {
  int c = -1;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (rx_head != rx_tail) {
      c = rx_buffer[rx_tail];
      rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    }
  }
  return c;
}
typedef void (*LineCallback)(String line);
// -----------------------
// Process Input with Callback
// -----------------------
//
// This function reads available characters from the ring buffer, appending them
// into a static line buffer. When the specified terminator is encountered,
// the line buffer is null‑terminated, converted into a String, and then the callback
// is invoked with that String.
// Parameters:
//    terminator - The character that indicates the end of a line (e.g. '\n').
//    callback   - A function pointer that receives a String when a complete line is ready.
void processInputWithCallback(char terminator, LineCallback callback) {
  int c;
  while ((c = readChar()) != -1) {
    if (lineIndex < LINE_BUFFER_SIZE - 1)  // Leave space for the null terminator.
    {
      lineBuffer[lineIndex++] = (char)c;
      // Check for the terminator.
      if ((char)c == terminator) {
        lineBuffer[lineIndex] = '\0';  // Null-terminate the string.
        lineCompleteFlag = true;
        // Create an Arduino String from the lineBuffer.
        String line(lineBuffer);
        callback(line);
        lineIndex = 0;  // Reset for the next line.
      }
    } else {
      // If the line buffer is full without a terminator, reset it.
      lineIndex = 0;
    }
  }
}

#endif  // SERIAL_COMM_H