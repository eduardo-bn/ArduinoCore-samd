/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

#include <WVariant.h>

// General definitions
// -------------------

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK     (48000000ul)

// Pins
// ----

// Number of pins defined in PinDescription array
#ifdef __cplusplus
extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (18u)
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (0u)

// Low-level pin register query macros
// -----------------------------------
#define digitalPinToPort(P)      (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P)   (1 << g_APinDescription[P].ulPin)
//#define analogInPinToBit(P)    ()
#define portOutputRegister(port) (&(port->OUT.reg))
#define portInputRegister(port)  (&(port->IN.reg))
#define portModeRegister(port)   (&(port->DIR.reg))
#define digitalPinHasPWM(P)      (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LCD
// ---
#define RS_PIN      (8u)
#define E_PIN       (9u)
#define DB4_PIN     (24u)
#define DB5_PIN     (10u)
#define DB6_PIN     (7u)
#define DB7_PIN     (0u)
#define LITE_PIN    (1u)

// LEDs
// ----
#define LED_PIN     (6u)
#define LED_BUILTIN LED_PIN

// Keypad
// ------
#define ROW1_PIN    (18u)
#define ROW2_PIN    (15u)
#define ROW3_PIN    (17u)
#define ROW4_PIN    (16u)
#define COL1_PIN    (31u)
#define COL2_PIN    (30u)
#define COL3_PIN    (13u)
#define COL4_PIN    (14u)

// Relay
// -----
#define RELAY_PIN   (19u)

// Photodiode
// ----------
#define PHOTO_PIN   (32u)

// Analog pins
// -----------
#define PIN_A0      (15u)
#define VOLTAGE_PIN (21u)
#define CURRENT_PIN (20u)
static const uint8_t A0  = PIN_A0;
static const uint8_t A5  = CURRENT_PIN;
static const uint8_t A6  = VOLTAGE_PIN;
#define ADC_RESOLUTION 12

// USB
// ---
#define PIN_USB_DM          (22ul)
#define PIN_USB_DP          (23ul)

// Serial ports
// ------------
#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

// Instances of SERCOM
extern SERCOM sercom0;

// AngazaSerial
extern Uart AngazaSerial;
#define UART_RXD_PIN    (3u)
#define UART_TXD_PIN    (2u)
#define PAD_ANGAZA_TX   (UART_TX_PAD_2)
#define PAD_ANGAZA_RX   (SERCOM_RX_PAD_3)
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
unsigned int PINCOUNT_fn();
#ifdef __cplusplus
}
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB

// Alias Serial to SerialUSB
#define Serial                      SerialUSB
