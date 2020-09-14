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
#define NUM_DIGITAL_PINS     (20u)
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
#define ROW1_PIN    (21u)
#define ROW2_PIN    (20u)
#define ROW3_PIN    (19u)
#define ROW4_PIN    (18u)
#define COL1_PIN    (33u)
#define COL2_PIN    (32u)
#define COL3_PIN    (31u)
#define COL4_PIN    (30u)

// Relay
// -----
#define RELAY_PIN   (11u)

// Photodiode
// ----------
#define PHOTO_PIN   (12u)

// Analog pins
// -----------
#define PIN_A0      (15u)
#define VOLTAGE_PIN (16u)
#define CURRENT_PIN (17u)
static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = VOLTAGE_PIN;
static const uint8_t A2  = CURRENT_PIN;
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
extern SERCOM sercom2;
extern SERCOM sercom5;

// SigFoxSerial
extern Uart SigFoxSerial;
#define SF_SLEEP_PIN    (26u)
#define SF_RST_PIN      (27u)
#define SF_RXD_PIN      (29u)
#define SF_TXD_PIN      (28u)
#define PAD_SIGFOX_TX   (UART_TX_PAD_2)
#define PAD_SIGFOX_RX   (SERCOM_RX_PAD_3)
#define SIGFOX_SERIAL   (SigFoxSerial)

// RS485Serial
extern Uart RS485Serial;
#define RS485_RE_PIN    (2u)
#define RS485_DE_PIN    (3u)
#define RS485_TXD_PIN   (4u)
#define RS485_RXD_PIN   (5u)
#define PAD_RS485_TX    (UART_TX_PAD_2)
#define PAD_RS485_RX    (SERCOM_RX_PAD_3)
#define RS485_SERIAL    (RS485Serial)

// AngazaSerial
extern Uart AngazaSerial;
#define UART_RXD_PIN    (13u)
#define UART_TXD_PIN    (14u)
#define PAD_ANGAZA_TX   (UART_TX_PAD_2)
#define PAD_ANGAZA_RX   (SERCOM_RX_PAD_3)

// Battery Voltage Parameters
#define BATTERY_24V

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
