/*
 * LWCloneU2
 * Copyright (C) 2013 Andreas Dittrich <lwcloneu2@cithraidt.de>
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program;
 * if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#define LED_MAPPING_TABLE(_map_) \
	\
	_map_( A, 0, 0 ) /* ( AD0 )               Digital pin 22 */ \
	_map_( A, 1, 0 ) /* ( AD1 )               Digital pin 23 */ \
	_map_( A, 2, 0 ) /* ( AD2 )               Digital pin 24 */ \
	_map_( A, 3, 0 ) /* ( AD3 )               Digital pin 25 */ \
	_map_( A, 4, 0 ) /* ( AD4 )               Digital pin 26 */ \
	_map_( A, 5, 0 ) /* ( AD5 )               Digital pin 27 */ \
	_map_( A, 6, 0 ) /* ( AD6 )               Digital pin 28 */ \
	_map_( A, 7, 0 ) /* ( AD7 )               Digital pin 29 */ \
	_map_( C, 7, 0 ) /* ( A15 )               Digital pin 30 */ \
	_map_( C, 6, 0 ) /* ( A14 )               Digital pin 31 */ \
	_map_( C, 5, 0 ) /* ( A13 )               Digital pin 32 */ \
	_map_( C, 4, 0 ) /* ( A12 )               Digital pin 33 */ \
	_map_( C, 3, 0 ) /* ( A11 )               Digital pin 34 */ \
	_map_( C, 2, 0 ) /* ( A10 )               Digital pin 35 */ \
	_map_( C, 1, 0 ) /* ( A9 )                Digital pin 36 */ \
	_map_( C, 0, 0 ) /* ( A8 )                Digital pin 37 */ \
	_map_( D, 7, 0 ) /* ( T0 )                Digital pin 38 */ \
	_map_( G, 2, 0 ) /* ( ALE )               Digital pin 39 */ \
	_map_( G, 1, 0 ) /* ( RD )                Digital pin 40 */ \
	_map_( G, 0, 0 ) /* ( WR )                Digital pin 41 */ \
	_map_( L, 7, 0 ) /*                       Digital pin 42 */ \
	_map_( L, 6, 0 ) /*                       Digital pin 43 */ \
	_map_( L, 5, 0 ) /* ( OC5C )              Digital pin 44 (PWM) */ \
	_map_( L, 4, 0 ) /* ( OC5B )              Digital pin 45 (PWM) */ \
	_map_( L, 3, 0 ) /* ( OC5A )              Digital pin 46 (PWM) */ \
	_map_( L, 2, 0 ) /* ( T5 )                Digital pin 47 */ \
	_map_( L, 1, 0 ) /* ( ICP5 )              Digital pin 48 */ \
	_map_( L, 0, 0 ) /* ( ICP4 )              Digital pin 49 */ \
	_map_( B, 3, 0 ) /* ( MISO/PCINT3 )       Digital pin 50 (MISO) */ \
	_map_( B, 2, 0 ) /* ( MOSI/PCINT2 )       Digital pin 51 (MOSI) */ \
	_map_( B, 1, 0 ) /* ( SCK/PCINT1 )        Digital pin 52 (SCK) */ \
	_map_( B, 0, 0 ) /* ( SS/PCINT0 )         Digital pin 53 (SS) */ \
	\
	/* end */

#if (USE_MOUSE)
#define MOUSE_X_CLK_INDEX    9
#define MOUSE_X_DIR_INDEX   10
#define MOUSE_Y_CLK_INDEX   11
#define MOUSE_Y_DIR_INDEX   12
#endif

#define SHIFT_SWITCH_INDEX   15

// connect MMA8151 ON PIN 20 and 21
//	_map_( D, 1,    KEY_7,           0				   ) /* ( SDA/INT1 )          Digital pin 20 (SDA) */ \
//	_map_( D, 0,    KEY_8,           0				   ) /* ( SCL/INT0 )          Digital pin 21 (SCL) */ \
//
#define PANEL_MAPPING_TABLE(_map_) \
	\
	_map_( E, 4,    MOD_LeftShift,   AC_VolumeDown     ) /* ( OC3B/INT4 )         Digital pin 2 (PWM) */ \
	_map_( E, 5,    MOD_RightShift,  AC_VolumeUp       ) /* ( OC3C/INT5 )         Digital pin 3 (PWM) */ \
	_map_( G, 5,    MOD_LeftControl, 0                 ) /* ( OC0B )              Digital pin 4 (PWM) */ \
	_map_( E, 3,    MOD_RightControl,0                 ) /* ( OC3A/AIN1 )         Digital pin 5 (PWM) */ \
	_map_( H, 3,    KEY_Esc,         KEY_F3            ) /* ( OC4A )              Digital pin 6 (PWM) */ \
	_map_( H, 4,    KEY_Enter,       0                 ) /* ( OC4B )              Digital pin 7 (PWM) */ \
	_map_( H, 5,    KEY_1,           KEY_P             ) /* ( OC4C )              Digital pin 8 (PWM) */ \
	_map_( H, 6,    KEY_5,           0                 ) /* ( OC2B )              Digital pin 9 (PWM) */ \
	_map_( B, 4,    KEY_A,           0                 ) /* ( OC2A/PCINT4 )       Digital pin 10 (PWM) */ \
	_map_( B, 5,    KEY_S,           0                 ) /* ( OC1A/PCINT5 )       Digital pin 11 (PWM) */ \
	_map_( B, 6,    KEY_D,           0                 ) /* ( OC1B/PCINT6 )       Digital pin 12 (PWM) */ \
	_map_( B, 7,    0,               0   			   ) /* ( OC0A/OC1C/PCINT7 )  Digital pin 13 (PWM) */ \
	_map_( J, 1,    KEY_2,           0			       ) /* ( TXD3/PCINT10 )      Digital pin 14 (TX3) */ \
	_map_( J, 0,    KEY_3,           0                 ) /* ( RXD3/PCINT9 )       Digital pin 15 (RX3) */ \
	_map_( H, 1,    KEY_4,           0                 ) /* ( TXD2 )              Digital pin 16 (TX2) */ \
	_map_( H, 0,    KEY_End,         0                 ) /* ( RXD2 )              Digital pin 17 (RX2) */ \
	_map_( F, 6,    KEY_9,           0				   ) /* ( ADC6 )              Analog pin 6 */ \
	_map_( F, 7,    KEY_0,           0				   ) /* ( ADC7 )              Analog pin 7 */ \
	_map_( K, 0,    J1_Button1,      0				   ) /* ( ADC8/PCINT16 )      Analog pin 8 */ \
	_map_( K, 1,    J1_Button2,      0				   ) /* ( ADC9/PCINT17 )      Analog pin 9 */ \
	_map_( K, 2,    J1_Button3,      0				   ) /* ( ADC10/PCINT18 )     Analog pin 10 */ \
	_map_( K, 3,    J1_Button4,      0				   ) /* ( ADC11/PCINT19 )     Analog pin 11 */ \
	_map_( K, 4,    J1_Button5,      0				   ) /* ( ADC12/PCINT20 )     Analog pin 12 */ \
	_map_( K, 5,    J1_Button6,      0                 ) /* ( ADC13/PCINT21 )     Analog pin 13 */ \
	_map_( K, 6,    J1_Button7,      0                 ) /* ( ADC14/PCINT22 )     Analog pin 14 */ \
	_map_( K, 7,    J1_Button8,      0                 ) /* ( ADC15/PCINT23 )     Analog pin 15 */ \
	\
	/* end */


//
// (port, pin, mux, value_min, value_max, joyid, axis)
#define ADC_MAPPING_TABLE(_map_) \
	\
		_map_( F, 0, 0x00, 0.000, 1.000, ID_AccelGyro, 0 ) /* Analog Pin 0 X   */ \
		_map_( F, 1, 0x01, 0.000, 1.000, ID_AccelGyro, 1 ) /* Analog Pin 1 Y   */ \
		_map_( F, 2, 0x02, 0.000, 1.000, ID_AccelGyro, 2 ) /* Analog Pin 2 Z   */ \
		_map_( F, 3, 0x03, 0.000, 1.000, ID_AccelGyro, 3 ) /* Analog Pin 3 Rx  */ \
		_map_( F, 4, 0x04, 0.000, 1.000, ID_AccelGyro, 4 ) /* Analog Pin 4 Ry  */ \
		_map_( F, 5, 0x05, 0.000, 1.000, ID_AccelGyro, 5 ) /* Analog Pin 5 Rz  */ \
	\
	/* end */
