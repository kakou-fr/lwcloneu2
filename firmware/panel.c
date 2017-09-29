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

 /* Name: hid_input.c
 * Project: V-USB Mame Panel
 * Author: Andreas Oberdorfer
 * Creation Date: 2009-09-19
 * Copyright 2009 - 2011 Andreas Oberdorfer
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 */

#include <string.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include <hwconfig.h>
#include "panel.h"
#include "comm.h"
#include "keydefs.h"
#include "clock.h"

#if !defined(PANEL_TASK)
	void panel_init(void) {}
	uint8_t panel_get_report(uint8_t **ppdata) { return 0; }
#else


const int DEBOUNCE = 5;

// derive the number of inputs from the table, let the compiler check that no pin is used twice
enum { 
	#define MAP(port, pin, normal_id, shift_id) port##pin##_index,
	PANEL_MAPPING_TABLE(MAP)
	NUMBER_OF_INPUTS,
	#undef MAP
	#if defined(LED_MAPPING_TABLE)
	#define MAP(port, pin, inv) port##pin##_index,
	LED_MAPPING_TABLE(MAP)
	#undef MAP
	#endif
	#if defined(ADC_MAPPING_TABLE)
	#define MAP(port, pin, mux, minval, maxval, joyid, axis) port##pin##_index,
	ADC_MAPPING_TABLE(MAP)
	#undef MAP
	#endif
};

#if defined(ADC_MAPPING_TABLE)
enum {
	#define MAP(port, pin, mux, minval, maxval, joyid, axis) port##pin##_adcindex,
	ADC_MAPPING_TABLE(MAP)
	#undef MAP
	NUM_ADC_CHANNELS
};
#endif

static uint8_t ReportBuffer[8];
static uint8_t InputState[NUMBER_OF_INPUTS];
static uint8_t shift_key = 0;
static uint8_t shift_key_cleanup = 0;
static uint8_t need_key_update = 0;
static uint8_t need_consumer_update = 0;

#if (NUM_JOYSTICKS >= 1)
static uint8_t need_joystick_update[NUM_JOYSTICKS];
#endif

#if defined(ENABLE_ANALOG_INPUT)
static uint16_t adc_values[NUM_ADC_CHANNELS] = {0};
static const uint8_t adc_mux_table[NUM_ADC_CHANNELS] = {
	#define MAP(port, pin, mux, minval, maxval, joyid, axis) mux,
	ADC_MAPPING_TABLE(MAP)
	#undef MAP
};
#endif


#if defined(ENABLE_ANALOG_INPUT)

uint16_t ADC_getvalue(uint8_t id)
{
	uint16_t x;

	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		x = adc_values[id];
	}

	return x;
}

static int16_t joyval12(uint16_t x, int16_t minval, int16_t maxval)
{
	return (int16_t)(((int32_t)x * (int32_t)(maxval - minval) + (1 << 9)) >> 10) + minval - 2047;
}

static int8_t joyval8(uint16_t x, int16_t minval, int16_t maxval)
{
	return (int8_t)(((int32_t)x * (int32_t)(maxval - minval) + (1 << 9)) >> 10) + minval - 127;
}

#endif


#if (USE_ACCELGYRO) || (ENABLE_MMA8451_NUDGE)
static uint8_t need_accelgyro_update = 0;
#endif


#if (ENABLE_MMA8451_NUDGE)

#include "MMA8451.h"
#include <util/delay.h>
#include <math.h>

#define ORIENTATION_PORTS_AT_RIGHT
//#define DEBUG_PRINTF
#ifdef DEBUG_PRINTF
#include "uart.h"
#endif

// Calculate the value needed for
// the CTC match value in OCR1A.
#define CTC_MATCH_OVERFLOW ((F_CPU / 1000) / 8)

volatile unsigned long timer1_millis;
long milliseconds_since;

unsigned long start;

ISR (TIMER3_COMPA_vect)
{
        timer1_millis++;
}

unsigned long millis (void)
{
        unsigned long millis_return;
        // Ensure this cannot be disrupted
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
                millis_return = timer1_millis;
        }
        return millis_return;
}

// accelerometer input history item, for gathering calibration data
typedef struct AccHist
{

        // reading for this entry
        float x;
        float y;

        // distance from previous entry
        float d;

        // total and count of samples averaged over this period
        float xtot, ytot;
        int cnt;
} AccHist_t;

typedef struct AccPos
{
        int x;
        int y;
} AccPos_t;


float distance(AccHist_t *cur, AccHist_t *prev)
{
        return sqrt(square(prev->x - cur->x) + square(prev->y - cur->y));
}


void set(AccHist_t *cur, float x, float y, AccHist_t *prv)
{
        // save the raw position
        cur->x = x;
        cur->y = y;
        cur->d = distance(cur, prv);
}


int cnt=0;

void clearAvg(AccHist_t *cur) { cur->xtot = cur->ytot = 0.0; cur->cnt = 0; }
void addAvg(AccHist_t *cur, float x, float y) { cur->xtot += x; cur->ytot += y; cur->cnt=cur->cnt+1; }
const float xAvg(AccHist_t cur) { return cur.xtot/cur.cnt; }
const float yAvg(AccHist_t cur) { return cur.ytot/cur.cnt; }

/*************************************************/

// --------------------------------------------------------------------------
//
// Set up mappings for the joystick X and Y reports based on the mounting
// orientation of the KL25Z in the cabinet.  Visual Pinball and other
// pinball software effectively use video coordinates to define the axes:
// positive X is to the right of the table, negative X to the left, positive
// Y toward the front of the table, negative Y toward the back.  The KL25Z
// accelerometer is mounted on the board with positive Y toward the USB
// ports and positive X toward the right side of the board with the USB
// ports pointing up.  It's a simple matter to remap the KL25Z coordinate
// system to match VP's coordinate system for mounting orientations at
// 90-degree increments...
//
#if defined(ORIENTATION_PORTS_AT_FRONT)
# define JOY_X(x, y)   (y)
# define JOY_Y(x, y)   (x)
#elif defined(ORIENTATION_PORTS_AT_LEFT)
# define JOY_X(x, y)   (-(x))
# define JOY_Y(x, y)   (y)
#elif defined(ORIENTATION_PORTS_AT_RIGHT)
# define JOY_X(x, y)   (x)
# define JOY_Y(x, y)   (-(y))
#elif defined(ORIENTATION_PORTS_AT_REAR)
# define JOY_X(x, y)   (-(y))
# define JOY_Y(x, y)   (-(x))
#else
# error Please define one of the ORIENTATION_PORTS_AT_xxx macros to establish the accelerometer orientation in your cabinet
#endif

// --------------------------------------------------------------------------
//
// Joystick axis report range - we report from -JOYMAX to +JOYMAX
//
#define JOYMAX 4096


// last raw acceleration readings
float ax_, ay_, az_;

// integrated velocity reading since last get()
float vx_, vy_;

// timer for measuring time between get() samples
unsigned long tGet_;

// timer for measuring time between interrupts
unsigned long tInt_;

// Calibration reference point for accelerometer.  This is the
// average reading on the accelerometer when in the neutral position
// at rest.
float cx_, cy_;

// timer for atuo-centering
unsigned long tCenter_;

// Auto-centering history.  This is a separate history list that
// records results spaced out sparesely over time, so that we can
// watch for long-lasting periods of rest.  When we observe nearly
// no motion for an extended period (on the order of 5 seconds), we
// take this to mean that the cabinet is at rest in its neutral
// position, so we take this as the calibration zero point for the
// accelerometer.  We update this history continuously, which allows
// us to continuously re-calibrate the accelerometer.  This ensures
// that we'll automatically adjust to any actual changes in the
// cabinet's orientation (e.g., if it gets moved slightly by an
// especially strong nudge) as well as any systematic drift in the
// accelerometer measurement bias (e.g., from temperature changes).
int iAccPrv_, nAccPrv_;
#define maxAccPrv  5
AccHist_t accPrv_[maxAccPrv];


// interrupt router
////InterruptIn intIn_;

void reset(void)
{
        // clear the center point
        cx_ = cy_ = 0.0;

        // start the calibration timer
        tCenter_=millis();
        iAccPrv_ = nAccPrv_ = 0;

        // reset and initialize the MMA8451Q
        if (!MMA8451_begin(MMA8451_DEFAULT_ADDRESS, MMA8451_RANGE_2_G)) {
                //FIXME LIMITER LA BOUCLE
                while (1) ;
        }

        // set the initial integrated velocity reading to zero
        vx_ = vy_ = 0;

        // read the current registers to clear the data ready flag
        MMA8451_read();

        // start our timers
        tGet_=millis();
        tInt_=millis();
}

// adjust a raw acceleration figure to a usb report value
int rawToReport(float v)
{
        // scale to the joystick report range and round to integer
        int i = ((int)(round(v*JOYMAX)));

        // if it's near the center, scale it roughly as 20*(i/20)^2,
        // to suppress noise near the rest position
        static const int filter[] = {
                -18, -16, -14, -13, -11, -10, -8, -7, -6, -5, -4, -3, -2, -2, -1, -1, 0, 0, 0, 0,
                0,
                0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 10, 11, 13, 14, 16, 18
        };
        return (i > 20 || i < -20 ? i : filter[i+20]);
}

void get(AccPos_t *pos)
{
        // read the shared data and store locally for calculations
        float ax = ax_, ay = ay_;
        float vx = vx_, vy = vy_;

        // reset the velocity sum for the next run
        vx_ = vy_ = 0;

        // get the time since the last get() sample
        float dt = (millis()-tGet_)/1.0e3;
        tGet_=millis();

        // adjust the readings for the integration time
        vx /= dt;
        vy /= dt;

        // add this sample to the current calibration interval's running total
        AccHist_t *p = accPrv_ + iAccPrv_;
        addAvg(p, ax, ay);

        // check for auto-centering every so often
        if ((millis() - tCenter_) > 1000)
        {
                // add the latest raw sample to the history list
                AccHist_t *prv = p;
                iAccPrv_ = (iAccPrv_ + 1) % maxAccPrv;
                p = accPrv_ + iAccPrv_;
                set(p, ax, ay, prv);

                // if we have a full complement, check for stability
                if (nAccPrv_ >= maxAccPrv)
                {
                        // check if we've been stable for all recent samples
                        static const float accTol = .01;
                        AccHist_t *p0 = accPrv_;
                        if (p0[0].d < accTol
                                        && p0[1].d < accTol
                                        && p0[2].d < accTol
                                        && p0[3].d < accTol
                                        && p0[4].d < accTol)
                        {
                                // Figure the new calibration point as the average of
                                // the samples over the rest period
                                cx_ = (xAvg(p0[0]) + xAvg(p0[1]) + xAvg(p0[2]) + xAvg(p0[3]) + xAvg(p0[4]))/5.0;
                               cy_ = (yAvg(p0[0]) + yAvg(p0[1]) + yAvg(p0[2]) + yAvg(p0[3]) + yAvg(p0[4]))/5.0;
                        }
                }
                else
                {
                        // not enough samples yet; just up the count
                        ++nAccPrv_;
                }

                // clear the new item's running totals
                clearAvg(p);

                // reset the timer
                tCenter_=millis();
        }

        // report our integrated velocity reading in x,y
        pos->x = rawToReport(vx);
        pos->y = rawToReport(vy);

#ifdef DEBUG_PRINTF
        if (pos->x != 0 || pos->y != 0)
                printf("=> %f %f %d %d %f\r\n", vx, vy, pos->x, pos->y, dt);
#endif
}

void UpdateMMA8451_NUDGE(void){
        // Read the axes.  Note that we have to read all three axes
        // (even though we only really use x and y) in order to clear
        // the "data ready" status bit in the accelerometer.  The
        // interrupt only occurs when the "ready" bit transitions from
        // off to on, so we have to make sure it's off.
        MMA8451_read();
        float x, y, z;
        x = MMA8451_x_g;
        y = MMA8451_y_g;
        z = MMA8451_z_g;
        // calculate the time since the last interrupt
        float dt = (millis()-tInt_)/1.0e3;
        tInt_=millis();

        // integrate the time slice from the previous reading to this reading
        vx_ += (x + ax_ - 2*cx_)*dt/2;
        vy_ += (y + ay_ - 2*cy_)*dt/2;

        // store the updates
        ax_ = x;
        ay_ = y;
        az_ = z;
}



static uint8_t IsAccelGyroCode(uint8_t key)
{
	//TODO FIXME
	//return UpdateMMA8451_NUDGE();	
	return 1;	
}

int8_t map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

static uint8_t ReportAccelGyro()
{
        uint8_t id = ID_AccelGyro;
	int x = 0, y = 0;

	UpdateMMA8451_NUDGE();

	AccPos_t pos;pos.x=pos.y=0;
	get(&pos);
	int xa=pos.x, ya=pos.y;
	// confine the results to our joystick axis range
	if (xa < -JOYMAX) xa = -JOYMAX;
	if (xa > JOYMAX) xa = JOYMAX;
	if (ya < -JOYMAX) ya = -JOYMAX;
	if (ya > JOYMAX) ya = JOYMAX;

	// store the updated accelerometer coordinates
	x = xa;
	y = ya;
        int8_t joy_x = 0;
	int8_t joy_y = 0;
	int8_t joy_z = 0;
        int8_t joy_rx = 0;
        int8_t joy_ry = 0;
        int8_t joy_rz = 0;
	uint8_t joy_b = 0;

	#if defined(ENABLE_ANALOG_INPUT) && defined(ADC_MAPPING_TABLE)
	#define MAP(port, pin, mux, minval, maxval, joyid, axis) \
                if ((axis == 0) && (joyid == id)) { joy_x  = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
                if ((axis == 1) && (joyid == id)) { joy_y  = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
                if ((axis == 2) && (joyid == id)) { joy_z  = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
                if ((axis == 3) && (joyid == id)) { joy_rx = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
                if ((axis == 4) && (joyid == id)) { joy_ry = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
                if ((axis == 5) && (joyid == id)) { joy_rz = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); }	
	ADC_MAPPING_TABLE(MAP)
	#undef MAP
	#endif


	if((millis()-start>10000) &&(abs(x) > 100 || abs(y)>100)){
		joy_x=map(JOY_X(x,y),-JOYMAX,JOYMAX,-128,127);
		joy_y=map(JOY_Y(x,y),-JOYMAX,JOYMAX,-128,127);
	}else{
		joy_x=joy_y=0;
	}

        ReportBuffer[0] = id;

        ReportBuffer[1] = joy_x;
        ReportBuffer[2] = joy_y;
        ReportBuffer[3] = joy_z;
        ReportBuffer[4] = joy_rx;
        ReportBuffer[5] = joy_ry;
        ReportBuffer[6] = joy_rz;
        ReportBuffer[7] = joy_b;

        return 8;
}

#endif

#if (USE_MOUSE != 0)
static uint8_t need_mouse_update = 0;
static uint8_t mouse_x_last_clk_state = 0;
static uint8_t mouse_x_last_dir_state = 0;
static int8_t mouse_x_count = 0;
static uint8_t mouse_y_last_clk_state = 0;
static uint8_t mouse_y_last_dir_state = 0;
static int8_t mouse_y_count = 0;
#if !defined(MOUSE_X_DELTA)
#define MOUSE_X_DELTA 1
#endif
#if !defined(MOUSE_Y_DELTA)
#define MOUSE_Y_DELTA 1
#endif
#endif


// Shift switch off
PROGMEM const uint8_t NormalMapping[NUMBER_OF_INPUTS] =
{ 
	#define MAP(port, pin, normal_id, shift_id) normal_id,
	PANEL_MAPPING_TABLE(MAP)
	#undef MAP
};

// Shift switch on
PROGMEM const uint8_t ShiftMapping[NUMBER_OF_INPUTS] =
{
	#define MAP(port, pin, normal_id, shift_id) shift_id,
	PANEL_MAPPING_TABLE(MAP)
	#undef MAP
};


#define IsKeyDown(index) (InputState[index] & 0x80)

static uint8_t GetKeyNormalMap(unsigned char index) { return (index < NUMBER_OF_INPUTS) ? pgm_read_byte(NormalMapping + index) : 0; }
static uint8_t GetKeyShiftMap(unsigned char index) { return (index < NUMBER_OF_INPUTS) ? pgm_read_byte(ShiftMapping + index): 0; }
static uint8_t IsKeyboardCode(uint8_t key) { return (key >= KEY_A) && (key <= MOD_RightGUI); }
static uint8_t IsModifierCode(uint8_t key) { return (key >= MOD_LeftControl) && (key <= MOD_RightGUI); }
static uint8_t IsConsumerCode(uint8_t key) { return (key >= AC_VolumeUp) && (key <= AC_Mute); }
static uint8_t GetKey(uint8_t index) { return (shift_key != 0) ? GetKeyShiftMap(index) : GetKeyNormalMap(index); }


#if (NUM_JOYSTICKS >= 1)

static uint8_t IsJoystickCode(uint8_t key, uint8_t joy)
{
	key -= (joy * NR_OF_EVENTS_PER_JOY);

	return (key >= J1_Left) && (key < (J1_Left + NR_OF_EVENTS_PER_JOY));
}

static uint8_t NeedJoystickUpdate(void)
{
	uint8_t i;

	for (i = 0; i < NUM_JOYSTICKS; i++)
	{
		if (need_joystick_update[i]) {
			return 1;
		}
	}

	return 0;
}

#endif

#if (USE_ACCELGYRO) 

static uint8_t IsAccelGyroCode(uint8_t key)
{
	key -= (4 * NR_OF_EVENTS_PER_JOY);

	return (key >= J1_Left) && (key < (J1_Left + NR_OF_EVENTS_PER_JOY));
}

#endif

#if (USE_MOUSE != 0)

static uint8_t IsMouseButtonCode(uint8_t key)
{
	return (key >= MB_Left) && (key <= MB_Middle);
}

static void MouseMoveX(uint8_t direction)
{
	if (direction)
	{
		if (mouse_x_count > -(127 - MOUSE_X_DELTA)) {
		    mouse_x_count -= MOUSE_X_DELTA;
		}
	}
	else
	{
		if (mouse_x_count < (127 - MOUSE_X_DELTA)) {
		    mouse_x_count += MOUSE_X_DELTA;
		}
	}

	need_mouse_update = 1;
}

static void MouseMoveY(uint8_t direction)
{
	if (direction)
	{
		if (mouse_y_count > -(127 - MOUSE_Y_DELTA)) {
		    mouse_y_count -= MOUSE_Y_DELTA;
		}
	}
	else
	{
		if (mouse_y_count < (127 - MOUSE_Y_DELTA)) {
		    mouse_y_count += MOUSE_Y_DELTA;
		}
	}

	need_mouse_update = 1;
}

static void CheckMouseUpdate(void)
{
	#if defined(MOUSE_X_CLK_INDEX) && defined(MOUSE_X_DIR_INDEX)

	uint8_t mouse_clk_state = InputState[MOUSE_X_CLK_INDEX];
	uint8_t mouse_dir_state = InputState[MOUSE_X_DIR_INDEX];

	if (mouse_clk_state != mouse_x_last_clk_state)
	{
		if (mouse_dir_state == mouse_x_last_dir_state) {
			MouseMoveX(mouse_clk_state ^ mouse_dir_state);
		}

		mouse_x_last_clk_state = mouse_clk_state;
	}

	if (mouse_dir_state != mouse_x_last_dir_state)
	{
		if (mouse_clk_state == mouse_x_last_clk_state) {
			MouseMoveX(!(mouse_clk_state ^ mouse_dir_state));
		}

		mouse_x_last_dir_state = mouse_dir_state;
	}

	#endif

	#if defined(MOUSE_Y_CLK_INDEX) && defined(MOUSE_Y_DIR_INDEX)

	mouse_clk_state = InputState[MOUSE_Y_CLK_INDEX];
	mouse_dir_state = InputState[MOUSE_Y_DIR_INDEX];

	if (mouse_clk_state != mouse_y_last_clk_state)
	{
		if (mouse_dir_state == mouse_y_last_dir_state) {
			MouseMoveY(mouse_clk_state ^ mouse_dir_state);
		}

		mouse_y_last_clk_state = mouse_clk_state;
	}

	if (mouse_dir_state != mouse_y_last_dir_state)
	{
		if (mouse_clk_state == mouse_y_last_clk_state) {
			MouseMoveY(!(mouse_clk_state ^ mouse_dir_state));
		}

		mouse_y_last_dir_state = mouse_dir_state;
	}

	#endif
}

static uint8_t NeedMouseUpdate(void) { return need_mouse_update; }

#endif

void panel_init(void)
{
#if (NUM_JOYSTICKS >= 1)
	memset(&need_joystick_update[0], 0x00, sizeof(need_joystick_update));
#endif

#define MAP(port, pin, normal_id, shift_id) \
	PORT##port |= (1 << pin); \
	DDR##port &= ~(1 << pin);
	PANEL_MAPPING_TABLE(MAP)
#undef MAP

#if defined(ENABLE_ANALOG_INPUT) && defined(ADC_MAPPING_TABLE)
#define MAP(port, pin, mux, minval, maxval, joyid, axis) \
		PORT##port &= ~(1 << pin); \
		DDR##port &= ~(1 << pin);
		ADC_MAPPING_TABLE(MAP)
#undef MAP

		ADC_init();
#endif
#if (ENABLE_MMA8451_NUDGE)
	// CTC mode, Clock/8
	TCCR3B |= (1 << WGM12) | (1 << CS31);

	// Load the high byte, then the low byte
	// into the output compare
	OCR3AH = (CTC_MATCH_OVERFLOW >> 8);
	OCR3AL = CTC_MATCH_OVERFLOW;

	// Enable the compare match interrupt
	TIMSK3 |= (1 << OCIE3A);
	// Now enable global interrupts
	sei();
	if (!MMA8451_begin(MMA8451_DEFAULT_ADDRESS, MMA8451_RANGE_2_G)) {
		while (1) ;
	}
	_delay_ms(250);
	MMA8451_read();

	reset();

	// last accelerometer report, in joystick units (we report the nudge
	// acceleration via the joystick x & y axes, per the VP convention)

	start = millis();

#endif
}

static void SetNeedUpdate(uint8_t index)
{
	uint8_t key = GetKey(index);

	if (IsConsumerCode(key))
	{
		need_consumer_update = 1;
		return;
	}

	if (IsKeyboardCode(key))
	{
		need_key_update = 1;
		return;
	}

	#if (USE_MOUSE != 0)
	if (IsMouseButtonCode(key))
	{
		need_mouse_update = 1;
		return;
	}
	#endif

	#if (NUM_JOYSTICKS >= 1)
	{
		uint8_t i;

		for (i = 0; i < NUM_JOYSTICKS; i++)
		{
			if (IsJoystickCode(key, i))
			{
				need_joystick_update[i] = 1;
				return;
			}
		}
	}
	#endif

	#if (USE_ACCELGYRO) || (ENABLE_MMA8451_NUDGE)
	if (IsAccelGyroCode(key))
	{
		need_accelgyro_update = 1;
		return;
	}
	#endif
}

#if defined(SHIFT_SWITCH_INDEX)

static void ShiftKeyCleanUp(void)
{
	if (shift_key_cleanup == 1)
	{
		uint8_t i;
		shift_key_cleanup = 2;

		for (i = 0; i < NUMBER_OF_INPUTS; i++)
		{
			if (i != SHIFT_SWITCH_INDEX)
			{
				if (InputState[i] != 0)
				{
					if (GetKeyNormalMap(i) != GetKeyShiftMap(i))
					{
						SetNeedUpdate(i);
						InputState[i] = 0;
					}
				}
			}
		}
	}

	if (shift_key_cleanup == 2)
	{
		bool no_update = !need_consumer_update && !need_key_update;

		#if (USE_MOUSE != 0)
		no_update = no_update && !NeedMouseUpdate();
		#endif

		#if (NUM_JOYSTICKS >= 1)
		no_update = no_update && !NeedJoystickUpdate();
		#endif

		#if (USE_ACCELGYRO) || (ENABLE_MMA8451_NUDGE)
		no_update = no_update && !need_accelgyro_update;
		#endif

		if (no_update)
		{
			shift_key_cleanup = 0;
			shift_key = IsKeyDown(SHIFT_SWITCH_INDEX);
		}
	}
}

#endif

static uint8_t NeedUpdate(void)
{
	#if defined(SHIFT_SWITCH_INDEX)
	ShiftKeyCleanUp();
	#endif

	#if (USE_MOUSE != 0)
	if (NeedMouseUpdate())
	{
		need_mouse_update = 0;
		return ID_Mouse;
	}
	#endif

	if (need_key_update)
	{
		need_key_update = 0;
		return ID_Keyboard;
	}

	if (need_consumer_update)
	{
		need_consumer_update = 0;
		return ID_Consumer;
	}

	uint8_t analog_update[NUM_JOYSTICKS + 1] = {0};
	static uint8_t analog_counter[NUM_JOYSTICKS + 1];

	#if (NUM_JOYSTICKS >= 1)
	for (uint8_t i = 0; i < NUM_JOYSTICKS; i++)
	{
		if (analog_counter[i] < 0xFF && need_joystick_update[i])
		    analog_counter[i] += 1;
	}
	#endif

	#if (USE_ACCELGYRO) || (ENABLE_MMA8451_NUDGE)
	if (analog_counter[NUM_JOYSTICKS] < 0xFF && need_accelgyro_update)
		analog_counter[NUM_JOYSTICKS] += 1;
	#endif

	#if defined(ENABLE_ANALOG_INPUT) && defined(ADC_MAPPING_TABLE)

	#define MAP(port, pin, mux, minval, maxval, joyid, axis) \
	if (joyid >= ID_Joystick1 && (joyid - ID_Joystick1) < NUM_JOYSTICKS) { analog_update[joyid - ID_Joystick1] = 1; } else \
	if (joyid == ID_AccelGyro) { analog_update[NUM_JOYSTICKS] = 1; }
	ADC_MAPPING_TABLE(MAP)
	#undef MAP

	for (uint8_t i = 0; i < sizeof(analog_update) / sizeof(analog_update[0]); i++)
	{
		if (analog_counter[i] < 0xFF && analog_update[i] > 0)
		    analog_counter[i] += 1;
	}

	#endif

	uint8_t ac_max = 0;
	int8_t index_max = -1;

	for (int8_t i = 0; i < sizeof(analog_update) / sizeof(analog_update[0]); i++)
	{
		if (ac_max < analog_counter[i])
		{
			ac_max = analog_counter[i];
			index_max = i;
		}
	}

	if (index_max >= 0)
	{
		analog_counter[index_max] = 0;

		#if (NUM_JOYSTICKS >= 1)
		if (index_max < NUM_JOYSTICKS) {
			need_joystick_update[index_max] = 0;
			return index_max + ID_Joystick1;
		}
		#endif

		#if (USE_ACCELGYRO) || (ENABLE_MMA8451_NUDGE)
		need_accelgyro_update = 0;
		return ID_AccelGyro;
		#endif
	}

	return ID_Unknown;
}

static void SetInputCount(uint8_t index, uint8_t condition)
{
	#if (USE_MOUSE != 0) && defined(MOUSE_X_CLK_INDEX) && defined(MOUSE_X_DIR_INDEX)
	if ((index == MOUSE_X_CLK_INDEX) || (index == MOUSE_X_DIR_INDEX))
	{
		InputState[index] = condition;
		return;
	}
	#endif

	#if (USE_MOUSE != 0) && defined(MOUSE_Y_CLK_INDEX) && defined(MOUSE_Y_DIR_INDEX)
	if ((index == MOUSE_Y_CLK_INDEX) || (index == MOUSE_Y_DIR_INDEX))
	{
		InputState[index] = condition;
		return;
	}
	#endif

	if (index >= NUMBER_OF_INPUTS)
	{
		return;
	}

	#if defined(MULTIFIRE_INDEX)
	if (index == MULTIFIRE_INDEX)
	{
		static uint16_t ncycle = 0;
		static uint8_t ncount = 0;
		static uint8_t ndelay = 0;

		// simple debounce
		if (ndelay == 0 && condition)
		{
			ndelay = (100 / (DELTA_TIME_PANEL_REPORT_MS + 1));
			ncount += MULTIFIRE_COUNT;
		}
		else if (ndelay > 1)
		{
			ndelay--;
		}
		else if (ndelay == 1)
		{
			if (!condition)
				ndelay = 0;
		}

		condition = 0;

		// state machine to generate multiple events
		if (ncount > 0)
		{
			condition = (ncycle < (100 / (DELTA_TIME_PANEL_REPORT_MS + 1))) ? 1 : 0;

			if (ncycle >= (600 / (DELTA_TIME_PANEL_REPORT_MS + 1)))
			{
				ncycle = 0;
				ncount -= 1;
			}
			else
			{
				ncycle += 1;
			}
		}
	}
	#endif

	uint8_t changed = 0;
	uint8_t count = InputState[index];
	uint8_t state = count & 0x80;
	count &= 0x7f;

	if (condition)
	{
		if (count <= DEBOUNCE)
		{
			if ((count == DEBOUNCE) && !state)
			{
				changed = 1;
				state = 0x80;
			}

			count++;
		}
		else
		{
			return;
		}
	}
	else
	{
		if (count > 0)
		{
			if ((count == 1) && state)
			{
				changed = 1;
				state = 0;
			}

			count--;
		}
		else
		{
			return;
		}
	}

	InputState[index] = state | count;

	if (changed)
	{
		#if defined(SHIFT_SWITCH_INDEX)
		if (index == SHIFT_SWITCH_INDEX)
		{
			shift_key_cleanup = 1;
		}
		else
		#endif
		{
			SetNeedUpdate(index);
		}
	}
}

void panel_ScanInput(void)
{
	if (shift_key_cleanup) {
		return;
	}

	#define MAP(port, pin, normal_id, shift_id) SetInputCount(port##pin##_index, 0 == (PIN##port & (1 << pin)));
	PANEL_MAPPING_TABLE(MAP)
	#undef MAP

	#if (USE_MOUSE != 0)
	CheckMouseUpdate();
	#endif
}

#if (USE_CONSUMER != 0)
static uint8_t ReportConsumer(void)
{
	uint8_t i;
	uint8_t consumer = 0;

	for (i = 0; i < NUMBER_OF_INPUTS; i++)
	{
		if (IsKeyDown(i))
		{
			uint8_t key = GetKey(i);

			if (IsConsumerCode(key)) {
				consumer |= ConsumerBit(key);
			}
		}
	}

	ReportBuffer[0] = ID_Consumer;
	ReportBuffer[1] = consumer;

	return 2;
}
#endif

static uint8_t ReportKeyboard(void)
{
	uint8_t i;
	uint8_t r = 2;

	memset(&ReportBuffer[1], 0x00, sizeof(ReportBuffer) - 1);

	ReportBuffer[0] = ID_Keyboard;

	for (i = 0; i < NUMBER_OF_INPUTS; i++)
	{
		if (IsKeyDown(i))
		{
			uint8_t key = GetKey(i);

			if (IsKeyboardCode(key))
			{
				if (IsModifierCode(key))
				{
					ReportBuffer[1] |= ModifierBit(key);
				}
				else
				{
					if (r < sizeof(ReportBuffer))
					{
						switch (key)
						{
						case KM_ALT_F4:
							ReportBuffer[1] |= ModifierBit(MOD_LeftAlt);
							ReportBuffer[r] = KEY_F4;
							break;
						case KM_SHIFT_F7:
							ReportBuffer[1] |= ModifierBit(MOD_LeftShift);
							ReportBuffer[r] = KEY_F7;
							break;
						default:
							ReportBuffer[r] = key;
							break;
						}

						r++;
					}
				}
			}
		}
	}

	return sizeof(ReportBuffer);
}

#if (NUM_JOYSTICKS >= 1)

static uint8_t ReportJoystick(uint8_t id)
{
	uint8_t i;
	int16_t joy_x = 0;
	int16_t joy_y = 0;
	uint8_t joy_b = 0;

	ReportBuffer[0] = id;

	#if defined(ENABLE_ANALOG_INPUT) && defined(ADC_MAPPING_TABLE)
	#define MAP(port, pin, mux, minval, maxval, joyid, axis) \
		if ((axis == 0) && (joyid == id)) { joy_x = joyval12(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 4094), (int16_t)(maxval * 4094)); } \
		if ((axis == 1) && (joyid == id)) { joy_y = joyval12(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 4094), (int16_t)(maxval * 4094)); }
	ADC_MAPPING_TABLE(MAP)
	#undef MAP
	#endif

	id = (id - ID_Joystick1) * NR_OF_EVENTS_PER_JOY;

	for (i = 0; i < NUMBER_OF_INPUTS; i++)
	{
		if (IsKeyDown(i))
		{
			uint8_t key = GetKey(i) - id;
			switch (key)
			{
			case J1_Left:
				joy_x = -2047;
				break;
			case J1_Right:
				joy_x = +2047;
				break;
			case J1_Up:
				joy_y = -2047;
				break;
			case J1_Down:
				joy_y = +2047;
				break;

			case J1_Button1:
			case J1_Button2:
			case J1_Button3:
			case J1_Button4:
			case J1_Button5:
			case J1_Button6:
			case J1_Button7:
			case J1_Button8:
				joy_b |= JoyButtonBit(key);
				break;

			default:
				break;
			}
		}
	}

	ReportBuffer[1] = ((uint16_t)joy_x & 0xFF);
	ReportBuffer[2] = (((uint16_t)joy_y & 0x0F) << 4) | (((uint16_t)joy_x >> 8) & 0x0F);
	ReportBuffer[3] = (((uint16_t)joy_y >> 4) & 0xFF);
	ReportBuffer[4] = joy_b;

	return 5;
}

#endif

#if (USE_ACCELGYRO)

static uint8_t ReportAccelGyro()
{
	uint8_t id = ID_AccelGyro;

	int8_t joy_x = 0;
	int8_t joy_y = 0;
	int8_t joy_z = 0;
	int8_t joy_rx = 0;
	int8_t joy_ry = 0;
	int8_t joy_rz = 0;
	uint8_t joy_b = 0;

	ReportBuffer[0] = id;

	#if defined(ENABLE_ANALOG_INPUT) && defined(ADC_MAPPING_TABLE)
	#define MAP(port, pin, mux, minval, maxval, joyid, axis) \
		if ((axis == 0) && (joyid == id)) { joy_x  = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
		if ((axis == 1) && (joyid == id)) { joy_y  = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
		if ((axis == 2) && (joyid == id)) { joy_z  = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
		if ((axis == 3) && (joyid == id)) { joy_rx = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
		if ((axis == 4) && (joyid == id)) { joy_ry = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); } \
		if ((axis == 5) && (joyid == id)) { joy_rz = joyval8(ADC_getvalue(port##pin##_adcindex), (int16_t)(minval * 254), (int16_t)(maxval * 254)); }
	ADC_MAPPING_TABLE(MAP)
	#undef MAP
	#endif

	id = (id - ID_Joystick1) * NR_OF_EVENTS_PER_JOY;

	for (uint8_t i = 0; i < NUMBER_OF_INPUTS; i++)
	{
		if (IsKeyDown(i))
		{
			uint8_t key = GetKey(i) - id;
			switch (key)
			{
			case J1_Left:
				joy_x = -127;
				break;
			case J1_Right:
				joy_x = +127;
				break;
			case J1_Up:
				joy_y = -127;
				break;
			case J1_Down:
				joy_y = +127;
				break;

			case J1_Button1:
			case J1_Button2:
			case J1_Button3:
			case J1_Button4:
			case J1_Button5:
			case J1_Button6:
			case J1_Button7:
			case J1_Button8:
				joy_b |= JoyButtonBit(key);
				break;

			default:
				break;
			}
		}
	}

	ReportBuffer[1] = joy_x;
	ReportBuffer[2] = joy_y;
	ReportBuffer[3] = joy_z;
	ReportBuffer[4] = joy_rx;
	ReportBuffer[5] = joy_ry;
	ReportBuffer[6] = joy_rz;
	ReportBuffer[7] = joy_b;

	return 8;
}

#endif


#if (USE_MOUSE != 0)

static uint8_t ReportMouse(void)
{
	uint8_t i;
	uint8_t buttons = 0;

	for (i = 0; i < NUMBER_OF_INPUTS; i++)
	{
		if (IsKeyDown(i))
		{
			uint8_t key = GetKey(i);

			if (IsMouseButtonCode(key)) {
				buttons |= MouseButtonBit(key);
			}
		}
	}

	ReportBuffer[0] = ID_Mouse;
	ReportBuffer[1] = buttons;
	ReportBuffer[2] = mouse_x_count;
	ReportBuffer[3] = mouse_y_count;
	mouse_x_count = 0;
	mouse_y_count = 0;

	return 4;
}

#endif


static uint8_t BuildReport(uint8_t id)
{
	switch (id)
	{
	#if (USE_KEYBOARD != 0)
	case ID_Keyboard:
		return ReportKeyboard();
	#endif

	#if (USE_CONSUMER != 0)
	case ID_Consumer:
		return ReportConsumer();
	#endif

	#if (NUM_JOYSTICKS >= 1)
	case ID_Joystick4:
	case ID_Joystick3:
	case ID_Joystick2:
	case ID_Joystick1:
		return ReportJoystick(id);
	#endif

	#if (USE_ACCELGYRO) || (ENABLE_MMA8451_NUDGE)
	case ID_AccelGyro:
		return ReportAccelGyro(id);
	#endif

	#if (USE_MOUSE != 0)
	case ID_Mouse:
		return ReportMouse();
	#endif

	default:
		break;
	}

	return 0;
}

uint8_t panel_get_report(uint8_t **ppdata)
{
	if (ppdata == NULL) {
		return 0;
	}

	static uint16_t time_next_ms = 0;
	uint16_t const time_curr_ms = clock_ms();

	if (((int16_t)time_curr_ms - (int16_t)time_next_ms) < 0) {
		return 0;
	}

	time_next_ms = time_curr_ms + DELTA_TIME_PANEL_REPORT_MS;

	panel_ScanInput();

	uint8_t const id = NeedUpdate();

	if (id == ID_Unknown) {
		return 0;
	}

	uint8_t const ndata = BuildReport(id);
	*ppdata = ReportBuffer;

	return ndata;
}


#if defined(ENABLE_ANALOG_INPUT)

// ADC Interrupt Routine

ISR(ADC_vect)
{
	#if defined(ENABLE_PROFILING)
	profile_start();
	#endif

	static int i = 0;

	// get value

	adc_values[i] = ADC;

	// cycle

	i -= 1;

	if (i < 0) 
		i += NUM_ADC_CHANNELS;

	// set mux channel for the next conversion

	ADC_setmux(adc_mux_table[i]);

	// start new conversion

	ADCSRA |= (1 << ADSC);
}

#endif



#endif
