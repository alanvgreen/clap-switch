/*
 * Copyright 2019 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <atmel_start.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <utils/atomic.h>

//////////////////////////////////////////////////////////////////////
// Current tick updates 1024 times per second - approximately equal
// to a millisecond

// Current tick
uint32_t tick_millis;

// ISR to update the tick
// Fires every ms. May be delayed if running LED update code because that
// disallows interrupts.
ISR(RTC_PIT_vect)
{
	tick_millis++;

	// Clear interrupt flag to indicate that interrupt has been handled.
	RTC.PITINTFLAGS = RTC_PI_bm;
}

//////////////////////////////////////////////////////////////////////
// Configuration related routines

// Hue of light - represents entire 360 degrees in 6 regions, 
// each with 32 steps = 192 valid hues. Storing in 6 regions simplifies
// calculations
// TODO: just store as an int8_t - it will simplify the increment code greatly.
typedef struct {
	unsigned region : 3; // range 0 - 5
	unsigned val : 5;    // range 0 - 31
} Hue;

#define MAX_BRIGHT 64

// Values to save in EEPROM
typedef struct {
	// Whether lamp is on
	bool on : 1;
	// Brightness range 1-MAX_BRIGHT
	uint8_t brightness; 
	// Hue to take
	Hue hue;
} Eeprom;
Eeprom config;

void readConfig() {
	FLASH_0_read_eeprom_block(0, (uint8_t *) &config, sizeof(Eeprom));
	if (config.brightness < 1 || config.brightness > MAX_BRIGHT) {
		config.brightness = 8;
	}
}

// Last time the config was changed
uint32_t config_change_millis;

#define CONFIG_WAIT_MS 500

// Whether config has been written since last change
bool config_written = true;

// Whether LEDS have been updated to reflect new config
bool leds_updated = false;

void configChanged() {
	config_change_millis = tick_millis;
	config_written = false;
	leds_updated = false;
}

// Write the config, if necessary
void maybeWriteConfig() {
	if (config_written && tick_millis > config_change_millis + CONFIG_WAIT_MS) {
		FLASH_0_write_eeprom_block(0, (uint8_t *) &config, sizeof(Eeprom));
		config_written = true;
	}
}

// Updates brightness
// in must be in (-1, 0 +1)
// Increments brightness if in is +1, decrements brightness if input is -1
void updateBrightness(int8_t in) {
	if (in == -1 && config.brightness >= 1) {
		config.brightness--;
		configChanged();
	}
	if (in == 1 && config.brightness < MAX_BRIGHT) {
		config.brightness++;
		configChanged();
	}
}

// Updates hue
// in must be in (-1, 0 +1)
// Increments hue if in is +1, decrements hue if input is -1
// Return true if hue was updated
void updateHue(int8_t in) {
	if (in == -1) {
		if (config.hue.val == 0) {
			config.hue.val = 31;
			if (config.hue.region == 0) {
				config.hue.region = 5;
			} else {
				config.hue.region--;
			}
		} else {
			config.hue.val--;
		}
		configChanged();
	}
	if (in == 1) {
		if (config.hue.val == 31) {
			config.hue.val = 0;
			if (config.hue.region == 5) {
				config.hue.region = 0;
			} else {
				config.hue.region++;
			}
		} else {
			config.hue.val++;
		}
		configChanged();
	}
}


// toggle whether lamp is on
void toggleOn() {
	config.on = !config.on;
	configChanged();
}


//////////////////////////////////////////////////////////////////////
// Track whether controls were touched.
// Touches are often detected as a loud noise through the microphone.

// Last manual control input time
uint32_t last_touched_millis;

// Updates last_touched_millis if in shows that a control was touched
int8_t checkTouch(int8_t in) {
	if (in != 0) {
		last_touched_millis = tick_millis;
	}
	return in;
}

// How long to wait after a touch to begin detecting claps
#define TOUCH_TO_CLAP_MIN_MS 500

// How long to wait after a clap to be sure it wasn't a touch
#define CLAP_TO_TOUCH_MIN_MS 100

// Minimum and maximum time between claps
#define CLAP_INTER_MIN 200
#define CLAP_INTER_MAX 700

// Maximum length of a clap. If a clap goes longer than this,
// we assume it was some other noise
#define CLAP_LEN_MAX 100

// Threshold for ADC to decide it was a clap
#define CLAP_THRESHOLD 300

typedef enum {
	NO_CLAP, // reset - waiting for a cap
	FIRST_CLAP, // have a first clap - waiting for second or touch
	DOUBLE_CLAP, // have a second clap - waiting for touch timeout
} ClapState;
ClapState clap_state;

// Last clap detect time
uint32_t last_clap_millis;

// Average accumulator - upper 16 bits hold the average
uint32_t mic_avg_level_accum;

// Average total energy - 10 bits
uint32_t mic_avg_energy_accum;

// Last energy
uint8_t mic_last_energy;

// Returns 0..1023 -- actually only 3/4 of the range due to 
// 1.8V max, measured against 2.5 V reference
uint16_t micRawRead() {
	return ADC_0_get_conversion(ADC_MUXPOS_AIN6_gc);
}

// Get top 16 bits of avg accumulator
static inline uint16_t micGetAvgLevel() {
	return mic_avg_level_accum >> 16;
}

// Take 256 readings over quarter of a second
void micCalibrateAvgLevel() {
	uint32_t total = 0;
	for (uint16_t i = 0; i < 256; i++) {
		total += micRawRead();
		_delay_ms(1);
	}
	mic_avg_level_accum = total * 256;
}

// Update with new reading
void micUpdateAvgLevel(uint16_t reading) {
	mic_avg_level_accum -= micGetAvgLevel();
	mic_avg_level_accum += reading;
}

// Detects double claps - returns true on double-clap
bool clapDetect(uint16_t reading) {
	// Any touch forces reset for TOUCH_TO_CLAP_MIN_MS
	if (tick_millis < last_touched_millis + TOUCH_TO_CLAP_MIN_MS) {
		clap_state = NO_CLAP;
		return false;
	}
	
	// Are we detecting a clap right now?
	uint16_t avg = micGetAvgLevel();
	bool detected = reading <= avg - CLAP_THRESHOLD || reading >= avg + CLAP_THRESHOLD;
	
	// What to do depends on current state, the time, and whether a 
	// clap has been detected
	if (clap_state == NO_CLAP) {
		if (detected) {
			clap_state = FIRST_CLAP;
			last_clap_millis = tick_millis;
		}
		return false;
	} else if (clap_state == FIRST_CLAP) {
		if (tick_millis < last_clap_millis + CLAP_INTER_MIN) {
			// Just waiting
		} else if (tick_millis < last_clap_millis + CLAP_INTER_MAX) {
			if (detected) {
				clap_state = DOUBLE_CLAP;
				last_clap_millis = tick_millis;
			}
		} else {
			// Timed out with no new clap detected
			clap_state = NO_CLAP;
		}
		return false;
	} else { // clap_state = DOUBLE_CLAP
		if (tick_millis > last_clap_millis + CLAP_TO_TOUCH_MIN_MS) {
			// Timed out without touch - must be a clap
			clap_state = NO_CLAP;
			return true;
		}
		return false;
	}
}

// Read mic, run processing return true if double-clap detected
bool micRead() {
	uint16_t reading = micRawRead();
	micUpdateAvgLevel(reading);
	return clapDetect(reading);
}



//////////////////////////////////////////////////////////////////////
// Read controls on UI

// Encoder algorithm from
// https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/
// Sadly, this domain seems to have expired :/

// Look up table for intepreting encoder state transitions.
// Index is (last_encoder_reading << 2 | // current_reading)
static const int8_t enc_states [] PROGMEM = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// Returns -1 on turn one way, 1, the other way and zero for no change.
int8_t readEncoder1() {
	static uint8_t last = 0;
	uint8_t curr = (ENC_1A_get_level() ? 2 : 0) | (ENC_1B_get_level() ? 1 : 0);
	int8_t result = pgm_read_byte(&(enc_states[last << 2 | curr]));
	last = curr;
	return result;
}

// Same, but for encoder 2
int8_t readEncoder2() {
	static uint8_t last = 0;
	uint8_t curr = (ENC_2A_get_level() ? 2 : 0) | (ENC_2B_get_level() ? 1 : 0);
	int8_t result = pgm_read_byte(&(enc_states[last << 2 | curr]));
	last = curr;
	return result;
}

// Read button - -1 = pressed, 1 = released, 0 = no change
int8_t readButton() {
	static bool last = true;
	static uint32_t last_change;
	bool curr = BUTTON_get_level();
	if (curr == last) {
		return 0;
	}
	// Ignore changes with 20ms of previous change
	if (tick_millis - last_change < 20) {
		return 0;
	}
	last = curr;
	last_change = tick_millis;
	return curr ? -1 : 1;
}

//////////////////////////////////////////////////////////////////////
// Convert LED to RGB and set LEDs to match

// Send a single byte out to WS2812b LEDs on PB0
// Assumes 20MHz CPU and that interrupts are disabled
void sendByte(uint8_t v) {
	// Working with PB0
	register uint8_t on = VPORTB_OUT | 1;
	register uint8_t off = VPORTB_OUT & 254;
	for (uint8_t i = 0; i < 8; i++) {
		if (v & 0x80) {
			VPORTB_OUT = on;
			__builtin_avr_delay_cycles(13); // 0.65uS
			VPORTB_OUT = off;
			__builtin_avr_delay_cycles(8); // 0.4uS
			} else {
			VPORTB_OUT = on;
			__builtin_avr_delay_cycles(6); //0.3uS
			VPORTB_OUT = off;
			__builtin_avr_delay_cycles(15); // 0.75uS
			
		}
		v <<= 1;
	}
}

// Sends RG&B values in correct order to LEDs
void sendLeds(uint8_t r, uint8_t g, uint8_t b) {
	DISABLE_INTERRUPTS();
	for (uint8_t i = 0; i < 8; i++) {
		sendByte(g);
		sendByte(b);
		sendByte(r);
	}
	ENABLE_INTERRUPTS();
}

// Utility functions
static inline uint16_t min(uint16_t a, uint16_t b) { return a < b ? a : b; }
static inline uint16_t max(uint16_t a, uint16_t b) { return a > b ? a : b; }

// Calculate RGB and send to LEDs
void maybeUpdateLeds() {
	if (leds_updated) {
		return;
	}
	if (config.on) {
		// https://en.wikipedia.org/wiki/HSL_and_HSV#HSL_to_RGB
		// v is L (lowercase l looks like 1 and is confusing). 
		// v range 0-255 instead of 0-1. 
		uint16_t v = (config.brightness * config.brightness) / 16;
		v = min(v, 255); // in case config.brightness == 64
		
		// Artificially limit to 1 to ensure LEDs minimally on if 
		// lamp is supposed to be on
		v = max(v, 1);
		
		// S is assumed to be 1 (range 0-1)
		// Calculate chroma - 0 to 255.
		uint16_t c = (255 - abs((2 * (int16_t) v) - 255)); 
		// x is 0 - 255
		uint16_t xt = 8 * (config.hue.region & 1 ? config.hue.val : 32-config.hue.val);
		uint16_t x = (c * (256 - xt)) >> 8; // scale x down to range 0 to 256
		uint8_t r = 0, g = 0, b = 0;
		switch (config.hue.region ) {
		case 0: r = c; g = x; break;
		case 1: r = x; g = c; break;
		case 2: g = c; b = x; break;
		case 3: g = x; b = c; break;
		case 4: r = x; b = c; break;
		default: r = c; b = x;
		}
		uint16_t m = v - c/2;
		sendLeds(min(r + m, 255), min(g + m, 255), min(b + m, 255));
	} else {
		sendLeds(0, 0, 0);
	}
	leds_updated = true;
}


// How long to wait between last change and writing config eeprom
#define CONFIG_WRITE_MS 500

//////////////////////////////////////////////////////////////////////
// Main loop
// move written and write time out to globals

int main(void)
{
	
	// Initializes MCU, drivers and middleware 
	atmel_start_init();
	readConfig();
	maybeUpdateLeds();
	_delay_ms(500);
	micCalibrateAvgLevel();

	uint32_t last_awake = tick_millis;
	while (1) {
		// Sleep until there's a new millisecond
                // CPU wakes on RTC interrupt.
		while (last_awake == tick_millis) {
			__builtin_avr_sleep();
		}
		last_awake = tick_millis;
		
		// Figure out what's going on with the button
		int8_t b = checkTouch(readButton());
		if (b == 1) {
			toggleOn();
		}
		
		// Read the encoders - update hue and brightness if lamp on
		int8_t re1 = checkTouch(readEncoder1());
		updateBrightness(re1);
		int8_t re2 = checkTouch(readEncoder2());
		updateHue(re2);
		
		// Read mic and determine whether there has been a double clap
		bool double_clap = micRead();
		if (double_clap) {
			toggleOn();
		}
		
		// Update LEDs if anything changed
		maybeUpdateLeds();
		
		// Update config it required
		maybeWriteConfig();
	}
}
