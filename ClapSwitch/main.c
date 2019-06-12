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
#include <stdio.h>

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

#define MAX_BRIGHT 64
#define MAX_BRIGHT 191

// Values to save in EEPROM
typedef struct {
	// Whether lamp is on
	bool on : 1;
	// Brightness range 1-MAX_BRIGHT
	uint8_t brightness; 
	// Hue to take range is 0-191, 192 being divisible into 6 regions of 32 steps each
	uint8_t hue;
} Eeprom;
Eeprom config;

void readConfig() {
	FLASH_0_read_eeprom_block(0, (uint8_t *) &config, sizeof(Eeprom));
	if (config.brightness > MAX_BRIGHT) {
		config.brightness = 8;
	}
	if (config.hue > MAX_HUE) {
		config.hue = 0;
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
	if (!config_written && tick_millis > config_change_millis + CONFIG_WAIT_MS) {
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
	if (in == 0) {
		return;
	}
	config.hue += in;
	if (config.hue == 255) {
		config.hue = 191;
	} else if (config.hue == 192) {
		config.hue = 0;
	}
	configChanged();
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

//////////////////////////////////////////////////////////////////////
// Mic + Clap detection

// This is the expected mid-level audio reading (1.8/2.5 * 1024 / 2)
#define MID_READING 369

// >= this value is a loud value for audio_squares
#define LOUD_THRESHOLD 50000 

// <= this is a quiet value for audio_squares
#define QUIET_THRESHOLD 20000

// Accumulates squares of readings
uint32_t audio_squares;

// Number of samples in squares
uint8_t sample_count;

#define NUM_SAMPLES_PER_THRESHOLD 16

typedef enum {
	QUIET = 0,
	MID = 1,
	LOUD = 2,
} AudioLevel;

// Buffer. Each entry represent 16ms of data, so whole buffer is 128 * 16ms ~= 2s
#define LEVEL_BUFFER_LEN 128
AudioLevel level_buffer[LEVEL_BUFFER_LEN];
uint8_t level_buffer_index = 0;

// Do not detect claps until tick_millis is > this value
uint32_t clap_lockout_millis = 0;

// How long to lock out
#define CLAP_LOCKOUT_MS 2000

// Returns 0..1023 -- actually only 3/4 of the range due to 
// 1.8V max, measured against 2.5 V reference
uint16_t micRawRead() {
	return ADC_0_get_conversion(ADC_MUXPOS_AIN6_gc);
}

AudioLevel calculateCurrentLevel() {
	if (audio_squares >= LOUD_THRESHOLD) {
		return LOUD;
	} else if (audio_squares <= QUIET_THRESHOLD) {
		return QUIET;
	} 
	return MID;
}

inline uint8_t bufferIndexSubtract(uint8_t index, uint8_t amount) {
	if (amount > index) {
		return LEVEL_BUFFER_LEN - (amount - index);
	} else {
		return index - amount;
	}
}

inline uint8_t bufferIndexAdd(uint8_t index, uint8_t amount) {
	uint8_t result = index + amount;
	if (result >= LEVEL_BUFFER_LEN) {
		result -= LEVEL_BUFFER_LEN;
	}
	return result;
}

inline uint8_t bufferIndexNext(uint8_t index) {
	return bufferIndexAdd(index, 1);
}

void addToBuffer(AudioLevel level) {
	// Increment index, mod LEVEL_BUFFER_LEN
	level_buffer_index++;
	if (level_buffer_index >= LEVEL_BUFFER_LEN) {
		level_buffer_index = 0;
	}
	level_buffer[level_buffer_index] = level;
}

// For debugging - dump the buffer
void dumpBuffer() {
	puts_P(PSTR("\n\n\n"));
	printf_P(PSTR("%02hx\n"), level_buffer_index);
	printf_P(PSTR("%06lx\n"), audio_squares);
	uint8_t i = bufferIndexNext(level_buffer_index);
	for ( uint8_t count = 0; count < LEVEL_BUFFER_LEN; count++) {
		if (level_buffer[i] == QUIET) {
			putchar('.');
		} else if(level_buffer[i] == LOUD) {
			putchar('X');
		} else {
			putchar('_');
		}
		i = bufferIndexNext(i);
	}
	puts_P(PSTR("\n---\n---\n"));
}

// For debugging
void maybeDumpBuffer() {
	if (level_buffer_index == LEVEL_BUFFER_LEN - 1) {
		dumpBuffer();
	}
}


// All these are defined in terms of buffer indexes
#define POST_CLAP_MIN_QUIET 8

// Return true if found number entries consisting of allowed levels between min and max, going backward in buffer
// Decrements index as it goes.
bool checkPrior(uint8_t *i, uint8_t min_required, uint8_t max_allowed, bool quiet_allowed, bool mid_allowed, bool loud_allowed) {
	uint8_t count = 0;
	uint8_t end = bufferIndexSubtract(*i, max_allowed);
	bool condition_met = true;
	while (*i != end && condition_met) {
		AudioLevel level = level_buffer[*i];
		condition_met = (level == QUIET && quiet_allowed)  || (level == MID && mid_allowed) || (level == LOUD && loud_allowed);
		if (condition_met) {
			count++;
			*i = bufferIndexSubtract(*i, 1);
		}
	}
	
	return count >= min_required;
}

// Analyze the buffer. Return true if detected a double clap.
// A double clap looks like this, approximately, in regex notation:
// . = QUIET, _ = MID, X = LOUD
// .{32},_?X[_X]{0, 7}[._]{0,6}..{2,50}_?X[_X]{0, 8}X[._]{0,6}.{8}
bool analyzeBuffer() {
	uint8_t i = level_buffer_index;
	
	// This code looks backward over the buffer.
	
#define CHECK(min, max, q, m, l) \
	if (!checkPrior(&i, min, max, q, m, l)) { return false; }
	// Special check: the actual clap must start with a loud, or have its second be a loud. The rest can be mids.
#define CHECK_STARTED_LOUD \
	if (!((level_buffer[bufferIndexAdd(i, 1)] == LOUD) || (level_buffer[bufferIndexAdd(i, 2)] == LOUD)) ) { \
		return false; \
	} 

	// Ends with 8 quiet periods	
	CHECK(8, 8, 1, 0, 0);

	// Ramp down MID/QUIET
	CHECK(0, 1, 0, 1, 0);
	CHECK(0, 5, 1, 1, 0);
	
	// At least one LOUD
	CHECK(1, 1, 0, 0, 1);
	
	// Mix of mids and louds up to 9 periods - allows for some echo
	// This is the actual clap, so it must have started loud
	CHECK(0, 9, 0, 1, 1);
	CHECK_STARTED_LOUD;

	// Inter-clap quiet
	CHECK(2, 50, 1, 0, 0);
	
	// Ramp down MID/QUIET
	CHECK(0, 1, 0, 1, 0);
	CHECK(0, 5, 1, 1, 0);

	// At least one LOUD
	CHECK(1, 1, 0, 0, 1);

	// Mix of mids and louds up to 9 periods - allows for some echo
	// This is the actual clap, so it must have started loud
	CHECK(0, 9, 0, 1, 1);
	CHECK_STARTED_LOUD;

	// Must have had quiet beforehand	
	CHECK(32, 32, 1, 0, 0);
		
#undef CHECK

	clap_lockout_millis = tick_millis + CLAP_LOCKOUT_MS;
	return true;	
	
}


// Read mic, run processing return true if double-clap detected
// There are several parts to this
// (a) assemble 16 1-ms samples into a value related to root mean squares, but
//     not rooted or meaned.
// (b) Once an RMS block value has been assembled, determine if level is QUIET,
//     LOUD or MID and record in a circular buffer.
// (c) Detect whether there have been two claps in a row by examining record.
// (d) Lock out any further detection for a period.
bool micRead() {
	// diff will be in range 0-184 or 185
	uint16_t reading = micRawRead();
	int8_t diff = (reading > MID_READING ? reading - MID_READING : MID_READING - reading) / 2;
	
	// (a) calculate RMS, squared * 16
	audio_squares += diff * diff;
	sample_count++;
	if (sample_count == NUM_SAMPLES_PER_THRESHOLD) {
		// (b) Once we have a block, record it in buffer.
		addToBuffer(calculateCurrentLevel());
		maybeDumpBuffer();
		audio_squares = 0;
		sample_count = 0;
		
		// (d) Lock out any further detection
		if (clap_lockout_millis > tick_millis) {
			return false;
		}
		
		// (c) analyze buffer to detect double claps
		return analyzeBuffer();
	}
	
	return false;
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
// For timing see https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
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
			__builtin_avr_delay_cycles(6); // 0.3uS
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
		// v is L (because lowercase l looks like 1 and is confusing). 
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
		uint8_t hue_region = config.hue >> 5; // top 3 bits of region are hue range (0-5)
		uint8_t hue_val = config.hue & 0x1f; // bottom 5 bits are val
		uint16_t xt = 8 * (hue_region & 1 ? hue_val : 32-hue_val);
		uint16_t x = (c * (256 - xt)) >> 8; // scale x down to range 0 to 255
		uint8_t r = 0, g = 0, b = 0;
		switch (hue_region) {
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
	
	USART_0_enable();

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
		int8_t re2 = checkTouch(readEncoder2());
		if (config.on) {
			updateBrightness(re1);
			updateHue(re2);
		}
		
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
