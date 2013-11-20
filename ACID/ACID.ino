/*
** A.C.I.D.
** Copyright (C) 2011 Pat Deegan
** http://flyingcarsandstuff.com/contact/
**
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
** 
** The Aural Colour Identification Device, uses the TCS3200 chip to transform
** colours into sounds.
** 
** Full details available at 
** http://flyingcarsandstuff.com/projects/acid/ 
**
**
**
*/
#include <TimerOne.h>
#include <EEPROM.h>
#include <SPI.h>






/* Development switches... */

//#define OUTPUT_DEBUG_TO_SERIAL


// with SMOOTH_SPI_ADJUSTMENTS defined, changes to external SPI
// device settings will be progressive and smooth, such that the 
// resulting audio output is more musical and avoids pops in the speaker.
// The downside is extra traffic on the SPI lines, since we'll be going
// through each value from the current to the target setting.
//
// 
#define SMOOTH_SPI_ADJUSTMENTS



// uncomment the TEST_AUDIO_STAGE define in order 
// to loop the audio stage test tune...  
// useful, but gets annoying after a bit with a speaker hooked up ;-)
// Don't leave this enabled in production!
//  
// define TEST_AUDIO_STAGE


// uncomment CALIBRATE_ON_STARTUP define in order
// to... you guessed it, calibrate automatically on startup
// this is useful when you are building the system but don't 
// have the calib button setup yet.
// Probably don't want to leave this enabled in production

// #define CALIBRATE_ON_STARTUP


// the ENABLE_POST_PROCESSING define enables adjustments and 
// manipulations to the sample data -- usefull for the system
// but can be disabled when you want to see the actual data
// from the light-to-frequency converter unmodified.
// Good to leave enabled in production
#define ENABLE_POST_PROCESSING

// the RESPONSIVITY_ADJUSTMENT_ENABLE define activates 
// the adjustments related to the variation in responsivity
// for each color (e.g. the red sensors respond more easily to
// irradiance than the others).
// Good to leave enabled in production
#define RESPONSIVITY_ADJUSTMENT_ENABLE




/*
 * Audio output pins/bits
 *p
 * Each of our chord notes is generated and output on 
 * a specific pin of PORTD.  
 * Modify with caution, as we're using low-level access
 * to play with these bits.
 *
 * SCHEMATIC: PORTB 0 (digital 8) -> C3 (low note) RC
               PORTB 1 (digital 9) -> E4 (mid note) RC
               PORTB 2 (digital 10) -> G5 (high note) RC)
*/



/*
 * The MIN_GAIN_SETTING_TARGET sets the minimum amount for the feedback resistor
 * and thus the gain (value between 0-255) for the final summing amp output stage.
 * Higher values generate more output, but you get square waves earlier... 100 or so
 * seems decent.
 */
#define MIN_GAIN_SETTING_TARGET  205.0

// DEADBEEF: define AUDIO_OUT_PORT    PORTB

// G5 -- nominal: 783.99 Hz, measured 781.25
#define G5BIT             2
// E4 -- nomila 329.63, measured 327.87
#define E4BIT             1
// C3 -- nominal 130.81, measured 131.57
#define C3BIT             0


#define AUDIO_OUT_PIN_C3  3
#define AUDIO_OUT_PIN_E4  9
#define AUDIO_OUT_PIN_G5  10


// pin connected to TSC3200 sampler ~OE input--ensure it is below 8
// #define SAMPLER_DISABLE_PIN  7
// #undef SAMPLER_DISABLE_PIN



/*
  Using 128 microseconds between our sound generation "ticks" allows us
  to easily generate all our desired frequencies using multiples of this
  unit of time.
*/
#define DELAY_BETWEEN_TICKS_uSECONDS  128


/*
 * We need to loop back to 0 periodically with our tick counter.  This 
 * must happen at a count which is a common denominator of all three
 * audio frequencies... 120 is nice for this.
*/
#define TICKS_LOOP_MAX  240

/*   
  Number of ticks for a given colour sample...
  
  Initial tests measured a range of between 6Hz and 325 kHz--impressive,
  and maybe it actually makes it up to 500kHz in full sunlight--that's 
  an interrupt every 2 us... too much. Using the 20% scale option reduces
  that to a max of 100kHz -- so 10 microseconds between INTs.
  
  We want to count the cycles for in the shortest amount 
  of time that still allows us to minimize error as much as 
  possible.
  
  In ~11ms (88 ticks), we'd get over 1100 INTs when the output is at 100 kHz.
  On the low side, we'd get 1 or two interrupts when the output is at only
  180 Hz--so we have a scale that runs from [180, 1100]... not bad.
  1
  
  At this rate, we can make around 89 full single colour measurements
  in a second (22 "full" measurements of all three colours plus ambient light levels), which 
  should be ok.
*/        
#define FREQ_SAMPLE_LEN_TICKS  88


/* Select the pins on which to recieve external interrupts... choose
 * wisely -- not all pins support external interrupts.
 *
 *  SCHEMATIC:  PORTD 2 (dig 2) -> TCS3200 OUT
 *              PORTD 3 (dig 3) -> Calibration debounced MOM switch
*/
#define CALIBRATE_REQ_INPUT_PIN 0
#define FREQ_SAMPLE_INPUT_PIN  2

/*
  Sets FREQ_SAMPLE_INTERRUPT_NUM based on the input pin
  A value of 0 is generally an interrupt on pin 2, 1 on pin 3... there 
  are more for the mega.  Should happen automagically.
*/

#if FREQ_SAMPLE_INPUT_PIN == 2
  #define FREQ_SAMPLE_INTERRUPT_NUM  0

#elif FREQ_SAMPLE_INPUT_PIN == 3
  #define FREQ_SAMPLE_INTERRUPT_NUM  1

#elif FREQ_SAMPLE_INPUT_PIN == 21
  #define FREQ_SAMPLE_INTERRUPT_NUM  2

#elif FREQ_SAMPLE_INPUT_PIN == 20
  #define FREQ_SAMPLE_INTERRUPT_NUM  3

#elif FREQ_SAMPLE_INPUT_PIN == 19
  #define FREQ_SAMPLE_INTERRUPT_NUM  4

#elif FREQ_SAMPLE_INPUT_PIN == 18
  #define FREQ_SAMPLE_INTERRUPT_NUM  5

#else
  #define FREQ_SAMPLE_INTERRUPT_NUM -1
  #error You have selected an invalid frequency sample input pin! (set FREQ_SAMPLE_INPUT_PIN)
#endif



/*
  Sets CALIBRATE_REQ_INTERRUPT_NUM based on the input pin
  A value of 0 is generally an interrupt on pin 2, 1 on pin 3... there 
  are more for the mega.  Should happen automagically.
*/

#if CALIBRATE_REQ_INPUT_PIN == FREQ_SAMPLE_INPUT_PIN

  #define CALIBRATE_REQ_INTERRUPT_NUM -1
  #error Calibration request input pin same as frequency sample input pin! (set CALIBRATE_REQ_INPUT_PIN)

#elif CALIBRATE_REQ_INPUT_PIN == 2
  #define CALIBRATE_REQ_INTERRUPT_NUM  0

#elif CALIBRATE_REQ_INPUT_PIN == 3
  #define CALIBRATE_REQ_INTERRUPT_NUM  1

#elif CALIBRATE_REQ_INPUT_PIN == 21
  #define CALIBRATE_REQ_INTERRUPT_NUM  2

#elif CALIBRATE_REQ_INPUT_PIN == 20
  #define CALIBRATE_REQ_INTERRUPT_NUM  3

#elif CALIBRATE_REQ_INPUT_PIN == 19
  #define CALIBRATE_REQ_INTERRUPT_NUM 4

#elif CALIBRATE_REQ_INPUT_PIN == 18
  #define CALIBRATE_REQ_INTERRUPT_NUM 5
#else
  #define CALIBRATE_REQ_INTERRUPT_NUM -1
  #define CALIBRATE_REQ_HANDLED_MANUALLY
  #warning You have selected a calibration request input pin without an interrupt attached...
#endif


// number of full samples to average over during calibration
#define CALIBRATION_AVERAGE_OVER_NUM  12








// NOTE: Using PORTC bits 0 and 1 to select sample type
// all this, below, depends on that fact and on the
// datasheet to specify how things are setup.
// On the Arduino, PORTC is the "analog input", so
// PORTC0 is analog input 0 and PORTC1 in analog input 1.

// SCHEMATIC: Analog Input 0 -> TCS3200 S2, Analog input 1 -> TCS3200 S3
#define STYPE_PORT      PORTC

#define STYPE_CLEARMASK B11111100

#define STYPE_BLUE      B00000010
#define STYPE_GREEN     B00000011
#define STYPE_RED       B00000000
#define STYPE_LEVEL     B00000001



// the sample type struct holds data related to samples for one colour (or clear)
// from the light-to-frequency converter.  It's mainly static, but holds 
// sample_count and resulting_setting that change dynamically and are used to
// set the output levels
typedef struct sample_type_struct {
  byte name;
  byte type_mask;
  float responsivity_adjustment;
  float calibration_factor;
  float equalizer_setting;
  unsigned int sample_count;
  unsigned int resulting_setting;
} sample_type;



// Certain color detectors respond more than other to 
// irradiance... The responsivity here was determined from 
// the tcs3200 datasheet

#define RESP_BLUE 331
#define RESP_GREEN 386
#define RESP_RED 474

// maximum value is for red... use that to normalize everything
#define RESP_MAX 474.0


// Our actual sample type settings and storage
#define NUM_SAMPLE_TYPES  4

sample_type stypes[NUM_SAMPLE_TYPES] = {
  { 'B', STYPE_BLUE, RESP_MAX/RESP_BLUE, 1.0, 1.85, 0, 0},
  { 'G', STYPE_GREEN, RESP_MAX/RESP_GREEN, 1.0, 1.30, 0, 0},
  { 'R', STYPE_RED, RESP_MAX/RESP_RED, 1.0, 0.70, 0, 0},
  { 'C', STYPE_LEVEL, 1.0, 1.0, 1.0, 0, 0}
};

#define STYPE_BLUE_IDX  0
#define STYPE_GREEN_IDX  1
#define STYPE_RED_IDX    2
// the "C" (clear) indicates global light levels and is 
// a bit special... we'll want to access it directly and
// sometimes handle all the colours disregarding it, so we'll
// keep track of it's index in our sample_type list:
#define STYPE_LEVEL_IDX  3


typedef struct spi_adjust_struct {
 
 boolean changes_required;
 boolean current_toggle[NUM_SAMPLE_TYPES];
 byte current_settings[NUM_SAMPLE_TYPES];
 byte target_settings[NUM_SAMPLE_TYPES];
} spi_adjust_details;



/* Full scale frequency... for the device in 
  LowSpeed mode, this is 12kHz.
  In our "high speed" mode (20% of actual device clock) it goes up to 120 kHz.
  For full clock speed, this can reach 600 kHz
  This means that maximum possible sample size (for 88 128us ticks) is:
  
    Lowspeed (2%): 135
    HighSpeed (20%): 1351
    Full Device speed (100%): 6758
    
  NOTE: This is only true below saturation... at that point things can go wild (I've seen sample counts of 20000+)
  Also note that our uC can't handle too high a frequency, as the interrupts overload it -- more on this below.
  
*/

#define HIGH_SPEED_MAX_CLOCK_FREQUENCY  600000.00
#define MED_SPEED_MAX_CLOCK_FREQUENCY  120000.00 
#define LOW_SPEED_MAX_CLOCK_FREQUENCY   12000.00 


// NOTE: There's a harcoded fudge factor in the equations, which was found empirically... (2, 0.15...) 
// It makes things more sensible on the output but is a bit of mystery meat... sorry ;-)
#define MICROSECONDS_PER_SECOND 1000000.00
#define MAX_NUM_PULSES_LOW_SPEED_SAMPLE (int)(LOW_SPEED_MAX_CLOCK_FREQUENCY * 2 * (1.0 * FREQ_SAMPLE_LEN_TICKS * (1.0 * DELAY_BETWEEN_TICKS_uSECONDS/MICROSECONDS_PER_SECOND)))
#define MAX_NUM_PULSES_MED_SPEED_SAMPLE (int)(MED_SPEED_MAX_CLOCK_FREQUENCY * 0.15 * (1.0 * FREQ_SAMPLE_LEN_TICKS * (1.0 * DELAY_BETWEEN_TICKS_uSECONDS/MICROSECONDS_PER_SECOND)))
#define MAX_NUM_PULSES_HIGH_SPEED_SAMPLE (int)(HIGH_SPEED_MAX_CLOCK_FREQUENCY  * 0.15 * (1.0 * FREQ_SAMPLE_LEN_TICKS * (1.0 * DELAY_BETWEEN_TICKS_uSECONDS/MICROSECONDS_PER_SECOND)))


typedef struct sample_speed_dets_struct {
  byte percentage;    
  byte pin_settings;
  float scaling_factor;
} sample_speed_details;


/*
 * Sensor output scaling... 
 * We control the output scaling ("sample speed") using bits 2 and 3 of PORTC,
 * "analog in" ports 2 and 3 in arduino-speak.  S0 goes to 2 and S1 to 3.
 *
 * SCHEMATIC: Analog In 2 -> TCS3200 S0, Analog In 3 -> TCS3200 S1
 */

#define SET_SAMPLE_SPEED_PORT  PORTC

#define SET_SAMPLE_SPEED_MASK  B11110011

#define NUM_SAMPLE_SPEED_SETTINGS 3

sample_speed_details  sample_speed_settings[NUM_SAMPLE_SPEED_SETTINGS] = {
  
  {2,   B00001000, 255.000000 / MAX_NUM_PULSES_LOW_SPEED_SAMPLE},
  {20,  B00000100, 255.000000 / MAX_NUM_PULSES_MED_SPEED_SAMPLE},
  {100, B00001100, 255.000000 / MAX_NUM_PULSES_HIGH_SPEED_SAMPLE}
};


#define DEFAULT_SAMPLE_SPEED_IDX  1

byte current_sample_speed_idx = DEFAULT_SAMPLE_SPEED_IDX;


// we will switch between sensor clock divisors (100, 20 or 2% of clock) automatically.
// it would be nice to stay on high speed, but the high frequency signal at 
// high light levels causes so many interrupts
// that we just can't handle it, so
// the MAX and MIN here are the points at which switch-over occurs
// 
// The values defined here are abstracted a bit, so we can vary our tick delay or the number of ticks to keeps sampling
// for and it'll auto-adjust.  Using, for instance, 128 us between each tick, and 88 ticks per sample, it gives us
//  * Max before slowing down: 1150 count
//  * Min before speeding up:  60 count
// Beyond about a 1200 count (within the 88 x 128 us period), we can definitely hear disruption on the audio generation.
// Below about 30-40 count, the deltas between colors become hard to distinguish.
#define MAX_SAMPLE_COUNT_FOR_SLOWDOWN  (int)( (10.21/100) * DELAY_BETWEEN_TICKS_uSECONDS  * FREQ_SAMPLE_LEN_TICKS)
#define MIN_SAMPLE_COUNT_FOR_SPEEDUP  (int)( (5.327/1000) *  DELAY_BETWEEN_TICKS_uSECONDS  * FREQ_SAMPLE_LEN_TICKS)



#define MIN_TOTAL_COLOUR_FOR_OPS   5


// MAX_LEVEL_FUDGE_FACTOR:
// Any global light level between [(255 - MAX_LEVEL_FUDGE_FACTOR), 255] is considered "very bright out"
// and we skip the normalization step alltogether.
#define MAX_LEVEL_FUDGE_FACTOR 10


/*
 * Headlight control
 *
 * If it's sample level too low on max speed... turn on the headlights.
 * If it's sample level too high on med speed and headlights are on, turn 'em off.
 *
 * Headlights are controled by bit 5 of PORTC
 * SCHEMATIC: Arduino analog in 4 to headlight transistor switch.
 */
#define HEADLIGHT_CTRL_PORT  PORTC
#define HEADLIGHT_OFF_MASK  B11101111
#define HEADLIGHT_ON_BIT    B00010000



/*
 *
 * Digital pots SELECT switches
 *
 *
 * SCHEMATIC:
 *              PORTD4 (digital 4) -> C3 dig pot ~CS pin
 *              PORTD5 (digital 5) -> E4 dig pot ~CS pin
 *              PORTD6 (digital 6) -> G5 dig pot ~CS pin
 *              PORTD7 (digital 7) -> summing amp feedback pot ~CS pin
 *
*/

typedef struct spi_ctrl_dets_struct {
  byte command;
  void (*chip_select)(byte);
  void (*line_release)(byte);
  byte clear_mask;
} spi_control;

void select_on_portb(byte clearmask)
{
  PORTB |= clearmask; PORTB &= ~clearmask;
}
void release_on_portb(byte clearmask)
{
  PORTB |= clearmask;
}


void select_on_portd(byte clearmask)
{
  PORTD |= clearmask; PORTD &= ~clearmask;
}
void release_on_portd(byte clearmask)
{
  PORTD |= clearmask;
}

spi_control spi_control_details[NUM_SAMPLE_TYPES] = {
    {'B', select_on_portb, release_on_portb, B00000001},
    {'G', select_on_portb, release_on_portb, B00000001},
    {'R', select_on_portb, release_on_portb, B00000001},
    {0, select_on_portd, release_on_portd, B10000000}  
};



/*
#define SPI_OUT_SELECT_PORT  PORTD
#define  SPI_OUT_SELECT_CLEAR  B10000000


// spi_out_select enables the selected index (between 0-3) and makes the appropriate pin LOW (~CS pins on SPI pots)
#define  SPI_OUT_SELECT(index)    {SPI_OUT_SELECT_PORT |= SPI_OUT_SELECT_CLEAR; SPI_OUT_SELECT_PORT &= ~(1<< (index + 4) );}

#define SPI_GAIN_CHANNEL  3
*/



#ifdef ENABLE_POST_PROCESSING
  // these should normally all be activated, but the defines
  // can be damn useful to disable stuff while debugging :)
  #define SAMPLE_DELTA_STRETCH_ENABLE
  
  #define INCIDENT_LIGHT_CORRECTION_ENABLE

  #define BOOST_GREEN_ENABLE
  
  #define NORMALIZE_SETTING_RANGE_ENABLE
  
  #define EQUALIZER_ENABLE
#else
  #warning Post processing is currently disabled.
#endif




#ifdef OUTPUT_DEBUG_TO_SERIAL
  // a few shorthand methods to avoid needing #ifdefs everywhere
  // just to add some debug output.
  #define SERIAL_OUTLN(ln)  Serial.println(ln);
  #define SERIAL_OUTLN2(ln, tp)  Serial.println(ln, tp);
  #define SERIAL_OUT(ln)    Serial.print(ln);
  #define SERIAL_OUT2(ln, tp)  Serial.print(ln, tp);

  #define OUTPUT_DEBUG_INFO_EVERY 200
  
  void output_sample_counts();

#else

  // if we don't want output, we just ignore all these output methods
  // and wipe them from the compiled code
  #define SERIAL_OUTLN(ln)    ;
  #define SERIAL_OUTLN2(ln, tp)  ;
  #define SERIAL_OUT(ln)   ;
  #define SERIAL_OUT2(ln, tp)  ;
#endif





#ifdef SAMPLE_DELTA_STRETCH_ENABLE
  // how much we "stretch out" closely similar colour channels
  // more is... more! (nominal around 2.5 seems ok)
  //#define SAMPLE_DELTA_STRETCH_FACTOR 2.5
  //#define SAMPLE_DELTA_STRETCH_FACTOR 2.9
  #define SAMPLE_DELTA_STRETCH_FACTOR 3.5
  
#endif

#ifdef INCIDENT_LIGHT_CORRECTION_ENABLE
  // what fraction of the incident (global) light level
  // that's common to everyone we remove
  // more is less (i.e. a greater number is a smaller proportion)
  // Below 2.0 is probably not a good idea.
  // #define INCIDENT_LIGHT_CORRECTION_FACTOR 2.3
  #define INCIDENT_LIGHT_CORRECTION_FACTOR 2.8
#endif




/*  MUSIC !
 * In order to test the audio output stage, and to provide a bit of 
 * feedback on calibration, we can play some tunes with our minimal
 * 3-note synth.
 *
 * A few defines are used to keep things (semi-)clear, then we can 
 * create "musical" output using arrays of colour_chord (structs that
 * specify values for each tone and duration).
 *
 * To play a song, pass a pointer to the first note in the list to
 * song_play_start() and you're done (as long as you used the start/stop 
 * markers appropriately -- more below).
 */
 
 
// To keep things simple, our defines all assume a 4/4 partition
#define MSDURATION_PER_QUARTER_NOTE_IN_44(bpm) (unsigned int)(1000/(bpm/60))

#define MS_QUARTER_NOTE(bpm) MSDURATION_PER_QUARTER_NOTE_IN_44(bpm)

#define MS_SIXTEENTH_NOTE(bpm)    (unsigned int)(MS_QUARTER_NOTE(bpm)/4)
#define MS_EIGTH_NOTE(bpm)    (unsigned int)(MS_QUARTER_NOTE(bpm)/2)
#define MS_HALF_NOTE(bpm) MS_QUARTER_NOTE(bpm) * 2
#define MS_WHOLE_NOTE(bpm) MS_QUARTER_NOTE(bpm) * 4






#define REST_SIXTEENTH_NOTE(bpm)  {{0,0,0}, MS_SIXTEENTH_NOTE(bpm)}
#define REST_EIGTH_NOTE(bpm)  {{0,0,0}, MS_EIGTH_NOTE(bpm)}
#define REST_QUARTER_NOTE(bpm)  {{0,0,0}, MS_QUARTER_NOTE(bpm)}
#define REST_HALF_NOTE(bpm)  {{0,0,0}, MS_HALF_NOTE(bpm)}
#define REST_WHOLE_NOTE(bpm)  {{0,0,0}, MS_WHOLE_NOTE(bpm)}

#define NOTE_C(level, duration) {{level,0,0}, duration}
#define NOTE_E(level, duration) {{0,level,0}, duration}
#define NOTE_G(level, duration) {{0,0,level}, duration}

// Song Markers: 
// SONG_START_MARKER *should* be the first "note" in the array
// SONG_END_MARKER *must* be the last note or we'll go out-of-bounds
#define SONG_START_MARKER {{0,0,0},0}

// The end marker is basically a unique duration that is checked for...
// any number will do, as long as no notes have that many ms as a duration
// What to use?  The answer is of course:
#define SONG_END_MARKER_DURATION 4242
#define SONG_END_MARKER {{0,0,0},SONG_END_MARKER_DURATION}



// colour_chord is a struct containing the output value {0-255} for 
// each colour and the duration in ms
typedef struct colour_chord_struct {
    byte components[STYPE_LEVEL_IDX];
    unsigned int duration;
} colour_chord;


#ifdef TEST_AUDIO_STAGE

// we setup a little tune to use when testing the audio stage.
// Will be ignored unless TEST_AUDIO_STAGE is defined at the top.
#define TEST_SONG_BPM  220
#define TEST_SONG_DEFAULT_LEVEL  160

#define TEST_SONG_EXTREME_LEVEL  175
colour_chord test_song_xtreme[] = {
  SONG_START_MARKER,
  
  NOTE_C(TEST_SONG_EXTREME_LEVEL, MS_HALF_NOTE(120)),
  {{TEST_SONG_EXTREME_LEVEL, TEST_SONG_EXTREME_LEVEL/3, 0}, MS_QUARTER_NOTE(120)},
  {{TEST_SONG_EXTREME_LEVEL, 0, TEST_SONG_EXTREME_LEVEL/4}, MS_QUARTER_NOTE(120)},
  NOTE_E(TEST_SONG_EXTREME_LEVEL, MS_HALF_NOTE(120)),
  {{0,TEST_SONG_EXTREME_LEVEL, TEST_SONG_EXTREME_LEVEL/3}, MS_QUARTER_NOTE(120)},
  NOTE_G(TEST_SONG_EXTREME_LEVEL, MS_HALF_NOTE(120)),
  SONG_END_MARKER
};

  
colour_chord test_song[] = {
  
  
  
  // No, not my best work... but it gets the job done ;-)
  SONG_START_MARKER,
  
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_G(TEST_SONG_DEFAULT_LEVEL, MS_EIGTH_NOTE(TEST_SONG_BPM)),
  REST_QUARTER_NOTE(TEST_SONG_BPM),
  NOTE_G(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  REST_QUARTER_NOTE(TEST_SONG_BPM),
  
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_G(TEST_SONG_DEFAULT_LEVEL, MS_EIGTH_NOTE(TEST_SONG_BPM)),
  REST_QUARTER_NOTE(TEST_SONG_BPM),
  NOTE_G(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  REST_QUARTER_NOTE(TEST_SONG_BPM),
  
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_G(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  REST_SIXTEENTH_NOTE(TEST_SONG_BPM),
  NOTE_C(TEST_SONG_DEFAULT_LEVEL + 30, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  
  REST_QUARTER_NOTE(TEST_SONG_BPM),
  
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_G(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_E(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  NOTE_C(TEST_SONG_DEFAULT_LEVEL, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  REST_SIXTEENTH_NOTE(TEST_SONG_BPM),
  NOTE_C(TEST_SONG_DEFAULT_LEVEL + 30, MS_QUARTER_NOTE(TEST_SONG_BPM)),
  
  REST_EIGTH_NOTE(TEST_SONG_BPM),
  
  {TEST_SONG_DEFAULT_LEVEL,TEST_SONG_DEFAULT_LEVEL,0, MS_EIGTH_NOTE(TEST_SONG_BPM)},
  {TEST_SONG_DEFAULT_LEVEL,TEST_SONG_DEFAULT_LEVEL,TEST_SONG_DEFAULT_LEVEL, MS_EIGTH_NOTE(TEST_SONG_BPM)},
 
  {{145,145,145}, MS_SIXTEENTH_NOTE(TEST_SONG_BPM)},
  {{160,160,160}, MS_SIXTEENTH_NOTE(TEST_SONG_BPM)},
  {{190,210,190}, MS_SIXTEENTH_NOTE(TEST_SONG_BPM)},
  {{230,250,230}, MS_QUARTER_NOTE(TEST_SONG_BPM)},
  {{100, 190, 190}, MS_QUARTER_NOTE(TEST_SONG_BPM)},
  {{0, 160, 160}, MS_HALF_NOTE(TEST_SONG_BPM)},
  {{0, 80, 120}, MS_HALF_NOTE(TEST_SONG_BPM)},
  {{0, 0, 85}, MS_HALF_NOTE(TEST_SONG_BPM)},
  {{0,0,60}, MS_HALF_NOTE(TEST_SONG_BPM)},
  {{235, 0, 0}, MS_EIGTH_NOTE(TEST_SONG_BPM)},
  
  REST_SIXTEENTH_NOTE(TEST_SONG_BPM),
  {{240, 0, 0}, MS_EIGTH_NOTE(TEST_SONG_BPM)},
  
  
  REST_SIXTEENTH_NOTE(TEST_SONG_BPM),
  {{255, 0, 0}, MS_EIGTH_NOTE(TEST_SONG_BPM)},
  
  REST_WHOLE_NOTE(TEST_SONG_BPM),
  
  SONG_END_MARKER
  
};

// #define TEST_AUDIO_SONG_TO_PLAY test_song_xtreme
#define TEST_AUDIO_SONG_TO_PLAY test_song

#endif



#define CALIB_SONG_BPM  220
#define CALIB_SONG_LEVEL  200

colour_chord startup_song[] = {
  SONG_START_MARKER,
  
  NOTE_C(200, MS_EIGTH_NOTE(220)),
  NOTE_G(100, MS_SIXTEENTH_NOTE(220)),
  
  
  REST_SIXTEENTH_NOTE(220),
  
  
  SONG_END_MARKER
};
  

colour_chord calib_start_song[] = {
  
  SONG_START_MARKER,
  
  NOTE_G(CALIB_SONG_LEVEL + 50, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)),
  NOTE_E(CALIB_SONG_LEVEL, MS_EIGTH_NOTE(CALIB_SONG_BPM)),
  {{0, CALIB_SONG_LEVEL - 50, CALIB_SONG_LEVEL + 70}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  
  // NOTE_G(CALIB_SONG_LEVEL + 70, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)),
  
  REST_HALF_NOTE(CALIB_SONG_BPM),
  
  SONG_END_MARKER
  
};


colour_chord calib_done_song[] = {
  
  SONG_START_MARKER,
  
  NOTE_C(CALIB_SONG_LEVEL, MS_EIGTH_NOTE(CALIB_SONG_BPM)),
  NOTE_E(CALIB_SONG_LEVEL, MS_EIGTH_NOTE(CALIB_SONG_BPM)),
  NOTE_G(CALIB_SONG_LEVEL, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)),
  
  REST_EIGTH_NOTE(CALIB_SONG_BPM),
  
  {{0, CALIB_SONG_LEVEL + 50, CALIB_SONG_LEVEL + 50}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  {{0, CALIB_SONG_LEVEL + 20, CALIB_SONG_LEVEL + 20}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  {{0, CALIB_SONG_LEVEL, CALIB_SONG_LEVEL}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  {{0, CALIB_SONG_LEVEL - 50, CALIB_SONG_LEVEL + 50}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  
  REST_HALF_NOTE(CALIB_SONG_BPM),
  
  SONG_END_MARKER
};


colour_chord calib_headlights_done_song[] = {
  
  SONG_START_MARKER,
  
  NOTE_C(CALIB_SONG_LEVEL, MS_QUARTER_NOTE(CALIB_SONG_BPM)),
  NOTE_E(CALIB_SONG_LEVEL, MS_EIGTH_NOTE(CALIB_SONG_BPM)),
  NOTE_G(CALIB_SONG_LEVEL, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)),
  
  REST_EIGTH_NOTE(CALIB_SONG_BPM),
  
  {{CALIB_SONG_LEVEL + 50, CALIB_SONG_LEVEL + 50, 0}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  {{CALIB_SONG_LEVEL + 20, CALIB_SONG_LEVEL + 20, 0}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  {{CALIB_SONG_LEVEL, CALIB_SONG_LEVEL, 0}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  {{CALIB_SONG_LEVEL - 50, CALIB_SONG_LEVEL - 50 , 0}, MS_SIXTEENTH_NOTE(CALIB_SONG_BPM)},
  
  REST_HALF_NOTE(CALIB_SONG_BPM),
  
  SONG_END_MARKER
  
};







// flipbit -- little shorthand for toggling a given bit in a byte.
#define flipbit(sfr, bit) (_SFR_BYTE(sfr) ^= 1 << bit)

// shorthand to hold little tweaks necessary to jump into resting state.
#define RESET_TO_RESTING_STATE()  sampling = false; current_sample_type_idx = 0; continue;


#define SUMMING_AMP_IN_RES  5100.0
#define SUMMING_AMP_FEEDBACK_RES_MAX  50200.0



/* 
 *****************  GLOBAL VARIABLES *****************
*/



unsigned int calib_samples[CALIBRATION_AVERAGE_OVER_NUM][NUM_SAMPLE_TYPES];
boolean headlights_currently_on = false;

spi_adjust_details spi_adjust_values;

byte currentstate = 0xF0;



// vars altered by interrupts (must be volatile)
// tick counter ISR
volatile boolean waitingfortick = true;
volatile byte tickcount = 0;

// frequency counter ISR
volatile unsigned int freq_sample_count = 0;

// calib request ISR
volatile boolean calibrating = false;
volatile byte calibrate_samples_to_collect = 0;

// A few global used to track our song playing state
volatile colour_chord * song_playing_chord = 0;
volatile unsigned long song_playing_next_note_time = 0;





/* 
 *****************  ISRs *****************
 *
 * The interrupt service routines used to
 * generate audio timer ticks, count our input
 * samples and setup calibration.
 *
*/

#define TIMER_ONE_PERIOD_DIVISOR  4
volatile byte tick_divisor = 0;
void generateTick()
{
  
      tick_divisor++;
      if (tick_divisor >= TIMER_ONE_PERIOD_DIVISOR)
      {
          tick_divisor = 0;
            
          if (tickcount >= TICKS_LOOP_MAX - 1)
          {
            // reset the tick count.  NOTE: We do TICKS_LOOP_MAX - 1 because the (even)
            // TICKS_LOOP_MAX would likely trigger a skip, as it matches some of the same 
            // stuff as the upcomming 0 value.
            tickcount = 0;
          } else {
            
            // normal increment here
            tickcount++;
          }
          
          // set the flag to notify driver we have ticked...
          waitingfortick = false;
      }
}

void freq_count_isr()
{
  // just need to increment our sample count by one here,
  // nice and quick!
  freq_sample_count++;
}

void calibrate_isr()
{
  // set us in calibration mode
  SERIAL_OUTLN("Calibration requested");
  calibrating = true;
  calibrate_samples_to_collect = CALIBRATION_AVERAGE_OVER_NUM;
  
  song_play_start(calib_start_song);
}



/* 
 *****************  Utility Functions *****************
*/











// an arbitrary starting addy for our 3 bytes of calibration data...
#define CALIBRATION_STORAGE_ADDRESS_OFFSET  13
#define CALIBRATION_FOR_HEADLIGHTS_STORAGE_ADDRESS_OFFSET    CALIBRATION_STORAGE_ADDRESS_OFFSET + 3

// we want to store the calib values in a single byte, so we multiply it up
// the larger the multiplier, the greater the precision, but we don't want to overflow
// the byte boundry... so a choice between 50 - 100 should be safe... higher more precise but
// risks overflow.
#define CALIBRATION_STORAGE_FACTOR_MULTIPLIER  90.000
void store_colour_calibration(byte color_index, float calib_factor, byte addr_offset=CALIBRATION_STORAGE_ADDRESS_OFFSET)
{
  // store it as a byte... we start with a float val and store it as an byte, by multiplying it by some factor
  
  byte val = (byte)(round(CALIBRATION_STORAGE_FACTOR_MULTIPLIER * calib_factor));
  
  SERIAL_OUT("Storing calibration setting: ");
  SERIAL_OUTLN2(val, DEC);
  EEPROM.write(addr_offset + color_index, val);
  
}

float get_colour_calibration(byte color_index, byte addr_offset=CALIBRATION_STORAGE_ADDRESS_OFFSET)
{
  byte val = EEPROM.read(addr_offset + color_index);
  
  float actual_calib_factor = (float)(val/CALIBRATION_STORAGE_FACTOR_MULTIPLIER);
  
  SERIAL_OUT("Got calibration setting: ");
  SERIAL_OUTLN2(actual_calib_factor, DEC);
  
  return actual_calib_factor;
}


#define SIG_ADDRESS_OFFSET 0
byte dev_init_sig[] = {80,115,121,99,104,111,103,101,110,105,99,0};
void verify_device_initialization()
{
  // if device has been initialized, it will have our sig string in EEPROM memory and
  // valid default values for both calibration sets.
  byte i = 0;
  boolean init_calib_done = true;
  byte val;
  while (dev_init_sig[i] != 0)
  {
    val = EEPROM.read(SIG_ADDRESS_OFFSET + i);
    
    if (val != dev_init_sig[i])
    {
      
      // oh, seems like we've never done this before... stick our sig letter in there
      // and note the fact we need to store init calib
      init_calib_done = false;
      EEPROM.write(SIG_ADDRESS_OFFSET + i, dev_init_sig[i]);
    }
    
    // don't forget to move on to next letter
    i++;
    
  }
  if (! init_calib_done)
  {
    // Init was never done, store default sane values for calibration in EEPROM
    
    SERIAL_OUTLN("Initial calibration never done... storing default values now");
    for (i=0; i<STYPE_LEVEL_IDX; i++)
    {
      store_colour_calibration(i, 1.0000, CALIBRATION_STORAGE_ADDRESS_OFFSET);
      store_colour_calibration(i, 1.0000, CALIBRATION_FOR_HEADLIGHTS_STORAGE_ADDRESS_OFFSET);
    }
  }
  
}
  
void setup_colour_calibration()
{
  
  byte addr_offset = headlights_currently_on? CALIBRATION_FOR_HEADLIGHTS_STORAGE_ADDRESS_OFFSET : CALIBRATION_STORAGE_ADDRESS_OFFSET;
  
  for (byte i=0; i<STYPE_LEVEL_IDX; i++)
  {
    SERIAL_OUTLN2(i, DEC);
    stypes[i].calibration_factor = get_colour_calibration(i, addr_offset);
  }
}


/* calibration routine
 * Called when we are in calibration mode and have collected all 
 * required samples.
*/




void perform_calibration()
{
  unsigned long sample_totals[STYPE_LEVEL_IDX];
  
  byte i;
  
  for (i=0; i< STYPE_LEVEL_IDX; i++)
  {
    sample_totals[i] = 0;
  }
  
  // run through all the sample sets, one sample type at a time
  for (i=0; i< STYPE_LEVEL_IDX; i++)
  {
    unsigned int high_val = calib_samples[0][i];
    unsigned int low_val = high_val;
    unsigned int cur_val;
    for (byte samp_set=0; samp_set < CALIBRATION_AVERAGE_OVER_NUM; samp_set++)
    {
      cur_val = calib_samples[samp_set][i];
      
      sample_totals[i] += cur_val; // keep running total
      
      
      // keep track of highest and lowest
      if (cur_val > high_val)
      {
        high_val = cur_val;
      }
      
      if (cur_val < low_val)
      {
        low_val = cur_val;
      }
    }
    
    // remove EXTREMES from sample totals for this type
    SERIAL_OUT("Eliminating calib extreme samples: ");
    SERIAL_OUT2(low_val, DEC);
    SERIAL_OUT("/");
    SERIAL_OUTLN2(high_val, DEC);
    sample_totals[i] -= (high_val + low_val);
      
  }
  
  unsigned long grand_total = 0;
  for (i=0; i<STYPE_LEVEL_IDX; i++)
  {
    
    // Remove the effect of the previous calibration factor, to ensure we are operating on "raw" data
    sample_totals[i] = sample_totals[i] / stypes[i].calibration_factor;
  
  
    if (sample_totals[i] < 1)
    {
      // just to make sure we never have a 0 in here
      sample_totals[i] = 1;
    }
    
    grand_total += sample_totals[i];
  }

  // get the average over all signal...
  // NOTE: Remember that we are discounting the high/low value from each sample type, so the number of effective samples is
  //       samples sets collected - 2 !!!
  float avg_color_sig = (float)(grand_total * 1.0 / (STYPE_LEVEL_IDX * (CALIBRATION_AVERAGE_OVER_NUM - 2) ));
  
  
#ifdef OUTPUT_DEBUG_TO_SERIAL
  
  // output a little debug info...
  SERIAL_OUTLN("Calibration totals");
  for (i=0; i<NUM_SAMPLE_TYPES; i++)
  {
    SERIAL_OUTLN2(sample_totals[i], DEC);
  }
  
  SERIAL_OUT("Average color sig: ");
  SERIAL_OUTLN2(avg_color_sig, DEC);
  
#endif

  byte calibration_addr_offset = headlights_currently_on ? CALIBRATION_FOR_HEADLIGHTS_STORAGE_ADDRESS_OFFSET : CALIBRATION_STORAGE_ADDRESS_OFFSET;
  float calib_correction_factor;
  
  for (i=0; i<STYPE_LEVEL_IDX; i++)
  {
    calib_correction_factor = avg_color_sig * (CALIBRATION_AVERAGE_OVER_NUM - 2) / sample_totals[i];
    SERIAL_OUT("Correction factor for ");
    SERIAL_OUT2(i, DEC)
    SERIAL_OUT(": ");
    SERIAL_OUTLN2(calib_correction_factor, DEC);
    
    store_colour_calibration(i, calib_correction_factor, calibration_addr_offset);
  }
  
  // reload it all...
  setup_colour_calibration();
  
  // tell the user it's done with a little tune
  colour_chord * done_song = headlights_currently_on ? calib_headlights_done_song : calib_done_song;
  song_play_start(done_song);
  
  
}



void reset_colour_calibration()
{
  SERIAL_OUTLN("Reset colour calib");
  
  for (byte i=0; i<STYPE_LEVEL_IDX; i++)
  {
    store_colour_calibration(i, 1.0000);
  }
}












/* 
 *****************  Sample Speed *****************
 *
 * SAMPLE THROTTLING
 *
 * Routines related to setting up the sensor's 
 * output frequency range, i.e. the speed at which 
 * our counter interrupt is triggered.
 *
 * More is better, as it gives us more precision, but
 * there's no way we can handle the full 600kHz pulse
 * train.
 * The fact is, with our audio generation and all the 
 * processing, we can basically only handle around
 * 90 kHz without the sound pulses starting to be 
 * affected.
 *
 * The code will notice changes in light levels 
 * that cause too high (or too low) a rate of input
 * and throttle accordingly. 
 *
*/

void sample_speed_throttle_down()
{
  if (current_sample_speed_idx > 0)
  {
    set_sample_speed(current_sample_speed_idx - 1);
  }
}

void sample_speed_throttle_up()
{
  if (current_sample_speed_idx < (NUM_SAMPLE_SPEED_SETTINGS - 1))
  {
    set_sample_speed(current_sample_speed_idx + 1);
  }
}

 

void set_sample_speed(byte speed_idx)
{
  
  // fetch the value first -- don't 0 in place or we'll put the device to sleep...
  
  byte curval = SET_SAMPLE_SPEED_PORT & SET_SAMPLE_SPEED_MASK; // current value with our bits of interest cleared
  
  curval |= sample_speed_settings[speed_idx].pin_settings;
  
  SET_SAMPLE_SPEED_PORT = curval;
  
  current_sample_speed_idx = speed_idx;
  
  
#ifdef OUTPUT_DEBUG_TO_SERIAL
  SERIAL_OUT("Sampling speed now at ");
  SERIAL_OUT2(sample_speed_settings[current_sample_speed_idx].percentage, DEC)
  SERIAL_OUTLN("%");
  delay(1000);
#endif
  
}




/*
 *****************  Headlight control  ***********************
 *
*/
void headlights_on()
{
  if (!headlights_currently_on)
  {
    SERIAL_OUTLN("Turning headlights ON");
    
    headlights_currently_on = true;
    
    HEADLIGHT_CTRL_PORT  |= HEADLIGHT_ON_BIT;

    setup_colour_calibration();
  }
}

void headlights_off()
{
  if (headlights_currently_on)
  {
    SERIAL_OUTLN("Turning headlights OFF");
    
    headlights_currently_on = false;
    
    HEADLIGHT_CTRL_PORT &= HEADLIGHT_OFF_MASK;
    
    setup_colour_calibration();
  }
}

/*
 ***********************  SPI   ****************************
 *
 * SPI (digital pots/SPI interface) related methods
 *
 */
void setup_spi()
{
  
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  SPI.begin();
  
  pinMode(12, OUTPUT);
  
}




void spi_write(byte channel, byte value, byte address=0)
{
  spi_control_details[channel].chip_select(spi_control_details[channel].clear_mask);
  // delayMicroseconds(4);
  
  if (address == 0)
  {
    address = spi_control_details[channel].command;
  }
  
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
 
 
  spi_control_details[channel].line_release(spi_control_details[channel].clear_mask);
  
  
}



void spi_write_all(byte value, byte address=0) 
{
  // select ALL channels
  // TODO:FIXME -- SPI SELECT??
  // SPI_OUT_SELECT_PORT &= ~SPI_OUT_SELECT_CLEAR;
  
  
  // delayMicroseconds(4);
  
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
 
  // SPI_OUT_SELECT_PORT |= SPI_OUT_SELECT_CLEAR ;
  
}



void quiet_all()
{
  // quiet everybody
  spi_write_all(0);
}






/*
 * Output results using SPI/summing amp:
 *
 */


void adjust_outputs_for_sampletypes(byte gain=0)
{
  
  byte max_setting = stypes[0].resulting_setting;
  float eq = stypes[0].equalizer_setting;
  
  unsigned int colour_total = 0;
  
  
  // adjust our target settings, while tracking the colour total and max value
  for (byte i=0; i< STYPE_LEVEL_IDX; i++)
  {  
    if (stypes[i].resulting_setting > max_setting)
    {
      max_setting = stypes[i].resulting_setting;
      eq = stypes[i].equalizer_setting;
    }
    
    colour_total += stypes[i].resulting_setting;
    spi_adjust_values.target_settings[i] = stypes[i].resulting_setting;
    
  }
  
  if (gain)
  {
    // gain was specified in call, just use that
    spi_adjust_values.target_settings[STYPE_LEVEL_IDX] = gain;
    
  } else {
    
    
    
    
    // gain not specified in call, figure it out
    float desired =  1.0;
    if (colour_total > MIN_TOTAL_COLOUR_FOR_OPS && max_setting < MIN_GAIN_SETTING_TARGET)
    // if (colour_total > MIN_TOTAL_COLOUR_FOR_OPS && colour_total < MIN_GAIN_SETTING_TARGET)
    {
      desired =  MIN_GAIN_SETTING_TARGET / (max_setting + 1.0);
      if (desired > 10.0)
      {
        desired = 10.0;
      }
    }
    
    /*
    SERIAL_OUT("Desired = ");
    SERIAL_OUTLN2(desired, DEC);
    */
    
    /* DEADBEEF... 
    // if we have a signal present in low lighting conditions, it is in fact more 
    // important than a greater signal in high lighting--since it's getting through regardless
    // of the lack of light
    
    // with our circuit, the feedback network is 5k on input/200R-50k on feedback, meaning a gain between
    // 0.04 - 10, passing by 1 at 5k (i.e. around the "25" mark in the gain pot setting)
    // thus desired gain between ~0 and 10) is:
    // setting = desired * ((5.1k/50k) * 255)
    
    // proportional to the colour_total - level diff
    // inversely proportional to color total...
    SERIAL_OUT("Desired = 1.8 * ");
    SERIAL_OUT2(colour_total, DEC);
    SERIAL_OUT(" / 1.0 + ");
    SERIAL_OUTLN2(stypes[STYPE_LEVEL_IDX].resulting_setting, DEC);
    
     (1.8  * colour_total) / (stypes[STYPE_LEVEL_IDX].resulting_setting + 1.0);
    
    if  ((desired * colour_total) > (255 * 2))
    {
      SERIAL_OUT("Desired (");
      SERIAL_OUT2(desired, DEC);
      SERIAL_OUTLN(") too high for current color total, reduced to max");
      desired = 7;
      
      // desired = colour_total/(255 * (SUMMING_AMP_IN_RES/SUMMING_AMP_FEEDBACK_RES_MAX));
      
    } else if (desired < MIN_GAIN_SETTING_TARGET)
    {
      
      SERIAL_OUT("Desired (");
      SERIAL_OUT2(desired, DEC);
      SERIAL_OUTLN(") below min, augmented to minimum.");
      
      desired = MIN_GAIN_SETTING_TARGET;  
      
    }
   
    
    // float desired = MIN_GAIN_SETTING_TARGET / (stypes[STYPE_LEVEL_IDX].resulting_setting + 1);
    
    */
    
    
    byte setting = desired * (( SUMMING_AMP_IN_RES/SUMMING_AMP_FEEDBACK_RES_MAX)*250.0);
    /*
    SERIAL_OUT("Setting :");
    SERIAL_OUTLN2(setting, DEC);
    */
#ifdef EQUALIZER_ENABLE

    if ( eq * setting > 255.0)
    {
      setting = 255;
    } else {
      setting = (byte) (round(eq * (float)setting));
    }
    
    /*
    SERIAL_OUT("After EQ:");
    SERIAL_OUTLN2(setting, DEC);
    */
    
#endif
    /*
    
    unsigned int target_gain_from_light_level = MIN_GAIN_SETTING_TARGET * 255/((stypes[STYPE_LEVEL_IDX].resulting_setting + 1) * 2);
    
    if (target_gain_from_light_level > 255)
    {
      target_gain_from_light = 255;
    }
    
    */
    
    // byte gain_setting = MIN_GAIN_SETTING_TARGET + (max_setting * (255 - MIN_GAIN_SETTING_TARGET)/255);
    
    /*
    if (gain_setting < 128)
    {
      gain_setting *= 2;
        
        if (gain_setting < 10)
        {
          gain_setting = 10;
        } 
    } else {
      gain_setting = 255;
    }
    */
    
    spi_adjust_values.target_settings[STYPE_LEVEL_IDX] = setting;
    // spi_adjust_values.target_settings[STYPE_LEVEL_IDX] = max_setting;
    
    // spi_write(SPI_GAIN_CHANNEL_BIT, gain_setting);
  }
  
  // make note that we've made changes to our target values
  spi_adjust_values.changes_required = true;
  
}









// song_play_start() sets up the song to play on next pass through 
// main loop of state machine
void song_play_start(struct colour_chord_struct * start_of_song_array)
{
  
  song_playing_next_note_time = 0;
  song_playing_chord = start_of_song_array;
}


// song_play_done() ensures we cease playing (called when the song end marker reached)
void song_play_done()
{
  // reset our pointer
  song_playing_chord = 0;
  song_playing_next_note_time = 0;
  // quiet_all();

}

/*
 ***************** Main Arduino functions ********************
*/


void setup()
{
  // start by setting *everything* to output...
  DDRD = 0xFF;
  DDRC = 0xFF;
  DDRB = 0xFF;
  
  
  //
  // TCCR0B = _BV(CS01) ; //| _BV(CS00);
 
  // 
  //
  
  // TCCR1B = _BV(CS10);
  
  Timer1.initialize(DELAY_BETWEEN_TICKS_uSECONDS/TIMER_ONE_PERIOD_DIVISOR);
  
  // 
  TCCR2B = _BV(CS20);
  
  // we have two inputs that we want to monitor with interrupts.  Set 'em as inputs firstly:
  pinMode(FREQ_SAMPLE_INPUT_PIN, INPUT);
  pinMode(CALIBRATE_REQ_INPUT_PIN, INPUT);
  
  pinMode(AUDIO_OUT_PIN_C3, OUTPUT);
  analogWrite(AUDIO_OUT_PIN_C3, 0);
  
  pinMode(AUDIO_OUT_PIN_E4, OUTPUT);
  //analogWrite(AUDIO_OUT_PIN_E4, 0);
  Timer1.pwm(AUDIO_OUT_PIN_E4, 0);
  
  pinMode(AUDIO_OUT_PIN_G5, OUTPUT);
  // analogWrite(AUDIO_OUT_PIN_G5, 0);
  
  Timer1.pwm(AUDIO_OUT_PIN_G5, 0);
  
  
  
  
  
#ifdef OUTPUT_DEBUG_TO_SERIAL
  
  Serial.begin(57600);        // connect to the serial port
  delay(800);
  
  Serial.println("startup!");
  delay(300);
#endif


  
  // ensure our device has been properly initialized
  verify_device_initialization();
  
  
  // we'll be communicating our findings via SPI, set that up:
  setup_spi();
  
  quiet_all();
  
  // init the spi_adjust_values to 0 for all
  spi_adjust_values.changes_required = false;
  for (byte i=0; i < NUM_SAMPLE_TYPES; i++)
  {
    spi_adjust_values.current_toggle[i] = false;
    spi_adjust_values.current_settings[i] = 0;
    spi_adjust_values.target_settings[i] = 0;
  }
  
#ifdef CALIBRATE_ON_STARTUP
  calibrate_isr();
#else
  
  setup_colour_calibration();
#endif

  // DEADBEEF: delay for testing...  TODO:FIXME/REMOVEME...
  // delay(2000);
  
#ifndef CALIBRATE_REQ_HANDLED_MANUALLY
  attachInterrupt(CALIBRATE_REQ_INTERRUPT_NUM, calibrate_isr, RISING);
#endif

#ifdef TEST_AUDIO_STAGE  
  song_play_start(TEST_AUDIO_SONG_TO_PLAY);
#endif

  song_play_start(startup_song);
}


void loop()
{


  byte current_sample_type_idx = 0;
  
  boolean all_samples_collected = false;
  boolean ready_to_adjust_settings = false;
  
  
  float scale_factor = 1.0;
          
  unsigned int colour_total = 0;
  boolean sampling = false;
  byte sample_freq_end_in = 0;
  byte sample_freq_start_in = 0;
  

  Timer1.attachInterrupt(generateTick);
  

#ifdef OUTPUT_DEBUG_TO_SERIAL
  unsigned int output_count = 0;
  boolean output_debug_info = false;
#endif
  
  while(1)
  {

#ifdef OUTPUT_DEBUG_TO_SERIAL
  if (output_count % OUTPUT_DEBUG_INFO_EVERY ==  0)
  {
    output_debug_info = true;
    // SERIAL_OUTLN(" ");
    
  } else {
    output_debug_info = false;
  }
#endif    

      while (waitingfortick)
      {
        delayMicroseconds(8);
      }
      waitingfortick = true;
      
      // first things first: do our time sensitive audio output stuff first.
      // G5 is the highest frequency sound, flips a bit every 5 ticks
      if (tickcount % 5 == 0)
      {
        if(spi_adjust_values.current_toggle[G5BIT])
        {
          // analogWrite(AUDIO_OUT_PIN_G5, 0);
          
          Timer1.pwm(AUDIO_OUT_PIN_G5,0);
          spi_adjust_values.current_toggle[G5BIT] = false;
        } else {
          // analogWrite(AUDIO_OUT_PIN_G5, spi_adjust_values.current_settings[G5BIT]);
          
          Timer1.pwm(AUDIO_OUT_PIN_G5, spi_adjust_values.current_settings[G5BIT] * 4);
          spi_adjust_values.current_toggle[G5BIT] = true;
        }
        
        // DEADBEEF: flipbit(currentstate, G5BIT);
        
        // we piggy back here, to keep our sampling subsystem going
        // at a decent rate.
        
        // start sampling at next *odd* tickcount
        // to ensure we're not interfering with any of our sound
        // generation
        
        if (sampling)
        {
          
              
          if (freq_sample_count > MAX_SAMPLE_COUNT_FOR_SLOWDOWN && current_sample_speed_idx > 0)
          {
              // we've already got too many samples to handle... sloooow down
              // as long as we're not currently calibrating
              
              if (headlights_currently_on && current_sample_speed_idx == 1)
              {
                headlights_off();
                RESET_TO_RESTING_STATE();
              }
              
              if (! calibrating)
              {
                sample_speed_throttle_down();
  
                // restart our sampling
                RESET_TO_RESTING_STATE();
              }
          }
     
        } else if (! (all_samples_collected || ready_to_adjust_settings))
        {
          // neither sampling, nor handling collected samples.
          
          // the next odd tick count is 1 or 2 ticks away, but since 
          // we're checking this stuff right _after_ this code, in the same
          // loop, we augment both counts by 1.
          if (tickcount % 10 == 0)
          {
            sample_freq_start_in = 2;
          } else {
            sample_freq_start_in = 3;
          }
          
          
        }
      }
      
      
      // E4, the middle frequency, need a bit flip every 12 ticks
      if (tickcount % 12 == 0)
      {
        
        if(spi_adjust_values.current_toggle[E4BIT])
        {
          //analogWrite(AUDIO_OUT_PIN_E4, 0);
          Timer1.pwm(AUDIO_OUT_PIN_E4, 0);
          spi_adjust_values.current_toggle[E4BIT] = false;
        } else {
          // analogWrite(AUDIO_OUT_PIN_E4, spi_adjust_values.current_settings[E4BIT]);
          
          Timer1.pwm(AUDIO_OUT_PIN_E4, spi_adjust_values.current_settings[E4BIT] * 4);
          
          spi_adjust_values.current_toggle[E4BIT] = true;
        }
        // DEADBEEF flipbit(currentstate, E4BIT);
      }
      
      
      // C3, the lowest freq, gets flipped every 30 ticks.
      if (tickcount % 30 == 0)
      {
        
        if(spi_adjust_values.current_toggle[C3BIT])
        {
          analogWrite(AUDIO_OUT_PIN_C3, 0);
          spi_adjust_values.current_toggle[C3BIT] = false;
        } else {
          analogWrite(AUDIO_OUT_PIN_C3, spi_adjust_values.current_settings[C3BIT]);
          
          spi_adjust_values.current_toggle[C3BIT] = true;
        }
        
        // DEADBEEF: flipbit(currentstate, C3BIT);
      }
      
      
      // output current state
      // DEADBEEF: AUDIO_OUT_PORT = currentstate;
      

#ifdef CALIBRATE_REQ_HANDLED_MANUALLY
      if (digitalRead(CALIBRATE_REQ_INPUT_PIN))
      {
        // the calib request pin is low
        if (! calibrating)
        {
          // the calib_request has not been made yet... do it.
          calibrate_isr();
        }
      }
#endif
        
        
      
      
      
      // now that the audio generation has been handled, ensure that we are at our target output values
      // or increment towards them
      if (spi_adjust_values.changes_required && tickcount % 3 == 0)
      {
        // we need to make some adjustments... find out which and perform 'em
        
        spi_adjust_values.changes_required = false; // assume no more changes after this-
        
        
        for (byte i=0; i < NUM_SAMPLE_TYPES; i++)
        {
          if (spi_adjust_values.current_settings[i] != spi_adjust_values.target_settings[i])
          {
            
            // an adjustment to this indexes setting needs to be made
            
        
#ifdef SMOOTH_SPI_ADJUSTMENTS

            // we are using smooth adjustments (which reduce popping and sound better)
            if (spi_adjust_values.current_settings[i] > spi_adjust_values.target_settings[i])
            {
              spi_adjust_values.current_settings[i]--;
            } else {
              spi_adjust_values.current_settings[i]++;
            }
#else
            // we aren't using smooth adjustments (thereby reducing the amount of SPI data transfered)
            spi_adjust_values.current_settings[i] = spi_adjust_values.target_settings[i];
#endif
            
            spi_adjust_values.changes_required = true;
           
            // TODO::FIXME XXXXXXX need to write RGB using "false" SPI select, and LEVEL using PD7...
            spi_write(i, spi_adjust_values.current_settings[i]);
            
          } // end if we are still approaching target value for this channel
         
        } // end loop over each SPI channel
        
      } // end if we have any SPI output changes to make
        

      
      
      /*
        Here is our little sampler/adjustment state machine.  It is big and monolithic, yes... it is designed to
        minimize overhead of function calling and parameter passing and use of global flags... not so pretty, but
        works nicely.
        
        It splits up its work such that different parts complete a task, change the state flags and wait for the next
        tick to proceed processing in the appropriate state.  In this manner, no operations disrupt the timing of our
        audio wave generation.
        
        It basically goes through these states, in order:
        
          * Resting... on the next beat of our highest frequency audio -> sample_freq_start_in set to a value > 1
      
          * Waiting to start sampling (sample_freq_start_in > 1) -> sample_freq_start_in --
          
          * Start sampling (sample_freq_start_in == 1) -> sampling = true
          
          * Sampling (sampling == true, sampling_freq_end > 1) -> sampling_freq_end --
          
          * Sampling Done (sample_freq_end == 1) -> sampling = false, all_samples_collected = true
          
          * Samples Collected (all_samples_collected == true) -> ready_to_adjust_settings = true
          
          * Adjusting Settings (ready_to_adjust_settings == true) -> ready_to_adjust_settings  = false, settings set. Resting.
          
          Note: there is are a few exception to this flow... 
            changing the sample input clock divisor resets us to resting phase.
            playing a song takes priority over everything, so we can provide feedback to user prior to doing stuff (e.g. calib)
            
          
       */
   
   
      // The state selector:
      if (song_playing_chord != 0)
      {
        // STATE: Playing a song
        
        // we're in the process of playing a song
        unsigned long now_time = millis();
        if (now_time >= song_playing_next_note_time)
        {
          // SERIAL_OUT("Time to play next note ( ");
          // time has arrived to switch notes
          song_playing_chord++;
          
          // SERIAL_OUT2((int)song_playing_chord, DEC);
          
          // SERIAL_OUT(") until ");
          
          
          
          if ( (*song_playing_chord).duration == SONG_END_MARKER_DURATION)
          {
            // we've hit our end of song marker
            song_play_done();
            
#ifdef TEST_AUDIO_STAGE
            // we'll just loop the test song, if we're testing the audio stage
            
            song_play_start(TEST_AUDIO_SONG_TO_PLAY);
#endif
          } else {
            // we have another note to play...
            
            // make note of "until when"
            
            song_playing_next_note_time = now_time + (*song_playing_chord).duration;
            
            // piggyback on our sampling system to play the note
            for (byte i=0; i < STYPE_LEVEL_IDX; i++)
            {
              stypes[i].resulting_setting = (*song_playing_chord).components[i];
            }
            
            // SERIAL_OUTLN2(song_playing_next_note_time, DEC);
            
            adjust_outputs_for_sampletypes();
            
          } // end if we've hit the end-of-song marker
        } // end if we've reached the next note to play
        
      } else if (sample_freq_start_in)
      {
        // STATE: WAITING TO START SAMPLING
        
        sample_freq_start_in--;
        if (sample_freq_start_in == 0)
        {
          // STATE TRANSITION: SAMPLING NOW
          
          
          // select frequency, arrr!
          
          // by first clearing me bits
          STYPE_PORT &= STYPE_CLEARMASK;
          
          // and then setting me bits
          STYPE_PORT |= stypes[current_sample_type_idx].type_mask;
          
         
          // reset counter
          freq_sample_count = 0;
          
          // enable freq counter interrupt.
          attachInterrupt(FREQ_SAMPLE_INTERRUPT_NUM, freq_count_isr, FALLING);
           
          sampling = true;
          
          sample_freq_end_in = FREQ_SAMPLE_LEN_TICKS;
 
        } // end if we're starting to sample now
        
      } else if (sampling)
      {
        // STATE: SAMPLING 
       
        sample_freq_end_in --;
        if (sample_freq_end_in == 0)
        {
          
          // STATE TRANSITION: SAMPLING DONE
          
          
          // disable freq counter interrupt
          detachInterrupt(FREQ_SAMPLE_INTERRUPT_NUM);
          
          // clear our "now sampling" flag
          sampling = false;
          
          
          if (freq_sample_count < MIN_SAMPLE_COUNT_FOR_SPEEDUP)
          {
            
            if ( (!headlights_currently_on) && current_sample_speed_idx >=  (NUM_SAMPLE_SPEED_SETTINGS - 1) )
            {
              headlights_on();
              
              RESET_TO_RESTING_STATE();
            }
            
            if (current_sample_speed_idx < (NUM_SAMPLE_SPEED_SETTINGS - 1) )
            {
              // switch to high speed, as long as we're not currently calibrating
              if (! calibrating)
              {
          
                SERIAL_OUTLN("Moving to higher speed sampling");
                sample_speed_throttle_up();
          
                RESET_TO_RESTING_STATE();
              }
            }
            
          }
          
#ifdef OUTPUT_DEBUG_TO_SERIAL
          if (output_debug_info)
          {
            SERIAL_OUT("Raw sample count for ");
            SERIAL_OUT(stypes[current_sample_type_idx].name);
            SERIAL_OUT(": ");
            SERIAL_OUTLN2(freq_sample_count, DEC);
          }
#endif
          
#ifdef RESPONSIVITY_ADJUSTMENT_ENABLE
          // get the sample count, adjusted for particular frequency, and add 1 to ensure always > 0
          stypes[current_sample_type_idx].sample_count = (int)(stypes[current_sample_type_idx].calibration_factor * freq_sample_count  *
                                                                   stypes[current_sample_type_idx].responsivity_adjustment);
#else
         
          stypes[current_sample_type_idx].sample_count = (int)(stypes[current_sample_type_idx].calibration_factor * freq_sample_count);
           
#endif


#ifdef OUTPUT_DEBUG_TO_SERIAL
          if (output_debug_info)
          {
            SERIAL_OUT(" -- After calib:");
            SERIAL_OUTLN2(stypes[current_sample_type_idx].sample_count, DEC);
          }
#endif



    
          if (stypes[current_sample_type_idx].sample_count < 1)
          {
            stypes[current_sample_type_idx].sample_count = 1;
          }
      
          
          
          // move on to next sample type
          current_sample_type_idx++;
          
          if (current_sample_type_idx >= NUM_SAMPLE_TYPES)
          {
            
            // STATE TRANSITION: ALL SAMPLES COLLECTED
              
            // loop back to first sample type, when we start sampling again
            current_sample_type_idx = 0;
            
            if (calibrating)
            {
              
              SERIAL_OUTLN("Done collecting calibration samples");
              // this sample is actually part of our calibration set, we'll just add it to our list of samples collected
              
              // stash our current sample set
              for (int i=0; i<NUM_SAMPLE_TYPES; i++)
              {
                calib_samples[calibrate_samples_to_collect - 1][i] = stypes[i].sample_count;
              }
              
              calibrate_samples_to_collect--;
              if (calibrate_samples_to_collect == 0)
              {
                
                // we've got our last sample... 
                // get out of calibration mode
                calibrating = false;
                
                // actually perform the calibration.

                perform_calibration();
              }
              
              RESET_TO_RESTING_STATE();
                
            }
            
            
            // oh myes, we seem to have all we need.  Do a bit of processing, and set the state to all_samples_collected.
            all_samples_collected = true;
            
            
            // process our sample counts to produce preliminary resulting_settings (between 0-255 for each sample type)
            
            
            
            
            
            
          } // end if we have all our required samples
          
        } // end if sampling is done 
      
      } else if (all_samples_collected) {
          
        // STATE: ALL SAMPLES COLLECTED
          
        // reset the all samples collected flag
        all_samples_collected = false;
        
            
            
        
         // because the diff can be large, on the kHz scale, but gets squished together, we need to eliminate a bit of the overhead/sameness from ambient light level...
         
         // first, figure out the highest/lowest sample counts
         int lowest_sample_count = stypes[0].sample_count;
         int highest_sample_count = stypes[0].sample_count;
         for (byte i=1; i<STYPE_LEVEL_IDX; i++) // start at index 1 because we've already assumed idx 0 had both highest and lowest...
         {
              if (stypes[i].sample_count < lowest_sample_count)
              {
                 lowest_sample_count = stypes[i].sample_count;
              }
              
              if (stypes[i].sample_count > highest_sample_count)
              {
                highest_sample_count = stypes[i].sample_count;
              }
              
         }
         
         
        
#ifdef BOOST_GREEN_ENABLE
       // our green detectors have a very narrow band and have a certain lack of presence in the signal without a little boost
       // when green is close/higher than blue and red
       if ((stypes[STYPE_GREEN_IDX].sample_count * 5) > (stypes[STYPE_BLUE_IDX].sample_count * 4)
         && (stypes[STYPE_RED_IDX].sample_count * 3 < stypes[STYPE_GREEN_IDX].sample_count * 4) )
       {
         stypes[STYPE_GREEN_IDX].sample_count +=  (stypes[STYPE_GREEN_IDX].sample_count * 5) - (stypes[STYPE_BLUE_IDX].sample_count * 4);
       #ifdef OUTPUT_DEBUG_TO_SERIAL
          if (output_debug_info)
          {
            SERIAL_OUTLN(" * After GREEN BOOST *");
            output_sample_counts();
          }  
      #endif
       }
#endif
         
            
#ifdef SAMPLE_DELTA_STRETCH_ENABLE
            
            // we want a given delta between lowest_sample_count and highest_sample_count to "count more" -- be stretched out -- when lighting is low
            // since a difference of, say, 50 samples for colours under low lighting is a more important diff than when light level is high
            
            // say 200 vs 250 @ 300 level compared to 
            // 200 vs 250 @ 1100 level
            
  #ifdef OUTPUT_DEBUG_TO_SERIAL
            if (output_debug_info)
            {
              SERIAL_OUTLN("Samples vals before delta stretch");
              output_sample_counts();
            }  
  #endif

            
            if (lowest_sample_count > 5 && (highest_sample_count  * 20 > (lowest_sample_count * 23))) // if highest sample count is at least 15% > than lowest
            {

         
              float adj = 1.0 + ((float)(highest_sample_count - lowest_sample_count) * SAMPLE_DELTA_STRETCH_FACTOR /  stypes[STYPE_LEVEL_IDX].sample_count);
              
              
            
              // int remove_this = (int)(lowest_sample_count * 99.0 / 100.0);
              // int remove_this = lowest_sample_count - 10;
              for (int i=0; i<STYPE_LEVEL_IDX; i++)
              {

                //Serial.println(stypes[i].sample_count, DEC);
                stypes[i].sample_count = stypes[i].sample_count  + (int)( (stypes[i].sample_count - lowest_sample_count) * adj);
                //Serial.println(stypes[i].sample_count, DEC);
                
              }
              
              
              
                
            }
            
              /* DEADBEEF?
              for (int i=0; i<STYPE_LEVEL_IDX; i++)
              {
                stypes[i].sample_count -= lowest_sample_count/2;
              }
              */
            
#endif
            
            
        /*
              now, we have all our samples nice and fresh and stored as a value between 0-255.
              What we want is:
                  * to adjust for low levels of lighting, by scaling upward
                  * to minimize distortion due to clipping, by keeping the max peak of the 
                    resulting output at a reasonable position
              
              Our technique: 
                  * Take the sum of all colours.  
                  * If signal is through the roof, scale down to reasonable level
                  * If there's "room at the top" (i.e. we can scale up without undue distortion),
                    do so, but in such a way that accounts for global light levels.
                    
              To clarify that last point, an example: the sum of all colours is, say, around 50% of max.
              If the light level is very high, this means that it is a dark colour... no upscaling!
              If the light level is very low, this means that it is a bright colour--in the dark... upscale to max.
              In between those two poles, scale linearly.
        */  
        
 
            int incident_light_correction;  
#ifdef INCIDENT_LIGHT_CORRECTION_ENABLE
            incident_light_correction = (int)((float)stypes[STYPE_LEVEL_IDX].sample_count / INCIDENT_LIGHT_CORRECTION_FACTOR);
#else
            incident_light_correction = 0;
#endif
            
                
            // scale all our sample values over a single byte (max 255) and keep track of the "total colour"
            colour_total = 0;
            // do the colours
            for (int i=0; i<STYPE_LEVEL_IDX; i++)
            {
              
                if (stypes[i].sample_count > incident_light_correction)
                {
                    stypes[i].resulting_setting = ((stypes[i].sample_count - incident_light_correction) * sample_speed_settings[current_sample_speed_idx].scaling_factor);
                } else {
                  
                  
                    
                    stypes[i].resulting_setting = 1;
                  
                    /*
                    stypes[i].resulting_setting = (lowest_sample_count - 
                                                    (lowest_sample_count * ((incident_light_correction - stypes[i].sample_count)/incident_light_correction)))
                                                    * sample_speed_settings[current_sample_speed_idx].scaling_factor;
                    */
                    
                    
                    
                    
                    
                    
                    
                    
                }
              
              
          
              // keep score of our total
              colour_total += stypes[i].resulting_setting;
            }
            
            // don't forget the general "level" sample...
            stypes[STYPE_LEVEL_IDX].resulting_setting = (stypes[STYPE_LEVEL_IDX].sample_count * sample_speed_settings[current_sample_speed_idx].scaling_factor);
            if (stypes[STYPE_LEVEL_IDX].resulting_setting < 1)
            {
              // never allow this to be 0... divide by zero: not c00l.
              stypes[STYPE_LEVEL_IDX].resulting_setting = 1;
            }
              
              
          // and we're done.
            
        // adjust everything on the next tick
        ready_to_adjust_settings = true;
              
      } else if (ready_to_adjust_settings)
      {
        // reset the flag
        ready_to_adjust_settings = false;
        
        
#ifdef OUTPUT_DEBUG_TO_SERIAL
        if (output_debug_info)
        {
          SERIAL_OUT("Colour total:");
          SERIAL_OUTLN2(colour_total, DEC);
          SERIAL_OUT("Ready To adjust... ****************************");
          output_sample_counts();
        }  
#endif

       
#ifdef NORMALIZE_SETTING_RANGE_ENABLE
        
        scale_factor = 1.0;
          
        // if total colour sample is too low, just forget about it, there's nothing to see here
        if (colour_total > MIN_TOTAL_COLOUR_FOR_OPS)
        {
          // ok, we do have a bit of input to work with... proceed with scaling.
           
          float max_scale_factor;
          
          max_scale_factor = 300.0/(float)colour_total;
          if (colour_total > 300)
          {
            // scale down...
            scale_factor = max_scale_factor;
          } else {
            // scale up, maybe...
            //
            // DEADBEEF: ? ?? if (stypes[STYPE_LEVEL_IDX].resulting_setting < (255 - MAX_LEVEL_FUDGE_FACTOR))
            if (1) // TODO: FIXME... 
            {
              // a simple 255/resulting_setting would scale up according to light level,
              // but we'd go through the roof sometimes and get a distorted signal.
              // to account for this, 
              // scale_factor = 255.0/(float)stypes[STYPE_LEVEL_IDX].resulting_setting;
              scale_factor = 255.0/(float)colour_total;
                
              
              // DEADBEEF ? (pointless)
              if (scale_factor > max_scale_factor)
              {
                // light's low but colour's too bright to scale all the way to compensate
                // just scale it as much as is reasonable.
                scale_factor = max_scale_factor;
              }
             
            } // end if we want to scale up
              
          } // end if colour_total exceeds bounds
          
          
        } // end if we have any colour signal actually present.
          
          
#ifdef OUTPUT_DEBUG_TO_SERIAL

        if (output_debug_info)
        {
          SERIAL_OUT("Colour total is ");
          SERIAL_OUT2(colour_total, DEC);
          SERIAL_OUT(" so scale factor calculated to be: " );
          SERIAL_OUTLN2(scale_factor, DEC);
          
        }
#endif
          for (int i=0; i< STYPE_LEVEL_IDX; i++)
          {
            stypes[i].resulting_setting = (unsigned int)((float)stypes[i].resulting_setting * scale_factor);
          }
#endif
            
          adjust_outputs_for_sampletypes();
          
          
        
#ifdef OUTPUT_DEBUG_TO_SERIAL

        // time to output all the colours
        if (output_debug_info)
        {
          output_sample_counts();
        }
        
        output_count++;
 
 #endif
 
        
      }  // end state machine flag switch
      
  } // end forever while loop
} // end loop()


#ifdef OUTPUT_DEBUG_TO_SERIAL

void output_sample_counts()
{
  
          Serial.println("Samples:");
            
          for(int i=0; i<NUM_SAMPLE_TYPES; i++)
          {
            Serial.print(stypes[i].name);
            Serial.print(": ");
            Serial.print(stypes[i].sample_count, DEC);
            Serial.print(" ");
          }
          Serial.println(" ");
          Serial.println("Settings:");
          for(int i=0; i<NUM_SAMPLE_TYPES; i++)
          {
            Serial.print(stypes[i].name);
            Serial.print(": ");
            Serial.print(stypes[i].resulting_setting, DEC);
            Serial.print(" ");
            
          }
          Serial.println(" ");
}

#endif
