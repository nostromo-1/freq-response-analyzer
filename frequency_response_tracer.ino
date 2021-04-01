
/*************************
This Arduino program uses the AD9834 DDS to test the frequency response of an audio amplifier.
It uses a peak detector and plots the result in a SSD1306 128x64 OLED display.
It is controlled by a rotary encoder with a push button.
If you press the push button after reset of Arduino, it enters SINGLE mode.
In this mode, it generates a single frequency, controlled by the rotary encoder.

Otherwise, it enters SWEEP mode. First, it generates a tone of 1000 Hz, and displays a scope screen
of the peak detector input. In this screen, you can adjust the input level (via the volume of the amplifier 
or the input potentiomenter of the peak detector). It should be a clean sine wave, not clipped.
When ready, press the pushbutton, and the sweep will start.
 
*************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ad983x.h>
#include <U8g2lib.h>

#define BASELINE (display.getDisplayHeight()-5)
#define MIN(a,b) (((a)<(b))?(a):(b))
#define HALT for(;;)


/* DDS declarations */
#define FSYNC_PIN 9   // Pin connected to FSYNC (SS) of AD983X
#define AD_MCLOCK 75  // Clock speed of AD983X in MHz
AD983X_SW AD(FSYNC_PIN, AD_MCLOCK);
static uint8_t bank;

/* Declaration for an SSD1306 display connected to I2C (SDA, SCL pins) */
#define DISPLAY_I2C_ADDR 0x3C
U8G2_SSD1306_128X64_NONAME_2_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

/* Encoder and pushbutton declarations */
#define encoderPinA PD2  // PORTD2=2. Must be an external interrupt pin. 2 or 3 in nano
#define encoderPinB PD4  // PORTD4=4. Must be in PORTD, as encoderPinA
#define pushButton  PD5  // Push button to pin 5, PD5
#define BUTTONPUSHED (~PIND & _BV(pushButton))

const uint8_t encoderPins = _BV(encoderPinA) | _BV(encoderPinB);
volatile int32_t encoderVal;
// Tasks set in interrupts for loop. Maximum 8. 
volatile uint8_t tasks;
enum {ROTARY=1, OTHER};


/* Sketch declarations */
#define PEAKDETECTORPORT A0   // ADC port where the peak detector is connected 
#define SCOPEPORT A1          // ADC port where the input of the peak detector is connected 
enum {SINGLE, SWEEP} mode;
const long f0 = 60;
const long f1 = 20000;
const float delta_f = 50.0f;  // Step size in Hz
uint8_t Vdc;
uint16_t Vcc;  // Voltage supply level

/* Linear frequency increase */
const int step_delay = 50;  // ms. Must be > 10 and > 3 times 1/f0
const long duration = (f1-f0)/(delta_f/step_delay);  // Milliseconds
const int steps = duration/step_delay;
uint8_t *samples;


const char *Hz = " Hz";


void setup() {
  int i, v;
  
  Serial.begin(9600);
  Wire.begin();
  SPI.begin();  
  display.setI2CAddress(DISPLAY_I2C_ADDR<<1);
  display.begin();
  AD.begin();
  AD.setOutputMode(OUTPUT_MODE_SINE);

  Vcc = readVcc();  // Read power supply voltage level (mV), for ADC reference
  Serial.print("Board power supply voltage: "); Serial.print(Vcc/1000.0f); Serial.println(" V");
  
  Serial.print("Steps: "); Serial.println(steps); 
  samples = malloc(steps+1);
  if (samples==NULL) {
    Serial.println(F("Memory allocation failed"));
    HALT;
  }

  Serial.print("Step delay: "); Serial.print(step_delay); Serial.println(" ms");
  Serial.print("Delta f: "); Serial.print(delta_f); Serial.println(Hz);
  Serial.print("Duration: "); Serial.print(duration/1000.0f); Serial.println(" sec");
  
  if (step_delay < 3000/f0 || step_delay < 10) {
    Serial.println(F("Step delay is too small"));
    HALT;
  }

  // Encoder and pushbutton
  DDRD &= ~_BV(DDD2);    // Pin D2 (PD2) as input, it is used as external interrupt INT0
  DDRD &= ~encoderPins;  // Set encoder0PinA and encoder0PinB as inputs
  PORTD |= encoderPins;  // Turn on pullup resistors in encoder0PinA and encoder0PinB
  DDRD &= ~_BV(pushButton);   // Set PD5 as input (push button)
  PORTD |= _BV(pushButton);   // Turn on pullup resistor in PD5
  EICRA = _BV(ISC00);  // sense any change on the INT0 pin
  EIMSK = _BV(INT0);   // enable INT0 interrupt
  delay(1000); // Wait for the pullup resistor to settle and for the peak detector to stabilize at null amplifier output

  // Read input voltage with no signal applied to amplifier, ie, base DC level of peak detector
  for (v=0, i=0; i<10; i++) {
    v += analogRead(PEAKDETECTORPORT);  
    delay(1);
  }
  v /= 10;
  Vdc = uint8_t(v>>2);   // 8 bits resolution

  if (BUTTONPUSHED) mode = SINGLE;
  else mode = SWEEP;

  if (mode == SINGLE) encoderVal = 1000;  // Initial frequency for generator
  else {
    adjust_input();
    bank = 1 - bank;
    AD.setFrequency(bank, float(f0));
    AD.selectFrequency(bank);
    delay(300);    
    }

}



void loop() {
  static int samples_count;
  static float freq=f0;
  static bool first=true;
  unsigned long currentMillis;
  static unsigned long previousMillis;  // store last time frequency was changed

  if (mode == SINGLE) {
    if (first || (tasks & _BV(ROTARY))) {  // If task was set in interrupt
      display.firstPage();
      display.setFont(u8g2_font_6x10_tr);
      display.setFontPosTop();  // Put the (0,0) at the top left corner of the glyph instead of bottom left
      do {
        display.setCursor(0,0);
        display.print("SINGLE");
        display.setCursor(0,10);
        display.print("Frequency: "); display.print(encoderVal); display.print(Hz);
      } while ( display.nextPage() );
      first = false;
      bank = 1 - bank;
      AD.setFrequency(bank, float(encoderVal));
      AD.selectFrequency(bank);
      tasks ^= _BV(ROTARY);  // Clear task. Must be last line
    }
    return; 
  }

  // SWEEP mode
  if (BUTTONPUSHED) {  // Emergency stop
    draw_freq_response(samples_count);
    HALT;  
  }

  currentMillis = millis();
  if (previousMillis == 0) previousMillis = currentMillis;

  if (currentMillis - previousMillis >= step_delay) {
    samples[samples_count++] = uint8_t(analogRead(PEAKDETECTORPORT)>>2);
    freq += delta_f;  // linear increase
    if (freq > f1) {  // end of sweep, draw graph
      draw_freq_response(samples_count);
      AD.setSleep(SLEEP_MODE_ALL);
      HALT;
    }
    bank = 1 - bank;
    AD.setFrequency(bank, freq);
    AD.selectFrequency(bank);
    draw_advance(freq);
    previousMillis = currentMillis;
  }

}



void adjust_input(void)
{
  uint8_t i, v1, v2, samples_count;
  int n_samples, x, y;
  unsigned long previousTime;
  bool timeout, found;
  const uint8_t ms_in_display = 4;
  const uint8_t conversion_time = 4 + 32*13/(F_CPU/1e6);  // Time to convert a sample in ADC, in microseconds. 32 is the prescaler
  const int half_screen = (display.getDisplayHeight()-1)/2;

  n_samples = 1000*ms_in_display/conversion_time;  // Number of samples to take during ms_in_display
  if (n_samples > steps) {  // Avoid buffer overflow  
    Serial.println(F("Warning: small buffer, had to limit number of samples in scope display")); 
    n_samples = steps;        
  }
  ADCSRA &= ~(_BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)); //  Clear ADC prescaler bits
  ADCSRA |= _BV(ADPS0) | _BV(ADPS2);                 //  Set prescaler to 32 => 500 KHz ADC clock, assuming a 16 MHz CPU clock
  bank = 1 - bank;
  AD.setFrequency(bank, float(1000));
  AD.selectFrequency(bank);
  delay(1);

  do {
    samples_count = 0;
    // Search for trigger point at Vdc, positive slope
    // Auto trigger: if trigger point not found in a certain time, go ahead anyway
    previousTime = millis();
    do {
      v1 = uint8_t(analogRead(SCOPEPORT)>>2);
      v2 = uint8_t(analogRead(SCOPEPORT)>>2);  // Throw away
      v2 = uint8_t(analogRead(SCOPEPORT)>>2);
      found = (v1 <= Vdc) && (v2 >= Vdc);
      timeout = (millis() > previousTime+ms_in_display);  // ms_in_display timeout
    } while (!found && !timeout);

    // Now, take a fixed number of samples
    samples[samples_count++] = v2;    
    while (samples_count < n_samples) {
      samples[samples_count++] = uint8_t(analogRead(SCOPEPORT)>>2);
    };

    // Put samples on display, only dots
    display.firstPage();
    do {
      // Draw reference lines
      display.drawHLine(0, display.getDisplayHeight()-1, display.getDisplayWidth());
      display.drawVLine(0, 0, display.getDisplayHeight()-1);
      y = display.getDisplayHeight()-1;
      do {
        display.drawHLine(0, y, 4);
        y -= half_screen*0.5*256/(Vcc/1000*Vdc);  // Vertical indicators every 0.5 V, Vdc level at half screen
      } while (y>0);
      // Draw input signal
      for (i=0; i<n_samples; i++) {
        x = i*(display.getDisplayWidth()-1)/n_samples;
        y = half_screen - half_screen*(samples[i]-Vdc)/Vdc;  // Vdc level at half screen
        display.drawPixel(x, y);
      }
    } while ( display.nextPage() );
  } while (!BUTTONPUSHED);  // Display scope until user presses button
  while (BUTTONPUSHED);     // Wait for button release

  // Set ADC clock to a better resolution
  ADCSRA &= ~(_BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)); //  Clear ADC prescaler bits
  ADCSRA |= _BV(ADPS1) | _BV(ADPS2);                 //  Set prescaler to 64 => 250 KHz ADC clock, assuming a 16 MHz CPU clock
  v1 = uint8_t(analogRead(SCOPEPORT)>>2);  // New read with new values
}



void draw_advance(unsigned f)
{
  uint8_t x;

  display.firstPage();
  display.setFont(u8g2_font_6x10_tr);
  display.setFontPosTop();  // Put the (0,0) at the top left corner of the glyph instead of bottom left
  do {
   display.setCursor(0,0); 
   display.print("SWEEP");
   display.setCursor(0,10);
   display.print("From "); display.print(f0); display.print(Hz);
   display.setCursor(0,20);
   display.print("To "); display.print(f1); display.print(Hz);
   display.setCursor(0,30);
   display.print("Steps: "); display.print(delta_f, 1); display.print(Hz);   
   
   x = (f-f0)*(display.getDisplayWidth()-1)/(f1-f0);
   display.drawBox(0, BASELINE, x, 4);
  } while ( display.nextPage() );
}


void draw_freq_response(int samples_count)
{
  uint8_t x, y, Vpmax, max_sample;
  int i;
  float freq;

  Vpmax = MIN(255-Vdc, Vdc); 
  for (max_sample=0, i=0; i<samples_count; i++) {
    if (samples[i]>max_sample) max_sample = samples[i];  
  }
  
  display.firstPage();
  display.setFont(u8g2_font_6x10_tr);
  display.setFontPosTop();  // Put the (0,0) at the top left corner of the glyph instead of bottom left
  do {
    // Draw scale
    display.drawHLine(0, BASELINE, display.getDisplayWidth()-1);
    for (long f = 1000; f <= f1; f += 1000) {
      x = (display.getDisplayWidth()-1)*(f-f0)/(f1-f0);
      display.drawVLine(x, BASELINE, 4);
    }
    // Draw curve
    for (freq=f0, i=0; i<samples_count; i++, freq += delta_f) {
      x = (freq-f0)/(f1-f0)*(display.getDisplayWidth()-1);
      y = BASELINE - BASELINE*(samples[i]-Vdc)/Vpmax;  
      display.drawPixel(x, y);
    }
    // Draw -3 dB line
    y = BASELINE - BASELINE*7U*(max_sample-Vdc)/Vpmax/10;     // 7 comes from 0.7, which is 1/sqrt(2)
    display.drawHLine(0, y, display.getDisplayWidth()-1);     // -3dB line
    display.setFontPosBaseline();
    display.setCursor(84, y); 
    display.print("-3 dB");
      
  } while ( display.nextPage() );
  
}



// Interrupt routine associated to external Pin 2 (PD2), INT0
ISR(INT0_vect) {
  /* If pinA and pinB are both high or both low, encoder is rotating CW.
     If they are different, it is rotating CCW.
  */
  uint32_t increment;
  uint8_t in_AB;
  
  if (tasks & _BV(ROTARY)) return;     // If task ist set but not served, return. Avoids bouncing issues
  in_AB = PIND & encoderPins;  // Read encoder pins, both are in PORTD

  if (BUTTONPUSHED) increment = 50;
  else increment = 500;

  // If both pins are the same rotation was CW, otherwise CCW
  if (in_AB == 0 || in_AB == encoderPins) encoderVal += increment;  
  else encoderVal -= increment;   
  // Limit encoder value 
  if (encoderVal < 0) encoderVal = 0;
  tasks |= _BV(ROTARY); // Set task for loop
}


// Return the Vcc voltage level in millivolts, by reading the internal reference
// This reference has an error of max 10%
// https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
uint16_t readVcc(void) {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  // Result is in ADC. Must read ADCL first - it then locks ADCH. Reading ADCW takes care of it.
  uint32_t result = 1126400L / ADCW; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return (uint16_t)result; 
}
