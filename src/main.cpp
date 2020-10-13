#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#define LOG_OUT 1 // use the log output function
#define FHT_N 64 // set to 128 point fht

#include <FHT.h> // include the library

#define PIN_MUSIC 4
#define PIN_LEDs 5
#define NUM_LEDS 139 // actually 139
#define FREQ_SUM_N 16 // must be > LEDSX

#define LEDSX 10
#define LEDSY 14
#define ABSPIXEL(x,y) (NUM_LEDS - 1 - (LEDSX * y + (x+dispRotation)%LEDSX))
#define DISPLAY_FPS 25
#define AMP_FACTOR ((255 / LEDSY)+1)

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN_LEDs, NEO_GRB + NEO_KHZ800);
uint8_t currSample = 0;
uint8_t rawADC, sMin = 255, sMax = 0;
uint8_t signalEnvelope = 0;
bool buffComplete = false;
uint8_t sumFreq[FREQ_SUM_N];
uint8_t lastFreq[FREQ_SUM_N];
uint32_t lastUpdate = 0;

uint8_t rotationCounter = 0;
uint8_t dispRotation = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  
  // set up the analog pin for continous ADC
  ADMUX =   bit(REFS0) | bit(ADLAR) | (PIN_MUSIC & 0x07);  // AVcc (5V) reference + left justify + pin
  ADCSRB = 0;
  ADCSRA = bit(ADPS2) | bit(ADPS1) | // 64 prescaler => 19.2 kHz sampling
  bit(ADATE) | // enable auto trigger
  bit(ADIE) |  // enable interrupts when measurement complete
  bit(ADEN) |  // enable ADC
  bit(ADSC);  // start ADC measurements

  // turn of board's LED in order to be invisible
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(255);
  strip.show(); // Initialize all pixels to 'off'

  lastUpdate = millis();

}

ISR(ADC_vect)
{
  rawADC = ADCH;
  fht_input[currSample] = rawADC; // read 8 bit value from ADC (left justified)
  fht_input[currSample] -= sMax-sMin;
  fht_input[currSample] <<= 6;  

  if (rawADC > sMax) sMax = rawADC;
  if (rawADC < sMin) sMin = rawADC;

  currSample++;
  if (currSample == FHT_N) {
    buffComplete = true;
    currSample = 0;
  }
}

/// Saves the continuous frequency measurements
/// in a max array so that no loud tones get
/// lost. The actualy display of frequencies/animations
/// only takes place at 25 FPS
void saveFreq(uint8_t* fData, uint8_t len) {
  uint8_t fac = len / FREQ_SUM_N;

  for(uint8_t i = 0; i < FREQ_SUM_N; i++) {
    uint32_t sum = 0;
    for(uint8_t j = 0; j < fac; j++) {
      sum += fData[i*fac + j];
    }
    sumFreq[i] = sumFreq[i]/2 + (sum / fac)/2;
  }
}

void processSamples() {
  cli();
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_log(); // take the output of the fht
  sei();
  saveFreq(fht_log_out, FHT_N / 2);
}

inline void setPixel(uint8_t x, uint8_t y, uint32_t col) {
  strip.setPixelColor(ABSPIXEL(x, y), col);
}

inline uint32_t getPixel(uint8_t x, uint8_t y) {
  return strip.getPixelColor(ABSPIXEL(x,y));
}

inline void getColComp(uint32_t& r, uint32_t& g, uint32_t& b, uint32_t oldCol) {
  r = oldCol >> 16;
  g = (oldCol >> 8) & 0xFF;
  b = (oldCol) & 0xFF;
}

inline uint32_t composeCol(uint32_t r, uint32_t g, uint32_t b) {
  return (r << 16) | (g << 8) | b;
}

void displayAnimation() {
  // rotate tower backwards
  rotationCounter++;
  dispRotation = rotationCounter / 5;

  // propagate lower pixels to upper pixels
  // -> "fire" effect
  for(uint8_t x = 0; x < LEDSX; x++) {
    for(uint8_t y = LEDSY-1; y >= 1; y--) {
      uint32_t r1, r2, g1, g2, b1, b2;
      //getColComp(r1, g1, b1, getPixel(x,y));
      getColComp(r2, g2, b2, getPixel(x,y-1));

      r2 = r2 * 4 / 5;
      g2 = g2 * 4 / 5;
      //b2 = b2 * 4 / 5;

      //uint32_t newCol = composeCol(r1/4+r2/2, g1/4+g2/2, b1/4+b2/2);
      uint32_t newCol = composeCol(r2, g2, b2);
      setPixel(x,y, newCol);
    }
  }

  // then, calculate the new lowest row (fire shape for the future)
  uint8_t sigDiff = sMax - sMin;
  uint8_t bassThreshold;
  uint8_t bass = sigDiff;
  if (signalEnvelope > 0) signalEnvelope--;
  //bassThreshold = max(32, (int)signalEnvelope - 5);
  if (bass > signalEnvelope) {
    signalEnvelope = bass;
    bassThreshold = max(45, signalEnvelope - 2);
  }
  

  uint32_t freqArea = 0;
  for(uint8_t x = 0; x < LEDSX; x++) {
    freqArea += sumFreq[x+1];
  }
  freqArea /= LEDSX;

  for(uint8_t x = 0; x < LEDSX; x++) {
    uint16_t freqVal = max(0, (int32_t)sumFreq[x+1] - 30) * 8 / freqArea;;
    uint32_t r, g, b;
    if (bass >= bassThreshold)
    {
      b = 64;
      r = g = 0;
    } else {
      b = 0;
      r = freqVal * sigDiff;
      g = max(0, (int)sumFreq[3] - 20) * sigDiff / 64;
    }
    

    uint32_t newCol = composeCol(r, g, b);
    setPixel(x, 0, newCol);
  }

  // make lastFreq sumFreq
  memcpy(lastFreq, sumFreq, sizeof(sumFreq));

  strip.show();
}

void loop() {

  if (buffComplete) {
    processSamples(); 
    buffComplete = false;
   
  }
  
  uint32_t currTime = millis();
  if (currTime - lastUpdate > 1000/DISPLAY_FPS) {
    displayAnimation();

    // reset after display

    for(uint8_t i = 0; i < FREQ_SUM_N; i++) {
      Serial.print(sumFreq[i]);
      Serial.print(' ');
      sumFreq[i] = 0;
    }
    Serial.println();
    lastUpdate = currTime;
    sMin = 255;
    sMax = 0;
  }
}