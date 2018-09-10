
// This is a program I have written that allows to play with :
// - a Flora Adafruit board
// - Adafruit NeoPixels and leds
// - a LSM9DS0 IMU chip
//
// The idea is to stich this hardware to a beatiful glove (it could be an other cloth)
// and produce subtile led effets according to the movements
//
// This is a raw base to be used by future e-wearing projects
//
// SetPix() and SetLed() are totally customizable and not really interesting at this stage
//
// This code is GNU GPL
//
// François Jarriges 2015

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

#include <SoftTimer.h>
#include <MsTimer2.h>

#include <math.h>

#include "led_colors.h"

#define GLOVE_NEO_PIXEL_PIN  6
#define FLORA_RED_LED_PIN    7
#define FLORA_NEO_PIXEL_PIN  8
#define GLOVE_WHITE_LED_PIN  9

#define GLOVE_NEO_PIXEL_NB  2

#define IMU_DELAY         100     // (ms)
#define LED_TEST_DELAY     75     // (ms)
#define TIMER_DELAY        25     // Timer 2 ISR calling period (ms)

#define PIX_COLOR_X_POS     RED
#define PIX_COLOR_X_NEG     RED
#define PIX_COLOR_Y_POS     GREEN
#define PIX_COLOR_Y_NEG     GREEN
#define PIX_COLOR_Z_POS     BLUE
#define PIX_COLOR_Z_NEG     BLUE

#define PIX_CHAIN_SIZE      GLOVE_NEO_PIXEL_NB        // Flora onboard pixel is not included in pixels chain animation
//#define PIX_CHAIN_SIZE      (GLOVE_NEO_PIXEL_NB + 1)  // Flora onboard pixel is included in pixels chain animation
#define PIX_CHAIN_ANIM         3                      // Refresh animation data in circular scheme (product of IMU_DELAY)

//#define PIX_GRAVITY                 // Use gravity vector for animation instead of relative inertial vector

#define PIX_EXP_FILT           2.7  // Exponential filter to translate physical data into brightness

#define PIX_REFRESH_DELAY      2    // Delay for refresh expressed in number of ISR cycles
#define PIX_BRIGHTNESS_GRAN    1    // Granularity (brightness value will be a product of BRIGHTNESS_GRAN)
#define PIX_BRIGHTNESS_MAX    18    // Maximum value for brightness from 1 to 255
#define PIX_BLINK_THRESHOLD  255    // Brightness threshold above which blinking mode is enabled
#define PIX_DECR_FACTOR      255    // Decrementation is 1 + brightness / DECR_FACTOR (0 means no auto-decreasing)
#define PIX_BLINK_FACTOR       9    // Number of ISR cycles with light on is 1 + (BRIGHTNESS_MAX - brightness) / BLINK_FACTOR
#define PIX_BLINK_RATIO        1    // Number of ISR cycles with light off is BLINK_RATIO x (number of cycles with light on)

#define LED_EXP_FILT           1.8  // Exponential filter to translate physical data into brightness

#define LED_REFRESH_DELAY      1    // Delay for refresh expressed in number of ISR cycles
#define LED_BRIGHTNESS_GRAN    6    // Granularity (brightness value will be a product of BRIGHTNESS_GRAN)
#define LED_BRIGHTNESS_MAX    72    // Maximum value for brightness from 1 to 255
#define LED_BLINK_THRESHOLD   18    // Brightness threshold above which blinking mode is enabled
#define LED_DECR_FACTOR       18    // Decrementation is 1 + brightness / DECR_FACTOR (0 means no auto-decreasing)
#define LED_BLINK_FACTOR       9    // Number of ISR cycles with light on is 1 + (BRIGHTNESS_MAX - brightness) / BLINK_FACTOR
#define LED_BLINK_RATIO        2    // Number of ISR cycles with light off is BLINK_RATIO x (number of cycles with light on)

#define ACCEL_RANGE_G 2

#define ACCEL_OFFSET_X     0.71
#define ACCEL_OFFSET_Y    -0.12
#define ACCEL_OFFSET_Z     0.35

#define ACCEL_FACTOR_X    -0.975
#define ACCEL_FACTOR_Y    -0.965
#define ACCEL_FACTOR_Z    -0.988

#define GYRO_OFFSET_X -2.4
#define GYRO_OFFSET_Y  1.1
#define GYRO_OFFSET_Z -6.2

#define GYRO_FACTOR	 9
#define COMPOUND_VECT(a, g) {(a + g*GYRO_FACTOR) / (1 + GYRO_FACTOR)}

#define GYRO_MOV_THRESHOLD       0.9  // Angle rate threshold under which the new attitude is got from accelerometer (dps)

#define PIX_GYRO_THRESH_HI     136.0  // Angle rate threshold above which a new animation is calculated (dps)
#define PIX_GYRO_THRESH_LO       6.0  // Angle rate threshold under which there is no new animation data (dps)
#define PIX_STRENGTH_THRESH_HI   1.0  // Relative acceleration value above which a new animation is calculated (m/s2)
#define PIX_STRENGTH_THRESH_LO   1.0  // Relative acceleration value under which there is no new animation data (m/s2)

#define LED_GYRO_THRESH_HI       6.0  // Angle rate threshold above which a new animation is calculated (dps)
#define LED_GYRO_THRESH_LO       6.0  // Angle rate threshold under which there is no new animation data (dps)
#define LED_STRENGTH_THRESH_HI  18.6  // Relative acceleration value above which a new animation is calculated (m/s2)
#define LED_STRENGTH_THRESH_LO  13.0  // Relative acceleration value under which there is no new animation data (m/s2)

#define UNDEF_DATA 0xFF
#define BRIGHT_SCALE 0xFF

#define PROPORTION_FLOAT_TO_UINT8(vIn, vRef, vMax) {(uint8_t)(float(vIn) * float(vRef) / float(vMax))}

#define TRY_TO_CONNECT
#define NB_SERIAL_CHK 500
#define SERIAL_BAUD   19200

#define SHOW_IMU
#define SHOW_VECTOR
//#define SHOW_ANIM

#define LED_TEST


typedef struct inertialVector_t {
  float m;
  struct {
    float x;
    float y;
    float z;
  } dirCos;
};

typedef struct ledAnimation_t {
  boolean   newData = false;
  boolean   active = false;
  boolean   lightOn = false;
  boolean   blinking = false;
  boolean   decreasing = false;
  uint8_t   brightness = 0;
  uint8_t   brightMax = BRIGHT_SCALE;
  uint8_t   color[3] = {0, 0, 0};
  uint8_t   decrVal = 0;
  uint16_t  blinkOn = 0;
  uint16_t  blinkOff = 0;
  uint16_t  blinkCount = 0;
  uint8_t   refrDelay = 1;
  uint8_t   refrCount = 0;
  uint8_t   digitalLed = 0;
  uint8_t   analogLed = 0;
  Adafruit_NeoPixel* neoPixel = 0;
  uint16_t  neoPixelIdx = 0;
};

Adafruit_NeoPixel FloraNeoPixel = Adafruit_NeoPixel(1, FLORA_NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800); // single NeoPixel on Flora board
Adafruit_NeoPixel GloveNeoPixel = Adafruit_NeoPixel(GLOVE_NEO_PIXEL_NB, GLOVE_NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800); // two NeoPixels on glove

Adafruit_LSM9DS0 Lsm9Ds0 = Adafruit_LSM9DS0(1000); // I2C protocol ID 1000

void MainLoop(Task*);
Task MainTask(IMU_DELAY, MainLoop);

sensors_vec_t LastGyroData;
inertialVector_t LastCompoundVector, GravityVector, RelativeVector;

const uint8_t PixColorXPos[3] = PIX_COLOR_X_POS;
const uint8_t PixColorXNeg[3] = PIX_COLOR_X_NEG;
const uint8_t PixColorYPos[3] = PIX_COLOR_Y_POS;
const uint8_t PixColorYNeg[3] = PIX_COLOR_Y_NEG;
const uint8_t PixColorZPos[3] = PIX_COLOR_Z_POS;
const uint8_t PixColorZNeg[3] = PIX_COLOR_Z_NEG;

volatile ledAnimation_t FloraNeoPixAnim, GloveNeoPixAnim[GLOVE_NEO_PIXEL_NB], GloveWLedAnim;
volatile ledAnimation_t* PixChain[PIX_CHAIN_SIZE];

boolean SerialConnected = false;

void setup() {
  int i;

#ifdef TRY_TO_CONNECT
  for (i = 0; i < NB_SERIAL_CHK; i++) {
    Serial.begin(SERIAL_BAUD);
    if (Serial == true) {
      SerialConnected = true;
      break;
    }
  }
#endif

  neoPixelInit(&FloraNeoPixel);
  neoPixelInit(&GloveNeoPixel);

  FloraNeoPixAnim.brightMax = PIX_BRIGHTNESS_MAX;
  FloraNeoPixAnim.refrDelay = PIX_REFRESH_DELAY;
  FloraNeoPixAnim.neoPixel = &FloraNeoPixel;
  FloraNeoPixAnim.neoPixelIdx = 0;
#if (PIX_CHAIN_SIZE==(GLOVE_NEO_PIXEL_NB + 1))
  PixChain[0] = &FloraNeoPixAnim;
#endif
  for (i = 0; i < GLOVE_NEO_PIXEL_NB; i++) {
    GloveNeoPixAnim[i].brightMax = PIX_BRIGHTNESS_MAX;
    GloveNeoPixAnim[i].refrDelay = PIX_REFRESH_DELAY;
    GloveNeoPixAnim[i].neoPixel = &GloveNeoPixel;
    GloveNeoPixAnim[i].neoPixelIdx = i;
    PixChain[(PIX_CHAIN_SIZE - GLOVE_NEO_PIXEL_NB)+i] = &(GloveNeoPixAnim[i]);
  }
  GloveWLedAnim.brightMax = LED_BRIGHTNESS_MAX;
  GloveWLedAnim.refrDelay = LED_REFRESH_DELAY;
  GloveWLedAnim.analogLed = GLOVE_WHITE_LED_PIN;
  
  if (!Lsm9Ds0.begin()) {
#ifdef TRY_TO_CONNECT
    if (SerialConnected) Serial.println("LSM9DS0 init failed");
#endif
    while (1);
  }
  Lsm9Ds0.setupAccel(Lsm9Ds0.LSM9DS0_ACCELRANGE_2G);
  //  Lsm9Ds0.setupMag(Lsm9Ds0.LSM9DS0_MAGGAIN_2GAUSS);
  Lsm9Ds0.setupGyro(Lsm9Ds0.LSM9DS0_GYROSCALE_245DPS);
  delay(IMU_DELAY);

  LastGyroData.status = UNDEF_DATA;
  memset(&LastCompoundVector, 0, sizeof(inertialVector_t));
  memset(&GravityVector, 0, sizeof(inertialVector_t));
  memset(&RelativeVector, 0, sizeof(inertialVector_t));

  pinMode(FLORA_RED_LED_PIN, OUTPUT);
  pinMode(GLOVE_WHITE_LED_PIN, OUTPUT);
#ifdef LED_TEST
  for (i = 0; i < 5; i++) {
    digitalWrite(FLORA_RED_LED_PIN, HIGH);
    analogWrite(GLOVE_WHITE_LED_PIN, LED_BRIGHTNESS_MAX);
    delay(LED_TEST_DELAY);
    digitalWrite(FLORA_RED_LED_PIN, LOW);
    analogWrite(GLOVE_WHITE_LED_PIN, 0);
    delay(LED_TEST_DELAY);
  }
#endif

  SoftTimer.add(&MainTask);
  MsTimer2::set(TIMER_DELAY, Animate);
  MsTimer2::start();

#ifdef TRY_TO_CONNECT
  if (SerialConnected) {
    Serial.println("********** NeoPixel Magic Glove Initialized ************");
    Serial.println("");
  }
#endif
}

void neoPixelInit(Adafruit_NeoPixel* neoPixel) {

  neoPixel->begin();
  neoPixel->show();
  delay(LED_TEST_DELAY);

#ifdef LED_TEST
  uint32_t color = 0xFF;
  uint16_t pix, nbPix;

  nbPix = neoPixel->numPixels();
  neoPixel->setBrightness(PIX_BRIGHTNESS_MAX);
  for (uint16_t c = 0; c < 4; c++) {
    for (pix = 0; pix < nbPix; pix++ ) {
      neoPixel->setPixelColor(pix, color);
    }
    neoPixel->show();
    color <<= 8;
    delay(LED_TEST_DELAY);
  }
  neoPixel->setBrightness(BRIGHT_SCALE);
#endif
}

void MainLoop(Task* me) {
  GetImu();
  SetPix();
  SetLed();
}

inline void GetImu() {
  sensors_event_t accel, gyro;
  inertialVector_t accelVector, gyroVector, compoundVector;
  float gyroDxRad, gyroDyRad, gyroDzRad;
  boolean moving;

  Lsm9Ds0.getEvent(&accel, 0, &gyro, 0);

  accel.acceleration.x = (accel.acceleration.x + ACCEL_OFFSET_X) * (ACCEL_FACTOR_X);
  accel.acceleration.y = (accel.acceleration.y + ACCEL_OFFSET_Y) * (ACCEL_FACTOR_Y);
  accel.acceleration.z = (accel.acceleration.z + ACCEL_OFFSET_Z) * (ACCEL_FACTOR_Z);

  normalizeVector(
    &accelVector,
    accel.acceleration.x,
    accel.acceleration.y,
    accel.acceleration.z);

  gyro.gyro.x -= GYRO_OFFSET_X;
  gyro.gyro.y -= GYRO_OFFSET_Y;
  gyro.gyro.z -= GYRO_OFFSET_Z;

  if (LastGyroData.status != UNDEF_DATA) {
    gyro.gyro.x = (gyro.gyro.x + LastGyroData.x) / 2;
    gyro.gyro.y = (gyro.gyro.y + LastGyroData.y) / 2;
    gyro.gyro.z = (gyro.gyro.z + LastGyroData.z) / 2;
  }
  gyroDxRad = gyro.gyro.x * SENSORS_DPS_TO_RADS * IMU_DELAY / 1000;
  gyroDyRad = gyro.gyro.y * SENSORS_DPS_TO_RADS * IMU_DELAY / 1000;
  gyroDzRad = gyro.gyro.z * SENSORS_DPS_TO_RADS * IMU_DELAY / 1000;

  gyroVector = LastCompoundVector;
  newAttitude(&gyroVector, gyroDzRad, gyroDyRad, gyroDxRad);
  newAttitude(&GravityVector, gyroDzRad, gyroDyRad, gyroDxRad);

  normalizeVector(
    &compoundVector,
    COMPOUND_VECT(accelVector.dirCos.x * accelVector.m, gyroVector.dirCos.x * gyroVector.m),
    COMPOUND_VECT(accelVector.dirCos.y * accelVector.m, gyroVector.dirCos.y * gyroVector.m),
    COMPOUND_VECT(accelVector.dirCos.z * accelVector.m, gyroVector.dirCos.z * gyroVector.m));

  if (
    (abs(gyro.gyro.x) < GYRO_MOV_THRESHOLD) &&
    (abs(gyro.gyro.y) < GYRO_MOV_THRESHOLD) &&
    (abs(gyro.gyro.z) < GYRO_MOV_THRESHOLD)) {
    GravityVector = accelVector;
    moving = false;
  } else {
    moving = true;
  }
  normalizeVector(
    &RelativeVector,
    accelVector.dirCos.x * accelVector.m - GravityVector.dirCos.x * GravityVector.m,
    accelVector.dirCos.y * accelVector.m - GravityVector.dirCos.y * GravityVector.m,
    accelVector.dirCos.z * accelVector.m - GravityVector.dirCos.z * GravityVector.m);

#ifdef TRY_TO_CONNECT
  if (moving && SerialConnected) {
#ifdef SHOW_IMU
    showSensor(&(accel.acceleration), 0, &(gyro.gyro), 0);
#endif SHOW_IMU
#ifdef SHOW_VECTOR
//    showVector(&GravityVector, &accelVector, &gyroVector, &compoundVector, &RelativeVector);
      showVector(&GravityVector, 0, 0, 0, &RelativeVector);
#endif SHOW_VECTOR
  }
#endif TRY_TO_CONNECT
  LastGyroData = gyro.gyro;
  LastCompoundVector = compoundVector;
}

inline void SetPix() {
  char pixName[4] = "PXn";
  uint8_t brightVal;
  float colorX, colorY, colorZ;
  static uint8_t colorVal[3];
  static boolean animCond =  false;
  inertialVector_t* refVector;

  if (
      ((abs(LastGyroData.x) >= PIX_GYRO_THRESH_HI) ||
       (abs(LastGyroData.y) >= PIX_GYRO_THRESH_HI) ||
       (abs(LastGyroData.z) >= PIX_GYRO_THRESH_HI))
      && (RelativeVector.m >= PIX_STRENGTH_THRESH_HI)) {
    animCond = true;
  }
  else if (
      ((abs(LastGyroData.x) < PIX_GYRO_THRESH_LO) &&
       (abs(LastGyroData.y) < PIX_GYRO_THRESH_LO) &&
       (abs(LastGyroData.z) < PIX_GYRO_THRESH_LO))
      || (RelativeVector.m <  PIX_STRENGTH_THRESH_LO)) {
    animCond = false;
  }

  if (animCond) {
#ifdef PIX_FILT_EXP
    brightVal = PROPORTION_FLOAT_TO_UINT8(PIX_BRIGHTNESS_MAX / PIX_BRIGHTNESS_GRAN, 
                                           pow(RelativeVector.m, PIX_FILT_EXP),
                                           pow(ACCEL_RANGE_G * SENSORS_GRAVITY_STANDARD, PIX_FILT_EXP));
#else
    brightVal = PROPORTION_FLOAT_TO_UINT8(PIX_BRIGHTNESS_MAX / PIX_BRIGHTNESS_GRAN, 
                                           RelativeVector.m,
                                           ACCEL_RANGE_G * SENSORS_GRAVITY_STANDARD);
#endif PIX_FILT_EXP
    brightVal = min(brightVal * PIX_BRIGHTNESS_GRAN, PIX_BRIGHTNESS_MAX);
    
    for (int c = 0; c < 3; c++) {
#ifdef PIX_GRAVITY
      refVector = &GravityVector;
#else
      refVector = &RelativeVector;
#endif PIX_GRAVITY
      if (refVector->dirCos.x >= 0)  colorX = refVector->dirCos.x  *  float(PixColorXPos[c]); 
      else                           colorX = refVector->dirCos.x  *  float(PixColorXNeg[c]) * (-1); 
      if (refVector->dirCos.y >= 0)  colorY = refVector->dirCos.y  *  float(PixColorYPos[c]); 
      else                           colorY = refVector->dirCos.y  *  float(PixColorYNeg[c]) * (-1);  
      if (refVector->dirCos.z >= 0)  colorZ = refVector->dirCos.z  *  float(PixColorZPos[c]); 
      else                           colorZ = refVector->dirCos.z  *  float(PixColorZNeg[c]) * (-1);  
     
      colorVal[c] = uint8_t((colorX + colorY + colorZ) / 3);
    }
  } else {
    brightVal = 0;
  }
#ifdef PIX_CHAIN_ANIM
  static uint8_t curPixId = 0, refreshCount = 0;

  if (++refreshCount == PIX_CHAIN_ANIM) {
   if (curPixId++ == (PIX_CHAIN_SIZE - 1)) curPixId = 0;
   refreshCount = 0; 
  }

  for (int p = 0; p <  PIX_CHAIN_SIZE; p++) {
    pixName[2] = '0' + p;
    updateAnimData(PixChain[p], pixName, (p == curPixId) ? brightVal : 0, PIX_DECR_FACTOR, 
                   PIX_BLINK_THRESHOLD, PIX_BLINK_FACTOR, PIX_BLINK_RATIO, (p == curPixId) ? colorVal : 0);
  }
#else
  for (int p = 0; p <  PIX_CHAIN_SIZE; p++) {
    pixName[2] = '0' + p;
    updateAnimData(PixChain[p], pixName, brightVal, PIX_DECR_FACTOR, 
                   PIX_BLINK_THRESHOLD, PIX_BLINK_FACTOR, PIX_BLINK_RATIO, colorVal);
  }
#endif PIX_CHAIN_ANIM
}

inline void SetLed() {
  uint8_t brightVal;
  static boolean animCond = false;

  if (
      ((abs(LastGyroData.x) >= LED_GYRO_THRESH_HI) ||
       (abs(LastGyroData.y) >= LED_GYRO_THRESH_HI) ||
       (abs(LastGyroData.z) >= LED_GYRO_THRESH_HI))
      && (RelativeVector.m >= LED_STRENGTH_THRESH_HI)) {
    animCond = true;
  }
  else if (
      ((abs(LastGyroData.x) < LED_GYRO_THRESH_LO) &&
       (abs(LastGyroData.y) < LED_GYRO_THRESH_LO) &&
       (abs(LastGyroData.z) < LED_GYRO_THRESH_LO))
      || (RelativeVector.m <  LED_STRENGTH_THRESH_LO)) {
    animCond = false;
  }
  if (animCond) {
#ifdef LED_FILT_EXP
    brightVal = PROPORTION_FLOAT_TO_UINT8(LED_BRIGHTNESS_MAX / LED_BRIGHTNESS_GRAN, 
                                          pow(RelativeVector.m, LED_FILT_EXP),
                                          pow(ACCEL_RANGE_G * SENSORS_GRAVITY_STANDARD, LED_FILT_EXP));
#else
    brightVal = PROPORTION_FLOAT_TO_UINT8(LED_BRIGHTNESS_MAX / LED_BRIGHTNESS_GRAN, 
                                          RelativeVector.m,
                                          ACCEL_RANGE_G * SENSORS_GRAVITY_STANDARD);
#endif LED_FILT_EXP
    brightVal = min(brightVal * LED_BRIGHTNESS_GRAN, LED_BRIGHTNESS_MAX);
  } else {
    brightVal = 0;
  }
  updateAnimData(&GloveWLedAnim, "LDW", brightVal, LED_DECR_FACTOR, 
                 LED_BLINK_THRESHOLD, LED_BLINK_FACTOR, LED_BLINK_RATIO, 0);
}

inline void updateAnimData(volatile ledAnimation_t* anim_p, char pix_name[],
                           uint8_t bright_val, uint8_t decr_fac, 
                           uint8_t blink_thr, uint8_t blink_fac, uint8_t blink_rat, 
                           uint8_t color[]) {

  noInterrupts();
  if (bright_val > 0) {
    anim_p->active = true;
    anim_p->decreasing = (decr_fac != 0);
    anim_p->brightness = bright_val;
    anim_p->newData = true;
  } else {
    if (anim_p->decreasing) {
      bright_val = anim_p->brightness;
    } else {
       anim_p->active = false;
    }
  }
  if (bright_val > 0) {
    if (anim_p->decreasing) {
      anim_p->decrVal = 1 + bright_val / decr_fac;
    }
    if (bright_val < blink_thr) {
      anim_p->blinking = false;
    } else {
      if (!anim_p->blinking) {
        anim_p->blinking = true;
        anim_p->blinkCount = 0;
      }
      anim_p->blinkOn = (uint16_t)(1 + (anim_p->brightMax - bright_val) / blink_fac);
      anim_p->blinkOff =(uint16_t)(anim_p->blinkOn * (1 + blink_rat));
    }
  }
  if ((uint8_t*)color != 0) {
    anim_p->color[0] = color[0];
    anim_p->color[1] = color[1];
    anim_p->color[2] = color[2];
  }
  interrupts();
#ifdef TRY_TO_CONNECT
#ifdef SHOW_ANIM
  if (anim_p->active && SerialConnected) showAnim(anim_p, pix_name);
#endif SHOW_ANIM
#endif TRY_TO_CONNECT
}

void Animate(void) {

#if (PIX_CHAIN_SIZE==(GLOVE_NEO_PIXEL_NB + 1))
  updateLed(PixChain[0]);
#endif
  for (int i = 0; i < GLOVE_NEO_PIXEL_NB; i++) {
    updateLed(PixChain[(PIX_CHAIN_SIZE - GLOVE_NEO_PIXEL_NB)+i]);
  }
  updateLed(&GloveWLedAnim);
}

inline void updateLed(volatile ledAnimation_t* led) {
  if (++(led->refrCount) >= led->refrDelay) {
    led->refrCount = 0;

    if (led->active) {
      if (led->blinking) {
        if (led->blinkCount >= led->blinkOff) {
          led->blinkCount = 0;
        }
        if (led->blinkCount >= led->blinkOn) {
          if (led->lightOn) {
            led->lightOn = false;
            led->newData = true;
          }
        }
        else if (led->blinkCount >= 0) {
          if (!led->lightOn) {
            led->lightOn = true;
            led->newData = true;
          }
        }
        led->blinkCount++;
      } else {
        if (!led->lightOn) {
          led->lightOn = true;
          led->newData = true;
        }
        led->blinkCount = 0;
      }
      if (led->decreasing) {
        if (led->brightness > led->decrVal) {
          led->brightness -= led->decrVal;
        } else {
          led->active = false;
          led->brightness = 0;          
          led->lightOn = false;
        }
        led->newData = true;
      }    
    } else if (led->lightOn) {
      led->lightOn = false;
      led->newData = true;
    }
    if (led->newData) {
        if (led->digitalLed != 0) {
          if (led->lightOn) digitalWrite(led->digitalLed, HIGH);
          else              digitalWrite(led->digitalLed, LOW);
        }
        else if (led->analogLed != 0) {
          if (led->lightOn) analogWrite(led->analogLed, led->brightness);
          else              analogWrite(led->analogLed, 0);
        }
        else if (led->neoPixel != 0) {
          if (led->lightOn) {
            led->neoPixel->setPixelColor(led->neoPixelIdx, 
                                         PROPORTION_FLOAT_TO_UINT8(led->color[0], led->brightness, BRIGHT_SCALE), 
                                         PROPORTION_FLOAT_TO_UINT8(led->color[1], led->brightness, BRIGHT_SCALE), 
                                         PROPORTION_FLOAT_TO_UINT8(led->color[2], led->brightness, BRIGHT_SCALE));
            led->neoPixel->show();
          }
          else {
            led->neoPixel->setPixelColor(led->neoPixelIdx, 0, 0, 0);
            led->neoPixel->show();
          }
        }
        led->newData = false;   
    }
  }  
}

inline void normalizeVector(inertialVector_t* vect_p, float x, float y, float z) {
  vect_p->m = sqrt(square(x) + square(y) + square(z));
  if (vect_p->m > 0) {
    vect_p->dirCos.x = x / vect_p->m;
    vect_p->dirCos.y = y / vect_p->m;
    vect_p->dirCos.z = z / vect_p->m;
  } else {
    vect_p->dirCos.x = 0;
    vect_p->dirCos.y = 0;
    vect_p->dirCos.z = 0;
  }
}

inline void newAttitude(inertialVector_t* vect_p, float d_yaw, float d_pitch, float d_roll) {
  inertialVector_t tmp_vect;
  float yaw, pitch, roll;
  float sin_yaw, sin_pitch, sin_roll;
  float cos_yaw, cos_pitch, cos_roll;

  roll = atan2(vect_p->dirCos.z, vect_p->dirCos.y) + d_roll;
  pitch = atan2(vect_p->dirCos.x, vect_p->dirCos.z) + d_pitch;
  yaw = atan2(vect_p->dirCos.y, vect_p->dirCos.x) + d_yaw;

  sin_roll = sin(roll); cos_roll = cos(roll);
  sin_pitch = sin(pitch); cos_pitch = cos(pitch);
  sin_yaw = sin(yaw); cos_yaw = cos(yaw);

  tmp_vect.dirCos.x = (sin_roll == 0) ? 0  : ((cos_roll == 0) ?  sin_pitch : (sin_pitch / sqrt(1 + (square(cos_pitch) / square(tan(roll))))));
  tmp_vect.dirCos.y = (sin_pitch == 0) ? 0  : ((cos_pitch == 0) ?  sin_yaw : (sin_yaw / sqrt(1 + (square(cos_yaw) / square(tan(pitch))))));
  tmp_vect.dirCos.z = (sin_yaw == 0) ? 0  : ((cos_yaw == 0) ?  sin_roll : (sin_roll / sqrt(1 + (square(cos_roll) / square(tan(yaw))))));

  *vect_p = tmp_vect;
}

#ifdef TRY_TO_CONNECT
#ifdef SHOW_IMU
void showSensor(sensors_vec_t* a, sensors_vec_t* m, sensors_vec_t* g, float* t) {

  Serial.println("**********************\n");
  if (a != 0) {
    Serial.print("Accel Data \t\tX: "); Serial.print(a->x);
    Serial.print("  \tY: "); Serial.print(a->y);
    Serial.print("  \tZ: "); Serial.print(a->z); Serial.println("  \tm/s^2");
  }
  if (m != 0) {
    Serial.print("Magn Data \t\tX: "); Serial.print(m->x);
    Serial.print("  \tY: "); Serial.print(m->y);
    Serial.print("  \tZ: "); Serial.print(m->z); Serial.println("  \tgauss");
  }
  if (g != 0 ) {
    Serial.print("Gyro  Data \t\tX: "); Serial.print(g->x);
    Serial.print("  \tY: "); Serial.print(g->y);
    Serial.print("  \tZ: "); Serial.print(g->z); Serial.println("  \tdps");
  }
  if (t != 0) {
    Serial.print("Temp: "); Serial.print(*t); Serial.println(" *C");
  }
  Serial.println("");
}
#endif SHOW_IMU
#ifdef SHOW_VECTOR
void showVector(inertialVector_t* p, inertialVector_t* a, inertialVector_t* g, inertialVector_t* c, inertialVector_t* r) {

  Serial.println("**********************\n");
  if (p != 0) {
    Serial.print("Gravity Vector \t\tM: "); Serial.print(p->m);
    Serial.print("  \tX: "); Serial.print(p->dirCos.x);
    Serial.print("  \tY: "); Serial.print(p->dirCos.y);
    Serial.print("  \tZ: "); Serial.println(p->dirCos.z);
  }
  if (a != 0) {
    Serial.print("Accel Vector \t\tM: "); Serial.print(a->m);
    Serial.print("  \tX: "); Serial.print(a->dirCos.x);
    Serial.print("  \tY: "); Serial.print(a->dirCos.y);
    Serial.print("  \tZ: "); Serial.println(a->dirCos.z);
  }
  if (g != 0) {
    Serial.print("Gyro  Vector \t\tM: "); Serial.print(g->m);
    Serial.print("  \tX: "); Serial.print(g->dirCos.x);
    Serial.print("  \tY: "); Serial.print(g->dirCos.y);
    Serial.print("  \tZ: "); Serial.println(g->dirCos.z);
  }
  if (c != 0) {
    Serial.print("Compound  Vector \tM: "); Serial.print(c->m);
    Serial.print("  \tX: "); Serial.print(c->dirCos.x);
    Serial.print("  \tY: "); Serial.print(c->dirCos.y);
    Serial.print("  \tZ: "); Serial.println(c->dirCos.z);
  }
  if (r != 0) {
    Serial.print("Relative  Vector \tM: "); Serial.print(r->m);
    Serial.print("  \tX: "); Serial.print(r->dirCos.x);
    Serial.print("  \tY: "); Serial.print(r->dirCos.y);
    Serial.print("  \tZ: "); Serial.println(r->dirCos.z);
  }
  Serial.println("");
}
#endif SHOW_VECTOR
#ifdef SHOW_ANIM
void showAnim(volatile ledAnimation_t* a, char n[]) {
  
  Serial.println("**********************\n"); 
  Serial.print("Animation Data "); Serial.println(n);
  Serial.print("\t newData : ");Serial.print(a->newData);
  Serial.print("\t lightOn : ");Serial.print(a->lightOn);
  Serial.print("\t blinking : ");Serial.print(a->blinking);
  Serial.print("\t decreasing : ");Serial.println(a->decreasing);
  Serial.print("\t brightness : ");Serial.println(a->brightness);
  Serial.print("\t color \t R : 0x");Serial.print(a->color[0], HEX);
  Serial.print("\t G : 0x");Serial.print(a->color[1], HEX);
  Serial.print("\t B : 0x");Serial.println(a->color[2], HEX);
  Serial.print("\t decrVal : ");Serial.println(a->decrVal);
  Serial.print("\t blinkOn : ");Serial.print(a->blinkOn);
  Serial.print("\t blinkOff : ");Serial.print(a->blinkOff);
  Serial.print("\t blinkCount : ");Serial.println(a->blinkCount);
  Serial.println("");
}
#endif SHOW_ANIM
#endif TRY_TO_CONNECT


