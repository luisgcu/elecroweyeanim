//
// Code adapted by Bodmer as an example for TFT_eSPI, this runs on any
// TFT_eSPI compatible processor so ignore the technical limitations
// detailed in the original header below. Assorted changes have been
// made including removal of the display mirror kludge.

//--------------------------------------------------------------------------
// Uncanny eyes for Adafruit 1.5" OLED (product #1431) or 1.44" TFT LCD
// (#2088).  Works on PJRC Teensy 3.x and on Adafruit M0 and M4 boards
// (Feather, Metro, etc.).  This code uses features specific to these
// boards and WILL NOT work on normal Arduino or other boards!
//
// SEE FILE "config.h" FOR MOST CONFIGURATION (graphics, pins, display type,
// etc).  Probably won't need to edit THIS file unless you're doing some
// extremely custom modifications.
//
// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing products
// from Adafruit!
//
// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.  SPI FIFO insight from Paul Stoffregen's ILI9341_t3 library.
// Inspired by David Boccabella's (Marcwolf) hybrid servo/OLED eye concept.
//--------------------------------------------------------------------------

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "eye_types.h"
#include "config.h"

// Function to control backlight brightness
void setBacklight(uint8_t brightness) {
#if defined(DISPLAY_BACKLIGHT) && (DISPLAY_BACKLIGHT >= 0)
  analogWrite(DISPLAY_BACKLIGHT, constrain(brightness, 0, 255));
#endif
}

// Forward declarations and constants
#define BUFFER_SIZE 1024

// External variables from main.cpp  
extern TFT_eSPI tft;
extern TFT_eSprite spr;
extern uint16_t pbuffer[2][BUFFER_SIZE]; // Changed to 2 for max buffers
extern bool dmaBuf;
extern eyeStruct_t eye[];
extern uint32_t startTime;
extern void user_loop(void);

// Function declarations
void split(int16_t startValue, int16_t endValue, uint32_t startTime, int32_t duration, int16_t range);

#if !defined(LIGHT_PIN) || (LIGHT_PIN < 0)
// Autonomous iris motion uses a fractal behavior to similate both the major
// reaction of the eye plus the continuous smaller adjustments that occur.
uint16_t oldIris = (IRIS_MIN + IRIS_MAX) / 2, newIris;
#endif

// Initialise eyes ---------------------------------------------------------
void initEyes(void)
{
  Serial.println("Initialise eye objects");

  // Initialise eye objects based on eyeInfo list in config.h:
  for (uint8_t e = 0; e < NUM_EYES; e++) {
    Serial.print("Create display #"); Serial.println(e);

    eye[e].tft_cs      = eyeInfo[e].select;
    eye[e].blink.state = NOBLINK;
    eye[e].xposition   = eyeInfo[e].xposition;

    pinMode(eye[e].tft_cs, OUTPUT);
    digitalWrite(eye[e].tft_cs, LOW);

    // Also set up an individual eye-wink pin if defined:
    if (eyeInfo[e].wink >= 0) pinMode(eyeInfo[e].wink, INPUT_PULLUP);
  }

#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
  pinMode(BLINK_PIN, INPUT_PULLUP); // Ditto for all-eyes blink pin
#endif
}

// UPDATE EYE --------------------------------------------------------------
void updateEye (void)
{
#if defined(LIGHT_PIN) && (LIGHT_PIN >= 0) // Interactive iris

  int16_t v = analogRead(LIGHT_PIN);       // Raw dial/photocell reading
#ifdef LIGHT_PIN_FLIP
  v = 1023 - v;                            // Reverse reading from sensor
#endif
  if (v < LIGHT_MIN)      v = LIGHT_MIN; // Clamp light sensor range
  else if (v > LIGHT_MAX) v = LIGHT_MAX;
  v -= LIGHT_MIN;  // 0 to (LIGHT_MAX - LIGHT_MIN)
#ifdef LIGHT_CURVE  // Apply gamma curve to sensor input?
  v = (int16_t)(pow((double)v / (double)(LIGHT_MAX - LIGHT_MIN),
                    LIGHT_CURVE) * (double)(LIGHT_MAX - LIGHT_MIN));
#endif
  // And scale to iris range (IRIS_MAX is size at LIGHT_MIN)
  v = map(v, 0, (LIGHT_MAX - LIGHT_MIN), IRIS_MAX, IRIS_MIN);
#ifdef IRIS_SMOOTH // Filter input (gradual motion)
  static int16_t irisValue = (IRIS_MIN + IRIS_MAX) / 2;
  irisValue = ((irisValue * 15) + v) / 16;
  frame(irisValue);
#else // Unfiltered (immediate motion)
  frame(v);
#endif // IRIS_SMOOTH

#else  // Autonomous iris scaling -- invoke recursive function

  newIris = random(IRIS_MIN, IRIS_MAX);
  split(oldIris, newIris, micros(), 10000000L, IRIS_MAX - IRIS_MIN);
  oldIris = newIris;

#endif // LIGHT_PIN
}

// EYE-RENDERING FUNCTION --------------------------------------------------
void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
  // Use native 32 bit variables where possible as this is 10% faster!
  uint8_t  e,       // Eye array index; 0 or 1 for left/right
  uint32_t iScale,  // Scale factor for iris
  uint32_t  scleraX, // First pixel X offset into sclera image
  uint32_t  scleraY, // First pixel Y offset into sclera image
  uint32_t  uT,      // Upper eyelid threshold value
  uint32_t  lT) {    // Lower eyelid threshold value

  uint32_t  screenX, screenY, scleraXsave;
  int32_t  irisX, irisY;
  uint32_t p, a;
  uint32_t d;
  uint32_t t = micros(); // Get current time for blink calculations

  uint32_t pixels = 0;

  // Set up raw pixel dump to smaller view window.  Although such writes can wrap
  // around automatically from end of rect back to beginning, the region is
  // reset on each frame here in case of an SPI glitch.
  digitalWrite(eye[e].tft_cs, LOW);
  tft.startWrite();
  
  // Center the smaller window where the 200x200 eye was
  const int16_t viewW = VIEW_W;
  const int16_t viewH = VIEW_H;
  const int16_t eyeLeft = eye[e].xposition + (SCLERA_WIDTH - viewW) / 2;
  const int16_t eyeTop  = (240 - viewH) / 2;   // center vertically on 320x240

  tft.setAddrWindow(eyeLeft, eyeTop, viewW, viewH);

  // Final safety clamp (use viewW/viewH, not SCLERA dims)
  if ((int)scleraX + viewW  > SCLERA_WIDTH)  scleraX = SCLERA_WIDTH  - viewW;
  if ((int)scleraY + viewH  > SCLERA_HEIGHT) scleraY = SCLERA_HEIGHT - viewH;

  // Now just issue raw 16-bit values for every pixel...

  scleraXsave = scleraX; // Save initial X value to reset on each line
  irisY       = scleraY - (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;

  // Eyelid image is left<>right swapped for two displays
  uint16_t lidX = 0;
  uint16_t dlidX = -1;
  if (e) dlidX = 1;
  for (screenY = 0; screenY < viewH; screenY++, scleraY++, irisY++) { // Use viewH instead of drawHeight
    scleraX = scleraXsave;
    irisX   = scleraXsave - (SCLERA_WIDTH - IRIS_WIDTH) / 2;
    
    // Calculate eyelid lookup coordinates - scale from sclera space to eyelid space (128x128)
    uint16_t eyelidY = (screenY * 128) / SCLERA_HEIGHT;
    
    for (screenX = 0; screenX < viewW; screenX++, scleraX++, irisX++) { // Use viewW instead of drawWidth
      // Calculate eyelid X coordinate - scale from sclera space to eyelid space (128x128)
      uint16_t eyelidX = (screenX * 128) / SCLERA_WIDTH;
      if (e) {
        eyelidX = eyelidX;
      } else {
        eyelidX = 127 - eyelidX;
      }
      
      // Simple geometric blink instead of complex eyelid arrays
      bool isBlinking = (eye[e].blink.state == ENBLINK || eye[e].blink.state == DEBLINK);
      bool pixelCoveredByEyelid = false;
      
      // Create an oval eye shape mask for more natural appearance
      float centerX = viewW / 2.0;
      float centerY = viewH / 2.0;
      float eyeRadiusX = viewW * 0.51; // Slightly reduced from 0.55 - a bit smaller
      float eyeRadiusY = viewH * 0.34; // Slightly reduced from 0.37 - keeping ratio
      
      // Calculate distance from center
      float dx = screenX - centerX;
      float dy = screenY - centerY;
      float ellipseValue = (dx * dx) / (eyeRadiusX * eyeRadiusX) + (dy * dy) / (eyeRadiusY * eyeRadiusY);
      
      // If outside the eye shape, make it black (background)
      bool outsideEyeShape = ellipseValue > 1.0;
      
      if (isBlinking) {
        // Create a simple blink by covering top and bottom portions of the eye
        float blinkProgress = 0.0;
        if (eye[e].blink.state == ENBLINK) {
          // Closing: progress from 0 to 1
          blinkProgress = (float)(t - eye[e].blink.startTime) / (float)eye[e].blink.duration;
        } else { // DEBLINK
          // Opening: progress from 1 to 0  
          blinkProgress = 1.0 - ((float)(t - eye[e].blink.startTime) / (float)eye[e].blink.duration);
        }
        blinkProgress = constrain(blinkProgress, 0.0, 1.0);
        
        // Calculate how much of the eye height should be covered
        int16_t blinkHeight = (int16_t)(blinkProgress * viewH / 2);
        
        // Cover top and bottom portions
        if (screenY < blinkHeight || screenY >= (viewH - blinkHeight)) {
          pixelCoveredByEyelid = true;
        }
      }
      
      if (outsideEyeShape || pixelCoveredByEyelid) {
        p = 0; // Black pixel for background or eyelid
      } else if ((irisY < 0) || (irisY >= IRIS_HEIGHT) ||
                 (irisX < 0) || (irisX >= IRIS_WIDTH)) { // In sclera
        p = pgm_read_word(sclera + scleraY * SCLERA_WIDTH + scleraX);
      } else {                                          // Maybe iris...
        p = pgm_read_word(polar + irisY * IRIS_WIDTH + irisX);                        // Polar angle/dist
        d = (iScale * (p & 0x7F)) / 128;                // Distance (Y)
        if (d < IRIS_MAP_HEIGHT) {                      // Within iris area
          a = (IRIS_MAP_WIDTH * (p >> 7)) / 512;        // Angle (X)
          p = pgm_read_word(iris + d * IRIS_MAP_WIDTH + a);                           // Pixel = iris
        } else {                                        // Not in iris
          p = pgm_read_word(sclera + scleraY * SCLERA_WIDTH + scleraX);               // Pixel = sclera
        }
      }
      *(&pbuffer[dmaBuf][0] + pixels++) = p >> 8 | p << 8;

      if (pixels >= BUFFER_SIZE) {
        yield();
#ifdef USE_DMA
        tft.pushPixelsDMA(&pbuffer[dmaBuf][0], pixels);
        dmaBuf  = !dmaBuf;
#else
        tft.pushPixels(&pbuffer[0][0], pixels);
#endif
        pixels = 0;
      }
    }
  }

  if (pixels) {
#ifdef USE_DMA
    tft.pushPixelsDMA(&pbuffer[dmaBuf][0], pixels);
#else
    tft.pushPixels(&pbuffer[0][0], pixels);
#endif
  }
  tft.endWrite();
  digitalWrite(eye[e].tft_cs, HIGH);
  
  // Add random text displays
  static uint32_t lastTextTime = 0;
  static bool showTexts = false;
  static uint32_t textShowTime = 0;
  
  if (t - lastTextTime > 3000000) { // Every 3 seconds, check for text display
    lastTextTime = t;
    if (random(100) < 30) { // 30% chance to show both texts
      showTexts = true;
      textShowTime = t;
    }
  }
  
  // Display both texts at the same time when triggered
  if (showTexts) {
    // Display "I am watching you..." at top - white, bigger, centered
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2); // Bigger text
    String topText = "..i am watching you..";
    int16_t textWidth = topText.length() * 12; // Approximate width for size 2
    int16_t centerX = (320 - textWidth) / 2;
    tft.setCursor(centerX, 25); // Move closer to eye (was 15)
    tft.print(topText);
    
    // Display "The Angel Eye" at bottom - white, bigger, centered
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2); // Bigger text
    String bottomText = "..the Angel Eye..";
    textWidth = bottomText.length() * 12; // Approximate width for size 2
    centerX = (320 - textWidth) / 2;
    tft.setCursor(centerX, 195); // Move closer to eye (was 210)
    tft.print(bottomText);
    
    // Clear both texts after 2 seconds
    if (t - textShowTime > 2000000) {
      // Clear top text area - exact match to text position
      tft.fillRect(0, 25, 320, 16, TFT_BLACK); // Height 16 for size 2 text
      // Clear bottom text area - exact match to text position
      tft.fillRect(0, 195, 320, 16, TFT_BLACK); // Height 16 for size 2 text
      showTexts = false;
    }
  }
}

// EYE ANIMATION -----------------------------------------------------------

const uint8_t ease[] = { // Ease in/out curve for eye movements 3*t^2-2*t^3
  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,   // T
  3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9, 10, 10,   // h
  11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23,   // x
  24, 25, 26, 27, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39,   // 2
  40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58,   // A
  60, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 78, 80,   // l
  81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 98, 100, 101, 103, // e
  104, 106, 107, 109, 110, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127, // c
  128, 130, 131, 133, 134, 136, 137, 139, 140, 142, 143, 145, 146, 148, 149, 151, // J
  152, 154, 155, 157, 158, 159, 161, 162, 164, 165, 167, 168, 170, 171, 172, 174, // a
  175, 177, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 195, // c
  197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 213, 214, 215, // o
  216, 217, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 228, 229, 230, 231, // b
  232, 233, 234, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242, 243, 243, 244, // s
  245, 245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 251, 251, 251, 252, 252, // o
  252, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255
}; // n

#ifdef AUTOBLINK
uint32_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
#endif

// Process motion for a single frame of left or right eye
void frame(uint16_t iScale) // Iris scale (0-1023)
{
  static uint32_t frames   = 0; // Used in frame rate calculation
  static uint8_t  eyeIndex = 0; // eye[] array counter
  int16_t         eyeX, eyeY;
  uint32_t        t = micros(); // Time at start of function

  if (!(++frames & 255)) { // Every 256 frames...
    float elapsed = (millis() - startTime) / 1000.0;
    if (elapsed) Serial.println((uint16_t)(frames / elapsed)); // Print FPS
  }

  if (++eyeIndex >= NUM_EYES) eyeIndex = 0; // Cycle through eyes, 1 per call

  // X/Y movement

#if defined(JOYSTICK_X_PIN) && (JOYSTICK_X_PIN >= 0) && \
    defined(JOYSTICK_Y_PIN) && (JOYSTICK_Y_PIN >= 0)

  // Read X/Y from joystick, constrain to circle
  int16_t dx, dy;
  int32_t d;
  eyeX = analogRead(JOYSTICK_X_PIN); // Raw (unclipped) X/Y reading
  eyeY = analogRead(JOYSTICK_Y_PIN);
#ifdef JOYSTICK_X_FLIP
  eyeX = 1023 - eyeX;
#endif
#ifdef JOYSTICK_Y_FLIP
  eyeY = 1023 - eyeY;
#endif
  dx = (eyeX * 2) - 1023; // A/D exact center is at 511.5.  Scale coords
  dy = (eyeY * 2) - 1023; // X2 so range is -1023 to +1023 w/center at 0.
  if ((d = (dx * dx + dy * dy)) > (1023 * 1023)) { // Outside circle
    d    = (int32_t)sqrt((float)d);               // Distance from center
    eyeX = ((dx * 1023 / d) + 1023) / 2;          // Clip to circle edge,
    eyeY = ((dy * 1023 / d) + 1023) / 2;          // scale back to 0-1023
  }

#else // Autonomous X/Y eye motion
  // Periodically initiates motion to a new random point, random speed,
  // holds there for random period until next motion.

  static bool  eyeInMotion      = false;
  static int16_t  eyeOldX = 512, eyeOldY = 512, eyeNewX = 512, eyeNewY = 512;
  static uint32_t eyeMoveStartTime = 0L;
  static int32_t  eyeMoveDuration  = 0L;

  int32_t dt = t - eyeMoveStartTime;      // uS elapsed since last eye event
  if (eyeInMotion) {                      // Currently moving?
    if (dt >= eyeMoveDuration) {          // Time up?  Destination reached.
      eyeInMotion      = false;           // Stop moving
      eyeMoveDuration  = random(3000000); // 0-3 sec stop
      eyeMoveStartTime = t;               // Save initial time of stop
      eyeX = eyeOldX = eyeNewX;           // Save position
      eyeY = eyeOldY = eyeNewY;
    } else { // Move time's not yet fully elapsed -- interpolate position
      int16_t e = ease[255 * dt / eyeMoveDuration] + 1;   // Ease curve
      eyeX = eyeOldX + (((eyeNewX - eyeOldX) * e) / 256); // Interp X
      eyeY = eyeOldY + (((eyeNewY - eyeOldY) * e) / 256); // and Y
    }
  } else {                                // Eye stopped
    eyeX = eyeOldX;
    eyeY = eyeOldY;
    if (dt > eyeMoveDuration) {           // Time up?  Begin new move.
      int16_t  dx, dy;
      uint32_t d;
      do {                                // Pick new dest in circle
        eyeNewX = random(1024);
        eyeNewY = random(1024);
        dx      = (eyeNewX * 2) - 1023;
        dy      = (eyeNewY * 2) - 1023;
      } while ((d = (dx * dx + dy * dy)) > (1023 * 1023)); // Keep trying
      eyeMoveDuration  = random(72000, 144000); // ~1/14 - ~1/7 sec
      eyeMoveStartTime = t;               // Save initial time of move
      eyeInMotion      = true;            // Start move on next frame
    }
  }
#endif // JOYSTICK_X_PIN etc.

  // Blinking
#ifdef AUTOBLINK
  // Similar to the autonomous eye movement above -- blink start times
  // and durations are random (within ranges).
  if ((t - timeOfLastBlink) >= timeToNextBlink) { // Start new blink?
    timeOfLastBlink = t;
    uint32_t blinkDuration = random(36000, 72000); // ~1/28 - ~1/14 sec
    // Set up durations for both eyes (if not already winking)
    for (uint8_t e = 0; e < NUM_EYES; e++) {
      if (eye[e].blink.state == NOBLINK) {
        eye[e].blink.state     = ENBLINK;
        eye[e].blink.startTime = t;
        eye[e].blink.duration  = blinkDuration;
#ifdef DYNAMIC_BACKLIGHT
        // Dim backlight when new blink starts
        analogWrite(DISPLAY_BACKLIGHT, BACKLIGHT_DIM);
#endif
      }
    }
    timeToNextBlink = blinkDuration * 3 + random(4000000);
  }
#endif

  if (eye[eyeIndex].blink.state) { // Eye currently blinking?
    // Check if current blink state time has elapsed
    if ((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration) {
      // Yes -- increment blink state, unless...
      if ((eye[eyeIndex].blink.state == ENBLINK) && ( // Enblinking and...
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
            (digitalRead(BLINK_PIN) == LOW) ||           // blink or wink held...
#endif
            ((eyeInfo[eyeIndex].wink >= 0) &&
             digitalRead(eyeInfo[eyeIndex].wink) == LOW) )) {
        // Don't advance state yet -- eye is held closed instead
      } else { // No buttons, or other state...
        if (++eye[eyeIndex].blink.state > DEBLINK) { // Deblinking finished?
          eye[eyeIndex].blink.state = NOBLINK;      // No longer blinking
#ifdef DYNAMIC_BACKLIGHT
          // Restore normal backlight when blink finishes
          analogWrite(DISPLAY_BACKLIGHT, BACKLIGHT_NORMAL);
#endif
        } else { // Advancing from ENBLINK to DEBLINK mode
          eye[eyeIndex].blink.duration *= 2; // DEBLINK is 1/2 ENBLINK speed
          eye[eyeIndex].blink.startTime = t;
#ifdef DYNAMIC_BACKLIGHT
          // Dim backlight during blink
          analogWrite(DISPLAY_BACKLIGHT, BACKLIGHT_DIM);
#endif
        }
      }
    }
  } else { // Not currently blinking...check buttons!
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
    if (digitalRead(BLINK_PIN) == LOW) {
      // Manually-initiated blinks have random durations like auto-blink
      uint32_t blinkDuration = random(36000, 72000);
      for (uint8_t e = 0; e < NUM_EYES; e++) {
        if (eye[e].blink.state == NOBLINK) {
          eye[e].blink.state     = ENBLINK;
          eye[e].blink.startTime = t;
          eye[e].blink.duration  = blinkDuration;
        }
      }
    } else
#endif
      if ((eyeInfo[eyeIndex].wink >= 0) &&
          (digitalRead(eyeInfo[eyeIndex].wink) == LOW)) { // Wink!
        eye[eyeIndex].blink.state     = ENBLINK;
        eye[eyeIndex].blink.startTime = t;
        eye[eyeIndex].blink.duration  = random(45000, 90000);
      }
  }

  // Process motion, blinking and iris scale into renderable values

  // Scale eye X/Y positions (0-1023) to pixel units used by drawEye()
  // Use smaller view window to allow panning motion within the 200x200 bitmap
  const int16_t rangeX = SCLERA_WIDTH  - VIEW_W;  // e.g. 200 - 160 = 40px travel
  const int16_t rangeY = SCLERA_HEIGHT - VIEW_H;  // e.g. 40px travel
  eyeX = constrain(map(eyeX, 0, 1023, 0, rangeX), 0, rangeX);
  eyeY = constrain(map(eyeY, 0, 1023, 0, rangeY), 0, rangeY);

  // Horizontal position is offset so that eyes are very slightly crossed
  // to appear fixated (converged) at a conversational distance.  Number
  // here was extracted from my posterior and not mathematically based.
  // I suppose one could get all clever with a range sensor, but for now...
  if (NUM_EYES > 1) {
    if (eyeIndex == 1) eyeX += 4;
    else eyeX -= 4;
  }
  if (eyeX > (SCLERA_WIDTH - 128)) eyeX = (SCLERA_WIDTH - 128);

  // Eyelids are rendered using a brightness threshold image.  This same
  // map can be used to simplify another problem: making the upper eyelid
  // track the pupil (eyes tend to open only as much as needed -- e.g. look
  // down and the upper eyelid drops).  Just sample a point in the upper
  // lid map slightly above the pupil to determine the rendering threshold.
  static uint8_t uThreshold = 128;
  uint8_t        lThreshold, n;
#ifdef TRACKING
  int16_t sampleX = SCLERA_WIDTH  / 2 - (eyeX / 2), // Reduce X influence
          sampleY = SCLERA_HEIGHT / 2 - (eyeY + IRIS_HEIGHT / 4);
  // Eyelid is slightly asymmetrical, so two readings are taken, averaged
  if (sampleY < 0) n = 0;
  else            n = (pgm_read_byte(upper + sampleY * SCREEN_WIDTH + sampleX) +
                         pgm_read_byte(upper + sampleY * SCREEN_WIDTH + (SCREEN_WIDTH - 1 - sampleX))) / 2;
  uThreshold = (uThreshold * 3 + n) / 4; // Filter/soften motion
  // Lower eyelid doesn't track the same way, but seems to be pulled upward
  // by tension from the upper lid.
  lThreshold = 254 - uThreshold;
#else // No tracking -- eyelids full open unless blink modifies them
  uThreshold = lThreshold = 0;
#endif

  // The upper/lower thresholds are then scaled relative to the current
  // blink position so that blinks work together with pupil tracking.
  if (eye[eyeIndex].blink.state) { // Eye currently blinking?
    uint32_t s = (t - eye[eyeIndex].blink.startTime);
    if (s >= eye[eyeIndex].blink.duration) s = 255;  // At or past blink end
    else s = 255 * s / eye[eyeIndex].blink.duration; // Mid-blink
    s          = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;
    n          = (uThreshold * s + 254 * (257 - s)) / 256;
    lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
  } else {
    n          = uThreshold;
  }

  // Pass all the derived values to the eye-rendering function:
  drawEye(eyeIndex, iScale, eyeX, eyeY, n, lThreshold);

  if (eyeIndex == (NUM_EYES - 1)) {
    user_loop(); // Call user code after rendering last eye
  }
}

// AUTONOMOUS IRIS SCALING (if no photocell or dial) -----------------------

#if !defined(LIGHT_PIN) || (LIGHT_PIN < 0)

// Autonomous iris motion uses a fractal behavior to similate both the major
// reaction of the eye plus the continuous smaller adjustments that occur.

void split( // Subdivides motion path into two sub-paths w/randimization
  int16_t  startValue, // Iris scale value (IRIS_MIN to IRIS_MAX) at start
  int16_t  endValue,   // Iris scale value at end
  uint32_t startTime,  // micros() at start
  int32_t  duration,   // Start-to-end time, in microseconds
  int16_t  range) {    // Allowable scale value variance when subdividing

  if (range >= 8) {    // Limit subdvision count, because recursion
    range    /= 2;     // Split range & time in half for subdivision,
    duration /= 2;     // then pick random center point within range:
    int16_t  midValue = (startValue + endValue - range) / 2 + random(range);
    uint32_t midTime  = startTime + duration;
    split(startValue, midValue, startTime, duration, range); // First half
    split(midValue  , endValue, midTime  , duration, range); // Second half
  } else {             // No more subdivisons, do iris motion...
    int32_t dt;        // Time (micros) since start of motion
    int16_t v;         // Interim value
    while ((dt = (micros() - startTime)) < duration) {
      v = startValue + (((endValue - startValue) * dt) / duration);
      if (v < IRIS_MIN)      v = IRIS_MIN; // Clip just in case
      else if (v > IRIS_MAX) v = IRIS_MAX;
      frame(v);        // Draw frame w/interim iris scale value
    }
  }
}
#endif // !LIGHT_PIN
