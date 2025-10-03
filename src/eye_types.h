#ifndef EYE_TYPES_H
#define EYE_TYPES_H

#include <stdint.h>

// Blink state constants
#define NOBLINK 0       // Not currently engaged in a blink
#define ENBLINK 1       // Eyelid is currently closing  
#define DEBLINK 2       // Eyelid is currently opening

// Blink structure
typedef struct {
  uint8_t  state;       // NOBLINK/ENBLINK/DEBLINK
  uint32_t duration;    // Duration of blink state (micros)
  uint32_t startTime;   // Time (micros) of last state change
} eyeBlink;

// Eye structure
typedef struct {
  int16_t   tft_cs;     // Chip select pin for each display
  eyeBlink  blink;      // Current blink/wink state
  int16_t   xposition;  // x position of eye image
} eyeStruct_t;

#endif // EYE_TYPES_H
