# ESP32 Eye Animation for Elecrow 2.4" Display

An animated eye project for ESP32 using the Elecrow 2.4" ILI9341 display, built with PlatformIO.

## Features

- 🎯 **Realistic Eye Animation** - Natural eye movement with saccades and smooth tracking
- 👁️ **Eye Shape Masking** - Oval-shaped eye rendering for realistic appearance  
- 😉 **Automatic Blinking** - Smooth geometric blinking animation
- 💬 **Text Messages** - Random display of "I am watching you..." and "The Angel Eye"
- 🎨 **Multiple Eye Types** - 10 different eye designs to choose from
- 💡 **PWM Backlight Control** - Adjustable brightness with dynamic effects
- 🔧 **PlatformIO Ready** - Modern build system with proper dependency management

## Hardware Requirements

- ESP32 Development Board
- Elecrow 2.4" ILI9341 Display (240x320 pixels)
- Connecting wires

## Pin Configuration

```cpp
TFT_MISO = 12
TFT_MOSI = 13  
TFT_SCLK = 14
TFT_CS   = 15
TFT_DC   = 2
TFT_BL   = 27   // Backlight (PWM controlled)
```

## Available Eye Types

Switch between different eye designs by editing `src/config.h`:

- `defaultEye.h` - Standard human-ish hazel eye
- `dragonEye.h` - Slit pupil fiery dragon/demon eye
- `catEye.h` - Feline eye with vertical pupil
- `owlEye.h` - Large owl eye
- `terminatorEye.h` - Robotic/cyborg eye
- `doeEye.h` - Gentle doe eye
- `goatEye.h` - Horizontal rectangular pupil
- `newtEye.h` - Amphibian eye
- `naugaEye.h` - Fantasy creature eye
- `noScleraEye.h` - Large iris, no sclera

## Installation

1. **Clone this repository:**
   ```bash
   git clone https://github.com/luisgcu/elecroweyeanim.git
   cd elecroweyeanim
   ```

2. **Install PlatformIO** (if not already installed):
   - VS Code Extension: Install "PlatformIO IDE"
   - Or CLI: `pip install platformio`

3. **Build and Upload:**
   ```bash
   pio run --target upload
   ```

## Configuration

### Changing Eye Type
Edit `src/config.h` and uncomment the desired eye:

```cpp
// Enable ONE of these #includes:
#include "data/defaultEye.h"      // Standard human-ish hazel eye -OR-
//#include "data/dragonEye.h"     // Slit pupil fiery dragon/demon eye -OR-
//#include "data/catEye.h"        // Feline eye -OR-
// ... etc
```

### Adjusting Eye Shape
Modify the oval parameters in `src/eye_functions.cpp`:

```cpp
// Adjust these values to change eye shape
float horizontalRadius = 0.51;  // Eye width (0.1 to 0.6)
float verticalRadius = 0.34;    // Eye height (0.1 to 0.5)
```

### Backlight Control
Change brightness levels in `src/config.h`:

```cpp
#define BACKLIGHT_NORMAL 200    // Normal brightness (0-255)
#define BACKLIGHT_DIM 100       // Dimmed brightness for blinks
```

## Project Structure

```
├── src/
│   ├── main.cpp              # Main application
│   ├── eye_functions.cpp     # Eye rendering and animation
│   ├── config.h              # Configuration settings
│   ├── User_Setup.h          # TFT display configuration
│   └── data/                 # Eye graphics data
│       ├── defaultEye.h
│       ├── dragonEye.h
│       └── ... (other eye types)
├── lib/
│   └── TFT_eSPI/            # Display library
├── platformio.ini           # PlatformIO configuration
└── README.md               # This file
```

## Customization

### Text Messages
Edit the text arrays in `src/eye_functions.cpp`:

```cpp
const char* messages[] = {
  "I am watching you...",
  "Always observing...",
  "Never blinking..."
};
```

### Animation Timing
Adjust timing constants in `src/config.h`:

```cpp
#define TRACKING true           // Enable eye tracking
#define AUTOBLINK true          // Enable automatic blinking
#define BLINK_PIN -1           // Manual blink trigger pin (-1 to disable)
```

## Troubleshooting

**Display shows corrupted graphics:**
- Check wiring connections
- Verify pin configuration in `User_Setup.h`
- Ensure stable 3.3V/5V power supply

**Eye appears as square instead of oval:**
- Check that oval masking code is enabled
- Verify VIEW_W and VIEW_H settings in `config.h`

**No eye movement:**
- Ensure TRACKING is enabled in `config.h`
- Check that eye movement range is not zero

## License

This project is based on the original Adafruit "Uncanny Eyes" project and has been adapted for PlatformIO and ESP32 with additional features.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Credits

- Original concept: Adafruit Industries
- PlatformIO adaptation and enhancements: @luisgcu
- TFT_eSPI library: Bodmer

---

**Enjoy your creepy animated eye! 👁️**