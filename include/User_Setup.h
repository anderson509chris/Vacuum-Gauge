// TFT_eSPI display configuration for LILYGO T-QT Pro S3
// (GC9A01 round 128x128 display)
// Used only when building the lilygo_t_qt environment.
// Verify pin numbers against the actual board schematic if display doesn't init.

#define USER_SETUP_ID 211

#define GC9A01_DRIVER

#define TFT_WIDTH  128
#define TFT_HEIGHT 128

// SPI display pins
#define TFT_MOSI  2
#define TFT_SCLK  3
#define TFT_CS    5
#define TFT_DC    6
#define TFT_RST   1
#define TFT_BL    10   // backlight, active HIGH

// Fonts to include
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7

#define SPI_FREQUENCY  40000000
