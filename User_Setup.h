#define USER_SETUP_INFO "User_Setup"

// #define STM32 // doesnt work with this on, why?????
#define ST7789_DRIVER
#define TFT_RGB_ORDER TFT_BGR
#define TFT_INVERSION_OFF

#define TFT_WIDTH  240
#define TFT_HEIGHT 320


#define TFT_MOSI PA7
#define TFT_SCLK PA5
#define TFT_CS   PA4
#define TFT_DC   PA3
#define TFT_RST  PA2

#define LOAD_GFXFF
#define LOAD_FONT2
#define LOAD_FONT4
#define SMOOTH_FONT

#define SPI_FREQUENCY 80000000