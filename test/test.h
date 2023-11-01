#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <Update.h>
#include <WebServer.h>
#include <DNSServer.h>

#define PANEL_RES_X 64    // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 64     // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1      // Total number of panels chained one to another
#define USE_FLOATHACK      // To boost float performance, comment if this doesn't work. 

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

MatrixPanel_I2S_DMA *dma_display = nullptr;

// inspired by
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
#ifdef USE_FLOATHACK
// cast float as int32_t
  int32_t intfloat(float n){ return *(int32_t *)&n; }
// cast int32_t as float
  float floatint(int32_t n){ return *(float *)&n; }
// fast approx sqrt(x)
  float floatsqrt(float n){ return floatint(0x1fbb4000+(intfloat(n)>>1)); }
// fast approx 1/x
  float floatinv(float n){ return floatint(0x7f000000-intfloat(n)); }
// fast approx log2(x)
  float floatlog2(float n){ return (float)((intfloat(n)<<1)-0x7f000000)*5.9604645e-08f; }
#else
  float floatinv(float n){ return 1.f/n;}
  float floatsqrt(float n){ return std::sqrt(n); }
  float floatlog2(float n){ return std::log2f(n); }
#endif

////////////////////////////////////////
// Escape time mandelbrot set function,
// with arbitrary start point zx, zy
// and arbitrary seed point ax, ay
//
// For julia set
// zx = pos_x,  zy = pos_y;
// ax = seed_x, ay = seed_y;
//
// For mandelbrot set
// zx = 0, zy = 0;
// ax = pos_x,  ay = pos_y;
//
const float  bailOut = 4;     // Escape radius
const int32_t itmult = 1<<10; // Color speed
//
// https://en.wikipedia.org/wiki/Mandelbrot_set
int32_t iteratefloat(float ax, float ay, float zx, float zy, uint16_t mxIT) {
  float zzl = 0;
  for (int it = 0; it<mxIT; it++) {
    float zzx = zx * zx;
    float zzy = zy * zy;
    // is the point is escaped?
    if(zzx+zzy>=bailOut){
      if(it>0){
        // calculate smooth coloring
        float zza = floatlog2(zzl);
        float zzb = floatlog2(zzx+zzy);
        float zzc = floatlog2(bailOut);
        float zzd = (zzc-zza)*floatinv(zzb-zza);
        return it*itmult+zzd*itmult;
      }
    };
    // z -> z*z + c
    zy = 2.f*zx*zy+ay;
    zx = zzx-zzy+ax;
    zzl = zzx+zzy;
  }
  return 0;
}

float sint[256]; // precalculated sin table, for performance reasons

// Palette color taken from:
// https://editor.p5js.org/Kouzerumatsukite/sketches/DwTiq9D01
// color palette originally made by piano_miles, written in p5js
// hsv2rgb(IT, cos(4096*it)/2+0.5, 1-sin(2048*it)/2-0.5)
void drawPixelPalette(int x, int y, uint32_t m){
  float r = 0.f, g = 0.f, b = 0.f;
  if(m){
    char  n =         m>> 4                               ;
    float l =abs(sint[m>> 2&255]         )*255.f          ;
    float s =   (sint[m    &255]+     1.f)*0.5f           ;
    r = (max(min(sint[n    &255]+0.5f,1.f),0.f)*s+(1-s))*l;
    g = (max(min(sint[n+ 85&255]+0.5f,1.f),0.f)*s+(1-s))*l;
    b = (max(min(sint[n+170&255]+0.5f,1.f),0.f)*s+(1-s))*l;
  }
  dma_display->drawPixelRGB888(x,y,r,g,b);
}

void drawCanvas() {
  uint32_t lastMicros = micros();
  double t = (double)lastMicros/8000000;
  double k = sin(t*3.212/2)*sin(t*3.212/2)/16+1;
  float cosk = (k-cos(t))/2;
  float xoff = (cos(t)*cosk+k/2-0.25);
  float yoff = (sin(t)*cosk         );
  for(uint8_t y=0;y<PANEL_RES_Y;y++){
    for(uint8_t x=0;x<PANEL_RES_X;x++){
      uint32_t itcount = iteratefloat(xoff,yoff,((x-64)+1)/64.f,(y)/64.f,64);
      uint32_t itcolor = itcount?floatsqrt(itcount)*4+t*1024:0;
      drawPixelPalette(x,y,itcolor);
    }
  }
}

volatile int frameCounts=0;
void Task1code(void *parameter){
  while(true){
    drawCanvas();
    delay(1);
    frameCounts++;
  }
}

// // Pin definition
// //ESP32-HUB75E牛角扣-RGB-LED
// #define R1_PIN 14
// #define G1_PIN 27
// #define B1_PIN 26
// #define R2_PIN 25
// #define G2_PIN 33
// #define B2_PIN 32
// #define A_PIN 13
// #define B_PIN 15
// #define C_PIN 2
// #define D_PIN 4
// #define E_PIN 16 
// #define LAT_PIN 5
// #define OE_PIN 18
// #define CLK_PIN 17

void setup() {
  HUB75_I2S_CFG::i2s_pins _pins={
//  R1, G1, B1, R2, G2, B2,  A,  B,  C,  D,  E,LAT, OE,CLK, 
    // 25, 26, 27, 14, 12, 13, 23, 19,  5, 17, 18,  4, 15, 16,
    14, 27, 26, 25, 33, 32, 13, 15,  2, 4, 16,  5, 18, 17,
  };
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X, // Module width
    PANEL_RES_Y, // Module height
    PANEL_CHAIN, // chain length
    _pins
  );

  // Display Setup
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->clearScreen();
  dma_display->setBrightness(64);
  setCpuFrequencyMhz(240);

  for(int i=0;i<256;i++){
    sint[i] = sinf(i/256.f*2.f*PI);
  }

  xTaskCreatePinnedToCore(\
      Task1code, /* Function to implement the task */\
      "Task1", /* Name of the task */\
      10000,  /* Stack size in words */\
      NULL,  /* Task input parameter */\
      4,  /* Priority of the task */\
      NULL,  /* Task handle. */\
      0); /* Core where the task should run */

  Serial.begin(115200);
}

uint64_t lastMillis=0;
void loop() {
  if(millis()-lastMillis>=1000){
    // log frame rate to serial
    Serial.print("fps: ");
    Serial.println(frameCounts);
    lastMillis += 1000;
    frameCounts=0;
  }
}


#include <Arduino.h>
#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <Update.h>
#include <WebServer.h>
#include <DNSServer.h>

/*********************************************************************
 * AnimatedGif LED Matrix Panel example where the GIFs are 
 * stored on a SD card connected to the ESP32 using the
 * standard GPIO pins used for SD card acces via. SPI.
 *
 * Put the gifs into a directory called 'gifs' (case sensitive) on
 * a FAT32 formatted SDcard.
 ********************************************************************/
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h> 
#include <AnimatedGIF.h>

/********************************************************************
 * Pin mapping below is for LOLIN D32 (ESP 32)
 *
 * Default pin mapping used by this library is NOT compatable with the use of the 
 * ESP32-Arduino 'SD' card library (there is overlap). As such, some of the pins 
 * used for the HUB75 panel need to be shifted.
 * 
 * 'SD' card library requires GPIO 23, 18 and 19 
 *  https://github.com/espressif/arduino-esp32/tree/master/libraries/SD
 * 
 */

/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */

/**** SD Card GPIO mappings ****/
#define SS_PIN          5
//#define MOSI_PIN        23
//#define MISO_PIN        19
//#define CLK_PIN         18


/**** HUB75 GPIO mapping ****/
// GPIO 34+ are on the ESP32 are input only!!
// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// #define E_PIN   12
// #define A_PIN   33  // remap esp32 library default from 23 to 33
// #define B_PIN   35  // remap esp32 library default from 19 to 32
// #define C_PIN   22   // remap esp32 library defaultfrom 5 to 22

//#define R1_PIN  25  // library default for the esp32, unchanged
//#define G1_PIN  26  // library default for the esp32, unchanged
//#define B1_PIN  27  // library default for the esp32, unchanged
//#define R2_PIN  14  // library default for the esp32, unchanged
//#define G2_PIN  12  // library default for the esp32, unchanged
//#define B2_PIN  13  // library default for the esp32, unchanged
//#define D_PIN   17  // library default for the esp32, unchanged
//#define E_PIN   -1 // IMPORTANT: Change to a valid pin if using a 64x64px panel.
            
//#define LAT_PIN 4  // library default for the esp32, unchanged
//#define OE_PIN  15  // library default for the esp32, unchanged
//#define CLK_PIN 16  // library default for the esp32, unchanged

#define R1_PIN 25
#define G1_PIN 26
#define B1_PIN 27
#define R2_PIN 14
#define G2_PIN 12
#define B2_PIN 13
#define A_PIN 33
#define B_PIN 35
#define C_PIN 22
#define D_PIN 17
#define E_PIN 32 
#define LAT_PIN 4
#define OE_PIN 15
#define CLK_PIN 16

/***************************************************************
 * HUB 75 LED DMA Matrix Panel Configuration
 **************************************************************/
#define PANEL_RES_X 64      // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 64     // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 2      // Total number of panels chained one to another

/**************************************************************/

AnimatedGIF gif;
MatrixPanel_I2S_DMA *dma_display = nullptr;

static int totalFiles = 0; // GIF files count

static File FSGifFile; // temp gif file holder
static File GifRootFolder; // directory listing

std::vector<std::string> GifFiles; // GIF files path

const int maxGifDuration    = 30000; // ms, max GIF duration

#include "gif_functions.hpp"
#include "sdcard_functions.hpp"
 

/**************************************************************/
void draw_test_patterns();
int gifPlay( const char* gifPath )
{ // 0=infinite

  if( ! gif.open( gifPath, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw ) ) {
    log_n("Could not open gif %s", gifPath );
  }

  Serial.print("Playing: "); Serial.println(gifPath);

  int frameDelay = 0; // store delay for the last frame
  int then = 0; // store overall delay

  while (gif.playFrame(true, &frameDelay)) {

    then += frameDelay;
    if( then > maxGifDuration ) { // avoid being trapped in infinite GIF's
      //log_w("Broke the GIF loop, max duration exceeded");
      break;
    }
  }

  gif.close();

  return then;
}

void draw_test()
{
 // fix the screen with green
  dma_display->fillRect(0, 0, dma_display->width(), dma_display->height(), dma_display->color444(0, 15, 0));
  delay(500);

  // draw a box in yellow
  dma_display->drawRect(0, 0, dma_display->width(), dma_display->height(), dma_display->color444(15, 15, 0));
  delay(500);

  // draw an 'X' in red
  dma_display->drawLine(0, 0, dma_display->width()-1, dma_display->height()-1, dma_display->color444(15, 0, 0));
  dma_display->drawLine(dma_display->width()-1, 0, 0, dma_display->height()-1, dma_display->color444(15, 0, 0));
  delay(500);

  // draw a blue circle
  dma_display->drawCircle(10, 10, 10, dma_display->color444(0, 0, 15));
  delay(500);

  // fill a violet circle
  dma_display->fillCircle(40, 21, 10, dma_display->color444(15, 0, 15));
  delay(500);
  delay(1000);

}

void setup()
{
    Serial.begin(115200);

    // **************************** Setup SD Card access via SPI ****************************
    if(!SD.begin(SS_PIN)){
        //  bool begin(uint8_t ssPin=SS, SPIClass &spi=SPI, uint32_t frequency=4000000, const char * mountpoint="/sd", uint8_t max_files=5, bool format_if_empty=false);      
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    //listDir(SD, "/", 1, false);

    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));



    // **************************** Setup DMA Matrix ****************************
    HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
    HUB75_I2S_CFG mxconfig(
    PANEL_RES_X, // Module width
    PANEL_RES_Y, // Module height
    PANEL_CHAIN, // chain length
    _pins // pin mapping
    );

    mxconfig.gpio.e = E_PIN;
    mxconfig.clkphase = false;
    mxconfig.driver = HUB75_I2S_CFG::FM6124;

    // Display Setup
    dma_display = new MatrixPanel_I2S_DMA(mxconfig);

    // Allocate memory and start DMA display
    if( not dma_display->begin() )
        Serial.println("****** !KABOOM! HUB75 memory allocation failed ***********");
 
    dma_display->setBrightness8(128); //0-255
    dma_display->clearScreen();

    draw_test();
    // **************************** Setup Sketch ****************************
    Serial.println("Starting AnimatedGIFs Sketch");

    // SD CARD STOPS WORKING WITH DMA DISPLAY ENABLED>...

    File root = SD.open("/gifs");
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(!file.isDirectory())
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());

            std::string filename = "/gifs/" + std::string(file.name());
            Serial.println(filename.c_str());
            
            GifFiles.push_back( filename );
         //   Serial.println("Adding to gif list:" + String(filename));
            totalFiles++;
    
        }
        file = root.openNextFile();
    }

    file.close();
    Serial.printf("Found %d GIFs to play.", totalFiles);
    //totalFiles = getGifInventory("/gifs");



  // This is important - Set the right endianness.
  gif.begin(LITTLE_ENDIAN_PIXELS);

}

void loop(){

      // Iterate over a vector using range based for loop
    for(auto & elem : GifFiles)
    {
        gifPlay( elem.c_str() );
        gif.reset();
        delay(500);
    }

}


