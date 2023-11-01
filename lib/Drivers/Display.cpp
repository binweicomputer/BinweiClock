#include "Display.h"

// // Commons
#include "Common.h"
// #include "Config.h"
#include <spi.h>

#include <AnimatedGIF.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

//Log message persistence
//Is a log message currently displayed?
bool logMessageActive = false;
//When was the message shown?
unsigned long messageDisplayMillis = 0;

// Flags to trigger display updates
bool clockStartingUp = true;
bool newSensorData = false;
bool sensorDead = true;

// Sensor data
float sensorTemp;
int sensorHumi;

AnimatedGIF gif;

static int totalFiles = 0; // GIF files count

static File FSGifFile; // temp gif file holder
static File GifRootFolder; // directory listing

std::vector<std::string> GifFiles; // GIF files path

const int maxGifDuration    = 30000; // ms, max GIF duration

#include "gif_functions.hpp"
#include "sdcard_functions.hpp"


Display::Display() {
};

Display::~Display() {
};

uint16_t Display::colorWheel(uint8_t pos) {
  if(pos < 85) {
    return dma_display->color565(pos * 3, 255 - pos * 3, 0);
  } else if(pos < 170) {
    pos -= 85;
    return dma_display->color565(255 - pos * 3, 0, pos * 3);
  } else {
    pos -= 170;
    return dma_display->color565(0, pos * 3, 255 - pos * 3);
  }
}

void Display::DisplayInit(bool swapBlueGreen, uint8_t displayBright) {
    // **************************** Setup DMA Matrix ****************************    
    HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
    HUB75_I2S_CFG mxconfig(
      PANEL_RES_X, // Module width
      PANEL_RES_Y, // Module height
      PANEL_CHAIN, // chain length
      _pins // pin mapping
    ); 

    if (swapBlueGreen)
    {
        // 交换蓝色和绿色引脚，因为面板是RBG而不是RGB
        mxconfig.gpio.b1 = 27;
        mxconfig.gpio.b2 = 33;
        mxconfig.gpio.g1 = 26;
        mxconfig.gpio.g2 = 32;
    }
    // 需要重新映射这些HUB75 DMA引脚，因为SPI SD卡正在使用它们.
    // 否则SD卡将无法工作.
    // mxconfig.gpio.a = A_PIN;
    // mxconfig.gpio.b = B_PIN;
    // mxconfig.gpio.c = C_PIN;
    // mxconfig.gpio.b1 = B1_PIN;
    // mxconfig.gpio.b2 = B2_PIN;
    // mxconfig.gpio.g1 = G1_PIN;
    // mxconfig.gpio.g2 = G2_PIN;
    // mxconfig.gpio.r1 = R1_PIN;
    // mxconfig.gpio.r2 = R2_PIN;    
    // mxconfig.gpio.d = D_PIN; 
    // mxconfig.gpio.e = E_PIN;  
    // mxconfig.gpio.oe = OE_PIN ; 

    mxconfig.gpio.e = E_PIN;
    mxconfig.clkphase = false;
    mxconfig.driver = HUB75_I2S_CFG::FM6126A;

    // Display Setup
    dma_display = new MatrixPanel_I2S_DMA(mxconfig);

    // Allocate memory and start DMA display
    if( not dma_display->begin() )
        Serial.println("****** !KABOOM! HUB75 memory allocation failed ***********");
 
    dma_display->setBrightness8(displayBright); //0-255
    dma_display->clearScreen();    
}

// 显示测试
void Display::DisplyTest(int delayMs){
  for(int i = 0;i<PANEL_HEIGHT;i++){
    dma_display->drawFastHLine(0,i,PANEL_WIDTH,dma_display->color565(map(i,0,128,0,255),map(i,0,128,0,255),map(i,0,128,0,255)));
    delay(10);
  }
  for(int i = 0;i<PANEL_HEIGHT;i++){
    dma_display->drawFastHLine(0,i,PANEL_WIDTH,dma_display->color565(map(i,0,128,0,255),map(i,0,128,255,0),map(i,0,128,0,255)));
    delay(10);
  }
  for(int i = 0;i<PANEL_HEIGHT;i++){
    dma_display->drawFastHLine(0,i,PANEL_WIDTH,dma_display->color565(map(i,0,128,255,0),map(i,0,128,0,255),map(i,0,128,0,255)));
    delay(10);
  }
  for(int i = 0;i<PANEL_HEIGHT;i++){
    dma_display->drawFastHLine(0,i,PANEL_WIDTH,dma_display->color565(map(i,0,128,0,255),map(i,0,128,0,255),map(i,0,128,255,0)));
    delay(10);
  }
  dma_display->fillScreen(dma_display->color444(0, 0, 0));
  // 简单的R/G/B屏幕填充，用于测试显示器
  dma_display->fillRect(0, 0, PANEL_WIDTH, PANEL_HEIGHT, dma_display->color565(255, 0, 0));
  delay(delayMs);
  dma_display->fillRect(0, 0, PANEL_WIDTH, PANEL_HEIGHT, dma_display->color565(0, 255, 0));
  delay(delayMs);
  dma_display->fillRect(0, 0, PANEL_WIDTH, PANEL_HEIGHT, dma_display->color565(0, 0, 255));
  delay(delayMs);
  dma_display->fillRect(0, 0, PANEL_WIDTH, PANEL_HEIGHT, dma_display->color565(0, 0, 0));
  delay(delayMs);
  // 画黄色边框
  dma_display->drawRect(0, 0, PANEL_WIDTH, PANEL_HEIGHT, dma_display->color444(15, 15, 0));
  delay(delayMs);
  // 画红色X
  dma_display->drawLine(0, 0, PANEL_WIDTH-1, PANEL_HEIGHT-1, dma_display->color444(15, 0, 0));
  dma_display->drawLine(PANEL_WIDTH-1, 0, 0, PANEL_HEIGHT-1, dma_display->color444(15, 0, 0));
  delay(delayMs);
  // 画蓝色圈
  dma_display->drawCircle(PANEL_WIDTH/2, PANEL_HEIGHT/2, 20, dma_display->color444(0, 0, 15));
  delay(delayMs);
  // 画实心圈
  dma_display->fillCircle(PANEL_WIDTH/2, PANEL_HEIGHT/2, 10, dma_display->color444(15, 0, 15));
  delay(delayMs);
  // fill the screen with 'black'
  dma_display->fillScreen(dma_display->color444(0, 0, 0));
  //drawText(0);
}

void Display::LogStatusMessage(const char *message) {
  Serial.println(message);
  // 先清除最后一行!
  dma_display->fillRect(0, 56, 192, 8, 0);
  dma_display->setTextSize(1);     // size 1 == 8 pixels high
  dma_display->setTextWrap(false); // Don't wrap at end of line - will do ourselves
  dma_display->setCursor(0, 56);   // Write on last line
  dma_display->setTextColor(LOG_MESSAGE_COLOR);
  dma_display->print(message);
  messageDisplayMillis = millis();
  logMessageActive = true;
}


void Display::LogStatusMessage(String message) {
  Serial.println(message);
  // 先清除最后一行!
  dma_display->fillRect(0, 56, 192, 8, 0);
  dma_display->setTextSize(1);     // size 1 == 8 pixels high
  dma_display->setTextWrap(false); // Don't wrap at end of line - will do ourselves
  dma_display->setCursor(0, 56);   // Write on last line
  dma_display->setTextColor(dma_display->color444(255,0,0));
  dma_display->print(message);
  messageDisplayMillis = millis();
  logMessageActive = true;
}

void Display::ClearStatusMessage() {
   dma_display->fillRect(0, 56, 192, 8, 0); 
   logMessageActive = false;
}

void Display::DisplaySensorData() {
  if (sensorDead) {
    dma_display->fillRect(SENSOR_DATA_X, SENSOR_DATA_Y, SENSOR_DATA_WIDTH, SENSOR_DATA_HEIGHT, 0);
    dma_display->setTextSize(1);     // size 1 == 8 pixels high
    dma_display->setTextWrap(false); // Don't wrap at end of line - will do ourselves
    dma_display->setTextColor(SENSOR_ERROR_DATA_COLOR);
    
    dma_display->setCursor(SENSOR_DATA_X, SENSOR_DATA_Y);   
    dma_display->print("No sensor data!");
  }

  if (newSensorData) {
    dma_display->fillRect(SENSOR_DATA_X, SENSOR_DATA_Y, SENSOR_DATA_WIDTH, SENSOR_DATA_HEIGHT, 0);
    dma_display->setTextSize(1);     // size 1 == 8 pixels high
    dma_display->setTextWrap(false); // Don't wrap at end of line - will do ourselves
    dma_display->setTextColor(SENSOR_DATA_COLOR);

//    dma_display->setFont(&FreeSerifBold12pt7b);
    dma_display->setCursor(SENSOR_DATA_X, SENSOR_DATA_Y);   
    dma_display->printf("%4.1f C           %3d%%", sensorTemp, sensorHumi);
    
    // 手动绘制度数符号
    dma_display->fillRect(SENSOR_DATA_X + 25, SENSOR_DATA_Y, 2, 2, SENSOR_DATA_COLOR);    
    newSensorData = false;
  }
}

void Display::DisplayLightData(float luxValue) {
  dma_display->fillRect(LIGHT_DATA_X, LIGHT_DATA_Y, LIGHT_DATA_WIDTH, LIGHT_DATA_HEIGHT, 0);
  
  dma_display->setTextSize(1);     // size 1 == 8 pixels high
  dma_display->setTextWrap(false); // Don't wrap at end of line - will do ourselves
  dma_display->setTextColor(SENSOR_DATA_COLOR);
  //    dma_display->setFont(&FreeSerifBold12pt7b);

  dma_display->setCursor(LIGHT_DATA_X, LIGHT_DATA_Y);   
  dma_display->printf("%4.1f lx", luxValue);
    

}

/**************************************************************/
int Display::GifPlay( const char* gifPath )
{ // 0=infinite

  if( ! gif.open( gifPath, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw ) ) {
    log_n("Could not open gif %s", gifPath );
  }

  Serial.print("Playing: "); Serial.println(gifPath);

  int frameDelay = 0; // 最后一帧的存储延迟
  int then = 0; // 存储总延迟

  while (gif.playFrame(true, &frameDelay)) {

    then += frameDelay;
    if( then > maxGifDuration ) { // 避免陷入无限GIF
      //log_w（“破坏GIF循环，超过最大持续时间”）；
      break;
    }
  }

  gif.close();

  return then;
}

void Display::Sdbegin(){
// **************************** 通过SPI设置SD卡访问 ****************************
    // SPI.begin(SCLK,DO,DI,CS);
    // if(!SD.begin(CS)){
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

// **************************** 设置草图****************************
    Serial.println("Starting AnimatedGIFs Sketch");

    // SD卡在启用DMA显示的情况下停止工作>。。。

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



  // 这一点很重要——设置正确的endianness。
  gif.begin(LITTLE_ENDIAN_PIXELS);
}

void Display::AutoGif(int delayMs){
// 使用基于范围的for循环在矢量上迭代
    for(auto & elem : GifFiles)
    {   dma_display->clearScreen();
        GifPlay( elem.c_str() );
        gif.reset();
        delay(delayMs);
    }

}

void Display::Clear(){
  dma_display->clearScreen();  
}