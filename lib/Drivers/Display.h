#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

// library includes
#include <ESP32-VirtualMatrixPanel-I2S-DMA.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

class Display {
public:
    Display(void);
    ~Display(void);

    uint16_t colorWheel(uint8_t pos); 
    void DisplayInit(bool swapBlueGreen, uint8_t displayBright);
    void DisplyTest(int delayMs);
    void LogStatusMessage(const char *message);
    void LogStatusMessage(String message);
    void ClearStatusMessage();
    void DisplaySensorData();
    void DisplayLightData(float luxValue);
    int GifPlay( const char* gifPath );
    void Sdbegin();
    void AutoGif(int delayMs);
    void Clear();


private:
};

#endif //DISPLAY_H