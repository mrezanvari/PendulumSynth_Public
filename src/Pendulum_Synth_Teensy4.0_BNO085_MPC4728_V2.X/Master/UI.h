/*
        UI class for the display

            @Mrezanvari 2021
*/

#ifndef UI_h
#define UI_h

#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>

class UI
{
  public:
    UI(Adafruit_SSD1306* displayOBJ);
    void setDataSet(float* dataBuffer);
    void test();
    void update();
    void nextMode();
    void resetTime();
    int getMode();
    int getGraph();
    void nextGraph();
    void splashScreen(float firmwareVersion);
    void setGraphRanges(float rangeX, float rangeY, float rangeZ);
    void setMode(int mode);
    void resetBuffer();
  private:
    Adafruit_SSD1306* _display;
    uint8_t _width;
    uint8_t _height;
    int modeIndex;
    long memTimeStamp;
    void _update();
    void drawGraph(int xCoor, int dataPoint);
    int bufferIndex;
    float* _dataBuffer;
    int graphIndex;
    float _rangeX;
    float _rangeY;
    float _rangeZ;
    void UI::modeInfoPage();

    const String labelDictionary[3] = // used on the label bar on top of the screen
    {
      "Gx: ",
      "Gy: ",
      "Gz: ",
    };
};

#endif
