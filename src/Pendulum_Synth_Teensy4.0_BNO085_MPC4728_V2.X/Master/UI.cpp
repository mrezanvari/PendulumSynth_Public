/*
        UI class for the display

            @Mrezanvari 2021
*/

#include "Arduino.h"
#include "UI.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CircularBuffer.h>

#define menuThreshold 1000 // timer after user input

#define SCREEN_WIDTH 128

int modeIndex = 0; // index of the used mode; (gravity:linear acceleration, gravity+: abs,peak2trough, peak2peak, trough2trough, diff)
int bufferIndex = 0; // index of the current buffer for the graph
long memTimeStamp = millis(); // saves the current cpu time

#define maxMode 6

int graphIndex = 0; // index of the current graph shown

CircularBuffer<int, SCREEN_WIDTH> graphBuffer;  // must be at the end...

Adafruit_SSD1306 *_display;
float* _dataBuffer; // pointer to the receieved data form the node

UI::UI(Adafruit_SSD1306* displayOBJ)
{
  _display = displayOBJ;
  _width = _display->width();
  _height = _display->height();
  //  modeIndex = 5;
}

void UI::setDataSet(float* dataBuffer)
{
  _dataBuffer = dataBuffer;
}

void UI::setGraphRanges(float rangeX, float rangeY, float rangeZ)
{
  _rangeX = rangeX;
  _rangeY = rangeY;
  _rangeZ = rangeZ;
}

void UI::splashScreen(float _firmwareVersion)
{
  const unsigned char pendulumSynth [] = {
    0x3f, 0x80, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
    0x3f, 0xe0, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
    0x30, 0x60, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
    0x30, 0x30, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x30, 0x30, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x30, 0x30, 0xf8, 0x13, 0x80, 0xf3, 0x18, 0x31, 0x8c, 0x18, 0x9c, 0x70, 0x00, 0x01, 0x00, 0x00,
    0x30, 0x71, 0xfc, 0x3f, 0xc1, 0xff, 0x18, 0x31, 0x8c, 0x18, 0xfe, 0xf8, 0x00, 0x00, 0x80, 0x00,
    0x30, 0xe3, 0x8e, 0x3c, 0x63, 0x8f, 0x18, 0x31, 0x8c, 0x18, 0xe7, 0x8c, 0x00, 0x00, 0x80, 0x00,
    0x3f, 0xc7, 0x07, 0x38, 0x63, 0x07, 0x18, 0x31, 0x8c, 0x18, 0xc3, 0x0c, 0x00, 0x00, 0x80, 0x00,
    0x30, 0x06, 0x03, 0x30, 0x66, 0x03, 0x18, 0x31, 0x8c, 0x18, 0xc3, 0x0c, 0x00, 0x00, 0x40, 0x00,
    0x30, 0x07, 0xff, 0x30, 0x66, 0x03, 0x18, 0x31, 0x8c, 0x18, 0xc3, 0x0c, 0x00, 0x00, 0x40, 0x00,
    0x30, 0x06, 0x00, 0x30, 0x66, 0x03, 0x18, 0x31, 0x8c, 0x18, 0xc3, 0x0c, 0x00, 0x00, 0x40, 0x00,
    0x30, 0x07, 0x02, 0x30, 0x63, 0x07, 0x18, 0x31, 0x8c, 0x38, 0xc3, 0x0c, 0x00, 0x00, 0x20, 0x00,
    0x30, 0x03, 0x8e, 0x30, 0x63, 0x8f, 0x1c, 0x71, 0x8c, 0x38, 0xc3, 0x0c, 0x00, 0x00, 0x20, 0x00,
    0x30, 0x01, 0xfc, 0x30, 0x61, 0xff, 0x0f, 0xf1, 0x87, 0xf8, 0xc3, 0x0c, 0x00, 0x00, 0x20, 0x00,
    0x30, 0x00, 0x70, 0x10, 0x20, 0xf3, 0x07, 0xb1, 0x83, 0xd8, 0xdb, 0x0c, 0x00, 0x00, 0x30, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x06, 0x00, 0x00, 0x02, 0x18, 0x00, 0x08, 0x00, 0x18, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x18, 0x00, 0x0e, 0x00, 0x1f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x18, 0x00, 0x03, 0x80, 0x30, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x41, 0xb7, 0x9f, 0x9b, 0x80, 0x10, 0xff, 0x60, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x61, 0xbf, 0xcf, 0x9f, 0xc0, 0x1c, 0x00, 0x40, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x33, 0x38, 0xc2, 0x1c, 0x60, 0x07, 0x80, 0x40, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x33, 0x30, 0xc2, 0x18, 0x60, 0x00, 0xfe, 0x40, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1a, 0x30, 0xc2, 0x18, 0x60, 0x00, 0x00, 0x40, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1e, 0x30, 0xc2, 0x18, 0x60, 0x00, 0x00, 0x60, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x0e, 0x30, 0xc2, 0x18, 0x60, 0x00, 0x00, 0x30, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x86, 0x0c, 0x30, 0xc2, 0x18, 0x60, 0x00, 0x00, 0x1f, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x0c, 0x30, 0xc2, 0x18, 0x60, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };


  _display->clearDisplay();
  _display->drawBitmap(0, 0,  pendulumSynth, 128, 32, WHITE);

  int16_t  x1, y1;
  uint16_t w, h;

  String str = "v" + String(_firmwareVersion);
  _display->setTextColor(SSD1306_WHITE);
  _display->getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
  _display->setCursor(0, _height - h);
  _display->println(str);
  _display->display();
  resetBuffer();
}

void UI::test()
{
  _display->clearDisplay();
  _display->setTextSize(1);      // Normal 1:1 pixel scale
  _display->setTextColor(SSD1306_WHITE); // Draw white text
  _display->setCursor(0, 0);     // Start at top-left corner
  _display->cp437(true);         // Use full 256 char 'Code Page 437' font
  for (int16_t i = 0; i < 256; i++) {
    if (i == '\n') _display->write(' ');
    else          _display->write(i);
  }

  _display->display();
}

void UI::update()
{
  if (millis() % 15 != 0) return; // update every 15ms
  _update();
}

void UI::nextMode()
{
  modeIndex >= maxMode ? modeIndex = 0 : modeIndex = modeIndex + 1;
  modeInfoPage();
}

void UI::modeInfoPage()
{
  memTimeStamp = millis();

  int16_t  x1, y1;
  uint16_t w, h;

  switch (modeIndex)
  {
    case 0:
      {
        String title = "Gravity";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;

    case 1:
      {
        String title = "Gravity+";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;

    case 2:
      {
        String title = "P2T";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;

    case 3:
      {
        String title = "P2P";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;

    case 4:
      {
        String title = "T2T";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;

      case 5:
      {
        String title = "DIV3";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;

    case 6:
      {
        String title = "DIFF";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;

    case 7:
      {
        String title = "JUGGLE!";
        _display->clearDisplay();
        _display->setTextSize(2); // Draw 2X-scale text
        _display->setTextColor(SSD1306_WHITE);
        _display->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        _display->setCursor((_width / 2) - (w / 2), (_height / 2) - (h / 2));
        _display->println(title);
        _display->display();
      }
      break;
  }
}

void UI::resetTime() // resets the timer so the 3 second threshhold is set
{
  memTimeStamp = millis();
}

void UI::_update()
{
  _display->setTextSize(1);
  _display->setTextColor(WHITE, BLACK);
  _display->setCursor(0, 0);
  _display->clearDisplay();

  float currentData = _dataBuffer[graphIndex];
  int range =
    (graphIndex == 0) ? _rangeX :
    (graphIndex == 1) ? _rangeY :
    (graphIndex == 2) ? _rangeZ : 0; // each sensor has a specific range; set them...

  int dataPoint = map(currentData, -range, range, 0, 1023); // map data to a good range
  if (getMode() == 1)
    dataPoint = map(abs(currentData), -range, range, 0, 1023); // map data to a good range

  graphBuffer.push(dataPoint);

  bufferIndex = bufferIndex >= _width ? 0 : bufferIndex + 1;

  _display->fillRect(0, 0, _width, 8, SSD1306_BLACK);
  _display->setCursor(0, 0);
  _display->print(labelDictionary[graphIndex]);
  _display->print(_dataBuffer[graphIndex]);

  int16_t x1, y1; // this is to get the rectangle for the label bar
  uint16_t w, h;

  String currentPage = getMode() == 0 ? "Gravity" // set the title of the page on the top right
                       : getMode() == 1 ? "Gravity+"
                       : getMode() == 2 ? "P2T"
                       : getMode() == 3 ? "P2P"
                       : getMode() == 4 ? "T2T"
                       : getMode() == 5 ? "DIV3"
                       : getMode() == 6 ? "DIFF"
                       : getMode() == 7 ? "JUGGLE!" : "";

  _display->getTextBounds(currentPage, 0, 0, &x1, &y1, &w, &h);
  _display->setCursor(_width - w, 0);
  _display->print(currentPage); // print the current output on top right

  int xCoor = 0; // scrolling x coordinates

  for (int i = 0; i < _width; i++)
  {
    int currentVal = graphBuffer[i];
    drawGraph(xCoor, currentVal);
    xCoor++;
  }

  if (millis() - memTimeStamp > menuThreshold) // just to have a delay befor showing data after any user input
    _display->display();
}

void UI::drawGraph(int xCoor, int dataPoint) // used to draw a pixel for each datapoint
{
  int graphHeight = _height - 10; // to account for the label bar
  int mappedYCoor = map(dataPoint, 0, 1023, 0, graphHeight);
  int yCoor = _height - mappedYCoor;
  _display->drawPixel(xCoor, yCoor, SSD1306_WHITE);
}

//void UI::_viewCurrentPot()
//{
//  String str = getMode() == 0 ? "Range" : "Constant" ;
//  String strVal = getGraph() == 0 ? "X:" + String(_rangeX) : getGraph() == 1 ? "Y:" + String(_rangeY) : getGraph() == 2 ? "Z:" + String(_rangeZ) : "";
//
//  _display->setCursor(0, _height - 8);
//  _display->println(str + strVal);
//  _display->display();
//}

void UI::nextGraph() // changes the graph
{
  graphIndex >= 2 ? graphIndex = 0 : graphIndex = graphIndex + 1;
  //memset(graphBuffer, 0, 128 * sizeof(*graphBuffer)); // reset buffer array
  resetBuffer();
}

void UI::setMode(int mode)
{
  if (mode > maxMode && mode != 7) return;
  
  modeIndex = mode;
  modeInfoPage();
}

void UI::resetBuffer()
{
  for (int i = 0; i < SCREEN_WIDTH; i++)
    graphBuffer.push(0);
}

int UI::getMode()
{
  return modeIndex;
}

int UI::getGraph()
{
  return graphIndex;
}
