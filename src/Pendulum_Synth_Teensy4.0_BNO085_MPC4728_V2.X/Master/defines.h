float firmwareVersion = 2.7;

#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "UI.h"
#include <Adafruit_MCP4728.h>
#include <T4_PowerButton.h>
//#include "TeensyThreads.h"

int IMU_Mode = 0;

bool debug = false;

double map_f(double x, double in_min, double in_max, double out_min, double out_max); // maps floats;
void reset();
void change_IMU_Mode(int mode, uint16_t node);
void updateSensorMode();
void resetAllNodes();
int clicks = 0;

#define usrKey 15
#define builtIn_LED 14

#define CE_PIN 9
#define CSN_PIN 10

long mem_delay = millis();
long firstRun; // used to keep track of the millis of the moment init is done; used to prevent pot info to show at start

float diff_mem[3] = {0.0}; // this will save the prevous value that was checked against the range value to see if enough movement was done to triger the gte

#define POT_ThreshVal 0.4
#define keyDownThreshVal 800

RF24 NRF(CE_PIN, CSN_PIN); // CE, CSN
RF24Network network(NRF);

bool buttonState = false; // used for the userKey
bool saveState = false;

const uint16_t this_node = 00; // address of master
const uint16_t node01 = 01; // address of the node01

const uint16_t nodes[] =
{
  node01,
};

float data_networkBuffer[3]; // buffer for the data recived form the node
float dataBuffer[3]; // buffer for all of the datas

bool keyDown = false; // indicates that the key is held down

long keyDown_memTime = 0; // saves the time of the first key down event
bool pageChanged = false; // indicates if the page has changed; this way if the sensor output is changed the graph will remain the same

// for display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_MCP4728 DAC;

UI ui(&display); // make a ui object using the display object

int MIDI_Channel = 1;
#define MIDI_CC1 0
#define MIDI_CC2 1
#define MIDI_CC3 2
#define MIDI_Trig1 36
#define MIDI_Trig2 -1
#define MIDI_Trig3 -1

bool MIDI_Trigs_MEM [] = {false, false, false}; //this is used to prevent sending the trig signal if already sent

int MIDI_CCs[] = {MIDI_CC1, MIDI_CC2, MIDI_CC3};
int MIDI_Trigs[] = {MIDI_Trig1, MIDI_Trig2, MIDI_Trig3};

int MIDI_CCs_Toggle_MEM[] = {MIDI_CC1, MIDI_CC2, MIDI_CC3}; // mem used for toggle settings
int MIDI_Trigs_Toggle_MEM[] = {MIDI_Trig1, MIDI_Trig2, MIDI_Trig3};

// Pot input pins
#define POT0 22
#define POT1 21
#define POT2 23

// CV output pins
#define CV1 1
#define CV2 2
#define CV3 3

int POT_Pins[] =
{
  POT0, POT1, POT2
};

int CV_Pins[] =
{
  CV1, CV2, CV3,
}; // defined as array pins so the init process will be easier

// used arrays so it is easier to use a for loop to check all the pots and ranges and etc...

float POTS_MEM[3]; //save the pots to check for change nad show info if needed
float ranges[3]; // range values for the axes; [X Y Z]
float ranges_MEM[3];// save the ranges to check for change nad show info if needed

float rangeMaxValue = 16.0;

void view_PotInfo()
{
  if (millis() < firstRun + 100 || pageChanged) return; // delays the view info for only the first time; this way the very first display will not be the infos...

  String str = ui.getMode() == 0 ? "Range" : "Constant" ;
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(str + "X:  ");
  display.println(ranges[0]);
  display.setCursor(0, 12);
  display.print(str + "Y:  ");
  display.println(ranges[1]);
  display.setCursor(0, 24);
  display.print(str + "Z:  ");
  display.println(ranges[2]);
  display.display();

  ui.resetTime();
}

void readNetwork()
{
  network.update();
  while ( network.available() )
  {
    digitalWrite(builtIn_LED, LOW);
    RF24NetworkHeader header;
    network.read(header, &data_networkBuffer, sizeof(data_networkBuffer)); // Read the incoming data

    // for now we have only one node so we dont need to check for header.from_node values...
    switch (char((int)header.type))
    {
      case 'G':
        {
          for (int i = 0; i < 3; i++)
          {
            dataBuffer[i] = data_networkBuffer[i];
            if (debug)
            {
              Serial.print("Data");
              Serial.print(i);
              Serial.print(":");
              Serial.print(data_networkBuffer[i]);
              Serial.print('\t');
            }
          }
          if (debug) Serial.println();
        }
        break;
    }
    digitalWrite(builtIn_LED, HIGH);
  }
}

void pass2Output()
{
  int mode = ui.getMode();
  if (mode > 5) mode = 5;
  switch (mode)
  {
    case 0: // regular gravity mode
      {
        for (int i = 0; i < 3; i++)
        {
          float inData = dataBuffer[i];
          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
          float cvVal;
          cvVal = map(inData, -range, range, 0, 4095);
          if (cvVal > 4095) cvVal = 4095;
          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
                              i == 1 ? MCP4728_CHANNEL_B :
                              MCP4728_CHANNEL_C, cvVal);


          cvVal = map(inData, -range, range, 0, 255);
          analogWrite(CV_Pins[i], cvVal);

          cvVal = constrain(map(inData, -range, range, 0, 127), 0, 127);
          usbMIDI.sendControlChange(MIDI_CCs[i], cvVal, MIDI_Channel);
        }
      }
      break;

    case 1: // gravity+ mode
      {
        for (int i = 0; i < 3; i++)
        {
          float inData = abs(dataBuffer[i]);
          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
          float cvVal;
          cvVal = map(inData, 0, range, 0, 4095);
          if (cvVal > 4095) cvVal = 4095;
          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
                              i == 1 ? MCP4728_CHANNEL_B :
                              MCP4728_CHANNEL_C, cvVal);


          cvVal = map(inData, 0, range, 0, 255);
          analogWrite(CV_Pins[i], cvVal);

          cvVal = constrain(map(inData, -range, range, 0, 127), 0, 127);
          usbMIDI.sendControlChange(MIDI_CCs[i], cvVal, MIDI_Channel);
        }
      }
      break;

    case 2: // peak2trough
      {
        for (int i = 0; i < 3; i++)
        {
          float inData = dataBuffer[i];
          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
          float cvVal;
          if (abs(inData) > range)
          {
            cvVal = 4095;
            if (MIDI_Trigs_MEM[i] == false)
            {
              usbMIDI.sendNoteOn(MIDI_Trigs[i], 127, MIDI_Channel);
              MIDI_Trigs_MEM[i] = true;
            }
          }
          else
          {
            cvVal = 0;
            if (MIDI_Trigs_MEM[i] == true)
            {
              usbMIDI.sendNoteOff(MIDI_Trigs[i], 0, MIDI_Channel);
              MIDI_Trigs_MEM[i] = false;
            }
          }

          analogWrite(CV_Pins[i], cvVal);
          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
                              i == 1 ? MCP4728_CHANNEL_B :
                              MCP4728_CHANNEL_C, cvVal);
        }
        break;
      }

    case 3: // peak2peak
      {
        for (int i = 0; i < 3; i++)
        {
          float inData = dataBuffer[i];
          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
          float cvVal;

          if (inData > range)
          {
            cvVal = 4095;
            if (MIDI_Trigs_MEM[i] == false)
            {
              usbMIDI.sendNoteOn(MIDI_Trigs[i], 127, MIDI_Channel);
              MIDI_Trigs_MEM[i] = true;
            }
          }
          else
          {
            cvVal = 0;
            if (MIDI_Trigs_MEM[i] == true)
            {
              usbMIDI.sendNoteOff(MIDI_Trigs[i], 0, MIDI_Channel);
              MIDI_Trigs_MEM[i] = false;
            }
          }

          analogWrite(CV_Pins[i], cvVal);
          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
                              i == 1 ? MCP4728_CHANNEL_B :
                              MCP4728_CHANNEL_C, cvVal);
        }
        break;
      }

    case 4: //trough2trough
      {
        for (int i = 0; i < 3; i++)
        {
          float inData = dataBuffer[i];
          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
          float cvVal;
          if (inData < range)
          {
            cvVal = 4095;
            if (MIDI_Trigs_MEM[i] == false)
            {
              usbMIDI.sendNoteOn(MIDI_Trigs[i], 127, MIDI_Channel);
              MIDI_Trigs_MEM[i] = true;
            }
          }
          else
          {
            cvVal = 0;
            if (MIDI_Trigs_MEM[i] == true)
            {
              usbMIDI.sendNoteOff(MIDI_Trigs[i], 0, MIDI_Channel);
              MIDI_Trigs_MEM[i] = false;
            }
          }

          analogWrite(CV_Pins[i], cvVal);
          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
                              i == 1 ? MCP4728_CHANNEL_B :
                              MCP4728_CHANNEL_C, cvVal);
        }
        break;
      }

       case 5: //div3
      {
        for (int i = 0; i < 3; i++)
        {
          float inData = dataBuffer[i];
          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
          float cvVal;
          if (abs(inData) > range || ( abs(inData) <= 0.6) )
          {
            cvVal = 4095;
            if (MIDI_Trigs_MEM[i] == false)
            {
              usbMIDI.sendNoteOn(MIDI_Trigs[i], 127, MIDI_Channel);
              MIDI_Trigs_MEM[i] = true;
            }
          }
          
          else
          {
            cvVal = 0;
            if (MIDI_Trigs_MEM[i] == true)
            {
              usbMIDI.sendNoteOff(MIDI_Trigs[i], 0, MIDI_Channel);
              MIDI_Trigs_MEM[i] = false;
            }
          }

          analogWrite(CV_Pins[i], cvVal);
          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
                              i == 1 ? MCP4728_CHANNEL_B :
                              MCP4728_CHANNEL_C, cvVal);
        }
        break;
      }

    case 6: //diff
      {
        for (int i = 0; i < 3; i++)
        {
          float inData = dataBuffer[i];
          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
          float cvVal;
          if (abs(abs(inData) - diff_mem[i]) >= (range)) // the dif const is there to just add some headroom
          {
            cvVal = 4095;
            diff_mem[i] = abs(inData);
            usbMIDI.sendNoteOn(MIDI_Trigs[i], 127, MIDI_Channel);
          }
          else
          {
            cvVal = 0;
            usbMIDI.sendNoteOff(MIDI_Trigs[i], 0, MIDI_Channel);
          }

          analogWrite(CV_Pins[i], cvVal);

          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
                              i == 1 ? MCP4728_CHANNEL_B :
                              MCP4728_CHANNEL_C, cvVal);
        }
        break;
      }

//    case 7: //jusggle! only activates using serial protocol
//      {
//        for (int i = 0; i < 3; i++)
//        {
//          float inData = dataBuffer[i];
//          float range = i == 0 ? ranges[0] : i == 1 ? ranges[1] : i == 2 ? ranges[2] : 0;
//          float cvVal;
//          if (abs(inData) > range)
//          {
//            cvVal = 4095;
//            if (MIDI_Trigs_MEM[i] == false)
//            {
//              usbMIDI.sendNoteOn(MIDI_Trigs[i], 127, MIDI_Channel);
//              MIDI_Trigs_MEM[i] = true;
//            }
//          }
//          else
//          {
//            cvVal = 0;
//            if (MIDI_Trigs_MEM[i] == true)
//            {
//              usbMIDI.sendNoteOff(MIDI_Trigs[i], 0, MIDI_Channel);
//              MIDI_Trigs_MEM[i] = false;
//            }
//          }
//
//          analogWrite(CV_Pins[i], cvVal);
//          DAC.setChannelValue(i == 0 ? MCP4728_CHANNEL_A :
//                              i == 1 ? MCP4728_CHANNEL_B :
//                              MCP4728_CHANNEL_C, cvVal);
//        }
//        break;
//      }
  }
}

void ui_checkUsrInput()
{
  // get pots
  for (int i = 0; i < 3; i++)
  {
    float currVal = analogRead(POT_Pins[i]);
    if (POTS_MEM[i] != currVal)
    {
      ranges[i] = constrain(
                    map_f(currVal, 0, 1024, ui.getMode() >= 3 ? -0.01 : 1, rangeMaxValue + 2) // -0.001 so the diff value can reach 0
                    , ui.getMode() >= 3 ? -0.0001 : 1, rangeMaxValue); //1024 will get rid of the flicker of the pots and that +2 is there so the rangeMaxValue is reached since the pots range is now more than actual value

      if (abs(ranges_MEM[i] - ranges[i]) > POT_ThreshVal) // if the value is changed show the info screen
      {
        view_PotInfo();
        ranges_MEM[i] = ranges[i];
        POTS_MEM[i] = currVal; // save em'
      }
    }
  }

  ui.setGraphRanges(ranges[0], ranges[1], ranges[2]);

  // check key
  if (!digitalRead(usrKey))
  {
    if (!keyDown) keyDown_memTime = millis();

    keyDown = true;
    if (millis() - keyDown_memTime > keyDownThreshVal)
    {
      ui.nextMode();
      keyDown_memTime = millis();
      pageChanged = true;


      updateSensorMode();
    }
  }

  if (digitalRead(usrKey) && keyDown) // do not proceed if the key is held down; once released, proceed...
  {
    if (!pageChanged)
      ui.nextGraph();

    keyDown = false;
    pageChanged = false;
  }
}

void checkSerial()
{
  String cmd[10] = {NULL}; // will hold all of the commands and values for processing...

  if (Serial.available())
  {
    String serialIn = Serial.readStringUntil('\n');
    serialIn.remove(serialIn.length() - 1); // remove \r

    String token; // tokenizer
    int index = 0;
    int cmdIndex = -1;

    while (serialIn.length() > 0)
    {
      index = serialIn.indexOf(" ");
      if (index == -1) index = serialIn.length(); // this is for the last part or single word commands...
      token = serialIn.substring(0, index); // get the token
      serialIn.remove(0, index + 1); // remove upto the space
      cmd[++cmdIndex] = token; // save the token
      //      Serial.println(cmd[cmdIndex]);
    }
  }

  int cmdSize = 0;

  for (int i = 0; i < sizeof(cmd) / sizeof(cmd[0]); i++)
  {
    if (cmd[i] == NULL) break;
    cmdSize++;
  }

  if (cmd[0].toLowerCase() == "reset" || cmd[0].toLowerCase() == "r") reset(); // arduino only supports switch case for int so we have to use if cases...

  if (cmd[0].toLowerCase() == "dump")
  {
    Serial.println("\n                       ..:: Settings Dump::..");
    Serial.println("---------------------------------------------------------------------------------\n");
    Serial.print("MIDI Channel : ");
    Serial.println(MIDI_Channel);
    Serial.println();

    for (int i = 0; i < sizeof(MIDI_CCs) / sizeof(MIDI_CCs[0]); i++)
    {
      Serial.print("MIDI_CC");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(MIDI_CCs[i]);
    }
    Serial.println();

    for (int i = 0; i < sizeof(MIDI_Trigs) / sizeof(MIDI_Trigs[0]); i++)
    {
      Serial.print("MIDI_Trig");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(MIDI_Trigs[i]);
    }
    Serial.println();

    Serial.print("Mode : ");
    String currentPage = ui.getMode() == 0 ? "Gravity"
                         : ui.getMode() == 1 ? "Gravity+"
                         : ui.getMode() == 2 ? "P2T"
                         : ui.getMode() == 3 ? "P2P"
                         : ui.getMode() == 4 ? "T2T"
                         : ui.getMode() == 5 ? "DIFF" : ""; // set the title of the page on the top right
    Serial.println(currentPage);

    Serial.println("\n---------------------------------------------------------------------------------");
  }

  if (cmd[0].toLowerCase() == "help" || cmd[0].toLowerCase() == "h")
  {
    Serial.println("\n                       ..::PendulumSynth Settings::..");
    Serial.println("---------------------------------------------------------------------------------\n\nCommands:\n");
    Serial.println("set midi cc MIDI MIDI_CC: sets the midi cc number to  MIDI_CC");
    Serial.println("set midi trig MIDI MIDI_NOTE: sets the midi trigger note number to  MIDI_NOTE");
    Serial.println("set midi channel CHANNEL: sets the midi channel to CHANNEL");
    Serial.println("set mode MODE: sets the mode of the device and the outputs");
    Serial.println("toggle midi cc MIDI: toggles the given midi cc");
    Serial.println("toggle midi trig MIDI: toggles the given midi note");
    Serial.println("toggle debug: toggles the serial monitor debugger (RESERVED)");
    Serial.println("reset/r: will reset the device");
    Serial.println("help/h: will show all the commands\n");
    Serial.println("---------------------------------------------------------------------------------");
  }

  // switch case cannot be used for string in arduino ide!
  else if (cmd[0].toLowerCase() == "set")
  {
    if (cmd[1].toLowerCase() == "midi")
    {

      if (cmd[2].toLowerCase() == "cc")
      {
        if (cmdSize < 5)
        {
          Serial.println("Command Error!  use \"help\"");
          return;
        }
        MIDI_CCs[cmd[3].toInt()] = cmd[4].toInt();
        Serial.print("MIDI CC ");
        Serial.print(cmd[3].toInt());
        Serial.print(" changed to: ");
        Serial.println(MIDI_CCs[cmd[3].toInt()]);
      }

      else if (cmd[2].toLowerCase() == "trig")
      {
        if (cmdSize < 5)
        {
          Serial.println("Command Error!  use \"help\"");
          return;
        }
        MIDI_Trigs[cmd[3].toInt()] = cmd[4].toInt();
        Serial.print("MIDI Trig ");
        Serial.print(cmd[3].toInt());
        Serial.print(" changed to: ");
        Serial.println(MIDI_Trigs[cmd[3].toInt()]);
      }

      else if (cmd[2].toLowerCase() == "channel")
      {
        if (cmdSize < 4)
        {
          Serial.println("Command Error!  use \"help\"");
          return;
        }
        MIDI_Channel = cmd[3].toInt();

        Serial.print("MIDI Channel changed to: ");
        Serial.println(MIDI_Channel);
      }
    }

    else if (cmd[1].toLowerCase() == "mode")
    {
      if (cmdSize < 3)
      {
        Serial.println("Command Error!  use \"help\"");
        return;
      }

      ui.setMode(cmd[2].toInt());

      updateSensorMode();
    }
  }

  else if (cmd[0].toLowerCase() == "toggle")
  {
    if (cmd[1].toLowerCase() == "midi")
    {
      if (cmdSize < 4)
      {
        Serial.println("Command Error!  use \"help\"");
        return;
      }

      if (cmd[2].toLowerCase() == "cc")
      {
        if ( MIDI_CCs[cmd[3].toInt()] == -1)
        {
          MIDI_CCs[cmd[3].toInt()] =  MIDI_CCs_Toggle_MEM[cmd[3].toInt()] ;
        }
        else
        {
          MIDI_CCs_Toggle_MEM[cmd[3].toInt()] = MIDI_CCs[cmd[3].toInt()];
          MIDI_CCs[cmd[3].toInt()] = -1;
        }
        Serial.print("MIDI CC ");
        Serial.print(cmd[3].toInt());
        Serial.println(" toggled...");
      }

      else if (cmd[2].toLowerCase() == "trig")
      {
        if ( MIDI_Trigs[cmd[3].toInt()] == -1)
        {
          MIDI_Trigs[cmd[3].toInt()] =  MIDI_Trigs_Toggle_MEM[cmd[3].toInt()] ;
        }
        else
        {
          MIDI_Trigs_Toggle_MEM[cmd[3].toInt()] = MIDI_Trigs[cmd[3].toInt()];
          MIDI_Trigs[cmd[3].toInt()] = -1;
        }
        Serial.print("MIDI Trig ");
        Serial.print(cmd[3].toInt());
        Serial.println(" toggled...");
      }
    }

    else if (cmd[1].toLowerCase() == "debug") debug = !debug;
  }
}

void updateSensorMode()
{
  rangeMaxValue = 16;
  if (ui.getMode() == 7)
  {
    change_IMU_Mode(2, node01);
    rangeMaxValue = 60;
  }
  else if ((ui.getMode() <= 1 || ui.getMode() > 5) && IMU_Mode != 0) change_IMU_Mode(0, node01);
  else if (ui.getMode() > 1 && IMU_Mode != 1) change_IMU_Mode(1, node01);
}

void reset() // soft reset the host
{
  for (int i = 0; i < 3; i++) // clear all notes and cc's
  {
    usbMIDI.sendNoteOff(MIDI_Trigs[i], 0, MIDI_Channel);
    usbMIDI.sendControlChange(MIDI_CCs[i], 0, MIDI_Channel);
    delay(1);
  }
  SCB_AIRCR = 0x05FA0004; // soft reset
}

void resetAllNodes() // send reset signall to all the nodes in the network
{
  int transmitBuffer[1];
  network.update();
  transmitBuffer[0] = 1;
  for (int i = 0; i < sizeof(nodes) / sizeof(nodes[0]); i++)
  {
    RF24NetworkHeader header1(nodes[i], 'R');
    network.write(header1, &transmitBuffer, sizeof(transmitBuffer)); // trasmit the buffer
  }
}

//int getConfirmationFormNode(uint16_t node) // does not work!
//{
//  long timoutMem = millis();
//  while (!network.available())
//  {
//    NRF.flush_rx();
//    network.update();
//  }
//
//  Serial.println("Got something!");
//  //    while (millis() - timoutMem < 200)
//  //    {
//  //      network.update();
//  while (char((int)header.type)
//  int buff[1];
//  NRF.flush_rx();
//  RF24NetworkHeader header;
//  network.peek(header, &buff, sizeof(buff)); // Read the incoming data
//  Serial.println(char((int)header.type));
//  Serial.println(buff[0]);
//  //      if (char((int)header.type) == 'C')
//  //        return buff[0];
//
//  //    else network.update();
//  //    }
//  while (1)
//  {
//    digitalWrite(builtIn_LED, LOW);
//    delay(500);
//    digitalWrite(builtIn_LED, HIGH);
//    delay(500);
//  }
//  return 0;
//}

void change_IMU_Mode(int mode, uint16_t node) // send mode change signal to the node
{
  int transmitBuffer[1];
  network.update();
  transmitBuffer[0] = mode;
  RF24NetworkHeader header1(node, 'M');
  network.write(header1, &transmitBuffer, sizeof(transmitBuffer)); // trasmit the buffer
  IMU_Mode = mode;
}

double map_f(double x, double in_min, double in_max, double out_min, double out_max) // maps floats
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
