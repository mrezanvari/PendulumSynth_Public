#include "defines.h"

void setup()
{
  //display.setRotation(2);

  set_arm_power_button_callback(&reset);  // Immediate shut off from on/off button press

  for (int i = 0; i <= sizeof(POT_Pins) / sizeof(POT_Pins[0]); i++)
  {
    pinMode(POT_Pins[i], INPUT);
  }

  pinMode(usrKey, INPUT_PULLUP);
  pinMode(builtIn_LED, OUTPUT);
  digitalWrite(builtIn_LED, HIGH);

  pinMode(CSN_PIN, OUTPUT);

  for (int i = 0; i <= sizeof(CV_Pins) / sizeof(CV_Pins[0]); i++)
  {
    pinMode(CV_Pins[i], OUTPUT);
    digitalWrite(CV_Pins[i], LOW);
  }

  digitalWrite(builtIn_LED, LOW);
  delay(100);
  digitalWrite(builtIn_LED, HIGH);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  ui.setDataSet(dataBuffer);
  ui.splashScreen(firmwareVersion);

  delay(1000);

  Serial.begin(115200);

  bool DAC_status = DAC.begin();


  NRF.begin();
  NRF.setRetries(5, 15);
  NRF.setPALevel(RF24_PA_HIGH);
  NRF.setDataRate(RF24_2MBPS);
  network.begin(90, this_node);  //(channel, node address)

  bool key = digitalRead(usrKey);
  if (!key)
  {
    while (!key)
    {
      key = digitalRead(usrKey);
      digitalWrite(builtIn_LED, LOW);
    }
    while (!Serial)
    {
      digitalWrite(builtIn_LED, LOW);
      delay(100);
      digitalWrite(builtIn_LED, HIGH);
      delay(100);
    }

    NRF.printDetails();
    digitalWrite(builtIn_LED, HIGH);

    Serial.println("\n\nRESERVED!");

    while (true) digitalWrite(builtIn_LED, LOW);
  }

  if (!DAC_status) // if dac is not connected or not working ask if user wants to continue
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("DAC not found!\n\nDo you wish to \ncontinue?"));
    display.display();
    while (digitalRead(usrKey));
    while (!digitalRead(usrKey)) digitalWrite(builtIn_LED, LOW);
    digitalWrite(builtIn_LED, HIGH);
    ui.splashScreen(firmwareVersion);
    ui.resetTime();
    delay(100);
  }


  resetAllNodes();
  delay(3000); // just a simple delay to make sure init is done nad for he splash screen :)

  // just to know when its past the init phase and begins to read data
  digitalWrite(builtIn_LED, LOW);
  delay(100);
  digitalWrite(builtIn_LED, HIGH);
  delay(100);
  digitalWrite(builtIn_LED, LOW);
  delay(100);
  digitalWrite(builtIn_LED, HIGH);
  firstRun = millis();
}

void loop()
{

//  mem_delay = millis();

  readNetwork(); // check the incoming data from the node(s) and fill the data array.

  pass2Output(); // pass all of the recieved values to the corresponding output DAC + MIDI

  ui_checkUsrInput(); // checks all the pots and buttons

  ui.update(); // update the ui

  checkSerial();


//  Serial.print("delay:");
//  Serial.println(millis() - mem_delay);

  //  for(int i = 0; i < 3; i++)
  //   Serial.print('\t' + String(dataBuffer[i]));
  //
  //  Serial.println();

}
