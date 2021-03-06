#include "defines.h"

void setup()
{

  pinMode(usrKey, INPUT_PULLUP);
  pinMode(builtIn_LED, OUTPUT);
  digitalWrite(builtIn_LED, HIGH);

  pinMode(rstPIN, OUTPUT);
  digitalWrite(rstPIN, HIGH);

  pinMode(IMU_rstPIN, OUTPUT);
  digitalWrite(IMU_rstPIN, HIGH);

  digitalWrite(builtIn_LED, LOW);
  delay(100);
  digitalWrite(builtIn_LED, HIGH);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  delay(100); // delay befor starting the serial will give enough time for the serial monitor to init...
  Serial.begin(115200);

  delay(200);
  if (!IMU.begin_I2C())
  {
    Serial.println("Failed to find IMU");
    while (1)
    {
      digitalWrite(builtIn_LED, LOW);
      delay(500);
      digitalWrite(builtIn_LED, HIGH);
      delay(500);
    }
  }

  if (!IMU_init(IMU))
  {
    Serial.println("Failed to initialize IMU");
    while (1)
    {
      digitalWrite(builtIn_LED, LOW);
      delay(500);
      digitalWrite(builtIn_LED, HIGH);
      delay(500);
    }
  }

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
    digitalWrite(builtIn_LED, HIGH);
    Serial.println("RESERVED...\n");

    for (int n = 0; n < IMU.prodIds.numEntries; n++) {
      Serial.print("Part ");
      Serial.print(IMU.prodIds.entry[n].swPartNumber);
      Serial.print(": Version :");
      Serial.print(IMU.prodIds.entry[n].swVersionMajor);
      Serial.print(".");
      Serial.print(IMU.prodIds.entry[n].swVersionMinor);
      Serial.print(".");
      Serial.print(IMU.prodIds.entry[n].swVersionPatch);
      Serial.print(" Build ");
      Serial.println(IMU.prodIds.entry[n].swBuildNumber);
    }
    while (true);
  }

  NRF.begin();
  //  NRF.setRetries(5, 15); // not so important so...
  NRF.setPALevel(RF24_PA_HIGH); // highest range
  NRF.setDataRate(RF24_2MBPS); // highest speed
  network.begin(90, thisNodeAddr);

  //  delay(1000); // to make sure everything is now initialized and ready to go
  // just to know when its past the init phase and begins to send data
  digitalWrite(builtIn_LED, LOW);
  delay(100);
  digitalWrite(builtIn_LED, HIGH);
  delay(100);
  digitalWrite(builtIn_LED, LOW);
  delay(100);
  digitalWrite(builtIn_LED, HIGH);
}

void loop()
{
  //  long mem_delay = millis();
  //  if (IMU.wasReset())
  //  {
  //    IMU_resetCounter ++;  // if imu reset itself init again; if happened ever...
  //    IMU_init(IMU);
  //  }
  //
  IMU.getSensorEvent(&IMU_dataBuffer); // update sensor data

  if (Serial) // only used for debugging; will be ignored when not connected to save time
  {
    digitalWrite(builtIn_LED, LOW);
    //    Serial.print("delay:");
    //    Serial.print(millis() - mem_delay);
    //    Serial.print(" rst:");
    //    Serial.print(IMU_resetCounter);
    Serial.print("GravityX:");
    Serial.print(IMU_dataBuffer.un.gravity.x);
    Serial.print(" GravityY:");
    Serial.print(IMU_dataBuffer.un.gravity.y);
    Serial.print(" GravityZ:");
    Serial.println(IMU_dataBuffer.un.gravity.z);
    //    mem_delay = millis();
    digitalWrite(builtIn_LED, HIGH);
  }

  else // if not debugging then send to nrf network
  {
    network.update();

    RF24NetworkHeader header1(masterAddr, 'G'); // generate a new header with the master address and the type of data; G for gravity

    switch (IMU_dataBuffer.sensorId)
    {

      case SH2_LINEAR_ACCELERATION:
        {
          //          if(isGravity) break;
          transmitBuffer [0] = IMU_dataBuffer.un.linearAcceleration.x; // fill the buffer with the 3 axes
          transmitBuffer [1] = IMU_dataBuffer.un.linearAcceleration.y;
          transmitBuffer [2] = IMU_dataBuffer.un.linearAcceleration.z;
        }
        break;

      case SH2_GRAVITY:
        {
          //          if(!isGravity) break;
          transmitBuffer [0] = IMU_dataBuffer.un.gravity.x;
          transmitBuffer [1] = IMU_dataBuffer.un.gravity.y;
          transmitBuffer [2] = IMU_dataBuffer.un.gravity.z;
        }

      case SH2_GYROSCOPE_CALIBRATED:
        {
          transmitBuffer [0] = IMU_dataBuffer.un.gyroscope.x;
          transmitBuffer [1] = IMU_dataBuffer.un.gyroscope.y;
          transmitBuffer [2] = IMU_dataBuffer.un.gyroscope.z;
        }
        break;

      case SH2_ACCELEROMETER:
        {
          transmitBuffer [0] = IMU_dataBuffer.un.accelerometer.x;
          transmitBuffer [1] = IMU_dataBuffer.un.accelerometer.y;
          transmitBuffer [2] = IMU_dataBuffer.un.accelerometer.z;
        }
        break;
    }

    network.write(header1, &transmitBuffer, sizeof(transmitBuffer)); // trasmit the buffer
  }

  network.update();

  if (network.available())
  {
    digitalWrite(builtIn_LED, LOW);
    RF24NetworkHeader header;
    network.read(header, &data_networkBuffer, sizeof(data_networkBuffer)); // Read the incoming data
    if (header.from_node != masterAddr) return; // if the message is not form the master then do not proccess it

    switch (char((int)header.type))
    {
      case 'R':
        {
          if (data_networkBuffer[0] == 1)
            digitalWrite(rstPIN, LOW);
          while (1);
        }
        break;

      case 'M':
        {
          digitalWrite(IMU_rstPIN, LOW);
          delay(1);
          digitalWrite(IMU_rstPIN, HIGH);
          delay(250);


          switch (data_networkBuffer[0])
          {
            case 0:
              {
                if (!IMU_init(IMU))
                {
                  while (1)
                  {
                    digitalWrite(builtIn_LED, LOW);
                    delay(500);
                    digitalWrite(builtIn_LED, HIGH);
                    delay(500);
                  }
                }
              }
              break;

            case 1:
              {
                if (!IMU.enableReport(SH2_GRAVITY))
                {
                  while (1)
                  {
                    digitalWrite(builtIn_LED, LOW);
                    delay(500);
                    digitalWrite(builtIn_LED, HIGH);
                    delay(500);
                  }
                }
              }
              break;

            case 2:
              {
                if (!IMU.enableReport(SH2_ACCELEROMETER))
                {
                  while (1)
                  {
                    digitalWrite(builtIn_LED, LOW);
                    delay(500);
                    digitalWrite(builtIn_LED, HIGH);
                    delay(500);
                  }
                }
              }
              break;
          }
        }
        break;
    }
    digitalWrite(builtIn_LED, HIGH);
  }
  //  Serial.print("delay:");
  //  Serial.println(millis() - mem_delay);
}
