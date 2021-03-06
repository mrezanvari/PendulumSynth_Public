float firmwareVersion = 2.0;

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <Adafruit_BNO08x.h>

#define usrKey PA0
#define builtIn_LED PC13
#define rstPIN PC15
#define IMU_rstPIN PC14

#define CE_PIN PB0
#define CSN_PIN PA4

#define SDA_PIN PB7
#define SCL_PIN PB6

#define BNO08X_RST -1

#define BNO08x_I2CADDR_DEFAULT 0x29

Adafruit_BNO08x IMU(BNO08X_RST);
sh2_SensorValue_t IMU_dataBuffer;
long mem_delay;
int IMU_resetCounter;

bool buttonState = false;
bool saveState = false;

const uint16_t thisNodeAddr = 01;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t masterAddr = 00;

float transmitBuffer[3];
int data_networkBuffer[1]; // buffer for the data recived form the master

bool isGravity = false;

RF24 NRF(CE_PIN, CSN_PIN); // CE, CS
RF24Network network(NRF);

//void sendConfirmationToMaster(int confirmation) // does not work!
//{
//  network.update();
//  int buff[1];
//  buff[0] = confirmation;
//  RF24NetworkHeader header(masterAddr, 'C');
//  network.write(header, &buff, sizeof(buff)); // trasmit the buffer
//}

bool IMU_init(Adafruit_BNO08x _IMU)
{
  //  bool statusOut = _IMU.enableReport(SH2_ACCELEROMETER);
  //  bool statusOut = _IMU.enableReport(SH2_GYROSCOPE_CALIBRATED);
  //  bool statusOut = _IMU.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
  bool statusOut = _IMU.enableReport(SH2_LINEAR_ACCELERATION);
  //  bool statusOut = _IMU.enableReport(SH2_ROTATION_VECTOR);
  //  bool statusOut = _IMU.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR);
  //  bool statusOut = _IMU.enableReport(SH2_GAME_ROTATION_VECTOR);
  //  bool statusOut = _IMU.enableReport(SH2_STEP_COUNTER);
  //  bool statusOut = _IMU.enableReport(SH2_STABILITY_CLASSIFIER);
  //  bool statusOut = _IMU.enableReport(SH2_RAW_ACCELEROMETER);
  //  bool statusOut = _IMU.enableReport(SH2_RAW_GYROSCOPE);
  //  bool statusOut = _IMU.enableReport(SH2_RAW_MAGNETOMETER);
  //  bool statusOut = _IMU.enableReport(SH2_SHAKE_DETECTOR);
//  statusOut *= _IMU.enableReport(SH2_GRAVITY);
  return statusOut;
}
