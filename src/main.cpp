#include <Arduino.h>
#include <SPI.h>
#include <inttypes.h>
#include <driver/ledc.h>

#define SPI_CMD_READ 0x4000 /*!< flag indicating read attempt when using SPI interface */
#define SPI_CMD_NOP 0x0000
#define SPI_REG_AGC 0x3ffd    /*!< agc register when using SPI */
#define SPI_REG_MAG 0x3ffe    /*!< magnitude register when using SPI */
#define SPI_REG_DATA 0x3fff   /*!< data register when using SPI */
#define SPI_REG_CLRERR 0x0001 /*!< clear error register when using SPI */

const int vspiSCK = 18;
const int vspiMISO = 19;
const int vspiMOSI = 23;
const int vspiSS = 5;
const uint32_t maxSPIClk = 10000000;

const double invalidAngle = -1.0;
double lastAngle;

void readAS5048All(double *angle,
                   uint8_t *agc,
                   uint16_t *magnitude,
                   bool *alarmHi,
                   bool *alarmLo);
double readAS5048Angle();
uint16_t spiTransfer(uint16_t data);
uint8_t spiCalcEvenParity(uint16_t value);

void setup()
{
  delay(1500);

  Serial.begin(115200);
  Serial.println();
  Serial.printf("as5048a-test %s %s\r\n", __DATE__, __TIME__);

  SPI.begin(vspiSCK, vspiMISO, vspiMOSI, vspiSS);
  pinMode(vspiSS, OUTPUT);
  digitalWrite(vspiSS, HIGH);

  double angle = invalidAngle;
  uint8_t agc = 0;
  uint16_t magnitude = 0;
  bool alarmHi = false;
  bool alarmLo = false;

  readAS5048All(&angle, &agc, &magnitude, &alarmHi, &alarmLo);
  lastAngle = angle;
  Serial.printf("%6.2lf agc: %3d mag: %5d alarmHi: %s alarmLo: %s\r\n",
                angle, agc, magnitude, alarmHi ? "Y" : "N", alarmLo ? "Y" : "N");
}

void loop()
{

  double angle = readAS5048Angle();

  if (angle != lastAngle)
  {
    lastAngle = angle;

    Serial.printf("%6.2lf\r\n", angle);
  }
}

void readAS5048All(double *angle,
                   uint8_t *agc,
                   uint16_t *magnitude,
                   bool *alarmHi,
                   bool *alarmLo)
{
  uint16_t data; // 16-bool data buffer for SPI communication
  uint16_t magReg;
  uint16_t agcReg;
  uint16_t angleReg;
  uint16_t value;

  /* Send READ AGC command. Received data is thrown away: this data comes from the precedent
command (unknown)*/
  data = SPI_CMD_READ | SPI_REG_AGC;
  data |= spiCalcEvenParity(data) << 15;
  spiTransfer(data);

  /* Send READ MAG command. Received data is the AGC value: this data comes from the
precedent command (unknown)*/
  data = SPI_CMD_READ | SPI_REG_MAG;
  data |= spiCalcEvenParity(data) << 15;
  agcReg = spiTransfer(data);

  /* Send READ ANGLE command. Received data is the MAG value, from the precedent command */
  data = SPI_CMD_READ | SPI_REG_DATA;
  data |= spiCalcEvenParity(data) << 15;
  magReg = spiTransfer(data);

  /* Send READ ANGLE command. Received data is the ANGLE value, from the precedent command */
  data = SPI_CMD_READ | SPI_REG_DATA;
  data |= spiCalcEvenParity(data) << 15;
  angleReg = spiTransfer(data);

  if ((angleReg & 0x4000) || (agcReg & 0x4000) || (magReg & 0x4000))
  {
    /* error flag set - need to reset it */
    data = SPI_CMD_READ | SPI_REG_CLRERR;
    data |= spiCalcEvenParity(data) << 15;
    spiTransfer(data);
    Serial.println("Error!");
  }
  else
  {
    *agc = agcReg & 0xff;                       // AGC value (0..255)
    value = angleReg & (16384 - 31 - 1);        // Angle value (0.. 16384 steps)
    *angle = (double(value) * 360.0) / 16384.0; // Angle value in degree (0..359.9°)
    *magnitude = magReg & (16384 - 31 - 1);
    *alarmLo = ((agcReg >> 10) & 0x1) != 0;
    *alarmHi = ((agcReg >> 11) & 0x1) != 0;
  }
}

double readAS5048Angle()
{
  /* Send READ ANGLE command. Received data is the ANGLE value, from the precedent command */
  uint16_t data = SPI_CMD_READ | SPI_REG_DATA;
  data |= spiCalcEvenParity(data) << 15;
  int16_t angleReg = spiTransfer(data);

  if (angleReg & 0x4000)
  {
    /* error flag set - need to reset it */
    data = SPI_CMD_READ | SPI_REG_CLRERR;
    data |= spiCalcEvenParity(data) << 15;
    spiTransfer(data);
    Serial.println("Error!");
    return invalidAngle;
  }
  else
  {
    uint16_t value = angleReg & (16384 - 31 - 1); // Angle value (0.. 16384 steps)
    return (double(value) * 360.0) / 16384.0;     // Angle value in degree (0..359.9°)
  }
}

uint16_t spiTransfer(uint16_t data)
{
  uint16_t out = 0;

  digitalWrite(vspiSS, LOW);
  SPI.beginTransaction(SPISettings(maxSPIClk, SPI_MSBFIRST, SPI_MODE1));
  out = SPI.transfer16(data);
  digitalWrite(vspiSS, HIGH);
  SPI.endTransaction();

  return out;
}

/*!
*****************************************************************************
* Calculate even parity of a 16 bit unsigned integer
*
* This function is used by the SPI interface to calculate the even parity
* of the data which will be sent via SPI to the encoder.
*
* \param[in] value : 16 bool unsigned integer whose parity shall be calculated
*
* \return : Even parity
*
*****************************************************************************
*/
uint8_t spiCalcEvenParity(uint16_t value)
{
  uint8_t cnt = 0;
  uint8_t i;
  for (i = 0; i < 16; i++)
  {
    if (value & 0x1)
    {
      cnt++;
    }
    value >>= 1;
  }
  return cnt & 0x1;
}
