#include "MAX30102.h"
#include "Display.h"

// Status Registers
static const uint8_t MAX30102_INTSTAT1 = 0x00;
static const uint8_t MAX30102_INTSTAT2 = 0x01;
static const uint8_t MAX30102_INTENABLE1 = 0x02;
static const uint8_t MAX30102_INTENABLE2 = 0x03;

// FIFO Registers
static const uint8_t MAX30102_FIFOWRITEPTR = 0x04;
static const uint8_t MAX30102_FIFOOVERFLOW = 0x05;
static const uint8_t MAX30102_FIFOREADPTR = 0x06;
static const uint8_t MAX30102_FIFODATA = 0x07;

// Configuration Registers
static const uint8_t MAX30102_FIFOCONFIG = 0x08;
static const uint8_t MAX30102_MODECONFIG = 0x09;
static const uint8_t MAX30102_PARTICLECONFIG = 0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30102_LED1_PULSEAMP = 0x0C;    // LED1 and LED2 adresses swapped in documentation?
static const uint8_t MAX30102_LED2_PULSEAMP =	0x0D;
static const uint8_t MAX30102_LED3_PULSEAMP =	0x0E;
static const uint8_t MAX30102_LED_PROX_AMP = 0x10;
static const uint8_t MAX30102_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30102_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30102_DIETEMPINT = 0x1F;
static const uint8_t MAX30102_DIETEMPFRAC = 0x20;
static const uint8_t MAX30102_DIETEMPCONFIG = 0x21;

// Proximity Function Registers
static const uint8_t MAX30102_PROXINTTHRESH = 0x30;

// Part ID Registers
static const uint8_t MAX30102_REVISIONID = 0xFE;
static const uint8_t MAX30102_PARTID = 0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30102_INT_A_FULL_MASK =	(byte)~0b10000000;
static const uint8_t MAX30102_INT_A_FULL_ENABLE = 0x80;
static const uint8_t MAX30102_INT_A_FULL_DISABLE = 0x00;

static const uint8_t MAX30102_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 0x20;
static const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30102_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = (byte)~0b00000010;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_SAMPLEAVG_MASK = (byte)~0b11100000;
static const uint8_t MAX30102_SAMPLEAVG_1 = 0x00;
static const uint8_t MAX30102_SAMPLEAVG_2 = 0x20;
static const uint8_t MAX30102_SAMPLEAVG_4 = 0x40;
static const uint8_t MAX30102_SAMPLEAVG_8 = 0x60;
static const uint8_t MAX30102_SAMPLEAVG_16 = 0x80;
static const uint8_t MAX30102_SAMPLEAVG_32 = 0xA0;

static const uint8_t MAX30102_ROLLOVER_MASK = 0xEF;
static const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30102_A_FULL_MASK = 0xF0;

static const uint8_t MAX30102_SHUTDOWN_MASK = 0x7F;
static const uint8_t MAX30102_SHUTDOWN = 0x80;
static const uint8_t MAX30102_WAKEUP = 0x00;

static const uint8_t MAX30102_RESET_MASK = 0xBF;
static const uint8_t MAX30102_RESET = 0x40;

static const uint8_t MAX30102_MODE_MASK = 0xF8;
static const uint8_t MAX30102_MODE_REDONLY = 0x02;
static const uint8_t MAX30102_MODE_REDIRONLY = 0x03;
static const uint8_t MAX30102_MODE_MULTILED = 0x07;

static const uint8_t MAX30102_ADCRANGE_MASK = 0x9F;
static const uint8_t MAX30102_ADCRANGE_2048 = 0x00;
static const uint8_t MAX30102_ADCRANGE_4096 = 0x20;
static const uint8_t MAX30102_ADCRANGE_8192 = 0x40;
static const uint8_t MAX30102_ADCRANGE_16384 = 0x60;

static const uint8_t MAX30102_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30102_SAMPLERATE_50 = 0x00;
static const uint8_t MAX30102_SAMPLERATE_100 = 0x04;
static const uint8_t MAX30102_SAMPLERATE_200 = 0x08;
static const uint8_t MAX30102_SAMPLERATE_400 = 0x0C;
static const uint8_t MAX30102_SAMPLERATE_800 = 0x10;
static const uint8_t MAX30102_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30102_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30102_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30102_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30102_PULSEWIDTH_69 = 0x00;
static const uint8_t MAX30102_PULSEWIDTH_118 = 0x01;
static const uint8_t MAX30102_PULSEWIDTH_215 = 0x02;
static const uint8_t MAX30102_PULSEWIDTH_411 = 0x03;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;


//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------

static const uint8_t CURRENT_0_0_MA = 0x00;
static const uint8_t CURRENT_0_2_MA = 0x01;
static const uint8_t CURRENT_6_2_MA = 0x1F;
static const uint8_t CURRENT_7_2_MA = 0x24;
static const uint8_t CURRENT_12_6_MA = 0x3F;
static const uint8_t CURRENT_25_4_MA = 0x7F;
static const uint8_t CURRENT_50_1_MA = 0xFF;

#define MAX30102_IR_VALUE_FINGER_ON_SENSOR  2000 //bylo 1600
#define MAX30102_IR_VALUE_FINGER_OUT_SENSOR  100000

#define INT_A_FULL_BIT  7
#define INT_PPG_RDY_BIT  6
#define	INT_ALC_OVF_BIT  5

volatile uint8_t Status;

uint32_t IrBuffer[MAX30102_BUFFER_LENGTH]; // Ir LED sensor data
uint32_t RedBuffer[MAX30102_BUFFER_LENGTH]; //Red LED sensor data
uint32_t BufferHead;
float ratio,correl;  //SPO2 value
int32_t Sp02Value;
int8_t Sp02IsValid;  //indicator to show if the SPO2 calculation is valid
int32_t HeartRate; //heart rate value
int8_t  IsHrValid;  //indicator to show if the heart rate calculation is valid
uint32_t CollectedSamples;
uint8_t IsFingerOnScreen;
int32_t previous_HR;
int32_t current_HR;

typedef enum {
  MAX30102_STATE_BEGIN,
  MAX30102_STATE_CALIBRATE,
  MAX30102_STATE_CALCULATE_HR,
  MAX30102_STATE_COLLECT_NEXT_PORTION
} MAX30102_STATE;


MAX30102_STATE StateMachine;
MAX30102_STATE previousState;

//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------

MAX30102::MAX30102() {
  // Constructor
}

boolean MAX30102::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr) {

  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  // Step 1: Initial Communication and Verification
  // Check that a MAX30102 is connected
  if (readPartID() != MAX_30105_EXPECTEDPARTID) {
    // Error -- Part ID read from MAX30102 does not match expected part ID.
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
    return false;
  }

  // Populate revision ID
  readRevisionID();
  
  return true;
}

//Begin Interrupt configuration
uint8_t MAX30102::getINT1(void) {
  return (readRegister8(_i2caddr, MAX30102_INTSTAT1));
}
uint8_t MAX30102::getINT2(void) {
  return (readRegister8(_i2caddr, MAX30102_INTSTAT2));
}

void MAX30102::enableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}
void MAX30102::disableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void MAX30102::enableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void MAX30102::disableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void MAX30102::enableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void MAX30102::disableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void MAX30102::enablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}
void MAX30102::disablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void MAX30102::enableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30102::disableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

//End Interrupt configuration

void MAX30102::softReset(void) {
  bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister8(_i2caddr, MAX30102_MODECONFIG);
    if ((response & MAX30102_RESET) == 0) break; //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
}

void MAX30102::shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void MAX30102::wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void MAX30102::setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void MAX30102::setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

void MAX30102::setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void MAX30102::setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30102_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30102::setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED2_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED1_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeGreen(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED3_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeProximity(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED_PROX_AMP, amplitude);
}

void MAX30102::setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  writeRegister8(_i2caddr, MAX30102_PROXINTTHRESH, threshMSB);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX30102::setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX30102::clearFIFO(void) {
  writeRegister8(_i2caddr, MAX30102_FIFOWRITEPTR, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOOVERFLOW, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void MAX30102::enableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX30102::disableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

// Die Temperature
// Returns temp in C
float MAX30102::readTemperature() {
	
  //DIE_TEMP_RDY interrupt must be enabled
  //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19
  
  // Step 1: Config die temperature register to take 1 temperature sample
  writeRegister8(_i2caddr, MAX30102_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    //uint8_t response = readRegister8(_i2caddr, MAX30102_DIETEMPCONFIG); //Original way
    //if ((response & 0x01) == 0) break; //We're done!
    
	//Check to see if DIE_TEMP_RDY interrupt is set
	uint8_t response = readRegister8(_i2caddr, MAX30102_INTSTAT2);
    if ((response & MAX30102_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
  //TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readRegister8(_i2caddr, MAX30102_DIETEMPINT);
  uint8_t tempFrac = readRegister8(_i2caddr, MAX30102_DIETEMPFRAC); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float MAX30102::readTemperatureF() {
  float temp = readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}

// Set the PROX_INT_THRESHold
void MAX30102::setPROXINTTHRESH(uint8_t val) {
  writeRegister8(_i2caddr, MAX30102_PROXINTTHRESH, val);
}


//
// Device ID and Revision
//
uint8_t MAX30102::readPartID() {
  return readRegister8(_i2caddr, MAX30102_PARTID);
}

void MAX30102::readRevisionID() {
  revisionID = readRegister8(_i2caddr, MAX30102_REVISIONID);
}

uint8_t MAX30102::getRevisionID() {
  return revisionID;
}


//Given a register, read it, mask it, and then set the thing
void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(_i2caddr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(_i2caddr, reg, originalContents | thing);
}

//
// Low-level I2C Communication
//
uint8_t MAX30102::readRegister8(uint8_t address, uint8_t reg) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->endTransmission(false);

  _i2cPort->requestFrom((uint8_t)address, (uint8_t)1); // Request 1 byte
  if (_i2cPort->available())
  {
    return(_i2cPort->read());
  }

  return (0); //Fail

}

void MAX30102::writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->write(value);
  _i2cPort->endTransmission();
}

//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------

//*pun_red_led   - pointer that stores the red LED reading data
//*pun_ir_led    - pointer that stores the IR LED reading data
MAX30102::MAX30102_STATUS MAX30102::Max30102_readFifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
  uint32_t un_temp;
  uint8_t uch_temp;
  *pun_ir_led=0;
  *pun_red_led=0;
  getINT1();
  getINT2();
  Wire.beginTransmission(MAX30102_ADDRESS);
  Wire.write(MAX30102_FIFODATA);
  Wire.endTransmission();
  Wire.requestFrom(MAX30102_ADDRESS, 6);
  un_temp=Wire.read();
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=Wire.read();
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=Wire.read();
  *pun_ir_led+=un_temp;
  un_temp=Wire.read();
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=Wire.read();
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=Wire.read();
  *pun_red_led+=un_temp;
  Wire.endTransmission();
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  return MAX30102_OK;
}

void MAX30102::Max30102_InterruptCallback()
{
  Status = getINT1();
  
  // FIFO Almost Full Interrupt handler
  if(Status & (1<<INT_A_FULL_BIT))
  {
    Max30102_Collect_A_Full();
  }

  // New FIFO Data Ready Interrupt handler
  if(Status & (1<<INT_PPG_RDY_BIT))
  {
    Max30102_Collect_Ppg_Rdy();
  }

  // Ambient Light Cancellation Interrupt handler
  if(Status & (1<<INT_ALC_OVF_BIT))
  {
    Serial.println("\n Ambient light is affecting the measurements");
  }
}


int32_t MAX30102::Max30102_GetHeartRate(void)
{
  return HeartRate;
}

int32_t MAX30102::Max30102_GetSpO2Value(void)
{
  return Sp02Value;
}


void MAX30102::Max30102_Collect_A_Full()
{
  //Serial.println("kilka probek");
  for(uint8_t i = 0; i < MAX30102_FIFO_ALMOST_FULL_SAMPLES; i++)
  {
    while(MAX30102_OK != Max30102_readFifo((RedBuffer+BufferHead), (IrBuffer+BufferHead)));
    if(IsFingerOnScreen)
    {
      if(IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR) IsFingerOnScreen = 0;
    }
    else
    {
      if(IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR) IsFingerOnScreen = 1;
    }
    //Serial.print(IrBuffer[BufferHead]);
    //Serial.print(" ");
    //Serial.println(RedBuffer[BufferHead]);
    BufferHead = (BufferHead + 1) % MAX30102_BUFFER_LENGTH;
    CollectedSamples++;
  }
}

void MAX30102::Max30102_Collect_Ppg_Rdy()
{
  //Serial.println("jedna probka");
  while(MAX30102_OK != Max30102_readFifo((RedBuffer+BufferHead), (IrBuffer+BufferHead)));
  if(IsFingerOnScreen)
  {
    if(IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR) IsFingerOnScreen = 0;
  }
  else
  {
    if(IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR) IsFingerOnScreen = 1;
  }
  //Serial.print(IrBuffer[BufferHead]);
  //Serial.print(" ");
  //Serial.println(RedBuffer[BufferHead]);
  BufferHead = (BufferHead + 1) % MAX30102_BUFFER_LENGTH;
  CollectedSamples++;
}

void MAX30102::Max30102_Task(Display *display)
{
  switch(StateMachine)
  {
    case MAX30102_STATE_BEGIN:
      if (previousState != MAX30102_STATE_BEGIN)
      {
        clearFIFO();
        display->clearScreen();
        display->centerMsg("Place finger on sensor");
      }
      previousState = StateMachine;
      if(IsFingerOnScreen)
      {
        CollectedSamples = 0;
        BufferHead = 0;
        setPulseAmplitudeRed(CURRENT_12_6_MA); // 0x24
        setPulseAmplitudeIR(CURRENT_12_6_MA);
        delay(500);
        clearFIFO();
        //Serial.println("STOP");
        StateMachine = MAX30102_STATE_CALIBRATE;
      }
      break;

    case MAX30102_STATE_CALIBRATE:
      if (previousState != MAX30102_STATE_CALIBRATE)
      {
        display->clearScreen();
        display->centerMsg("Calculating...");
      }
      previousState = StateMachine;
      if(IsFingerOnScreen)
      {
        if(CollectedSamples >= MAX30102_BUFFER_LENGTH) // > (MAX30102_BUFFER_LENGTH - MAX30102_FIFO_SAMPLES_PER_SECOND)
        {
          StateMachine = MAX30102_STATE_CALCULATE_HR;
        }
      }
      else
      {
        setPulseAmplitudeRed(CURRENT_0_0_MA);
        setPulseAmplitudeIR(CURRENT_0_2_MA);
        delay(500);
        StateMachine = MAX30102_STATE_BEGIN;
      }
      break;

    case MAX30102_STATE_CALCULATE_HR:
      if (previousState != MAX30102_STATE_CALCULATE_HR && previousState != MAX30102_STATE_COLLECT_NEXT_PORTION)
      {
        display->clearScreen();
        display->valueUpdatesInit("Heart rate: --- bpm", "SpO2: --- %");
      }
      previousState = StateMachine;
      if(IsFingerOnScreen)
      {
        heart_rate_and_oxygen_saturation(IrBuffer, MAX30102_BUFFER_LENGTH, RedBuffer, &Sp02Value, &Sp02IsValid, &HeartRate, &IsHrValid, &ratio, &correl);
        if (IsHrValid == 1){
          display->clearScreen();
          display->valueUpdates("Heart rate: " + String(HeartRate) + " bpm", "SpO2: " + String(Sp02Value) + " %");
        }

        CollectedSamples = 0;
        BufferHead = 0;
        clearFIFO();
        StateMachine = MAX30102_STATE_COLLECT_NEXT_PORTION;
      }
      else
      {
        setPulseAmplitudeRed(CURRENT_0_0_MA);
        setPulseAmplitudeIR(CURRENT_0_2_MA);
        delay(500);
        StateMachine = MAX30102_STATE_BEGIN;
      }
      break;

    case MAX30102_STATE_COLLECT_NEXT_PORTION:
      previousState = StateMachine;
      if(IsFingerOnScreen)
      {
        if(CollectedSamples >= MAX30102_BUFFER_LENGTH) 
        {
          StateMachine = MAX30102_STATE_CALCULATE_HR;
        }
      }
      else
      {
        setPulseAmplitudeRed(CURRENT_0_0_MA);
        setPulseAmplitudeIR(CURRENT_0_2_MA);
        delay(500);
        StateMachine = MAX30102_STATE_BEGIN;
      }
      break;
    default:
      break;
  }
}


void MAX30102::MAX30102_Init()
{
  softReset();        // Reset all registers to start state: 0x00
  getINT1();          // Clearing interrupts from Interrupt Status 1 register
  clearFIFO();        // Resets all: fifo_wr_ptr, fifo_rd_ptr, ovf_counter to 0
  disableFIFORollover();  // disables rollover if fifo over flows
  setFIFOAlmostFull(0x00); // Set number of samples to trigger the almost full interrupt to 17 (0x00 to 32)
  setLEDMode(MAX30102_MODE_REDIRONLY); // set sensor to SPO2 mode (red ind ir led enabled)
  setADCRange(MAX30102_ADCRANGE_4096); // 15.63 pA per LSB
  setSampleRate(MAX30102_SAMPLERATE_400); // 800 samples per second spo2 rate
  setFIFOAverage(MAX30102_SAMPLEAVG_4); // fifo averaging (samplerate: 800, sampleavg: 8 --> we get 100 samples/second)
  //pulse width 411 us - 18 bits adc resolution.
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  setPulseWidth(MAX30102_PULSEWIDTH_411); 
  setPulseAmplitudeRed(CURRENT_0_0_MA); // Initially red led is off - before finger detection we safe energy 
  setPulseAmplitudeIR(CURRENT_0_2_MA); // initially 0.2mA ir current (CURRENT_0_2_MA = 0x01 -> current = 0.2 * Value mA)
  enableAFULL(); // sets the fifo almost full interrupt
  enableDATARDY(); // sets the new fifo data ready interrupt
  enableALCOVF(); // sets ambient light cancellation overflow interrupt - we can e.g. denay measur because it will be incorrect
  StateMachine = MAX30102_STATE_BEGIN;
}

bool MAX30102::CheckIfCalculationsDone(void)
{
    if (StateMachine == MAX30102_STATE_COLLECT_NEXT_PORTION)
    {
      return true;
    }
    return false;
}

//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------