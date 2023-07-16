#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Display.h"
#include "algorithm_v2.h"

#define MAX30102_ADDRESS          0x57 //7-bit I2C Address
//Note that MAX30102 has the same I2C address and Part ID

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------

/*
#define MAX30102_MEASUREMENT_SECONDS  5
#define MAX30102_FIFO_SAMPLES_PER_SECOND	 100 // 50, 100, 200, 400, 800, 100, 1600, 3200 sample rating
#define MAX30102_FIFO_ALMOST_FULL_SAMPLES  32

#define MAX30102_BUFFER_LENGTH	((MAX30102_MEASUREMENT_SECONDS+1)*MAX30102_FIFO_SAMPLES_PER_SECOND)

#define MAX30102_USE_INTERNAL_TEMPERATURE
*/
#define MAX30102_MEASUREMENT_SECONDS  5
#define MAX30102_FIFO_SAMPLES_PER_SECOND 100
#define MAX30102_FIFO_ALMOST_FULL_SAMPLES  32

#define MAX30102_BUFFER_LENGTH (MAX30102_MEASUREMENT_SECONDS * MAX30102_FIFO_SAMPLES_PER_SECOND)

//--------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------


class MAX30102 {
  private:
  TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
  uint8_t _i2caddr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  
  uint8_t revisionID; 

  void readRevisionID();

  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

  typedef enum {
    MAX30102_ERROR = 0,
    MAX30102_OK = 1
  } MAX30102_STATUS;

 public: 

  MAX30102(void);

  boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30102_ADDRESS);

  // Configuration
  void softReset();
  void shutDown(); 
  void wakeUp(); 

  void setLEDMode(uint8_t mode);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
  void setPulseWidth(uint8_t pulseWidth);

  void setPulseAmplitudeRed(uint8_t value);
  void setPulseAmplitudeIR(uint8_t value);
  void setPulseAmplitudeGreen(uint8_t value);
  void setPulseAmplitudeProximity(uint8_t value);

  void setProximityThreshold(uint8_t threshMSB);
  
  // Data Collection

  //Interrupts (page 13, 14)
  uint8_t getINT1(void); //Returns the main interrupt group
  uint8_t getINT2(void); //Returns the temp ready interrupt
  void enableAFULL(void); //Enable/disable individual interrupts
  void disableAFULL(void);
  void enableDATARDY(void);
  void disableDATARDY(void);
  void enableALCOVF(void);
  void disableALCOVF(void);
  void enablePROXINT(void);
  void disablePROXINT(void);
  void enableDIETEMPRDY(void);
  void disableDIETEMPRDY(void);

  //FIFO Configuration (page 18)
  void setFIFOAverage(uint8_t samples);
  void enableFIFORollover();
  void disableFIFORollover();
  void setFIFOAlmostFull(uint8_t samples);
  
  //FIFO Reading
  void clearFIFO(void); //Sets the read/write pointers to zero

  //Proximity Mode Interrupt Threshold
  void setPROXINTTHRESH(uint8_t val);

  // Die Temperature
  float readTemperature();
  float readTemperatureF();

  // Detecting ID/Revision
  uint8_t getRevisionID();
  uint8_t readPartID();  

  // Low-level I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

  //------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------
  
  void MAX30102_Init();
  MAX30102_STATUS Max30102_readFifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);
  void Max30102_InterruptCallback(void);
  int32_t Max30102_GetHeartRate(void);
  int32_t Max30102_GetSpO2Value(void);
  void Max30102_Task(Display *display);
  void Max30102_Collect_A_Full(void);
  void Max30102_Collect_Ppg_Rdy(void);
  bool CheckIfCalculationsDone(void);

  //------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------

};