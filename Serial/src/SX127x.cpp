/* based on code by Amon Schumann / DL9AS   https://github.com/DL9AS
*/ 

#include <Arduino.h>
#include <SPI.h>

#include "SX127x.h"
#include "SX127x_register.h"
#include "pins.h"


// Module functions
static void SX127x_write_reg(uint8_t address, uint8_t reg_value)
{
  digitalWrite(SX127x_SS, LOW);  // Select SX127x SPI device
  SPI.transfer(address | 0x80);             
  SPI.transfer(reg_value);
  digitalWrite(SX127x_SS, HIGH);  // Clear SX127x SPI device selection
}

static uint8_t SX127x_read_reg(uint8_t address)
{
  digitalWrite(SX127x_SS, LOW);  // Select SX127x SPI device
  SPI.transfer(address & 0x7F);
  uint8_t reg_value = SPI.transfer(0);
  digitalWrite(SX127x_SS, HIGH);  // Clear SX127x SPI device selection

  return reg_value;
}

void SX127x_read_buffer(uint8_t address, uint8_t* buffer,  uint8_t size)
{
   for (int a = 0; a<size; a++){
     buffer[a]= SX127x_read_reg(address);
   }
}

void SX127x_write_buffer(uint8_t address, uint8_t* buffer,  uint8_t size)
{
   for (int a = 0; a<size; a++){
     SX127x_write_reg(address, buffer[a] );
   }
}


// Exported functions
void SX127x_begin(void)
{
  SPI.begin(SX127x_SCK, SX127x_MISO, SX127x_MOSI, SX127x_SS); // Begin SPI communication

  pinMode(SX127x_SS, OUTPUT);
  digitalWrite(SX127x_SS, HIGH);  // Clear SX127x SPI device selection

  pinMode(SX127x_RESET, OUTPUT);
  SX127x_enable();
}

void SX127x_enable(void)
{
  digitalWrite(SX127x_RESET, HIGH); // Enable SX127x
}

void SX127x_disable(void)
{
  digitalWrite(SX127x_RESET, LOW); // Disable SX127x
}

void SX127x_sleep(void)
{
  // Set SX127x operating mode to sleep mode
  // 7: 0->FSK/OOK Mode
  // 6-5: 00->FSK
  // 4: 0 (reserved)
  // 3: 1->Low Frequency Mode
  // 2-0: 000->Sleep Mode
  SX127x_write_reg(REG_OP_MODE, 0x08);
}

void SX127x_reset(void)
{
  digitalWrite(SX127x_RESET, LOW); // Disable SX127x
  delay(5);
  digitalWrite(SX127x_RESET, HIGH); // Enable SX127x
  delay(5);
}

void SX127x_set_frequency(uint64_t *freq)
{
  uint64_t freq_tmp = *freq;
  // Frequency value is calculate by:
  // Freg = (Frf * 2^19) / Fxo
  // Resolution is 61.035 Hz if Fxo = 32 MHz
  freq_tmp = (((freq_tmp + SX127x_FREQUENCY_CORRECTION) << 19) / SX127x_CRYSTAL_FREQ);

  SX127x_write_reg(REG_FR_MSB, freq_tmp >> 16);  // Write MSB of RF carrier freq
  SX127x_write_reg(REG_FR_MID, freq_tmp >> 8);  // Write MID of RF carrier freq
  SX127x_write_reg(REG_FR_LSB, freq_tmp);  // Write LSB of RF carrier freq
}

int8_t SX127x_getRSSI(void){
  uint8_t value;
  value = SX127x_read_reg(REG_RSSI_VALUE);
  return -value/2;
}

uint8_t SX127x_getRSSI_raw(void){
  uint8_t value;
  value = SX127x_read_reg(REG_RSSI_VALUE);
  return value;
}

