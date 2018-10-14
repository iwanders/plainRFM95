/*
  The MIT License (MIT)
  Copyright (c) 2018 Ivor Wanders
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include "plainRFM95.h"

plainRFM95::plainRFM95(uint8_t cs_pin) : cs_pin_(cs_pin)
{
}

void plainRFM95::reset(uint8_t pin)
{  // function to send the RFM95 a hardware reset.
  // p 75 of datasheet;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(150);  // pull high for >100 uSec
  pinMode(pin, INPUT);     // release
  delay(10);               //  wait 10 milliseconds before SPI is possible.
}

void inline plainRFM95::chipSelect(bool enable)
{
  digitalWrite(cs_pin_, (enable) ? LOW : HIGH);
}

uint8_t plainRFM95::readRawRegister(uint8_t reg)
{
  return readRegister(reg);
}

void plainRFM95::writeRegister(uint8_t reg, uint8_t data)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
  chipSelect(true);                                                  // assert chip select
  SPI.transfer(RFM95_WRITE_REG_MASK | (reg & RFM95_READ_REG_MASK));
  SPI.transfer(data);
  chipSelect(false);     // deassert chip select
  SPI.endTransaction();  // release the SPI bus
}

uint8_t plainRFM95::readRegister(uint8_t reg)
{
  uint8_t foo;
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
  chipSelect(true);                                                  // assert chip select
  SPI.transfer((reg & RFM95_READ_REG_MASK));
  foo = SPI.transfer(0);
  chipSelect(false);     // deassert chip select
  SPI.endTransaction();  // release the SPI bus
  return foo;
}

void plainRFM95::writeMultiple(uint8_t reg, void* data, uint8_t len)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
  chipSelect(true);                                                  // assert chip select
  SPI.transfer(RFM95_WRITE_REG_MASK | (reg & RFM95_READ_REG_MASK));
  uint8_t* r = reinterpret_cast<uint8_t*>(data);
  for (uint8_t i = 0; i < len; i++)
  {
    SPI.transfer(r[len - i - 1]);
  }
  chipSelect(false);     // deassert chip select
  SPI.endTransaction();  // release the SPI bus
}

void plainRFM95::readMultiple(uint8_t reg, void* data, uint8_t len)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
  chipSelect(true);                                                  // assert chip select

  SPI.transfer((reg & RFM95_READ_REG_MASK));
  uint8_t* r = reinterpret_cast<uint8_t*>(data);
  for (uint8_t i = 0; i < len; i++)
  {
    r[len - i - 1] = SPI.transfer(0);
  }
  chipSelect(false);     // deassert chip select
  SPI.endTransaction();  // release the SPI bus
}

uint32_t plainRFM95::readRegister32(uint8_t reg)
{
  uint32_t f = 0;
  readMultiple(reg, &f, 4);
  return f;
}
uint32_t plainRFM95::readRegister24(uint8_t reg)
{
  uint32_t f = 0;
  readMultiple(reg, &f, 3);
  return f;
}
uint16_t plainRFM95::readRegister16(uint8_t reg)
{
  uint16_t f = 0;
  readMultiple(reg, &f, 2);
  return f;
}

void plainRFM95::writeFIFO(const void* buffer, uint8_t len)
{
  const uint8_t* r = reinterpret_cast<const uint8_t*>(buffer);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
  chipSelect(true);                                                  // assert chip select
  SPI.transfer(RFM95_WRITE_REG_MASK | (RFM95_FIFO & RFM95_READ_REG_MASK));
  for (uint8_t i = 0; i < len; i++)
  {
    // Serial.print("Writing to FIFO: "); Serial.println(r[i]);
    SPI.transfer(r[i]);
  }
  chipSelect(false);     // deassert chip select
  SPI.endTransaction();  // release the SPI bus
}

void plainRFM95::readFIFO(void* buffer, uint8_t len)
{
  uint8_t* r = reinterpret_cast<uint8_t*>(buffer);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
  chipSelect(true);                                                  // assert chip select

  SPI.transfer((RFM95_FIFO % RFM95_READ_REG_MASK));
  for (uint8_t i = 0; i < len; i++)
  {
    r[i] = SPI.transfer(0);
  }
  chipSelect(false);     // deassert chip select
  SPI.endTransaction();  // release the SPI bus
}


void plainRFM95::setFrequency(uint32_t freq)
{
  // 61 should be 61.03515625 for precision.
  uint32_t frf = freq / 61;
  writeMultiple(RFM95_FRF_MSB, &frf, 3);
}

uint8_t plainRFM95::readRxData(void* buffer)
{
  uint8_t pos = readRegister(RFM95_LORA_FIFO_RX_CURRENT_ADDR);
  seekFIFO(pos);
  uint8_t len = readRxLength();
  readFIFO(buffer, len);
  return len;
}

bool plainRFM95::readPacketCRCOn()
{
  return readRegister(RFM95_LORA_HOP_CHANNEL) & (1<<6);
}

int8_t plainRFM95::readPacketSNR()
{
  uint8_t raw_snr = readRegister(RFM95_LORA_PACKET_SNR);
  const int8_t& snr = reinterpret_cast<const int8_t&>(raw_snr);
  return snr / 4;
}

int8_t plainRFM95::readPacketRSSI()
{
  uint8_t raw_rssi = readRegister(RFM95_LORA_PACKET_RSSI);
  return -137 + raw_rssi;  
}

void plainRFM95::printPacketStats()
{
  Serial.print("   CRC: "); Serial.println(readPacketCRCOn());
  Serial.print("   SNR: "); Serial.println(readPacketSNR());
  Serial.print("   RSSI: "); Serial.println(readPacketRSSI());
  Serial.print("   len: "); Serial.println(readRxLength());
}

void plainRFM95::writeTxFIFO(const void* buffer, uint8_t length)
{
  seekFIFO(fifo_tx_);
  writeFIFO(buffer, length);
}

uint8_t plainRFM95::readRxLength()
{
  return readRegister(RFM95_LORA_FIFO_RX_BYTES);
}

bool plainRFM95::begin()
{
  bool success = true;

  // Switch to LORA
  writeRegister(RFM95_OPMODE, 0);
  writeRegister(RFM95_OPMODE, RFM95_OPMODE_LORA);
  success &= (readRegister(RFM95_OPMODE) == RFM95_OPMODE_LORA);

  standby();

  // Set up the Tx and Rx fifo addresses in the FIFO
  const uint8_t Rx_FIFO_addr = 0;
  const uint8_t Tx_FIFO_addr = 0;
  const uint8_t payload_max_length = 128;

  setRxFIFO(Rx_FIFO_addr);
  setTxFIFO(Tx_FIFO_addr);
  success &= (readRegister(RFM95_LORA_FIFO_TX_BASE_ADDR) == Tx_FIFO_addr);
  success &= (readRegister(RFM95_LORA_FIFO_RX_BASE_ADDR) == Rx_FIFO_addr);
  writeRegister(RFM95_LORA_PAYLOAD_MAX_LENGTH, payload_max_length);

  // set moderate power.
  setFrequency((uint32_t) 434*1000*1000); // set the frequency.
  setPower(10);
  setModemConfig2(true);
  return success;
}

void plainRFM95::seekFIFO(uint8_t position)
{
  writeRegister(RFM95_LORA_FIFO_ADDR_PTR, position);
}

void plainRFM95::setTxFIFO(uint8_t position)
{
  fifo_tx_ = position;
  writeRegister(RFM95_LORA_FIFO_TX_BASE_ADDR, fifo_tx_);
}

void plainRFM95::setRxFIFO(uint8_t position)
{
  fifo_rx_ = position;
  writeRegister(RFM95_LORA_FIFO_RX_BASE_ADDR, fifo_rx_);
}

void plainRFM95::setMode(uint8_t mode)
{
  writeRegister(RFM95_OPMODE, RFM95_OPMODE_LORA | (mode & 0b111));
}

void plainRFM95::setModemConfig2(bool crc_on, uint8_t spreading)
{
  writeRegister(RFM95_LORA_MODEM_CONFIG2, spreading | (crc_on ? RFM95_LORA_MODEM_CONFIG2_CRC_ON : 0));
}


void plainRFM95::preparePayload(const void* buffer, uint8_t length)
{
  setMode(RFM95_MODE_STANDBY);
  clearIRQ();
  writeTxFIFO(buffer, length);
  writeRegister(RFM95_LORA_PAYLOAD_LENGTH, length);
}

void plainRFM95::transmit()
{
  seekFIFO(fifo_tx_);
  writeRegister(RFM95_DIO_MAPPING1, RFM95_LORA_DIO0_TX_DONE << RFM95_DIO_MAPPING_DIO0_SHIFT);

  // Setup interrupt for Tx Done, mask all others.
  writeRegister(RFM95_LORA_IRQ_MASK, ~(RFM95_LORA_IRQ_TX_DONE));
  clearIRQ();

  setMode(RFM95_MODE_TX);
}

void plainRFM95::receive()
{
  standby();

  // rewind the fifo pointer to the rx position. It will still move if we are in receive mode for a while...
  // That's why we use the RX_CURRENT_ADDR to retrieve the packet.
  seekFIFO(fifo_rx_);

  // Change the DIO Mapping
  writeRegister(RFM95_DIO_MAPPING1, RFM95_LORA_DIO0_RX_DONE << RFM95_DIO_MAPPING_DIO0_SHIFT);

  // Set interrupts for Rx events only.
  writeRegister(RFM95_LORA_IRQ_MASK, ~(RFM95_LORA_IRQ_RX_TIMEOUT | RFM95_LORA_IRQ_RX_DONE | RFM95_LORA_IRQ_CRC_ERROR));
  clearIRQ();
  // Switch modes.
  setMode(RFM95_MODE_RX_CONTINUOUS);
}

void plainRFM95::standby()
{
  setMode(RFM95_MODE_STANDBY);
  writeRegister(RFM95_DIO_MAPPING1, RFM95_LORA_DIO0_NONE << RFM95_DIO_MAPPING_DIO0_SHIFT);
  writeRegister(RFM95_LORA_IRQ_MASK, 0);
  clearIRQ();  
}

void plainRFM95::sleep()
{
  standby();
  setMode(RFM95_MODE_SLEEP);
}

void plainRFM95::clearIRQ()
{
  writeRegister(RFM95_LORA_IRQ_FLAGS, 0xFF);
  writeRegister(RFM95_LORA_IRQ_FLAGS, 0xFF);
}


plainRFM95::IRQState plainRFM95::block(uint32_t ms)
{
  plainRFM95::IRQState res = NONE;
  elapsedMillis blocktime = 0;
  while (res == NONE)
  {
    if ((ms != 0) && (blocktime > ms))
    {
      return TIMEOUT;
    }
    delay(10);
    res = poll();
  }
  return res;
}


plainRFM95::IRQState plainRFM95::poll()
{
  // Multiple could be active if irq's weren't cleared properly...
  const uint8_t irqs = readRegister(RFM95_LORA_IRQ_FLAGS);

  if (irqs & RFM95_LORA_IRQ_TX_DONE)
  {
    return TX_DONE;
  }

  if (irqs & RFM95_LORA_IRQ_RX_DONE)
  {
    if (irqs & RFM95_LORA_IRQ_CRC_ERROR)
    {
      return RX_DONE_INVALID_PACKET;
    }
    else
    {
      return RX_DONE_VALID_PACKET;
    }
  }

  if (irqs & RFM95_LORA_IRQ_CAD_DONE)
  {
    if (irqs & RFM95_LORA_IRQ_CAD_DETECTED)
    {
      return CAD_DONE_SIGNAL;
    }
    else
    {
      return CAD_DONE_NO_SIGNAL;
    }
  }
  return NONE;
}


void plainRFM95::setPower(uint8_t Pout)
{
  // probably have PA_BOOST on the default module...
  // Pout = Pmax - (15 - OutputPower);
  // Pmax is 20 with PA_boosts
  // Enable boost:
  writeRegister(RFM95_MODE_RX_CAD, RFM95_PA_DAC_BOOST);
  const uint8_t Pmax = 20;
  uint8_t bounded = max(Pmax - (15 - 0), min(Pout, Pmax));
  // Pout = Pmax - (15 - OutputPower)
  // Pout = Pmax - 15 + OutputPower
  // Pout - Pmax + 15 = OutputPower
  // Not sure if RFM95_PA_CONFIG_MAX_POWER has influence without RFO? Best set it to max.
  writeRegister(RFM95_PA_CONFIG, RFM95_PA_CONFIG_PA_SELECT | RFM95_PA_CONFIG_MAX_POWER | (Pout - Pmax + 15));
}

