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


#ifndef PLAIN_RFM95_H
#define PLAIN_RFM95_H

#include <Arduino.h>
#include <SPI.h>
#include "plainRFM95_const.h"

#define RFM95_PLAIN_DEBUG
#ifdef RFM95_PLAIN_DEBUG
#define debug_rfm(a) Serial.print(a);
#define debug_rfmln(a) Serial.println(a);
#else
#define debug_rfm(a)
#define debug_rfmln(a)
#endif

#define RFM95_SPI_SETTING SPISettings(80000000, MSBFIRST, SPI_MODE0)

class plainRFM95
{
public:
  enum IRQState : uint8_t
  {
    NONE,                    //!< nothing yet, still listening or sending, or idling.

    RX_DONE_INVALID_PACKET,  //!< Rx Done happened, CRC is incorrect.
    RX_DONE_VALID_PACKET,    //!< Rx Done, CRC was correct, packet to read.

    CAD_DONE_NO_SIGNAL, //!< The CAD is finished, but no signal was detected.
    CAD_DONE_SIGNAL,    //!< The CAD is finished, and a signal was detected

    TX_DONE,            //!< Transmission was finished.

    TIMEOUT,            //!< Timeout has happened, can only happen if blocking with optional argument of Rx single.
  };
  enum Activity : uint8_t 
  {
    IDLE,
    RX,
    TX,
    CAD
  };
protected:
  uint8_t cs_pin_;  //!< chip select pin.
  uint8_t fifo_tx_;  //!< Fifo Tx base address.
  uint8_t fifo_rx_;  //!< Fifo Rx base address.
  Activity activity_ = IDLE;

  void writeRegister(uint8_t reg, uint8_t data);
  void writeMultiple(uint8_t reg, void* data, uint8_t len);

  uint8_t readRegister(uint8_t reg);
  uint16_t readRegister16(uint8_t reg);
  uint32_t readRegister24(uint8_t reg);
  uint32_t readRegister32(uint8_t reg);
  void readMultiple(uint8_t reg, void* data, uint8_t len);

  void inline chipSelect(bool enable);

  /**
   * @brief Set the FIFO cursor.
   */
  void seekFIFO(uint8_t position);

  /**
   * @brief Set the Tx base address.
   */
  void setTxFIFO(uint8_t position);

  /**
   * @brief Set the Tx base address.
   */
  void setRxFIFO(uint8_t position);

  /**
   * @brief Sets the mode.
   * @note LORA is always enabled.
   */
  void setMode(uint8_t mode);

  /**
   * @brief Writes bytes to the fifo.
   * @param buffer The address to read from.
   * @param len The number of bytes to write to the FIFO.
   */
  void writeFIFO(const void* buffer, uint8_t len);

  /**
   * @brief Reads bytes from the fifo to the buffer.
   * @param buffer The address to write to
   * @param len The number of bytes to read from the FIFO to the buffer.
   */
  void readFIFO(void* buffer, uint8_t len);

  /**
   * @brief Write the packet data into the fifo.
   */
  void writeTxFIFO(const void* buffer, uint8_t length);

  /**
   * @brief Clear all IRQ flags
   */
  void clearIRQ();

public:

  /**
   * @param cs_pin The chip select pin to use.
   */
  plainRFM95(uint8_t cs_pin);
  

  /**
   * @brief Performs a hardware reset by pulling the reset pin low.
   * @param reset_pin The pin connected to the reset of the radio module.
   */
  static void reset(uint8_t reset_pin);

  /**
   * @brief Switch to LORA mode and confirm spi connections.
   */
  bool begin();

  /**
   * @brief Provides raw register read access for debugging.
   * @param reg The register to read.
   * @return The value of the register.
   */
  uint8_t readRawRegister(uint8_t reg);


  /**
   * @brief Set output power.
   * @param Pout between 5 and 20, higher is more power.
   */
  void setPower(uint8_t Pout);

  /**
   * @brief Set the frequency to operate on in Hz (433 * 1000 * 1000) for MHz
   */
  void setFrequency(uint32_t freq);

  /**
   * @brief Sets the modem config 2 register, containing spreading and crc (also upper part of Rx timeout).
   */
  void setModemConfig2(bool crc_on, uint8_t spreading = RFM95_LORA_MODEM_CONFIG2_SPREADING_128);

  /**
   * @brief Switches to continuous Rx mode, switches DIO0 mapping to RxDone.
   */
  void receive();

  /**
   * @brief Read the number of bytes received, this is the length of the packet in the FIFO.
   */
  uint8_t readRxLength();

  /**
   * @brief Reads the current packet from the fifo. Automatically sets the cursor.
   * @param buffer The address to write the received packet to.
   */
  uint8_t readRxData(void* buffer);

  /**
   * @brief Reads the state whether the CRC was enabled in the packet header.
   */
  bool readPacketCRCOn();

  /**
   * @brief Reads the last packet's SNR value, in dB
   */
  int8_t readPacketSNR();
  /**
   * @brief Reads the last packet's SNR value, in dB
   */
  int8_t readPacketRSSI();

  void printPacketStats();

  /**
   * @brief Switches to Tx mode to transmit the setup data, switches DIO0 mapping to TxDone.
   */
  void transmit();

  /**
   * @brief Stores a payload in the fifo, sets the length register.
   */
  void preparePayload(const void* buffer, uint8_t length);

  /**
   * @brief Go into standby.
   */
  void standby();

  /**
   * @brief Go into sleep
   */
  void sleep();

  /**
   * @brief Polls the status register and returns enum based on the state.
   * The possible states depend on the mode.
   * Receive:
   *   RX_DONE_INVALID_PACKET
   *   RX_DONE_VALID_PACKET
   *   TIMEOUT
   *   NONE
   * Transmit:
   *   TX_DONE
   *   TIMEOUT
   *   NONE
   * CAD:
   *   CAD_DONE_NO_SIGNAL
   *   CAD_DONE_SIGNAL
   *   TIMEOUT
   *   NONE
   */
  IRQState poll();

  /**
   * @brief Blocks on poll() untill it returns something else than None, or timeout duration has been reached.
   */
  IRQState block(uint32_t ms=0);


};

// PLAIN_RFM95_H
#endif