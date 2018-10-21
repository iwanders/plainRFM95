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

/*
Symbol rate, p28 S1276

Rs = BW / 2**SF

Bw = Bandwidth
SF = Spreading factor

Tsym = 1.0 / Rs

p31 SX1276
Tpreamble = (Npreamble + 4.25) * Tsym

Npayload = 8 + max(ceil((8 * PL - 4 * SF + 28 + 16 * CRC - 20 * IH)/(2 * SF - 2 * DE))*(CR + 4), 0)
PL = Number of payload bytes (1-255)
SF = Spreading factor (6 - 12)
IH = Implicit header, 0 when header, 1 when no header.
DE = Low data rate optimize enabled (1 = enabled, 0 = disabled)
CRC = 1 if CRC enabled
CR = Coding rate (1 corresponds to 4/5, 4 to 4/8)

Tpayload = Npayload * Ts
Tpacket = Tpreamble + Tpayload

# Copy-paste ready Python3 code:

BW = 125e3
CR = 4/5
SPREAD = 128
Npreamble = 8
Npayload = 32

CRC = 1
IH = 0

from math import ceil, log2
Rs = BW / (SPREAD)
Tsym = 1.0 / Rs
Tpreamble = (Npreamble + 4.25) * Tsym
DE = (0 if Tsym < 16e-3 else 1)
Spayload = 8 + max(ceil(8 * Npayload - 4 * log2(SPREAD) + 28 + 16 * CRC - 20 * IH) / (4*(log2(SPREAD) - 2 * DE)) * ((1/(CR) * 4 - 4) + 4), 0)
Tpayload = Spayload * Tsym
Tpacket = Tpreamble + Tpayload
print("Low data rate should be: {}".format(DE))
print("Tpayload: {:f}\nTpreamble: {:f}\nTpacket: {:f}".format(Tpayload, Tpreamble, Tpacket))


 */

class plainRFM95
{
public:
  enum IRQState : uint8_t
  {
    NONE,  //!< nothing yet, still listening or sending, or idling.

    RX_DONE_INVALID_PACKET,  //!< Rx Done happened, CRC is incorrect.
    RX_DONE_VALID_PACKET,    //!< Rx Done, CRC was correct, packet to read.

    CAD_DONE_NO_SIGNAL,  //!< The CAD is finished, but no signal was detected.
    CAD_DONE_SIGNAL,     //!< The CAD is finished, and a signal was detected

    TX_DONE,  //!< Transmission was finished.

    TIMEOUT,  //!< Timeout has happened, can only happen if blocking with optional argument of Rx single.
  };
  enum Activity : uint8_t
  {
    IDLE,
    RX,
    TX,
    CAD
  };

protected:
  uint8_t cs_pin_;   //!< chip select pin.
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
   * @brief Performs a hardware reset by pulling the reset pin low.
   * @param reset_pin The pin connected to the reset of the radio module.
   */
  static void reset(uint8_t reset_pin);

  /**
   * @brief Switch to LORA mode and confirm spi connections.
   * @param cs_pin The chip select pin to use.
   * @return True if registers could be set correctly, confirming the SPI connection false otherwise.
   */
  bool begin(uint8_t cs_pin);

  /**
   * @brief If the module resets for any reason, the LORA bit in the opmode gets flipped to false after the reset.
   *        This can be used to detect whether or not begin() has to be called again.
   * @return True if the radio module is not in LORA mode and begin() needs to be called.
   */
  bool notInLORA();

  /**
   * @brief Provides raw register read access for debugging.
   * @param reg The register to read.
   * @return The value of the register.
   */
  uint8_t readRawRegister(uint8_t reg);

  /**
   * @brief Dump registers for debugging.
   */
  void dumpDebugRegisters();

  /**
   * @brief Set output power.
   * @param Pout between 5 and 20, higher is more power.
   */
  void setPower(uint8_t Pout);

  /**
   * @brief Set the frequency to operate on in Hz (433 * 1000 * 1000) for 433.0 MHz
   */
  void setFrequency(uint32_t freq);

  /**
   * @brief Set the preamble length, defaults to 8.
   */
  void setPreamble(uint16_t preamble_length);

  /**
   * @brief Set preamble length, in symbols.
   */

  /**
   * @brief Set the signal bandwith and coding rate of the transmission.
   * @param bandwidth The signal bandwidth to use. See the constants:
   *        RFM95_LORA_MODEM_CONFIG1_BW_7_8_KHZ - RFM95_LORA_MODEM_CONFIG1_BW_500_KHZ
   * @param coding_rate The error coding rate. RFM95_LORA_MODEM_CONFIG1_CODING_4_5 to 4_8
   * @note p25 SX1276: For all bandwidths lower than 62.5 kHz, it is advised to use a TCXO as a frequency reference.
   *       This is required to meet the frequency error tolerance specifications given in the Electrical Specification.
   */
  void setModemConfig1(uint8_t bandwidth, uint8_t coding_rate);

  /**
   * @brief Sets the modem config 2 register, containing spreading and crc (also upper part of Rx timeout).
   * @param spreading The number of chirps per symbol. RFM95_LORA_MODEM_CONFIG2_SPREADING_64 to 4096
   * @param crc True; CRC is used to verify packet integrity. Set in the header of the packet.
   * @note p30 SX1276: Note With SF = 6 (RFM95_LORA_MODEM_CONFIG2_SPREADING_64) selected, implicit header mode is the
   *       only mode of operation possible.
   */
  void setModemConfig2(bool crc_on, uint8_t spreading = RFM95_LORA_MODEM_CONFIG2_SPREADING_128);

  /**
   * @brief Sets LNA gain option and low data rate optimization.
   * @param low_data_rate_optimization mandated for when the symbol length exceeds 16ms, p114 SX1276.
   * @param agc_auto_on False: LNA gain set by register LnaGain, True: LNA gain set by the internal AGC loop
   */
  void setModemConfig3(bool low_data_rate_optimization, bool agc_auto_on = false);

  /**
   * @brief Sets the default modem configuration parameters, but does enable CRC (which defaults to off.
   *        crc = 1
   *        RFM95_LORA_MODEM_CONFIG1_BW_125_KHZ
   *        RFM95_LORA_MODEM_CONFIG1_CODING_4_5
   *        RFM95_LORA_MODEM_CONFIG2_SPREADING_128
   *        low_data_rate_optimization = false, agc_auto_on = false;
   *        BW = 125e3;CR = 4/5;SPREAD = 128;Npreamble = 8;Npayload = 32
   */
  void setModemConfigDefault();

  /**
   * @brief Takes default, but increases error coding rate and spreading rate for more robust transmissions. Bw is kept
   *        identical as to not run into transmission errors because of clock drift.
   *        crc = 1
   *        RFM95_LORA_MODEM_CONFIG1_BW_125_KHZ
   *        RFM95_LORA_MODEM_CONFIG1_CODING_4_6
   *        RFM95_LORA_MODEM_CONFIG2_SPREADING_256
   *        low_data_rate_optimization = false, agc_auto_on = false;
   *        BW = 125e3;CR = 4/6;SPREAD = 256;Npreamble = 8;Npayload = 64
   */
  void setModemConfigRobust();

  /**
   * @brief Switches to continuous Rx mode, switches DIO0 mapping to RxDone.
   */
  void receive();

  /**
   * @brief Switches the radio to single Rx mode. Switches DIO0 mapping to RxDone.
   */
  void receiveSingle();

  /**
   * @brief Read the number of bytes received, this is the length of the packet in the FIFO.
   */
  uint8_t readRxLength();

  /**
   * @brief Reads the current packet from the fifo.
   * @param buffer The address to write the received packet to.
   * @return The length of the packet that was just read.
   */
  uint8_t readRxData(void* buffer);

  /**
   * @brief Reads whether the last packet had it's CRC enabled in the header.
   */
  bool readPacketCRCOn();

  /**
   * @brief Reads the last packet's SNR value, in dB
   */
  int8_t readPacketSNR();

  /**
   * @brief Reads the last packet's SNR value, in dBm
   */
  int8_t readPacketRSSI();

  /**
   * @brief Print stats about the last received message. (CRC, SNR, RSSI, length)
   */
  void printPacketStats();

  /**
   * @brief Stores a payload in the fifo, sets the length register.
   */
  void preparePayload(const void* buffer, uint8_t length);

  /**
   * @brief Switches to Tx mode to transmit the setup data, switches DIO0 mapping to TxDone.
   */
  void transmit();

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
   * @return The possible states depend on the previously set mode:
   * Receive:
   *   RX_DONE_INVALID_PACKET
   *   RX_DONE_VALID_PACKET
   *   NONE
   * Transmit:
   *   TX_DONE
   *   NONE
   * CAD:
   *   CAD_DONE_NO_SIGNAL
   *   CAD_DONE_SIGNAL
   *   NONE
   */
  IRQState poll();

  /**
   * @brief Blocks on poll() untill it returns something else than NONE, or timeout duration has been reached.
   * @return The possible states depend on the previously set mode:
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
  IRQState block(uint32_t ms = 0);

  /**
   * @brief Return the current activity of the radio.
   * @return IDLE, RX, TX or CAD
   */
  Activity getActivity();
};

// PLAIN_RFM95_H
#endif