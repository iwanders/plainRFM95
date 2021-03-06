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

// Generic registers
#define RFM95_FIFO 0x00
#define RFM95_OPMODE 0x01

#define RFM95_BITRATE_MSB 0x02
#define RFM95_BITRATE_LSB 0x03

#define RFM95_FDEV_MSB 0x04
#define RFM95_FDEV_LSB 0x05

#define RFM95_FRF_MSB 0x06
#define RFM95_FRF_MID 0x07
#define RFM95_FRF_LSB 0x08

#define RFM95_PA_CONFIG 0x09
#define RFM95_PA_RAMP 0x0A
#define RFM95_OCP 0x0B
#define RFM95_LNA 0x0C

#define RFM95_PACKET_SNR 0x19
#define RFM95_PACKET_RSSI 0x1A

#define RFM95_DIO_MAPPING1 0x40
#define RFM95_DIO_MAPPING2 0x41
#define RFM95_VERSION 0x42
#define RFM95_TCXO 0x4B
#define RFM95_PA_DAC 0x4D

#define RFM95_FORMER_TEMP 0x5B

#define RFM95_AGC_REF 0x61
#define RFM95_AGC_THRES_1 0x62
#define RFM95_AGC_THRES_2 0x63
#define RFM95_AGC_THRES_3 0x64

/// Generic bits

#define RFM95_WRITE_REG_MASK 0x80
#define RFM95_READ_REG_MASK 0x7F

#define RFM95_OPMODE_LORA (1 << 7)
#define RFM95_OPMODE_ACCESS_SHARED_REG (1 << 6)
#define RFM95_OPMODE_LOW_FREQUENCY_MODE_ON (1 << 3)
#define RFM95_MODE_SLEEP 0b000
#define RFM95_MODE_STANDBY 0b001
#define RFM95_MODE_FS_TX 0b010
#define RFM95_MODE_TX 0b011
#define RFM95_MODE_FS_RX 0b100
#define RFM95_MODE_RX_CONTINUOUS 0b101
#define RFM95_MODE_RX_SINGLE 0b110
#define RFM95_MODE_RX_CAD 0b111

#define RFM95_DIO_MAPPING_DIO0_SHIFT 6

#define RFM95_PA_CONFIG_PA_SELECT (1 << 7)
#define RFM95_PA_CONFIG_MAX_POWER (0b111 << 4)

#define RFM95_PA_DAC_NORMAL 0x04
#define RFM95_PA_DAC_BOOST 0x07

/// LORA registers.
#define RFM95_LORA_FIFO_ADDR_PTR 0x0D
#define RFM95_LORA_FIFO_TX_BASE_ADDR 0x0E
#define RFM95_LORA_FIFO_RX_BASE_ADDR 0x0F

#define RFM95_LORA_FIFO_RX_CURRENT_ADDR 0x10

#define RFM95_LORA_IRQ_MASK 0x11
#define RFM95_LORA_IRQ_FLAGS 0x12

#define RFM95_LORA_FIFO_RX_BYTES 0x13

#define RFM95_LORA_PACKET_SNR 0x19
#define RFM95_LORA_PACKET_RSSI 0x1A
#define RFM95_LORA_HOP_CHANNEL 0x1C
#define RFM95_LORA_MODEM_CONFIG1 0x1D
#define RFM95_LORA_MODEM_CONFIG2 0x1E
#define RFM95_LORA_MODEM_CONFIG3 0x26

#define RFM95_LORA_PREAMBLE_MSB 0x20
#define RFM95_LORA_PREAMBLE_LSB 0x21

#define RFM95_LORA_PAYLOAD_LENGTH 0x22
#define RFM95_LORA_PAYLOAD_MAX_LENGTH 0x23



// LORA bits

#define RFM95_LORA_DIO0_RX_DONE 0b00
#define RFM95_LORA_DIO0_TX_DONE 0b01
#define RFM95_LORA_DIO0_CAD_DONE 0b10
#define RFM95_LORA_DIO0_NONE 0b11

#define RFM95_LORA_IRQ_RX_TIMEOUT (1 << 7)
#define RFM95_LORA_IRQ_RX_DONE (1 << 6)
#define RFM95_LORA_IRQ_CRC_ERROR (1 << 5)
#define RFM95_LORA_IRQ_VALID_HEADER (1 << 4)
#define RFM95_LORA_IRQ_TX_DONE (1 << 3)
#define RFM95_LORA_IRQ_CAD_DONE (1 << 2)
#define RFM95_LORA_IRQ_FHSS_CHANGE (1 << 1)
#define RFM95_LORA_IRQ_CAD_DETECTED (1 << 0)

#define RFM95_LORA_MODEM_CONFIG1_BW_7_8_KHZ (0b0000 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_10_4_KHZ (0b0001 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_15_6_KHZ (0b0010 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_20_8_KHZ (0b0011 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_31_25_KHZ (0b0100 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_41_7_KHZ (0b0101 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_62_5_KHZ (0b0110 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_125_KHZ (0b0111 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_250_KHZ (0b1000 << 4)
#define RFM95_LORA_MODEM_CONFIG1_BW_500_KHZ (0b1001 << 4)

#define RFM95_LORA_MODEM_CONFIG1_CODING_4_5 (0b001 << 1)
#define RFM95_LORA_MODEM_CONFIG1_CODING_4_6 (0b010 << 1)
#define RFM95_LORA_MODEM_CONFIG1_CODING_4_7 (0b011 << 1)
#define RFM95_LORA_MODEM_CONFIG1_CODING_4_8 (0b100 << 1)
#define RFM95_LORA_MODEM_CONFIG1_IMPLICIT_HEADER (1)

#define RFM95_LORA_MODEM_CONFIG2_SPREADING_64 (6 << 4)
#define RFM95_LORA_MODEM_CONFIG2_SPREADING_128 (7 << 4)
#define RFM95_LORA_MODEM_CONFIG2_SPREADING_256 (8 << 4)
#define RFM95_LORA_MODEM_CONFIG2_SPREADING_512 (9 << 4)
#define RFM95_LORA_MODEM_CONFIG2_SPREADING_1024 (10 << 4)
#define RFM95_LORA_MODEM_CONFIG2_SPREADING_2048 (11 << 4)
#define RFM95_LORA_MODEM_CONFIG2_SPREADING_4096 (12 << 4)

#define RFM95_LORA_MODEM_CONFIG3_LOW_DATA_RATE_OPTIMIZE (1 << 3)
#define RFM95_LORA_MODEM_CONFIG3_AGC_AUTO_ON (1 << 2)

// Technically the datasheet here states received from packet header, but that doesn't make sense.
#define RFM95_LORA_MODEM_CONFIG2_CRC_ON (1 << 2)
