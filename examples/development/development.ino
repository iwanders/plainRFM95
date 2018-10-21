#include <Arduino.h>
#include <SPI.h>
#include "plainRFM95.h"

// Generic pins
#define SPI_SCK_PIN 14

// RFM95 pins
#define RFM95_CS_PIN 10
#define RFM95_RST_PIN 5

// Unused DIO pins.
#define RFM95_DIO0_PIN 6
#define RFM95_DIO2_PIN 2

/*
  Instantaneous power usage by these radio modules is not to be underestimated. You can use the following to detect and
  correct brownout:
  if (rfm.notInLORA())
  {
    rfm.begin();
  }
*/

// Simple function to print a buffer in hexadecimal.
void hexprint(const uint8_t* data, uint8_t length)
{
  char buffer[2 * length + 1];
  memset(buffer, 0, 2 * length + 1);
  for (uint16_t i = 0; i < length; ++i)
  {
    sprintf(buffer + 2 * i, "%02x", data[i]);
  }
  Serial.println(buffer);
}

// Instantiate the class.
plainRFM95 rfm;

void setup()
{
  // Setup the USB port
  Serial.begin(9600);

  // Block on serial port for testing.
  pinMode(LED_BUILTIN, OUTPUT);
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  while (!Serial)
  //  {
  //  }
  //  digitalWrite(LED_BUILTIN, LOW);

  delay(1000);

  // Setup the SPI bus.
  SPI.setSCK(SPI_SCK_PIN);
  SPI.begin();

  // Setup GPIO pins.
  pinMode(RFM95_CS_PIN, OUTPUT);
  pinMode(RFM95_RST_PIN, OUTPUT);
  digitalWrite(RFM95_RST_PIN, HIGH);
  //  pinMode(RFM95_DIO2_PIN, INPUT_PULLUP);


  plainRFM95::reset(RFM95_RST_PIN);  // sent the RFM95 a hard-reset.

  // Do a count down to allow the serial connect.
  delay(10);
  for (uint8_t i = 5; i > 0; i--)
  {
    Serial.println(i);
    delay(500);
  }

  setupRadio();

  // from https://forum.pjrc.com/threads/25522-Serial-Number-of-Teensy-3-1/page2
  Serial.print("SIM_UIDL: ");
  Serial.println(SIM_UIDL, HEX);
}

void setupRadio()
{
  plainRFM95::reset(RFM95_RST_PIN);  // sent the RFM95 a hard-reset.
  if (rfm.begin(RFM95_CS_PIN))
  {
    Serial.println("Setup correct");
  }
  //  rfm.setPreamble(160);

  rfm.setModemConfigRobust();

}


void sender()
{
  Serial.println("Sender");

  uint8_t sbuf[255] = { 0 };
  uint8_t rbuf[255] = { 0 };
  uint32_t& s_ctr = reinterpret_cast<uint32_t&>(sbuf);
  uint32_t& r_ctr = reinterpret_cast<uint32_t&>(rbuf);
  int8_t& r_rssi = reinterpret_cast<int8_t&>(*(rbuf + sizeof(r_ctr)));

  const uint8_t len = 64;

  bool waiting_for_response = false;
  elapsedMillis tx_duration = 0;
  rfm.standby();
  while (1)
  {
    if (rfm.notInLORA())
    {
      Serial.println("Not in lora, calling setupRadio to fix this.");
      setupRadio();
    }

    if (waiting_for_response)
    {
      // Do the receive step, block up to 1000 ms on trying to receive a message.
      rfm.receiveSingle();
      switch (rfm.block(1000))
      {
        case plainRFM95::RX_DONE_VALID_PACKET:
          // done, got packet.
          break;  // show packet stats
        case plainRFM95::RX_DONE_INVALID_PACKET:
          Serial.println("Received invalid packet :(");
          waiting_for_response = false;
          break;  // show packet stats
        case plainRFM95::TIMEOUT:
          Serial.print("Timed out waiting on: 0x");
          Serial.println(s_ctr, HEX);
          waiting_for_response = false;
          continue;
        default:
          waiting_for_response = false;
          continue;
      }

      rfm.standby();  // stop listening.
      uint8_t rlen = rfm.readRxData(&rbuf);

      if (s_ctr != r_ctr)
      {
        Serial.print("Incorrect counters, sent: 0x");
        Serial.print(s_ctr, HEX);
        Serial.print(" received: 0x");
        Serial.println(r_ctr, HEX);
      }
      Serial.print("Read: ");
      hexprint(rbuf, rlen);
      rfm.printPacketStats();
      Serial.print("   Remote RSSI: "); Serial.println(r_rssi);

      waiting_for_response = false;
    }
    else
    {
      rfm.standby();  // stop listening.

      delay(100);

      s_ctr++;
      digitalWrite(LED_BUILTIN, HIGH);

      rfm.preparePayload(sbuf, len);
      tx_duration = 0;
      rfm.transmit();
      switch (rfm.block(1000))
      {
        case plainRFM95::TX_DONE:
          digitalWrite(LED_BUILTIN, LOW);
          Serial.print("Sent: ");
          hexprint(sbuf, len);
          break;
        default:
          Serial.println("Block returned other than TX_DONE");
          break;
      }
      Serial.print("Tx duration: "); Serial.println(tx_duration);

      waiting_for_response = true;
    }
  }
}

void receiver()
{
  Serial.println("Receiver");
  while (1)
  {
    if (rfm.notInLORA())
    {
      Serial.println("Not in lora, calling setupRadio to fix this.");
      setupRadio();
    }

    // Wait on message
    rfm.receive();
    switch (rfm.block(10000))  // wait for 10 seconds, then check if we dropped out of lora.
    {
      case plainRFM95::RX_DONE_INVALID_PACKET:
        rfm.receive();  // drops the packet.
        continue;
      default:
        rfm.receive();  // Clear IRQ and receive again.
        continue;
      case plainRFM95::RX_DONE_VALID_PACKET:
        break;
    }
    rfm.standby();  // stop listening.

    // valid packet if we got here.
    uint8_t buf[255] = { 0 };
    uint32_t& r_ctr = reinterpret_cast<uint32_t&>(buf);
    int8_t& r_rssi = reinterpret_cast<int8_t&>(*(buf + sizeof(r_ctr)));
    uint8_t rlen = rfm.readRxData(&buf);
    Serial.print("Data: ");
    hexprint(buf, rlen);
    rfm.printPacketStats();
    r_rssi = rfm.readPacketRSSI();  // add our reception strength to the message payload.

    delay(10);  // wait a little bit such that the other side can switch to receive.

    // going to transmit this back.
    rfm.preparePayload(buf, rlen);
    rfm.transmit();

    // block on transmission.
    switch (rfm.block(1000))
    {
      case plainRFM95::TX_DONE:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("Sent back: ");
        hexprint(buf, rlen);
        break;
      default:
        Serial.println("Block returned other than TX_DONE");
        break;
    }
    // should've sent it, go back into receive.
  }
}

void loop()
{
  // Go receiver or sender based on the unique ID in the chip.
  if (SIM_UIDL == 0x45164E45)
  {
    receiver();
  }
  else
  {
    sender();
  }
}
