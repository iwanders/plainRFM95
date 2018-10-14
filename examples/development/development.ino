#include <SPI.h>
#include <Arduino.h>
#include "plainRFM95.h"

// Generic pins
#define SPI_SCK_PIN 14

// RFM95 pins
#define RFM95_CS_PIN 10
#define RFM95_RST_PIN 5

// Unused DIO pins.
#define RFM95_DIO0_PIN 6
#define RFM95_DIO2_PIN 2


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
plainRFM95 rfm = plainRFM95(RFM95_CS_PIN);

void setup()
{
  // Setup the USB port
  Serial.begin(9600);

  delay(1000);

  // Setup the SPI bus.
  //  pinMode(11, OUTPUT);
  //  pinMode(12, INPUT_PULLUP);
  SPI.setSCK(SPI_SCK_PIN);
  SPI.begin();

  // Setup GPIO pins.
  pinMode(RFM95_CS_PIN, OUTPUT);
  pinMode(RFM95_RST_PIN, OUTPUT);
  digitalWrite(RFM95_RST_PIN, HIGH);
  //  pinMode(RFM95_DIO2_PIN, INPUT_PULLUP);

  // Setup led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  plainRFM95::reset(RFM95_RST_PIN); // sent the RFM95 a hard-reset.

  // Do a count down to allow the serial connect.
  delay(10);
  for (uint8_t i = 5; i > 0; i--)
  {
    Serial.println(i);
    delay(500);
  }
  if (rfm.begin())
  {
    Serial.println("Setup correct");
  }

  // from https://forum.pjrc.com/threads/25522-Serial-Number-of-Teensy-3-1/page2
  Serial.print("SIM_UIDL: "); Serial.println(SIM_UIDL, HEX);
}

void sender()
{
  Serial.println("Sender");
  uint8_t buf[255] = {0};
  uint32_t& ctr = *reinterpret_cast<uint32_t*>(buf);
  const uint8_t len = 32;
  while(1)
  {
    delay(1000);
    ctr++;
    digitalWrite(LED_BUILTIN, HIGH);
    rfm.preparePayload(buf, len);
    rfm.transmit();
    switch (rfm.block())
    {
      case plainRFM95::TX_DONE:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("Sent: "); hexprint(buf, len);
        break;
      case plainRFM95::CAD_DONE_SIGNAL:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("CAD!!");
        break;
      default:
        Serial.print("irq: 0x"); Serial.println(rfm.readRawRegister(RFM95_LORA_IRQ_FLAGS), HEX);
        Serial.println("Transmitting");
        break;
    }
  }
}

void receiver()
{
  Serial.println("Receiver");
  rfm.receive();
  while(1)
  {
    switch (rfm.block())
    {
      case plainRFM95::RX_DONE_INVALID_PACKET:
        rfm.receive(); // drops the packet.
        continue;
      default:
        Serial.print("irq: 0x"); Serial.println(rfm.readRawRegister(RFM95_LORA_IRQ_FLAGS), HEX);
        continue;
      case plainRFM95::RX_DONE_VALID_PACKET:
        break;
    }
    // valid packet if we got here.
    uint8_t buf[255] = {0};
    uint8_t rlen = rfm.readRxData(&buf);
    Serial.print("Data: "); hexprint(buf, rlen);
    rfm.printPacketStats();
    rfm.receive();
  }
}

void loop()
{
  while (Serial.available())
  {
    auto z = Serial.read();
    Serial.print("Read: "); Serial.println(z);
    Serial.print("C: "); Serial.write(z); Serial.println();
  }

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
