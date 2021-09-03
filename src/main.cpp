/*! @mainpage
//-------------------------------------------------------------------------------
*/
#include <Arduino.h>

//-- DEBUG ----------------------------------------------------------------------
#define DEBUG_PIN               false
#define DEBUG_VALUE             false

#if DEBUG_PIN
#define TEST_PIN(gpio_nr)          { Serial.print("TEST_PIN BEGIN:"); \
                                     Serial.println(gpio_nr); \
                                     for (uint8_t i = 0; i < 10; i++) { \
                                        Serial.println("TEST_PIN RUNNING"); \
                                        pinMode(gpio_nr, OUTPUT); \
                                        digitalWrite(gpio_nr, HIGH); \
                                        delay(1000); \
                                        digitalWrite(gpio_nr, LOW); \
                                        delay(1000); \
                                     } \
                                     Serial.println("TEST_PIN END"); \
                                   }
#else
#define TEST_PIN(gpio_nr)          {}           
#endif

#if DEBUG_VALUE
#define PR(msg, value)          { Serial.print(msg); Serial.println(value); }
#else
#define PR(msg, value)          {}           
#endif

/*
 * Uncomment to enable debug output.
 * Warning: Debug output will slow down the whole system significantly.
 *          Also, it will result in larger compiled binary.
 * Levels: debug - only main info
 *         verbose - full transcript of all SPI/UART communication
 */

#define RADIOLIB_DEBUG
// #define RADIOLIB_VERBOSE


//-- CONFIGURATIONS -------------------------------------------------------------
#define UART_ECHO               (0)
#define UART_BAUDRATE           (115200)

#define ADC1_0                  (36)
#define ADC1_3                  (39)

//-- 433 MHz RADIO --------------------------------------------------------------
#include <RadioLib.h>

// https://www.electrodragon.com/w/Si4432
// Si4432 has the following connections:
#define RADIO_nSEL      5
#define RADIO_nIRQ      15
// BUG: https://github.com/jgromes/RadioLib/issues/305
#define RADIO_SDN       13

Si4432 radio = new Module(RADIO_nSEL, RADIO_nIRQ, RADIO_SDN);

//-- MSG PACK -------------------------------------------------------------------
// RX PROTOCOL
uint8_t pan;
uint8_t tilt;
uint8_t outd;   // bit 3 = PIN 3, pin 4 = PIN 4
#define PACKET_RX_SIZE (sizeof(uint8_t) * 3)
// TX PROTOCOL
float analogPan;
float analogTilt;
float analogPanLast;
float analogTiltLast;
// #define MSGPACK_DEBUGLOG_ENABLE
#include <MsgPack.h>

MsgPack::Packer packer;
MsgPack::Unpacker unpacker;

//--- CONSOLE MENU ---------------------------------------------------------------
// https://github.com/neu-rah/ArduinoMenu/wiki/Menu-definition
#include <menu.h>
#include <menuIO/serialOut.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialIn.h>

using namespace Menu;

#define MAX_DEPTH   1

result menuShowIP();
result menuListAP();
result menuSetUser();
result menuSetPass();
result menuSave();
result menuInfo();

MENU(mainMenu,"telemetry config",doNothing,noEvent,wrapStyle
  ,OP("show IP",menuShowIP,enterEvent)
  ,OP("list AP",menuListAP,enterEvent)
  ,OP("set user",menuSetUser,enterEvent)
  ,OP("set password",menuSetPass,enterEvent)
  ,OP("save",menuSave,enterEvent)
  ,OP("nav info",menuInfo,enterEvent)
  ,OP("set channel",doNothing,enterEvent)
  ,OP("set power",doNothing,enterEvent)
  ,EXIT("<Back")
);

serialIn serial(Serial);
MENU_INPUTS(in,&serial);

MENU_OUTPUTS(out,MAX_DEPTH
  ,SERIAL_OUT(Serial)
  ,NONE//must have 2 items at least
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);


//-------------------------------------------------------------------------------
#include <TaskScheduler.h>

void sensorReading();
Task sensorTask(5000, TASK_FOREVER, &sensorReading);
Scheduler runner;

float voltageReading(float value) {
  return ((5*value)/1023)*4.103354632587859;
}

void sensorReading() {
  analogPan = voltageReading(ADC1_0);
  analogTilt = voltageReading(ADC1_3);
  if (analogPan == analogPanLast && analogTilt == analogTiltLast) return;

  analogPanLast = analogPan;
  analogTiltLast = analogTilt;
  packer.serialize(analogPan, analogTilt);
  int state = radio.transmit((uint8_t *)packer.data(), packer.size());

  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("I:Si4432:TX:success!"));

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("I:Si4432:TX:too long!"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("I:Si4432:TX:timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("I:Si4432:TX:failed, code "));
    Serial.println(state);
  }
}

//-------------------------------------------------------------------------------------------------------

result menuShowIP() {
  Serial.println("menuShowIP");
  return proceed;
}

result menuListAP() {
  Serial.println("menuListAP");
  return proceed;
}

result menuSetUser() {
  Serial.println("menuSetUser");
  return proceed;
}

result menuSetPass() {
  Serial.println("menuSetPass");
  return proceed;
}

result menuSave() {
  Serial.println("menuSave");
  return proceed;
}

result menuInfo() {
  Serial.println("I:CONSOLE");
  Serial.println("I:Use keys [+ up] [- down] [* enter] [/ esc]");
  Serial.println("I:to control the menu navigation");
  return proceed;
}

//-------------------------------------------------------------------------------
void setup() {
  Serial.begin(UART_BAUDRATE);
  // needed to keep leonardo/micro from starting too fast!
  while (!Serial) { delay(10); }

  TEST_PIN(RADIO_nSEL);
  TEST_PIN(RADIO_nIRQ);
  TEST_PIN(RADIO_SDN);
  TEST_PIN(18);
  Serial.println(F("I:Si4432:START"));
  PR("I:MOSI:", MOSI);
  PR("I:MISO:", MISO);
  PR("I:SCK:", SCK);
  PR("I:SS:", SS);
  int state = radio.begin();
  if (state == ERR_NONE) {
    Serial.println(F("I:Si4432:success!"));
  } else {
    Serial.print(F("I:Si4432:failed code: "));
    Serial.println(state);
    while (true);
  }

  runner.init();
  runner.addTask(sensorTask);
  sensorTask.enable();
}

void loop() {
  byte payload[PACKET_RX_SIZE];
  int state = radio.receive(payload, PACKET_RX_SIZE);

  if (state == ERR_NONE) {
    // packet was successfully received
    Serial.println(F("I:Si4432:RX:success!"));
    unpacker.feed(payload, PACKET_RX_SIZE);
    unpacker.deserialize(pan, tilt, outd);

    // print the data of the packet
    Serial.print(F("I:Si4432:RX:Data:\t\t"));
    Serial.println((char*)payload);

  } else if (state == ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("I:Si4432:RX:timeout!"));

  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("I:Si4432:RX:CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("I:Si4432:RX:failed code: "));
    Serial.println(state);

  }
  runner.execute();

  nav.doInput();
  if (nav.changed(0)) {
    nav.doOutput();
  }
}