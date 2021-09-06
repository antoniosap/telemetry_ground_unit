/*! @mainpage
//-------------------------------------------------------------------------------
*/
#include <Arduino.h>

//-- DEBUG ----------------------------------------------------------------------
#define DEBUG_PIN               false
#define DEBUG_VALUE             true

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

#define PR_VALUE(msg, value)    { Serial.print(F(msg)); Serial.println(value); }
#define PR_FLOAT(msg, value)    { char buf[16]; snprintf(buf, 16, "%.4f", value); Serial.print(F(msg)); Serial.println(buf); }

/*
 * Uncomment to enable debug output.
 * Warning: Debug output will slow down the whole system significantly.
 *          Also, it will result in larger compiled binary.
 * Levels: debug - only main info
 *         verbose - full transcript of all SPI/UART communication
 */

// #define RADIOLIB_DEBUG
// #define RADIOLIB_VERBOSE


//-- CONFIGURATIONS -------------------------------------------------------------
#define UART_ECHO               (0)
#define UART_BAUDRATE           (115200)

#define ADC1_0                  (36)
#define ADC1_3                  (39)

//------------------------------------------------------------------------------
template <typename T> T serialPrintBinary(T x, bool usePrefix = true)
{
  if (usePrefix) Serial.print("0b");
  for (uint8_t i = 0; i < 8 * sizeof(x); i++) {
    Serial.print(bitRead(x, sizeof(x) * 8 - i - 1));
  }
  Serial.println();
  return x;
}

//------------------------------------------------------------------------------
#include <FastLED.h>

#define LED_PIN     33
#define NUM_LEDS    8
#define CHIPSET     WS2811
#define COLOR_ORDER GRB
#define BRIGHTNESS  128
CRGB leds[NUM_LEDS];

#define RX_LED      6
#define TX_LED      7

#define LED_SHOW_COLOR(i, color)    { leds[i] = color; FastLED.show(); }

//-- 433 MHz RADIO --------------------------------------------------------------
#include <RadioLib.h>
#include <RadioDefs.h>

// https://www.electrodragon.com/w/Si4432
// Si4432 has the following connections:
#define RADIO_nSEL      5
#define RADIO_nIRQ      15
// BUG: https://github.com/jgromes/RadioLib/issues/305
#define RADIO_SDN       13

Module* module = new Module(RADIO_nSEL, RADIO_nIRQ, RADIO_SDN);
Si4432 radio = module;

float radioFreq = 434.0;
float radioBitRateKbSec = 48.0;
float radioFreqDev = 50.0;
float radioRxBw = 181.1;
int8_t radioPower = 10;
uint8_t radioPreambleLen = 40;

#define ITU_433_ISM_BAND_LOW      (433.050)
#define ITU_433_ISM_BAND_HIGH     (434.790)
#define ITU_433_ISM_BAND_SPACING    (0.025)  

//-- MSG PACK -------------------------------------------------------------------
// RX PROTOCOL
float analogA0;
float analogA1;
float analogA2;
float analogA3;
#define PACKET_RX_SIZE (20)    // matched bit-bit @ packer.size() transmitter 
// TX PROTOCOL
float txAnalogPan;
float txAnalogTilt;
uint8_t BTNBlkValue = HIGH;    // unpressed
uint8_t BTNRedValue = HIGH;    // unpressed
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

#define MAX_DEPTH   2

result menuShowIP();
result menuListAP();
result menuSetUser();
result menuSetPass();
result menuSave();
result menuInfo();
result menuSetChannel();
result menuSetPower();
result menuLoopbackTest();
result menuRadioStatus();
result menuDumpRadioRegisters();
result menuRadioScanner();

MENU(wifiMenu,"wifi",doNothing,noEvent,wrapStyle
  ,OP("show IP",menuShowIP,enterEvent)
  ,OP("list AP",menuListAP,enterEvent)
  ,OP("set user",menuSetUser,enterEvent)
  ,OP("set password",menuSetPass,enterEvent)
  ,OP("save",menuSave,enterEvent)
  ,EXIT("<Back")
);

MENU(radioMenu,"radio",doNothing,noEvent,wrapStyle
  ,OP("set channel",menuSetChannel,enterEvent)
  ,OP("set power",menuSetPower,enterEvent)
  ,OP("dump radio registers",menuDumpRadioRegisters,enterEvent)
  ,EXIT("<Back")
);

MENU(mainMenu,"system config",doNothing,noEvent,wrapStyle
  ,SUBMENU(wifiMenu)
  ,SUBMENU(radioMenu)
  ,OP("nav info",menuInfo,enterEvent)
  ,OP("loopback test",menuLoopbackTest,enterEvent)
  ,OP("radio status",menuRadioStatus,enterEvent)
  ,OP("radio scanner",menuRadioScanner,enterEvent)
  ,EXIT("<Back")
);

serialIn serial(Serial);
MENU_INPUTS(in,&serial);

MENU_OUTPUTS(out,MAX_DEPTH
  ,SERIAL_OUT(Serial)
  ,NONE//must have 2 items at least
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

//-- KEYBOARD ------------------------------------------------------------------
#include <Bounce2.h>
#define BUTTONS_BLK_PIN     (4)
#define BUTTONS_RED_PIN     (2)

Bounce btnBlk = Bounce(BUTTONS_BLK_PIN, 5);
Bounce btnRed = Bounce(BUTTONS_RED_PIN, 5); 



void buttonInit() {
  pinMode(BUTTONS_BLK_PIN,  INPUT_PULLUP);
  pinMode(BUTTONS_RED_PIN,  INPUT_PULLUP);
  
  BTNBlkValue = HIGH;    // unpressed
  BTNRedValue = HIGH;    // unpressed
}

void buttonProcess() {
  btnBlk.update();
  btnRed.update();

  if (BTNBlkValue != btnBlk.read()) {
    BTNBlkValue = btnBlk.read();
    if (BTNBlkValue == LOW) {
      PR_VALUE("BTN:BLK:", "LOW");
    }
    if (BTNBlkValue == HIGH) {
      PR_VALUE("BTN:BLK:", "HIGH");
    }
  }  
  
  if (BTNRedValue != btnRed.read()) {
    BTNRedValue = btnRed.read();
    if (BTNRedValue == LOW) {
      PR_VALUE("BTN:RED:", "LOW");
    }
    if (BTNRedValue == HIGH) {
      PR_VALUE("BTN:RED:", "HIGH");
    }
  }    
}

//-------------------------------------------------------------------------------
#include <TaskScheduler.h>

void radioProcess();
Task radioTask(1000, TASK_FOREVER, &radioProcess);
Scheduler runner;

float voltageReading12b(uint8_t pin) {
  return analogRead(pin) * (3.3 / 4096.0); // @3V3
}

void txSensor() {
  txAnalogPan = voltageReading12b(ADC1_0);
  txAnalogTilt = voltageReading12b(ADC1_3);
  packer.clear();
  packer.serialize(txAnalogPan, txAnalogTilt, BTNBlkValue, BTNRedValue);
  PR_FLOAT("\nI:TX:PAN:", txAnalogPan);
  PR_FLOAT("I:TX:TILT:", txAnalogTilt);
  PR("I:TX:BLK:", BTNBlkValue);
  PR("I:TX:RED:", BTNRedValue);
  PR("I:TX:SIZE:", packer.size());
  int state = radio.transmit((uint8_t *)packer.data(), packer.size());

  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    LED_SHOW_COLOR(TX_LED, CRGB::Green);
    Serial.println(F("I:Si4432:TX:success!"));

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    LED_SHOW_COLOR(TX_LED, CRGB::Blue);
    Serial.println(F("I:Si4432:TX:too long!"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    LED_SHOW_COLOR(TX_LED, CRGB::Violet);
    Serial.println(F("I:Si4432:TX:timeout!"));

  } else {
    // some other error occurred
    LED_SHOW_COLOR(TX_LED, CRGB::Red);
    Serial.print(F("I:Si4432:TX:failed, code "));
    Serial.println(state);
  }
  LED_SHOW_COLOR(TX_LED, CRGB::Black);
}

void rxTelemetry() {
  byte payload[PACKET_RX_SIZE];
  int state = radio.receive(payload, PACKET_RX_SIZE);

  if (state == ERR_NONE) {
    // packet was successfully received
    LED_SHOW_COLOR(RX_LED, CRGB::Green);
    Serial.println(F("\nI:Si4432:RX:success!"));
    unpacker.feed(payload, PACKET_RX_SIZE);
    unpacker.deserialize(analogA0, analogA1, analogA2, analogA3);

    // print the data of the packet
    // Serial.print(F("I:Si4432:RX:Data:"));
    // Serial.println((char*)payload);
#if DEBUG_VALUE
    PR_FLOAT("I:RX:A0:", analogA0);
    PR_FLOAT("I:RX:A1:", analogA1);
    PR_FLOAT("I:RX:A2:", analogA2);
    PR_FLOAT("I:RX:A3:", analogA3);
#endif
  } else if (state == ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    // LED_SHOW_COLOR(RX_LED, CRGB::Blue);
    // Serial.println(F("I:Si4432:RX:timeout!"));
    Serial.print(F("."));

  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    LED_SHOW_COLOR(RX_LED, CRGB::Violet);
    Serial.println(F("\nI:Si4432:RX:CRC error!"));

  } else {
    // some other error occurred
    LED_SHOW_COLOR(RX_LED, CRGB::Red);
    Serial.print(F("\nI:Si4432:RX:failed code: "));
    Serial.println(state);

  }
  LED_SHOW_COLOR(RX_LED, CRGB::Black);
}

void radioProcess() {
  buttonProcess();
  txSensor();
  rxTelemetry();
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
  Serial.println("\nI:CONSOLE");
  Serial.println("I:Use keys [+ up] [- down] [* enter] [/ esc]");
  Serial.println("I:to control the menu navigation");
  return proceed;
}

result menuSetChannel() {
  Serial.println("menuSetChannel");
  return proceed;
}

result menuSetPower() {
  Serial.println("menuSetPower");
  return proceed;
}

result menuLoopbackTest() {
  Serial.println("menuLoopbackTest");
  return proceed;
}

result menuRadioStatus() {
  PR_VALUE("\nradio freq MHz: ", radioFreq);
  PR_VALUE("bit rate kb/s: ", radioBitRateKbSec);
  PR_VALUE("TX power: ", radioPower);
  PR_VALUE("RSSI:", module->SPIgetRegValue(SI443X_REG_RSSI));
  PR_VALUE("RSSI THR:", module->SPIgetRegValue(SI443X_REG_RSSI_CLEAR_CHANNEL_THRESHOLD));
  return proceed;
}

result menuDumpRadioRegisters() {
  Serial.println("\ndump radio registers");
  for (uint8_t i = 0; i <= 0x7F; i++) {
    Serial.print("I:REG:");
    Serial.print(i, HEX);
    Serial.print(":");
    serialPrintBinary((uint8_t)module->SPIgetRegValue(i));
    // Serial.println(module->SPIgetRegValue(i), BIN);
  }
  return proceed;
}

result menuRadioScanner() {
  float minRSSI = 999;
  float minRF = 0;
  float maxRSSI = 0;
  float RSSI;
  float maxRF = 0;
  Serial.println("\nradio scanner");
  radioTask.disable();

  for (float rfCarrier = ITU_433_ISM_BAND_LOW; rfCarrier <= ITU_433_ISM_BAND_HIGH; rfCarrier += ITU_433_ISM_BAND_SPACING) {
    PR_FLOAT("check MHz: ", rfCarrier);
    radio.setFrequency(rfCarrier);
    rxTelemetry();
    Serial.print(F(" RSSI: "));
    RSSI = module->SPIgetRegValue(SI443X_REG_RSSI);
    Serial.println(RSSI);
    if (minRSSI > RSSI) { 
      minRSSI = RSSI;
      minRF = rfCarrier;
    }
    if (maxRSSI < RSSI) {
      maxRSSI = RSSI;
      maxRF = rfCarrier;
    }
  }
  PR_FLOAT("min RSSI MHz: ", minRF);
  PR_FLOAT("max RSSI MHz: ", maxRF);

  radio.setFrequency(radioFreq);
  radioTask.enable();
  return proceed;
}

//-------------------------------------------------------------------------------
void setup() {
  Serial.begin(UART_BAUDRATE);
  // needed to keep leonardo/micro from starting too fast!
  while (!Serial) { delay(10); }

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalSMD5050 );
  FastLED.setBrightness( BRIGHTNESS );

  TEST_PIN(RADIO_nSEL);
  TEST_PIN(RADIO_nIRQ);
  TEST_PIN(RADIO_SDN);
  TEST_PIN(18);
  Serial.println(F("I:Si4432:START"));
  PR("I:MOSI:", MOSI);
  PR("I:MISO:", MISO);
  PR("I:SCK:", SCK);
  PR("I:SS:", SS);
  int state = radio.begin(radioFreq, radioBitRateKbSec, radioFreqDev, radioRxBw, radioPower, radioPreambleLen);
  if (state == ERR_NONE) {
    Serial.println(F("I:Si4432:success!"));
    LED_SHOW_COLOR(RX_LED, CRGB::Green);
    LED_SHOW_COLOR(TX_LED, CRGB::Green);
  } else {
    Serial.print(F("I:Si4432:failed code: "));
    Serial.println(state);
    LED_SHOW_COLOR(RX_LED, CRGB::Red);
    LED_SHOW_COLOR(TX_LED, CRGB::Red);
    while (true);
  }
  delay(1000);
  LED_SHOW_COLOR(RX_LED, CRGB::Black);
  LED_SHOW_COLOR(TX_LED, CRGB::Black);
  buttonInit();

  runner.init();
  runner.addTask(radioTask);
  radioTask.enable();
}

void loop() {
  runner.execute();

  nav.doInput();
  if (nav.changed(0)) {
    nav.doOutput();
  }
}
