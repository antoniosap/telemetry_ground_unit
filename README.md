# telemetry_ground_unit

## schematic

https://www.electrodragon.com/w/Si4432
Si4432: 433mHZ radio SPI SLAVE
PIN 01  GND
PIN 02  
PIN 03
PIN 04
PIN 05  3V3
PIN 06  SDO     --> GPIO 19/MISO
PIN 07  SDI     <-- GPIO 23/MOSI
PIN 08  SCLK    <-- GPIO 18/SCK
PIN 09  NSEL    <-- GPIO 5/SS
PIN 10  NIRQ    <-- GPIO 15
PIN 11  SDN     <-- GPIO 13
PIN 12
...
PIN 13  ANTENNA
PIN 14

LOLIN 32 SPI MASTER:
RAW 5V0      <-- input from BEC
GND GND common
VCC 3V3      --> output to radio
GPIO 18/SCK  --> SLCK radio
GPIO 19/MISO <-- SDO radio
GPIO 23/MOSI --> SDI radio
GPIO 5/SS    --> NSEL radio + ON BOARD LED
GPIO 15      --> NIRQ
GPIO 13      --> SDN

INGRESSI DIGITAL:
GPIO 2  <-- RED BTN = HOME POSITION - FRONT VIEV
GPIO 4  <-- BLACK BTN = COMMAND BY POTS

INGRESSI ANALOGICI 0->3V3:
ADC1_0  <-- RP = PAN
ADC1_3  <-- LP = TILT

USCITE LED BAR RGB WS2812:
3V3         VIN
GPIO 33 --> DI
GND         GND

COLOR CODING:
NUMBER  FUNCTION            COLOR
LED 0 RSSI                  0 -> 6
LED 1 BAT ground base
LED 2 BAT mobile 1
LED 3 BAT mobile 2
LED 4 WIFI connected        BLUE = OFF GREEN = OK
LED 5 BLE connected         BLUE = OFF GREEN = OK
LED 6 RX PACKET             GREEN = OK
LED 7 TX PACKET             GREEN = OK

HUE:
0 VIOLET #8000ff 1 BLUE #0000ff 2 PINK #ff00ff 3 RED #ff0000 4 ORANGE #ff8000 5 YELLOW #ffff00 6 GREEN #00ff00 

## interfaces

MQTT messages
console menu

SPI BUS 1 CONFIGURATION:
I:MOSI:23
I:MISO:19
I:SCK:18
I:SS:5

## tools espressIF

https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension
https://blog.espressif.com/how-to-use-custom-partition-tables-on-esp32-69c0f3fa89c8
https://docs.platformio.org/en/latest/platforms/espressif32.html#partition-tables
https://github.com/espressif/arduino-esp32/tree/master/tools/partitions