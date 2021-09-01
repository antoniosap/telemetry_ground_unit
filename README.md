# telemetry_ground_unit

## schematic

https://www.electrodragon.com/w/Si4432
Si4432: 433mHZ radio
PIN 01  GND
PIN 02  
PIN 03
PIN 04
PIN 05  3V3
PIN 06  SDO     <-- GPIO 23/MOSI
PIN 07  SDI     <-- GPIO 19/MISO
PIN 08  SCLK    <-- GPIO 18/SCK
PIN 09  NSEL    <-- GPIO 23/SS
PIN 10  NIRQ    <-- GPIO 15
PIN 11  SDN     <-- GPIO 0
PIN 12
...
PIN 13  ANTENNA
PIN 14

LOLIN 32:
RAW 5V0      <-- input from BEC
GND GND common
VCC 3V3      --> output to radio
GPIO 18/SCK  --> SLCK radio
GPIO 19/MISO --> SDI radio
GPIO 23/MOSI <-- SDO radio
GPIO 23/SS   --> NSEL radio
GPIO 15      --> NIRQ
GPIO 0       --> SDN

INGRESSI DIGITAL:
GPIO 2  <-- RED BTN = HOME POSITION - FRONT VIEV
GPIO 4  <-- BLACK BTN = COMMAND BY POTS

INGRESSI ANALOGICI 0->3V3:
ADC1_0  <-- RP = PAN
ADC1_3  <-- LP = TILT

## interfaces

MQTT messages
console menu
