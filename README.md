# Ginlong Solis solar inverters

First of all my greates thanks to the work of Hajo Noerenberg (https://github.com/hn/ginlong-solis).
Excellent piece of Work - very clean, easy to extend and really works like a charm. Wish I could do the same...
This repo is a fork of his work

## Changes
- main-goal was to add MQTT-capabilities. This part is solved. Any read register gets published via MQTT
- project is getting bloated up by those further topics
- I am feeling that the control-loop of the solis is not well done. Its quite to often oscillating by either drawing to much or to less power. And this over more then 15minutes. So I am working on adding some more control. This function is not even alpha. Best to keep this function off via #define enable_anti_oscillation_control_loop 0
- The Seplos BMS is rather poor when it somes to balancing. So I decided to add a heltec active balancer. This one can be externally controlled if on or off. So I am adding a routine using a relais to turn on/off the balancer as required. State not even alpha, so best turn it off via #define heltec_active_balancer 0
- made some minor changes with Serial.print outputs and added a debug-flag for the queried registers to be more verbose

## Preamble
Hajo made a nice chapter on how to attach the RS485 enabled ESP32. 

Attention/Caveat/Disclaimer/Your own risk
I can not remember where I was reading it, but I think you MUST attach the WLAN Datalogger Stick and have your ESP-device sitting in-between the Solis and the WLAN-Stick without any further terminating resistors - this is how I am using it.
If there is no WLAN-Stick in use, then I think one shall add a 4.7k resistor from B to ground and add another 4.7kresistor from A to +3.3V (to be confirmed)
And I am using MAX3485, which is 3.3V to make sure not destroying the ESP. Feedback welcome on this and I am by no means taking any responsible when anything is breaking apart...

## Some words to mine overall environment
- SMA PV Inverters with old/rev1 Sunny-Home-Manager. Using  https://www.unifox.at/software/sma-em-daemon/ to get it into MQTT
- Seplos BMS - Use https://github.com/byte4geek/SEPLOS_MQTT to get Seplos RS485 into MQTT
- and the Fork of Hajo's work to read the Solis via RS485 and put into MQTT
- using Telegraf plugin to subscribe to MQTT and push into Influxdb
- Use Grafana for Virtualization. Unhappy here. Need to learn how to use SVG-Grafics and Apache Echarts.

## Credits

//Hajo Noerenberg - which I would call the original
https://github.com/hn/ginlong-solis

//read SMA and publish via MQTT
[datenschuft] https://github.com/datenschuft/SMA-EM 
 
//nice writeup how to subscribe to a MQTT-topic. Required to read Seplos BMS to control the active balancer
Karl Söderby https://docs.arduino.cc/tutorials/uno-wifi-rev2/uno-wifi-r2-mqtt-device-to-device

//Reading Seplos BMS via RS485
byte4geek https://github.com/byte4geek/SEPLOS_MQTT

//parsing JSON (SEPLOS BMS)
Liz Miller https://www.learnrobotics.org/blog/parse-json-data-arduino/

//Solis register xls. very complete, including writable holding-regs
[peufeu2] https://github.com/peufeu2/GrugBus

# Disclaimer
 This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 3.0 as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along
   with this program. If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
