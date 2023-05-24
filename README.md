# Ginlong Solis solar inverters

First of all my greates thanks to the work of Hajo Noerenberg (https://github.com/hn/ginlong-solis).
Excellent piece of Work - very clean, easy to extend and really works like a charm. Wish I could do the same...
This repo is a fork of his work

## Changes
- main-goal was to add MQTT-capabilities. This part is solved. Any read register gets published via MQTT
- mostly done is the work to turn on/off my heltec balancer based on the date from the Seplos BMS.
- project is getting bloated up by those further topics
- I am feeling that the control-loop of the solis is not well done. Its quite to often oscillating by either drawing to much or to less power. And this over more then 15minutes. So I am working on adding some more control. This function is not even alpha. Best to keep this function off via #define enable_anti_oscillation_control_loop 0
- made some minor changes with Serial.print outputs and added a debug-flag for the queried registers to be more verbose

## Preamble
Hajo made a nice chapter on how to attach the RS485 enabled ESP32. 

Attention/Caveat/Disclaimer/Your own risk
I can not remember where I was reading it, but I think you MUST attach the WLAN Datalogger Stick and have your ESP-device sitting in-between the Solis and the WLAN-Stick without any further terminating resistors - this is how I am using it.
If there is no WLAN-Stick in use, then I think one shall add a 4.7k resistor from B to ground and add another 4.7kresistor from A to +3.3V (to be confirmed)
And I am using MAX3485, which is 3.3V to make sure not destroying the ESP. Feedback welcome on this and I am by no means taking any responsible when anything is breaking apart...

## MQTT Changes
Hajos Original was nicely reading the Solis via RS485. With mine changes, those values are now beeing published via MQTT:

```
pi@rpi4:~ $ mosquitto_sub -t  solis/#
Battery_current A=11.500
Battery_voltage V=54.500

If you want to debug any register (including the MQTT stuff), then chnage the false-keywird to true

  { MB_INPUTREG,  33133,  0,     10,      SDT_U16,   10,     "V",  "Battery_voltage", false },
```

After enabling debugging, serial-console is more verbose:
  
```
reading register :33133 Battery_voltage = 54.50
MQTT: Sending message to topic: solis/Battery_voltage_V
Value: Battery_voltage V=54.500
```

Not sure if it was a good idea to have 
a) for each register an own topic
b) and in addition in the message-filed to reference same topic-string.

I did this, becasue then I could easily subscribe to solis/# and each register is beeing displayed with "name" and "value".

If you want to chnage it, then just visit the function "void publish_mqtt()". variable topic and message are beeing defined including the publish.

## Heltec Balancer
I can not yet state if this balancer is doing any good. By my Seplos passive balancer does not convince me. My cheap Solis Inverter can charge with more then 50Amps and the cells drift quite away when SOC is in the 90's%.
The cool think on the Heltec Active Balancer is its price and the capability to turn it on/off via switch.
So using this sketch I am controlling a relais based on charging-state, voltage-difference from weakest and strongest cell, with a small hystersys.

The overall flow could be described as:
- I am using github.com/byte4geek/SEPLOS_MQTT to read the Seplos BMS and publish it into MQTT
- this sketch subscribes to MQTT and derive 3 variables from it:
    - function: void parse_seplos_json_string (char *my_seplos_json_char)
    - bms_measured_cell_diff = doc["difference"].as<float>();
    - highest_cell_v = doc["highest_cell_v"].as<float>();
    - charge_discharge = doc["charge_discharge"].as<float>();
  
**Control Loop Heltec**
- turn on heltec balancer
   - Cell with highest volatge must be larger as trigger_heltec_on
   - and the the last charging-activity must be within the last max_balance_time_in_h_after_last_charge hours
- In short, we did charge and one cell is high enough to enjoy balancing
  
- turn off the balancer
  - the easy part is, to just turn it off if the difference between highest cell voltage and lowest cell voltage < trigger_heltec_off
  - we have a hysteresis. trigger_heltec_off < trigger_heltec_on value
  - if the charging time is more then max_balance_time_in_h_after_last_charge, heltec is turned off if we are below the initial balance-threshold (trigger_heltec_on)
  
Well, control-loop needs to be tested. Hence the code has the option to disable and and turn-on debug-info:
- #define heltec_active_balancer 0 
- #define debug_heltec_active_balancer 1
                                                                                                                                     
                                                                                                                                     




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

//excellent example on how to parse JSON
https://arduinojson.org/v6/example/parser/

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
