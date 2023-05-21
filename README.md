# Ginlong Solis solar inverters

First of all my greates thanks to the work of Hajo Noerenberg (https://github.com/hn/ginlong-solis).
Excellent piece of Work - very clean, easy to extend and really works like a charm. Wish I could do the same...
This repo is a fork of his work

## Changes
- main-goal was to add MQtt-capabilities. This part is solved. Any read register gets published via MQTT
- project is getting bloated up by those further topics
- I am feeling that the control-loop of the solis ist not well done. its quite to often oscillating by either drawing to much or to less power. And this over more then 15minutes. So I am working on adding some more control. This function is not even alpha. Best to keep this function off via #define enable_anti_oscillation_control_loop 0
- The Seplos is rather poor when it somes to balancing. So I decided to add a heltec active balancer. This one can be externally controlled if on or off. So I am adding a routine using a relais to turn on/off the balancer as required. state not even alpha, so best turn it off via #define heltec_active_balancer 0
- made some minor chnages with Serial.print outputs and added a debug-flag for the queried registers to be more verbose

## Preamble
Hajo made a nice chapter on how to attach the RS485 enabled ESP32. 

Attention/Caveat/Disclaimer/Your own risk
I can not remember where I was reading it, but I think you MUST attach the WLAN Datalogger Stick and have your ESP-device sitting in-between. This is what I am doing.
If there is no WLAN-Stick in use, then I think one shall add a 4.7k resistor from B to ground and add another 4.7kresistor from A to +3.3V
And I am using MAX3485, which is 3.3V to make sure not destroying the ESP. Feedback welcome on this and I am by no means taking any responsible when anything is breaking apart...

## Some words to mine overall environment
- SMA PV Inverters with old/rev1 Sunny-Home-Manager. Using  https://www.unifox.at/software/sma-em-daemon/


## Credits

- [datenschuft] https://github.com/datenschuft/SMA-EM 
- 
# further credits
//taken mqtt_subscribe to read the Seplso values
Karl Söderby https://docs.arduino.cc/tutorials/uno-wifi-rev2/uno-wifi-r2-mqtt-device-to-device

//Reading Seplos BMS via RS485
byte4geek https://github.com/byte4geek/SEPLOS_MQTT

//parsing JSON (SEPLOS BMS)
Liz Miller https://www.learnrobotics.org/blog/parse-json-data-arduino/

//Solis register xls. very complete, including writable holding-regs
[peufeu2] https://github.com/peufeu2/GrugBus
