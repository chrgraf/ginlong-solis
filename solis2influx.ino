/*

   solis2mqtt.ino

   this is a fork from the superb code of Hajo Noerenberg
   Use at own Risk. See below disclaimer 

   main changes
   -------------
   - added if statement did not detect the Solis solis-rhi-3k-48es-5g 48V battery-inverter
      -// (not true any longer..) } else if ((regvalue) == 8241) {        // RS485_MODBUS solis-rhi-3k-48es-5g
      - hardcoded the script to always run in ESVinv mode
   - removed influx
   - added export to MQTT 
   - modified the WLAN-setup. but unhappy with my own changes. need to improve here...
   - added some more registers to read and commented some out
   - added relais-output to turn on/off heltec active balancer (work in progress)
   - added a control-loop. not even alpha state..
           its a longer story. my solis heavily oscillates by charging to heavy. 
           so it takes power from grid during charge, realizes that it overdrains. 
           then toggles to discharge. from dicharge toggle to heavy charge,..
           Solution I am taking is to write register 43117 (which limits the charging power).
           this register seemes to be stored in eeprom, as it survives power-down
           eeprom has limited write-cycles. so we need to make sure writing only 2-3 times a day...
           in mine case, the solis oscillates over 15min duration and jumps from 1000W discharge to 1000W charge with decent frequency
           Control-loop recovery from a low-charge value to high-charge value seems working.
           Lowering-charging-power not yet..
           I am not a coder, so for sure my code needs a clean-up.

       - control-llop is actually disabled. refer to readme, if I feel it can be enabled. for now, keep it disabled
            - #define enable_anti_oscillation_control_loop 0
            I refined a second time the 3 registers via "osci_registers"for the control-loop. This is redundant and bad idea. Need to clean-up here as well.
       - Hajos Code is perfect cleaned-up and structed. Mine is far away from this...

   Hajo just added in the meantime support for the solis-rhi-3k-48es-5g inverter. if you just need this, then I recommned to stay with Hajos version
   Overall. MQTT works well.

   Overall, I am happy for any feedback.
   

   thanks

   btw, the most complete register-description I found on solis is here:
   https://github.com/peufeu2/GrugBus
   look here in the csv-files: https://github.com/peufeu2/GrugBus/tree/main/grugbus/devices



   source
   --------
   ESP8266 gateway to read Ginlong Solis inverter stats and statistics and push to influxdb

   (C) 2022 Hajo Noerenberg

   http://www.noerenberg.de/
   https://github.com/hn/ginlong-solis

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 3.0 as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along
   with this program. If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.

# further credits
//taken mqtt_subscribe to read the Seplso values
Karl Söderby https://docs.arduino.cc/tutorials/uno-wifi-rev2/uno-wifi-r2-mqtt-device-to-device

//Reading Seplos BMS via RS485
byte4geek https://github.com/byte4geek/SEPLOS_MQTT

//parsing JSON (SEPLOS BMS)
Liz Miller https://www.learnrobotics.org/blog/parse-json-data-arduino/
*/

//most visible chrgraf changes
#define enable_anti_oscillation_control_loop 0   // my solis is osciallation and having issues with its own control-loop. this function shoud give some enhancement to it
#define oscillation_debug 0                      //oscillation-stuff is not even alpha. enabling this add more verbosity for debugging. (1_ enable, (0) disable)
#define heltec_active_balancer 1                 // enables the capability to control the active-balancer via relais  

#define RETRY_ATTEMPTS 5                        // Number of retries to connect to WiFi and MQTT
#define ACTIVE_BALANCER_RELAIS_PIN  23         // pin to control the relais for active-balancer

#define MODBUSPINRX       16  //RX2==R0       esp32:GPIO16     esp8266:d7/gpio13
#define MODBUSPINTX       17  //TX2==DI       ESP32:GPIO17     ESP8266:D8/GPIO15
#define MODBUSPINENATX    18  // enable only if your RS485 adapter requires a TX enable pin */
#define MODBUSBAUD        9600
#define MODBUSINVERTERID  1   // wlan datalogger stick expects the inverter to be address 1

#define SNHEXWORDS        4
#define SNASCWORDS        16

#include <WiFi.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <ArduinoMqttClient.h>

//wlan
const char* ssid_home = "TP-LINK-ganser24";
const char* password_home = "mamaandpapa24";
int retry_counter = 30;                                       //this sketch will not forever try to get wlan up... same for the mqtt-connect
const int retry_interval_wlan = 500;

//mqtt
WiFiClient espClient;
MqttClient mqttClient(espClient);
//IP Address for MQTT Broker here. Will work with local name however more reliable with static IP
const char broker[] = "192.168.178.116"; //"core-mosquitto";
int        port     = 1883; //your port here
const char mqtt_user[] = "";
const char mqtt_pw[] = "";

int count = 0;
const char mqtt_topic[] = "solis";

/////////////////////////
//oscillation-vars - START
/////////////////////////
//somehow double-defintion of registers. in this section those regisyers are only be used for the anti-osciallation control-loop
typedef struct 
{
  char     name[40];
  uint16_t my_register;
  uint8_t  my_register_len;
  int      conversion;
  float    value;
}osci_registers;

osci_registers reg_33149 {"read_battery_power_ampere",33149,2,1,0};
osci_registers reg_33263 { "Meter_total_active_power", 33263,2,1,0};
osci_registers reg_43117 {"write_max_batt_charge",43117,1,10,0};

int ampere_values[]={8,16,40};             //charging-limits in ampere. the control-loop will iterate through those values
                                           //max-value must be most right and shall be the desired maximum allowed
                                           // starting with zero should work, but not tested.
int ampere_steps = (sizeof(ampere_values) / sizeof(ampere_values[0])-1);  //subtract 1, as we count from 0
int actual_choosen_ampere_index;

int control_loop_counter=0;                 // counter to reach loop_iterations 
int overshoots=0;                           //  counts if power-send-to-grid is higher then allowed_oscillations_power, means to much discharging
int undershoots=0;                          // opposite of overshoots
#define loop_iterations 30                  // number of loop-runs where we decide if we osciallte to heavy or not
#define allowed_oscillations 4              // absolute counter. 
#define allowed_oscillations_power 300      // deviation from zero. Exceeding  this value to count as oscillation and counts against allowed_oscillations
#define power_to_grid_needed_for_recovery 900      // we need to have at least xWatt exported to grid, before we allow to recover to higher charging values
#define wait_seconds_recovery             900  //if we reduced charging-power, then at least we have to wait x seconds before we can recover from it 
float timestamp_power_change=0;            // timestamp storing when last power-reduce was done
/////////////////////////
//osciallation-vars - END
/////////////////////////

/////////////////////////
//active balancer relais
/////////////////////////
//The Heltec-balancer can be controlled via a switch if enabled or not. 
//this sketch turns on/off the balancer as needed
// roughly turn on when charging and cell diff-voltage must be greater then 10mv
//turn off after min 2h balancing and if diff < 10mv
//values are aquired via subscribing to MQTT - in mine case a seplos BMS which has as well a MQTT-publish
//hence this routine might mostly be very specific to my own environment
float time_stamp_balance_went_on = 0;
float cell_diff;
const char subscribed_topic[]  = "seplos";


enum solisdatatype {
  SDT_U16,
  SDT_U32,
  SDT_S16,
  SDT_S32,
  SDT_H16,      // Hex number
  SDT_ITYPE,    // Inverter type definition
  SDT_SNHEX,    // Serial number, 4 words hex encoded
  SDT_SNASC,    // Serial number, 16 words direct ASCII
  SDT_DATETIME, // Composed date and time
  SDT_APP1,     // Appendix 1 - Product model
  SDT_APP2,     // Appendix 2 - Inverter status
  SDT_APP3,     // Appendix 3 - Grid standard
  SDT_APP4,     // Appendix 4 - Power curve number
  SDT_APP5,     // Appendix 5 - Fault status
  SDT_APP6,     // Appendix 6 - Working status
  SDT_APP7,     // Appendix 7
  SDT_APP8,     // Appendix 8 - Setting flag
};

enum modbusobjecttype {
  MB_COIL,            // 1 bit r/w   func 0x05 read - Solis '0X' type
  MB_DISCRETEINPUT,   // 1 bit r     func 0x02 read - Solis '1X' type
  MB_INPUTREG,        // 16 bit r    func 0x04 read - Solis '3X' type
  MB_HOLDINGREG,      // 16 bit r/w  func 0x03 read, 0x06 write and 0x10 write multiple - Solis '4X' type
};

typedef struct {
  modbusobjecttype modbusobject;
  unsigned int modbusaddr;
  unsigned int modbusoffset;
  unsigned int readdelay;           // poll frequency in seconds, -1 to disable
  solisdatatype datatype;
  unsigned int datadiv;
  const char *dataunit;
  const char *dataname;
  const bool debug;
} solisreg;

/* RS485_MODBUS map while inverter type is unknown */
const solisreg solisUNKNOWN[] = {
  { MB_INPUTREG,  35000,  0,     20,    SDT_ITYPE,    1,      "",  "Solis inverter type definition" },
  {}
};

/* RS485_MODBUS (INV-3000ID EPM-36000ID) inverter protocol */
const solisreg solisINV[] = {
  { MB_INPUTREG,   3000,  1,    300,     SDT_APP1,    1,      "",  "Product model", false },
  { MB_INPUTREG,   3005,  1,     60,      SDT_U32,    1,     "W",  "Active Power", false },
  { MB_INPUTREG,   3007,  1,     60,      SDT_U32,    1,     "W",  "Total DC output Power", false },
  { MB_INPUTREG,   3009,  1,     60,      SDT_U32,    1,   "kWh",  "Total energy", false },
  { MB_INPUTREG,   3011,  1,     90,      SDT_U32,    1,   "kWh",  "Energy this month", false },
  { MB_INPUTREG,   3013,  1,     90,      SDT_U32,    1,   "kWh",  "Energy last month", false },
  { MB_INPUTREG,   3015,  1,     90,      SDT_U16,   10,   "kWh",  "Energy today", false },
  { MB_INPUTREG,   3016,  1,     90,      SDT_U16,   10,   "kWh",  "Energy last day", false },
  { MB_INPUTREG,   3022,  1,     30,      SDT_U16,   10,     "V",  "DC voltage 1", false },
  { MB_INPUTREG,   3023,  1,     30,      SDT_U16,   10,     "A",  "DC current 1", false },
  { MB_INPUTREG,   3024,  1,     30,      SDT_U16,   10,     "V",  "DC voltage 2", false },
  { MB_INPUTREG,   3025,  1,     30,      SDT_U16,   10,     "A",  "DC current 2", false },
  { MB_INPUTREG,   3042,  1,     60,      SDT_U16,   10,    "°C",  "Inverter temperature", false },
  { MB_INPUTREG,   3043,  1,    120,      SDT_U16,  100,    "Hz",  "Grid frequency", false },
  { MB_INPUTREG,   3044,  1,     60,     SDT_APP2,    1,      "",  "Inverter status", false },
  { MB_INPUTREG,   3045,  1,     -1,      SDT_S32,    1,     "W",  "Limit active Power adjustment rated Power", false },
  { MB_INPUTREG,   3047,  1,     -1,      SDT_S32,    1,   "Var",  "Limit reactive Power adjustment rated Power", false },
  { MB_INPUTREG,   3050,  1,     -1,      SDT_U16,  100,     "%",  "Power limit actual", false },
  { MB_INPUTREG,   3054,  1,    300,     SDT_APP3,    1,      "",  "Country standard code", false },
  { MB_INPUTREG,   3056,  1,     -1,      SDT_S32,    1,   "Var",  "Reactive Power", false },
  { MB_INPUTREG,   3061,  1,    120,    SDT_SNHEX,    1,      "",  "Inverter SN", false },
  { MB_INPUTREG,   3072,  1,     60,     SDT_APP6,    1,      "",  "Working status", false },
  { MB_INPUTREG,   3073,  1,    300, SDT_DATETIME,    1,      "",  "System Time", false },
  { MB_INPUTREG,   3084,  1,     -1,      SDT_S32,    1,     "W",  "Meter active Power", false },
  {}
};

/* RS485_MODBUS (ESINV-33000ID) energy storage inverter protocol */
// modbusobjecttype modbusobject;  unsigned int modbusaddr;  unsigned int modbusoffset;  unsigned int readdelay; // poll frequency in seconds, -1 to disable  
// solisdatatype datatype;   unsigned int datadiv;   const char *dataunit;   const char *dataname;
const solisreg solisESINV[] = {
  { MB_INPUTREG,  33000,  0,     -1,     SDT_APP1,    1,      "",  "Model_no", false },
  { MB_INPUTREG,  33004,  0,     -1,    SDT_SNASC,    1,      "",  "Inverter_SN", false },
  { MB_INPUTREG,  33035,  0,     -1,      SDT_U16,   10,   "kWh",  "Today_energy_generation", false },
  { MB_INPUTREG,  33036,  0,     -1,      SDT_U16,   10,   "kWh",  "Yesterday_energy_generation", false },
  { MB_INPUTREG,  33049,  0,     -1,      SDT_U16,   10,     "V",  "DC_voltage_1", false },
  { MB_INPUTREG,  33050,  0,     -1,      SDT_U16,   10,     "A",  "DC_current_1", false },
  { MB_INPUTREG,  33051,  0,     -1,      SDT_U16,   10,     "V",  "DC_voltage_2", false },
  { MB_INPUTREG,  33052,  0,     -1,      SDT_U16,   10,     "A",  "DC_current_2", false },
  { MB_INPUTREG,  33053,  0,     -1,      SDT_U16,   10,     "V",  "DC_voltage_3", false },
  { MB_INPUTREG,  33054,  0,     -1,      SDT_U16,   10,     "A",  "DC_current_3", false },
  { MB_INPUTREG,  33055,  0,     -1,      SDT_U16,   10,     "V",  "DC_voltage_4", false },
  { MB_INPUTREG,  33056,  0,     -1,      SDT_U16,   10,     "A",  "DC_current_4", false },
  { MB_INPUTREG,  33057,  0,     -1,      SDT_U32,    1,     "W",  "Total_DC_output_Power", false },
  { MB_INPUTREG,  33071,  0,     -1,      SDT_U16,   10,     "V",  "DC_bus_voltage", false },
  { MB_INPUTREG,  33073,  0,     -1,      SDT_U16,   10,     "V",  "A_Phase_Volt", false },
  { MB_INPUTREG,  33074,  0,     -1,      SDT_U16,   10,     "V",  "B_Phase_Volt", false },
  { MB_INPUTREG,  33075,  0,     -1,      SDT_U16,   10,     "V",  "C_Phase_Volt", false },
  { MB_INPUTREG,  33076,  0,     -1,      SDT_U16,   10,     "A",  "A_Phase_Current", false },
  { MB_INPUTREG,  33077,  0,     -1,      SDT_U16,   10,     "A",  "B_Phase_Current", false },
  { MB_INPUTREG,  33078,  0,     -1,      SDT_U16,   10,     "A",  "C_Phase_Current", false },
  { MB_INPUTREG,  33079,  0,      5,      SDT_S32,    1,     "W",  "Active_Power", false },
  { MB_INPUTREG,  33093,  0,     30,      SDT_U16,   10,    "°C",  "Inverter_temperature", false },
  { MB_INPUTREG,  33094,  0,     30,      SDT_U16,  100,    "Hz",  "Grid_frequency", false },
  { MB_INPUTREG,  33104,  0,     30,      SDT_U16,    1,     "%",  "Limited_Power_actual_value", false },
  { MB_INPUTREG,  33133,  0,     10,      SDT_U16,   10,     "V",  "Battery_voltage", false },
  { MB_INPUTREG,  33134,  0,      5,      SDT_S16,   10,     "A",  "Battery_current", false },
  { MB_INPUTREG,  33135,  0,     -1,      SDT_U16,    1,     "",  "(0)Charge/(1)Dischrage", false },
  { MB_INPUTREG,  33139,  0,     60,      SDT_U16,    1,     "%",  "Battery_capacity_SOC", false },
  { MB_INPUTREG,  33140,  0,   3600,      SDT_U16,    1,     "%",  "Battery_health_SOH", false },
  { MB_INPUTREG,  33141,  0,     30,      SDT_U16,  100,     "V",  "Battery_voltage_BMS", false },
  { MB_INPUTREG,  33142,  0,      5,      SDT_S16,   10,     "A",  "Battery_current_BMS", false },
  { MB_INPUTREG,  33143,  0,     30,      SDT_U16,   10,     "A",  "Battery_charge_limitation_bms", false },
  { MB_INPUTREG,  33143,  0,     30,      SDT_U16,   10,     "A",  "Battery_discharge_limitation_bms", false },
  { MB_INPUTREG,  33147,  0,     -1,      SDT_U16,    1,     "W",  "Household_load_Power", false },
  { MB_INPUTREG,  33149,  0,     5,       SDT_S32,    1,     "W",  "Battery_Power", false },
  { MB_INPUTREG,  33151,  0,     -1,      SDT_S32,   -1,     "W",  "Grid_Power_pos_to_Grid", false },
  { MB_INPUTREG,  33163,  0,     30,      SDT_U16,   10,   "kWh",  "Today_battery_charge_energy", false },
  { MB_INPUTREG,  33164,  0,     36000,   SDT_U16,   10,   "kWh",  "Yesterday_battery_charge_energy", false },
  { MB_INPUTREG,  33167,  0,     60,      SDT_U16,   10,   "kWh",  "Today_battery_discharge_energy", false },
  { MB_INPUTREG,  33169,  0,     60,      SDT_U16,   10,   "kWh",  "Today_load_energy_consumption", false },
  { MB_INPUTREG,  33171,  0,     30,      SDT_U16,   10,   "kWh",  "Today_energy_imported_from_Grid", false },
  { MB_INPUTREG,  33172,  0,     36000,   SDT_U16,   10,   "kWh",  "Yesterday_energy_imported_from_Grid", false },
  { MB_INPUTREG,  33175,  0,     30,      SDT_U16,   10,   "kWh",  "Today_energy_fed_into_Grid", false },
  { MB_INPUTREG,  33176,  0,     3600,    SDT_U16,   10,   "kWh",  "Yesterday_energy_fed_into_Grid", false },
  { MB_INPUTREG,  33179,  0,     30,      SDT_U16,   10,   "kWh",  "Today_load_energy_consumption", false },
  { MB_INPUTREG,  33180,  0,     36000,   SDT_U16,   10,   "kWh",  "Yesterday_load_energy_consumption", false },
  { MB_INPUTREG,  33204,  0,     10,      SDT_U16,   1,     "",  "Charge_Direction", false },
  { MB_INPUTREG,  33263,  0,     -1,      SDT_S32,   1000, "KW",  "Meter_total_active_Power_Grid", false },
  { MB_INPUTREG,  33285,  0,     120,     SDT_U32,   1000, "KW",  "Meter_total_active_energy_to_Grid", false },
  { MB_INPUTREG,  33251,  0,     30,      SDT_U16,   10, "V",  "Meter_ac_voltage_A", false },
  { MB_INPUTREG,  33252,  0,     30,      SDT_U16,   100, "A",  "Meter_ac_current_A", false },
  { MB_INPUTREG,  33253,  0,     30,      SDT_U16,   10, "V",  "Meter_ac_voltage_B", false },
  { MB_INPUTREG,  33254,  0,     30,      SDT_U16,   100, "A",  "Meter_ac_current_B", false },
  { MB_INPUTREG,  33255,  0,     30,      SDT_U16,   10, "V",  "Meter_ac_voltage_C", false },
  { MB_INPUTREG,  33256,  0,     30,      SDT_U16,   100, "A",  "Meter_ac_current_C", false },
  { MB_INPUTREG,  33263,  0,     5,      SDT_U32,   1, "W",  "Meter_total_active_power", false },
  { MB_HOLDINGREG,  43117,  0,   120,     SDT_U16,   10, "A",  "Battery_Charge_current_max", false },
  {}
};



SoftwareSerial modbusSerial(MODBUSPINRX, MODBUSPINTX);
ModbusMaster modbus;

unsigned long readlast[(sizeof(solisINV) > sizeof(solisESINV) ? sizeof(solisINV) : sizeof(solisESINV) ) / sizeof(solisreg)];
char serialnumber[4 * SNHEXWORDS + 1];
int serialvalid = 0;
//const solisreg *solis = solisUNKNOWN;
const solisreg *solis = solisESINV;

#ifdef MODBUSPINENATX
void ModbusPreTransmission() {
  digitalWrite(MODBUSPINENATX, 1);
}

void ModbusPostTransmission() {
  digitalWrite(MODBUSPINENATX, 0);
}
#endif


/*
turn on/off of the heltec active balancer
 - subscribes to mqtt topic seplos (https://github.com/byte4geek/SEPLOS_MQTT)
 - its a json format, 
    -mosquitto_sub -t  seplos/#
   {"lowest_cell":"Cell 15 - 3306 mV","lowest_cell_v":"3306","lowest_cell_n":"15","highest_cell":"Cell 9 - 3309 mV","highest_cell_v":"3309","highest_cell_n":"9","difference":"3","cell01":"3309","cell02":"3307","cell03":"3308","cell04":"3309","cell05":"3309","cell06":"3309","cell07":"3309","cell08":"3309","cell09":"3309","cell10":"3309","cell11":"3307","cell12":"3309","cell13":"3307","cell14":"3308","cell15":"3306","cell16":"3308","cell_temp1":"22.7","cell_temp2":"22.7","cell_temp3":"23.0","cell_temp4":"23.0","env_temp":"26.2","power_temp":"26.4","charge_discharge":"-18.95","total_voltage":"52.93","residual_capacity":"239.02","soc":"85.3","cycles":"10","soh":"100.0","port_voltage":"52.85"}
   - start balancing if charging is active and min-max cell-diff > 10mv
   - stop balancing after2h and if cell-diff < 10mv

- functiononMqttMessage displays subscribed topic

*/


void onMqttMessage(int messageSize) {
  String messageTemp;
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    //Serial.print((char)mqttClient.read());
    messageTemp += (char)mqttClient.read();
    yield();
  }

  Serial.println(messageTemp);
  
}


bool get_oscillation_regs () {
uint8_t mbreadresult;
  // {"read_battery_power_ampere",33149,2}; 
  mbreadresult = modbus.readInputRegisters(reg_33149.my_register,reg_33149.my_register_len);
  if (mbreadresult != modbus.ku8MBSuccess) return false;
  reg_33149.value = (long)(modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1);


  //{ "Meter_total_active_power", 33263,2}; 
  mbreadresult = modbus.readInputRegisters(reg_33263.my_register,reg_33263.my_register_len);
  if (mbreadresult != modbus.ku8MBSuccess) return false;
  reg_33263.value = (long)(modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1);

  //lets read the register{"write_max_batt_charge",43117,1}; 
  mbreadresult = modbus.readHoldingRegisters(reg_43117.my_register,reg_43117.my_register_len);
  if (mbreadresult != modbus.ku8MBSuccess) return false;
  reg_43117.value = (long)(modbus.getResponseBuffer(0) / (float) reg_43117.conversion);

return true;
}



void recover_to_full_operation() {
    #if (oscillation_debug) 
       Serial.println("=============================================");
       Serial.println  ("recover_to_full_operation");
       Serial.print  (reg_33263.value);
       Serial.print (" > ");
       Serial.println  (power_to_grid_needed_for_recovery);
       Serial.print  (reg_43117.value);
       Serial.print (" < ");
       Serial.println  (ampere_values[ampere_steps]);
       Serial.println (" ampere  values:  ");
       Serial.println (ampere_values[0]);
       Serial.println (ampere_values[1]);
       Serial.println (ampere_values[2]);
       Serial.println (ampere_steps);
       Serial.println("=============================================");
     #endif
uint8_t mbreadresult;
uint8_t mbwriteresult;
char write_value_string[12];
//conditions required for recovery
// - registers must be aquired (in main loop done)
// - time-interval must be ok (in main loop done)
// - enough power on grid (33263 register)
// - 43117 register is not yet set to maximum, so we can still increase to higher level


if ( reg_33263.value > power_to_grid_needed_for_recovery  && reg_43117.value < ampere_values[ampere_steps]) {
  //so all is true, lets increase the allowed charging-power
        //iterate through the available steps, take next higer one
       #if (oscillation_debug) 
             Serial.println(">>>>>>>>>============================================<<<<<<<<<<<=");
             Serial.println( reg_33263.value);
             Serial.println( power_to_grid_needed_for_recovery);
             Serial.println( reg_43117.value);
             Serial.println( ampere_values[ampere_steps]);
             Serial.println(">>>>>>>>>============================================<<<<<<<<<<<=");
       #endif



         for (int i = 0; i<= ampere_steps; ++i) {
               
               //lets find the most highest value which is below the actual one configured
               #if (oscillation_debug)
                   Serial.print ("in the loop, ampere_value: ");
                   Serial.println (ampere_values[i]);
               #endif
               if (ampere_values[i]> reg_43117.value ) {
                  uint16_t write_value=ampere_values[i] * reg_43117.conversion;     // is a 0.1 value, if we want 3 ampere, we need to set 30 
                  timestamp_power_change = millis();
                  // write the new value 
                  mbwriteresult=modbus.writeSingleRegister(reg_43117.my_register, write_value);  //not checking if successful, next run will fix it
                  #if (oscillation_debug)
                    Serial.println(">>>>>>>>>=============================================");
                    Serial.println(write_value);
                    Serial.println(">>>>>>>>>=============================================");
                  #endif
                  char topic[]="solis/oscillations";
                  char message[50]="write_Battery_Charge_current_max A=";
                  dtostrf(ampere_values[i], -2, 0, write_value_string);
                  strcat (message, write_value_string);
                  mqttClient.beginMessage(topic);
                  mqttClient.print(message);
                  mqttClient.endMessage();
                  break;
               }
         }
}
}

void detect_oscillation() {
  // detecting oscialltions by reading register 33263 - meter-measurement grid 
  // (before I used 33204) if charging==0 or discharging==1 is ongoing
  uint8_t mbreadresult;
  uint8_t mbwriteresult;
  char topic[50];
  char message[50];
  char regvalue_string[12];
  char write_value_string[50];

  #if (oscillation_debug)
    Serial.println ("detect oscillations");
    Serial.println ("-------------------");
  #endif
  // loop until loop_iterations is reached
   if (control_loop_counter < loop_iterations) {
      ++control_loop_counter;
     //count under/overshoots
      if (reg_33263.value >=allowed_oscillations_power) { 
        ++overshoots;
      } 
      if (reg_33263.value<=-allowed_oscillations_power) { 
        ++undershoots;
      }    
    }
    else {
    //counter has reached its max== loop_iterations, then lets decide if oscillation is ongoing
      int seen_oscillations=min(overshoots,undershoots);
      #if (oscillation_debug)
        Serial.print ("Result of the osciallation_detection   ");
        Serial.println (seen_oscillations);
      
      
      //mqtt for debug only
      strcpy (topic, mqtt_topic);
      strcat (topic,"/oscillations");
      
      mqttClient.beginMessage(topic);
      dtostrf(seen_oscillations, -2, 0, regvalue_string);
      strcpy (message,"seen_oscillations =");
      strcat (message, regvalue_string);
      mqttClient.print(message);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic);
      dtostrf(overshoots, -2, 0, regvalue_string);
      strcpy (message,"overshoots oc=");
      strcat (message, regvalue_string);
      mqttClient.print(message);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic);
      dtostrf(undershoots, -2, 0, regvalue_string);
      strcpy (message,"undershoots uc=");
      strcat (message, regvalue_string);
      mqttClient.print(message);
      mqttClient.endMessage();
      #endif
      
      
      // if to many oscillations, then lower the charge value by changing register 43117
      if (seen_oscillations >= allowed_oscillations)  {
          //oscillation seen/ now lets search the next lowest value to program
          if (reg_43117.value!=ampere_values[0]) {
              // not yet minimum written. so we can lower the setting
          
              Serial.println("----------------------------unequal yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy");
              //loop through array of ampere_values, check which one is actually choosen
              for (int i = ampere_steps; i >=0 ; i--) {
                 //lets find the most highest value which is below the actual one configured
                 if (ampere_values[i]<reg_43117.value ) {
                   uint16_t write_value=ampere_values[i] * reg_43117.conversion;     // is a 0.1 value, if we want 3 ampere, we need to set 30 
                   timestamp_power_change = millis();
                   // write the new value 
                   mbwriteresult=modbus.writeSingleRegister(reg_43117.my_register, write_value);
                   //write_register(reg_43117.my_register,reg_43117.my_register_len, write_value);
                   char topic[]="solis/oscillations";
                   char message[50]="write_Battery_Charge_current_max A=";
                   dtostrf(ampere_values[i], -2, 0, write_value_string);
                   strcat (message, write_value_string);
                   mqttClient.beginMessage(topic);
                   mqttClient.print(message);
                   mqttClient.endMessage();
                  }
              }
              // reset all counters
              overshoots=0;
              undershoots=0;
              control_loop_counter=0;
          }
      }  
     // only for testing, enforces to write register independant of oscilattion ongoing
     // write_register(reg_43117.my_register,reg_43117.my_register_len);
    }
}


void setup_wifi(const int retry_interval, const char* nSSID = nullptr, const char* nPassword = nullptr)
{


    Serial.print("Connecting to WLAN ");
    if(nSSID) {
        WiFi.begin(nSSID, nPassword);
        Serial.println(nSSID);
    }
 
    while(WiFi.status()!= WL_CONNECTED)
    {
        delay(retry_interval);
        Serial.print(".");
    }
    Serial.println(" ");
    Serial.print ("connected to wlan. ");
    Serial.print ("IP address: ");
    Serial.println(WiFi.localIP()); 
    Serial.println("");
      
}


void connect_to_wlan() {

      while (WiFi.status() != WL_CONNECTED) {
        Serial.println("trying to connect to wlan tp-link-ganser");
        setup_wifi (retry_interval_wlan,ssid_home,password_home);
      } 
}

void connect_mqtt () {
//MQTT Connection here
  int i = 0;
  //if (!mqttClient.connect(broker, port)) {
    if (!mqttClient.connected()) {
      // Each client must have a unique client ID
      mqttClient.setId("SolisInv");
      // You can provide a username and password for authentication
      mqttClient.setUsernamePassword(mqtt_user, mqtt_pw);      // user/pw
      Serial.print("Attempting to connect to the MQTT broker: ");
      Serial.println(broker);
      while (!mqttClient.connect(broker, port) && i++ < retry_counter) {
         delay(500);
         Serial.print(".");
      }
      #if (heltec_active_balancer)
        // set the message receive callback
        mqttClient.onMessage(onMqttMessage);
        Serial.print("Subscribing to topic: ");
        Serial.println(subscribed_topic);
        Serial.println();
        // subscribe to a topic
        mqttClient.subscribe(subscribed_topic);
      #endif
  //Serial.println("You're connected to the MQTT broker!");
  //Serial.println();

}
}





void publish_mqtt (const char  *dataname, const char  *dataunit, const float regvalue, const bool debug) {

   //MQTT
    char topic[50];
    strcpy (topic, mqtt_topic);
    strcat (topic,"/");
    strcat (topic,dataname);
    strcat (topic,"_");
    strcat (topic,dataunit);

    char message[50];
    char regvalue_string[12];
    strcpy (message,dataname);
    strcat (message," ");
    strcat (message,dataunit);
    strcat (message, "=");
    dtostrf(regvalue, 5, 3, regvalue_string);
    strcat (message, regvalue_string);

 
    
    if (debug) {
    Serial.println("");
    Serial.print("MQTT: Sending message to topic: ");
    Serial.println(topic);
    Serial.print("Value: ");
    Serial.println(message);
    
    }  
    mqttClient.beginMessage(topic);
    mqttClient.print(message);
    mqttClient.endMessage();
    if (debug) Serial.print("MQTT: done");
    
  }

void setup() {
  Serial.begin(74880);
  // auf serielle Verbindung warten
  while (!Serial) {;}
  Serial.println ("_______SETUP______");
  Serial.println ("xxxxxxxxxxxxxxxxxxx");

  modbusSerial.begin(MODBUSBAUD);
  modbus.begin(MODBUSINVERTERID, modbusSerial);

#ifdef MODBUSPINENATX
  pinMode(MODBUSPINENATX, OUTPUT);
  digitalWrite(MODBUSPINENATX, 0);
  modbus.preTransmission(ModbusPreTransmission);
  modbus.postTransmission(ModbusPostTransmission);
#endif

  Serial.println("solis2mqtt started");

   connect_to_wlan();
   connect_mqtt();
   if (mqttClient.connected()) {
      Serial.print ("connected successful MQTT: ");
      Serial.print (broker);
      Serial.print (":");
      Serial.println (port);
      }


}

void loop() {
  /*
  Serial.println ("");
  Serial.println ("New Run");
  Serial.println ("#########");
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  */

  //connect_to_wlan();
  connect_mqtt();

  mqttClient.poll();

#if (enable_anti_oscillation_control_loop) 
      // my solis overshhots when charging battery in low-light conditions.
      //draws to much resulting in requing power from grid
      //then it stops charging and discharges.. this leads to oscillation
      // below control-loop is trying to cover it
      //Serial.print ("+++++++++++++++++++++++++++++++++++++");
      //Serial.print (millis());
      //Serial.print ("-");
      //Serial.print (timestamp_power_change);
      //Serial.print (">");
      //Serial.println (1000 * wait_seconds_recovery);
      //delay (15000);
      if ( millis() - timestamp_power_change  > 1000 * wait_seconds_recovery) {
        //Serial.println ("main-loop - before booth detect and recover. now wailting 10sec");
        //delay (10000);
        if (get_oscillation_regs())  {
            //detect_oscillation();
            //recover_to_full_operation();
        }

  }
#endif

  for (int i = 0; solis[i].modbusaddr; i++) {

    //Serial.println (solis[i].modbusaddr);

    if (!( millis() - readlast[i] >= (1000 * solis[i].readdelay) ) || (solis[i].readdelay < 0)) {
      continue;
    }
    readlast[i] = millis();

    unsigned int mbreqaddr = solis[i].modbusaddr - solis[i].modbusoffset;
    unsigned int mbreqlen;
    unsigned int mbreadresult;
    unsigned int datadiv = ( solis[i].datadiv > 1 ) ? solis[i].datadiv : 0;
    
    if (solis[i].debug) {
      Serial.print("reading register :");
      Serial.print(mbreqaddr);
      Serial.print(" ");
      Serial.print(solis[i].dataname);
      Serial.print(" = ");
    }

    switch (solis[i].datatype) {
      case SDT_SNHEX:
        mbreqlen = SNHEXWORDS;
        break;
      case SDT_SNASC:
        mbreqlen = SNASCWORDS;
        break;
      case SDT_DATETIME:
        mbreqlen = 6;
        break;
      case SDT_U32:
      case SDT_S32:
        mbreqlen = 2;
        break;
      default:
        mbreqlen = 1;
        break;
    }

    switch (solis[i].modbusobject) {
      case MB_COIL:
        mbreadresult = modbus.readCoils(mbreqaddr, mbreqlen);
        break;
      case MB_DISCRETEINPUT:
        mbreadresult = modbus.readDiscreteInputs(mbreqaddr, mbreqlen);
        break;
      case MB_INPUTREG:
        mbreadresult = modbus.readInputRegisters(mbreqaddr, mbreqlen);
        break;
      case MB_HOLDINGREG:
        mbreadresult = modbus.readHoldingRegisters(mbreqaddr, mbreqlen);
        break;
    }
    
    if (mbreadresult != modbus.ku8MBSuccess) {
      Serial.print ("Error: Modbus read failure for Register: ");
      Serial.println(mbreqaddr);
      readlast[i] += 1000 * 60;
      delay(500);
      continue;
    }

    switch (solis[i].datatype) {
      case SDT_U16:
        if (datadiv) {
          float regvalue = modbus.getResponseBuffer(0) / (float) datadiv;
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
          publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
         
        } else {
          unsigned int regvalue = modbus.getResponseBuffer(0);
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
           publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
          
        }
        
        break;

      case SDT_U32:
        if (datadiv) {
          float regvalue = ((modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1)) / (float) datadiv;
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
          publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
          
        } else {
          unsigned long regvalue = (unsigned long)(modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1);
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
          publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
        }
        
        break;

      case SDT_S16:
        if (datadiv) {
          float regvalue = (int) modbus.getResponseBuffer(0) / (float) datadiv;
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
          publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
        } else {
          int regvalue = (int) modbus.getResponseBuffer(0);
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
          publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
        }
        
        break;

      case SDT_S32:
        if (datadiv) {
          float regvalue = (((long)(modbus.getResponseBuffer(0) << 16)) | modbus.getResponseBuffer(1)) / (float) datadiv;
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
          publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
        } else {
          long regvalue = (long)(modbus.getResponseBuffer(0) << 16) | modbus.getResponseBuffer(1);
          if (solis[i].debug) Serial.print(regvalue);
          //influxdb.addField(solis[i].dataname, regvalue);
          publish_mqtt (solis[i].dataname, solis[i].dataunit, regvalue,solis[i].debug);
        }
        
        break;

      case SDT_ITYPE:  /* Solis inverter type definition, switches lookup table */
        {
          char buf[2 + 4 + 1];
          unsigned int regvalue = modbus.getResponseBuffer(0);
          
          sprintf(buf, "0x%04X", regvalue);
          Serial.print(buf);


          if ((regvalue / 100) == 10) {           /* RS485_MODBUS (INV-3000ID EPM-36000ID) inverter protocol */
            solis = solisINV;
          } else if ((regvalue / 100) == 20) {    /* RS485_MODBUS (ESINV-33000ID) energy storage inverter protocol */
            solis = solisESINV;
          } else if ((regvalue) == 8241) {        // RS485_MODBUS solis-rhi-3k-48es-5g
            solis = solisESINV;
          }
          else {
            solis = solisESINV;
          }
        }
        break;

      case SDT_SNHEX:  /* 16 characters serial number, solis-style hex encoded */
        for (int j = 0; j < SNHEXWORDS; j++) {
          unsigned int r = modbus.getResponseBuffer(j);
          sprintf(serialnumber + (j * 4), "%02x%02x",
                  (r & 0x0F) << 4 | (r & 0xF0) >> 4, (r & 0x0F00) >> 4 | (r & 0xF000) >> 12);
        }
        serialnumber[SNHEXWORDS * 4] = 0;
        serialvalid = 1;
        Serial.print(serialnumber);
        break;

      case SDT_SNASC:  /* Serial number, solis-style ASCII encoded */
        for (int j = 0; j < SNASCWORDS; j++) {
          unsigned int r = modbus.getResponseBuffer(j);
          serialnumber[j] = r & 0x0F;
        }
        serialnumber[SNASCWORDS] = 0;
        /* serialvalid = 1;   decoding logic above not tested, docs are confusing, please help */
        Serial.print(serialnumber);
        break;

      case SDT_DATETIME:  /* Composed date and time format */
        {
          char buf[8 + 1 + 8 + 1];
          sprintf(buf, "%02d-%02d-%02d %02d:%02d:%02d", modbus.getResponseBuffer(0), modbus.getResponseBuffer(1), modbus.getResponseBuffer(2),
                  modbus.getResponseBuffer(3), modbus.getResponseBuffer(4), modbus.getResponseBuffer(5));
          Serial.print(buf);
        }
        break;

      case SDT_APP6:  /* Working status */
        {
          const char *wstatus[] = {
            "Normal", "Initializing", "Grid off", "Fault to stop", "Standby", "Derating",
            "Limitating", "Backup OV Load", "Grid Surge Warn", "Fan fault Warn", "Reserved",
            "AC SPD ERROR VgSpdFail", "DC SPD ERROR DcSpdFail", "Reserved", "Reserved", "Reserved"
          };
          char buf[64];
          unsigned int regvalue = modbus.getResponseBuffer(0);
          for (int j = 0; j < 16; j++) {
            sprintf(buf, "%s - %s", solis[i].dataname, wstatus[j]);
            //influxdb.addField(buf, (bool)(regvalue & (1 << j)));

            if (regvalue & (1 << j)) {
              Serial.print(wstatus[j]);
              Serial.print(" ");
            }
          }
        }
        break;

      default:  /* Hex print */
        {
          char buf[2 + 4 + 1];
          unsigned int regvalue = modbus.getResponseBuffer(0);
          sprintf(buf, "0x%04X", regvalue);
          Serial.print(buf);
        }
        break;
  
    
    }  // end of switch (solis[i].datatype) {
    //Serial.println(solis[i].dataunit);

 
    yield();
  
  }
  mqttClient.poll();

  //Serial.println("Now waiting 1secs");
  //delay(1000);
}

