
#define MODBUSPINRX       16  //RX2==R0       esp32:GPIO16     esp8266:d7/gpio13
#define MODBUSPINTX       17  //TX2==DI       ESP32:GPIO17     ESP8266:D8/GPIO15
#define MODBUSPINENATX    18 // enable only if your RS485 adapter requires a TX enable pin */
#define MODBUSBAUD        9600
#define MODBUSINVERTERID  1   // wlan datalogger stick expects the inverter to be address 1


#include <SoftwareSerial.h>
#include <ModbusMaster.h>

//write register stuff
const uint16_t max_batt_charge_register=43117;
const unsigned int max_batt_charge_reg_len=1;

const uint16_t reg_43117_target=40;                  // value in ampere

const int conversion =10;                         // register is a 0.1 register

// modbus RTU
SoftwareSerial modbusSerial(MODBUSPINRX, MODBUSPINTX);
ModbusMaster modbus;

#ifdef MODBUSPINENATX
void ModbusPreTransmission() {
  digitalWrite(MODBUSPINENATX, 1);
}

void ModbusPostTransmission() {
  digitalWrite(MODBUSPINENATX, 0);
}
#endif


float read_register (const uint16_t reg, const unsigned int reg_len) {
  uint8_t mbreadresult;
  Serial.print("reading register: ");
  Serial.println(reg);
  mbreadresult = modbus.readHoldingRegisters(reg, reg_len);
  if (mbreadresult != modbus.ku8MBSuccess) {
      Serial.println("Error: Modbus read failure");
      }
  else {
    float regvalue = modbus.getResponseBuffer(0) / conversion;    
    return (regvalue);
    }
}

void write_register (const uint16_t target_ampere, const uint16_t reg, const unsigned int reg_len) {

   const uint16_t write_value=target_ampere * conversion;     // is a 0.1 value, if we want 3 ampere, we need to set 30
   uint8_t mbreadresult; 
   uint8_t mbwriteresult; 
 
  
  Serial.print ("New Target Value :");
  Serial.println (target_ampere);

  float regvalue=read_register(reg, reg_len);
  Serial.print ("Actual Value :");
  Serial.println (regvalue); 

  if (regvalue!=target_ampere) {
      Serial.println("value is different. Trying to set  new value");
      mbwriteresult=modbus.writeSingleRegister(reg, write_value);
      if (mbwriteresult==0) {
        delay (1000);
        Serial.println("Sucess Writing");
        Serial.println("Verification Step - reading register again");
        Serial.println("------------------------------------------");
        //float regvalue=read_register(reg, reg_len);
        float regvalue=modbus.readHoldingRegisters(reg, reg_len);
        Serial.print ("Value :");
        Serial.println (regvalue);
      }
      else {
        Serial.println("Writing failed");
      }
    }  
  else {
     Serial.println("no difference between old and new. doing nothing");
       }
 
}


void setup() {
  Serial.begin(74880);
  delay (1000);
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


write_register(reg_43117_target,max_batt_charge_register, max_batt_charge_reg_len);  
//read_register (max_batt_charge_register, max_batt_charge_reg_len); 
}

void loop() {

}

