#include "JuncTek_BatteryMonitor.h"
#include <Arduino.h>
#include <SimpleTimer.h>        // Simple Timer library

BatteryMonitor::BatteryMonitor(){
//empty constructor
}
BatteryMonitor::~BatteryMonitor(){ 
}

void BatteryMonitor::begin(int address, Stream &serialDevice, int pinForRSE){
  bm_serial=&serialDevice;
  bm_address=address;
  pinRSE = pinForRSE;
  pinMode(pinForRSE, OUTPUT);  // set Output enable pin to output mode

  setValues.deviceAddress=bm_address;
  basicInfo.deviceAddress=bm_address;
  measuredValues.deviceAddress=bm_address;
  cacheTime=CACHE_TIME; 
  
  getBasicInfo();
  delay(200);
  int i = 0;
  bool ret;
  do{
    ret = getSetValues();
    i++;
  }while(!ret || i <= 3);
  delay(300);
  //Serial.printf("=== Battery Monitor basic info ===\nmax Voltage: %i\nmax Current: %i\nSensor Type: %i\nVersion    : %i\nSerial Nr : %i",  basicInfo.maxVoltage, basicInfo.maxCurrent,basicInfo.sensorType,basicInfo.deviceVersion,basicInfo.deviceSerialNumber);
  getMeasuredValues();
  //Serial.printf("=== Battery Monitor measured values ===\nUptime: %i\nbattery life left: %i\nTemperature: %i\nVoltage: %f\nCurrent: %f\nint Resistanc: %f\nCapacity: %f\ncumulative capa: %f\nOutput State: %i\ncurrent direction: %i",measuredValues.uptime,measuredValues.batteryLifeLeft, measuredValues.temperature, measuredValues.voltage, measuredValues.current, measuredValues.internalResistance, measuredValues.remainingCapacity, measuredValues.cumulativeCapacity,measuredValues.outputState,measuredValues.currentDir);  
}

void BatteryMonitor::setNewAddress(uint8_t newAddress){
    sendCommand(bm_address, BM_F_SetAddress, newAddress);
}

bool BatteryMonitor::setOutput(bool output){
  if(output){
    return sendCommand(bm_address, BM_F_TurnOnOutput, 1);
   }else{
    return sendCommand(bm_address, BM_F_TurnOnOutput, 0);
   }
}
bool BatteryMonitor::setOverVoltageProtection(int voltage){
  int v=(int)(voltage*100);
  return sendCommand(bm_address, BM_F_SetOVProt, v);
}
bool BatteryMonitor::setOverVoltageProtection(float voltage){
  int v=(int)(voltage*100);
  return sendCommand(bm_address,BM_F_SetOVProt, v);
}

bool BatteryMonitor::setUnderVoltageProtection(int voltage){
    int v=(int)(voltage*100);
  return sendCommand(bm_address, BM_F_SetUVProt, v);
}
bool BatteryMonitor::setUnderVoltageProtection(float voltage){
    int v=(int)(voltage*100);
  return sendCommand(bm_address, BM_F_SetUVProt, v);
}

bool BatteryMonitor::setPositiveOverCurrentProtection(int current){
    int c=(int)(current*100);
    return sendCommand(bm_address, BM_F_SetPOCProt, c);
}
bool BatteryMonitor::setPositiveOverCurrentProtection(float current){
    int c=(int)(current*100);
    return sendCommand(bm_address, BM_F_SetPOCProt, c);
}

bool BatteryMonitor::setNegativeOverCurrentProtection(int current){
   int c=(int)(current*100);
    return sendCommand(bm_address, BM_F_SetNOCProt, c);
}
bool BatteryMonitor::setNegativeOverCurrentProtection(float current){
   int c=(int)(current*100);
    return sendCommand(bm_address, BM_F_SetNOCProt, c);
}

bool BatteryMonitor::setOverPowerProtection(int power){
   int p=(int)(power*100);
    return sendCommand(bm_address, BM_F_SetOPProt, p);
}
bool BatteryMonitor::setOverPowerProtection(float power){
   int p=(int)(power*100);
    return sendCommand(bm_address, BM_F_SetOPProt, p);
}

bool BatteryMonitor::setOverTemperatureProtection(int temperature){
   int t=temperature+100;
    return sendCommand(bm_address, BM_F_SetOTProt, t);
}

bool BatteryMonitor::setOverTemperatureProtection(float temperature){
   int t=(int)(temperature+100);
    return sendCommand(bm_address, BM_F_SetOTProt, t);
}

bool BatteryMonitor::setBatteryCapacity(int capacity){
  int c=capacity*10;
  return sendCommand(bm_address, BM_F_SetBattCapa, c);  
}
bool BatteryMonitor::setBatteryCapacity(float capacity){
  int c=(int)(capacity*10);
  return sendCommand(bm_address, BM_F_SetBattCapa, c);
}
bool BatteryMonitor::setVoltageCalibration(int calibrationVoltage){
  return sendCommand(bm_address, BM_F_SetVoltCalbr, calibrationVoltage);
}

bool BatteryMonitor::setCurrentCalibration(int calibrationCurrent){
  return sendCommand(bm_address, BM_F_SetCurrCalbr, calibrationCurrent);
}

bool BatteryMonitor::setTemperatureCalibration(int calibrationTemperature){
  int t=calibrationTemperature+100;
    return sendCommand(bm_address, BM_F_SetTempCalbr,t );
}

bool BatteryMonitor::setRelayType(int relayType){
  return sendCommand(bm_address, BM_F_SetRelayType, relayType);
}

void BatteryMonitor::resetFactorySettings(){
  sendMessage(bm_address, BM_F_ResumeFctSettings, 1);
  String message=readMessage();
}
bool BatteryMonitor::setCurrentMultiplier(int currentMultiplier){
  return sendCommand(bm_address, BM_F_SetCurrMult, currentMultiplier);
}

bool BatteryMonitor::setBatteryPercent(int batteryPercent){
  if(batteryPercent >=0 && batteryPercent<=100) {
    return sendCommand(bm_address,BM_F_SetBattPerc,batteryPercent);
  } else {
    return false;
  }
}
void BatteryMonitor::zeroCurrent(){
  bool rc=sendCommand(bm_address, BM_F_ZeroCurrent, 1);
}

void BatteryMonitor::clearAccountingData(){
    bool rc=sendCommand(bm_address, BM_F_ClearAccData, 1);
}

int BatteryMonitor::getUptime(){
  getMeasuredValues();
  return measuredValues.uptime;
}

int BatteryMonitor::getMeasuredValuesChecksum(){
  getMeasuredValues();
  return measuredValues.checksum;
}
int BatteryMonitor::getMeasuredValuesTstChecksum(){
  getMeasuredValues();
  return measuredValues.tst_checksum;
}

int BatteryMonitor::getSetValuesChecksum(){
  getMeasuredValues();
  return setValues.checksum;
}
int BatteryMonitor::getSetValuesTstChecksum(){
  getMeasuredValues();
  return setValues.tst_checksum;
}

int BatteryMonitor::getBatteryLifeLeft(){
    getMeasuredValues();
    return measuredValues.batteryLifeLeft;
}
int BatteryMonitor::getTemperature(){
    getMeasuredValues();
    return measuredValues.temperature;
}
   
int BatteryMonitor::getProtectionTemperature(){
    return setValues.protectionTemperature;
}

int BatteryMonitor::getProtectionRecoveryTime(){
    return setValues.protectionRecoveryTime;
}

int BatteryMonitor::getProtectionDelayTime(){
    return setValues.protectionDelayTime;
}

int BatteryMonitor::getCapacity(){
    return setValues.presetCapacity/10;
}

int BatteryMonitor::getVoltageCalibration(){
    return setValues.voltageCalibration;
}

int BatteryMonitor::getCurrentCalibration(){
    return setValues.currentCalibration;
}

int BatteryMonitor::getTemperatureCalibration(){
    return setValues.temperatureCalibration;
}

int BatteryMonitor::getVoltageScale(){  
    return setValues.voltageScale;
}
int BatteryMonitor::getCurrentScale(){
    return setValues.currentScale;
}

int BatteryMonitor::getRelayType(){
  return setValues.relayType;
}
float BatteryMonitor::getVoltage() {
  getMeasuredValues();
  return measuredValues.voltage;
}
float BatteryMonitor::getCurrent(){
  getMeasuredValues();
  return measuredValues.current;
}
float BatteryMonitor::getPower() {
  getMeasuredValues();
  return measuredValues.power;
}
float BatteryMonitor::getWattHours() {
  getMeasuredValues();
  return measuredValues.wattHours;
}
float BatteryMonitor::getInternalResistance(){
  getMeasuredValues();
  return measuredValues.internalResistance;
}
float BatteryMonitor::getRemainingCapacity(){
  getMeasuredValues();
  return measuredValues.remainingCapacity;
}
float BatteryMonitor::getCumulativeCapacity(){
  getMeasuredValues();
  return measuredValues.cumulativeCapacity;
}

float BatteryMonitor::getOverVoltageProtectionVoltage(){
  // getSetValues();  
  return setValues.OVPVoltage;
}
float BatteryMonitor::getUnderVoltageProtectionVoltage(){
  return setValues.UVPVoltage;
}
float BatteryMonitor::getOverCurrentProtectionForwardCurrent(){
  return setValues.OCPForwardCurrent;
}
float BatteryMonitor::getOverCurrentProtectionReverseCurrent(){
  return setValues.OCPReverseCurrent;
}
float BatteryMonitor::getOverPowerProtectionPower(){
  return setValues.OPPPower;
}
int BatteryMonitor::getCurrentDirection(){
  return measuredValues.currentDir;
}

void BatteryMonitor::getBasicInfo(){
  // result syntax: :r00=<addr>,<checksum>,<sensor_type:1><voltage:1><amperage:2>,<firmware_version>,<machine_serial_nr>,
  String message,field;
  
  debug("getting basic Info");
   
  sendMessage(bm_address, BM_F_ReadBasicInf, 1);
  message=readMessage();
  #ifdef DEBUG2
    Serial.print("getBasicInfo "+message);
  #endif  

  debug("get sensor type, max voltage and max current");
  field=getStringField(message,3);
  debug("field:");
  debug(field);
  basicInfo.sensorType=field.substring(0,1).toInt();
  debug("sensorType");
  debug(basicInfo.sensorType);
  basicInfo.maxVoltage=field.substring(1,2).toInt()*100;
  debug("maxVoltage");
  debug(basicInfo.maxVoltage);  
  basicInfo.maxCurrent=field.substring(2).toInt()*10;
  debug("maxCurrent");
  debug(basicInfo.maxCurrent);  
  
  
  debug("get device version");
  basicInfo.deviceVersion=getStringField(message,4).toInt();
  debug("get device serial number");
  basicInfo.deviceSerialNumber=getStringField(message,5).toInt();
}

// MP: read a raw message and return it
String BatteryMonitor::getRawMessage(){
  String message;
  message=readMessage();
  return(message);
}

void BatteryMonitor::getMeasuredValues(){
  int idx, i;
  String message, instring;
  float pwr;
  if(!checkCache()){
    sendMessage(bm_address, BM_F_ReadMsrdVals, 1);
    
    #ifdef DEBUG2
      Serial.print("getMeasuredValues before readMessage\n");
    #endif  
    // read until we get a message with measured values, containing "":R50="
    i = 0;
    do{
      message=readMessage();
      idx = message.indexOf(":R50=");
      Serial.print(idx);
      i++;
    }while(idx < 0 && i < 3);    
    //}while(idx < 0 || i < 10);  // endlosschleife wenn nichts gelesen, idx bleibt -1

    #ifdef DEBUG2
      Serial.print("getMeasuredValues after readMessage\n");
    #endif  
  
    /*
    * :r50=<addr>,
    * 01 - <addr>
    * 02 - <checksum>,
    * 03 - <Voltage (1/100V)>,
    * 04 - <Current (1/100A)>,
    * 05 - <remaining capacity mAh>,
    * 06 - <cumulative capacity mAh>,
    * 07 - <Capacity (mWh)>,
    * 08 - <running time (s)>,
    * 09 - <temperature (°C+100)>,
    * 10 - <power (1/100 W)>,
    * 11 - <output State>,
    * 12 - <current direction>,
    * 13 - <remaining battery life (minutes)>,
    * 14 - <internal resistance (mOhm/100)>
    */

    // get checksum as sent by KGF-110
    measuredValues.checksum = getStringField(message,2).toInt();   
    // calculate checksum from data fields: (sum of all fields after checksum) % 255 +1
    int tst = 0;
    for(i=3; i<=14; i++)
      tst += getStringField(message,i).toInt();
    tst = (tst % 255) + 1;
    measuredValues.tst_checksum = tst;

    #ifdef DEBUG2
      Serial.print("getMeasuredValues after readMessage\n");
    #endif 

    if(measuredValues.checksum == tst)
    {
      debug("get Voltage");
      measuredValues.voltage=getStringField(message,3).toFloat()/100;
      measuredValues.current=getStringField(message,4).toFloat()/100;
      measuredValues.remainingCapacity=getStringField(message,5).toFloat()/1000;
      measuredValues.cumulativeCapacity=getStringField(message,6).toFloat()/1000;
      measuredValues.wattHours=getStringField(message,7).toFloat()/1000;
      debug("get uptime");
      measuredValues.uptime=getStringField(message,8).toInt();    
      debug("get temperature");
      measuredValues.temperature=getStringField(message,9).toInt()-100;
      
      // Power kommt nicht als übertragenes Datum an, ist in der Message immer 0. Also berechnen wir es:
      measuredValues.power= getStringField(message,10).toFloat()/100;
      measuredValues.power=    measuredValues.voltage * measuredValues.current;
      
      measuredValues.outputState=getStringField(message,11).toInt();
      measuredValues.currentDir=getStringField(message,12).toInt();

      debug("get battery life left");
      measuredValues.batteryLifeLeft=getStringField(message,13).toInt();
      measuredValues.internalResistance=getStringField(message,14).toFloat()/100;

      measuredValues.lastReadTime=millis();
    }
  }
}
bool BatteryMonitor::getSetValues(){
  String message; 
  int idx, i;
  sendMessage(bm_address, BM_F_ReadSetVals,1);
  // read until we get a message with measured values, containing "":R50="
  i = 0;
  do{
    message=readMessage();
    idx = message.indexOf(":R51=");
    Serial.print(idx);
    i++;
  }while(idx < 0 && i < 3);   
  //message=readMessage();
    /*
       1: deviceAddress
       2: checksum
       3: OVPVoltage
       4: UVPVoltage
       5: OCPForwardCurrent
       6: OCPReverseCurrent
       7: OPPPower
       8: protectionTemperature
       9: protectionRecoveryTime
      10: protectionDelayTime
      11: presetCapacity
      12: voltageCalibration
      13: currentCalibration
      14: temperatureCalibration
      15: reserved
      16: relayType
      17: currentMultiple
      18: voltageScale
      19: currentScale
    */
  #ifdef DEBUG2
    Serial.print("getSetValues "+message);
  #endif  
    
  // get checksum as sent by KGF-110
  setValues.checksum = getStringField(message,2).toInt();   
  // calculate checksum from data fields: (sum of all fields after checksum) % 255 +1
  int tst = 0;
  for(int i=3; i<=19; i++)
    tst += getStringField(message,i).toInt();
  tst = (tst % 255) + 1;
  setValues.tst_checksum = tst;

  bool ret = false;
  if(setValues.checksum == tst)
  {
    
    setValues.OVPVoltage          = getStringField(message, 3).toFloat()/100;
    setValues.UVPVoltage          = getStringField(message, 4).toFloat()/100;
    setValues.OCPForwardCurrent   = getStringField(message, 5).toFloat()/100;
    setValues.OCPReverseCurrent   = getStringField(message, 6).toFloat()/100;
    setValues.OPPPower            = getStringField(message, 7).toFloat()/100;
    setValues.protectionTemperature= getStringField(message, 8).toInt()-100;
    setValues.protectionRecoveryTime  = getStringField(message, 9).toInt();
    setValues.protectionDelayTime = getStringField(message,10).toInt();
    setValues.presetCapacity      = getStringField(message,11).toInt();
      
    setValues.voltageScale        = getStringField(message,18).toInt();
    setValues.currentScale        = getStringField(message,19).toInt();
    setValues.relayType           = getStringField(message,16).toInt();
    ret= true;
  }  
    /*
        int 
        deviceAddress,
        checksum,
        protectionTemperature,
        protectionRecoveryTime,
        protectionDelayTime,
        presetCapacity,
        voltageCalibration,
        currentCalibration,
        temperatureCalibration,
        voltageScale,
        currentScale;
    float        
        OVPVoltage,
        UVPVoltage,
        OCPForwardCurrent,
        OCPReverseCurrent,
        OPPPower;
    enum relayType{
        normallyOpen=0,
        normallyClosed=1
   */
  return(ret);
}

bool BatteryMonitor::sendCommand(int address, int command, int parameter){
  String message;
  int command_r,returncode; //values as read back
  char verb_r,verb; 
  
  sendMessage(bm_address, command, parameter);
  message=readMessage();
  command_r = message.substring(3,5).toInt();
  returncode = getStringField(message, 2).toInt();
  getSetValues();
  if(command_r == command && returncode==0){
    return true;
  }else{
    return false;
  }
  /*
   * 
  //verb='w';
  //&verb_r     = message.substring(2,3).c_str();
  command_r   = message.substring(3,5).toInt();
  address_r   = getStringField(message, 1).toInt();
  checksum_r  = getStringField(message, 2).toInt();
  parameter_r = getStringField(message, 3).toInt();
  #ifdef DEBUG2
    Serial.println(message);
    //Serial.printf("==== verify command ====\n\t\t\tsent:\t|\t\treceived\n\tverb\t:\t%i\t|\t\t W\n\tcommand\t:%i\t|\t\t %i\n\taddress\t:%i\t|\t\t %i\n\tchecksum\t:\t%i\t|\t\t %i\n\tparameter\t:\t%i\t\t|\t\t %i\n\n", verb,command,command_r,address,address_r,checksum(parameter),checksum_r,parameter,parameter_r);
    Serial.printf("==== verify command ====\n\t\tsent:\t|\t\treceived\n\tcommand\t:%i\t|\t\t %i\n\taddress\t:%i\t|\t\t %i\n\tchecksum\t:\t%i\t|\t\t %i\n\tparameter\t:\t%i\t\t|\t\t %i\n\n", command,command_r,address,address_r,checksum(parameter),checksum_r,parameter,parameter_r);
  #endif
  if(command_r==command && address_r==address && parameter_r==parameter && checksum(parameter)==checksum_r){
    return true;
  } 
  else {
    return false;
  }
 */
  
}

String BatteryMonitor::getStringField(String message, int idx){
  String field, debugstring;
  int curr_idx,next_idx;
  // MP next_idx=message.indexOf("=");
  next_idx=message.lastIndexOf("=");
  /*
  #ifdef DEBUG
    debugstring = "Search for:"  + String(idx) + ". '=' found at "+ String(next_idx) + " in Message: "+message+"\n"; 
    Serial.print(debugstring);
  #endif  
  */
  //Serial.println(message);
  for(int i=1;i<=idx;i++){
    if(next_idx!=-1){
      curr_idx=next_idx;
      next_idx=message.indexOf(",",curr_idx+1);
      /*
      #ifdef DEBUG2
        debugstring = "i: "+ String(i) 
          + " ,current idx: " + String(curr_idx) 
          + " ,next idx: " + String(next_idx) + " : " + message.substring(curr_idx+1,next_idx) + "\n";
        Serial.print(debugstring);
      #endif  
      */
      //Serial.printf("Fieldnum: %i, current idx: %i, next idx: %i\n", i, curr_idx, next_idx);
    }else{
      curr_idx=0;         // setting the indexes to 0 and 1
      next_idx=1;         // will make sure the return string is length 0
      i=idx;              // exit the loop
    }
  }
  field=message.substring(curr_idx+1,next_idx);
  debug("getStringField returns");
  debug(field);
  #ifdef DEBUG2
    debugstring = "getStringField returns for Index: "+String(idx)+" : "+field+"\n";
    // Serial.print(debugstring);
  #endif 
  return field;
}

void BatteryMonitor::sendMessage(int address, int command, int parameter){
  char message[40];
  switch(command){
    case BM_F_ReadBasicInf:
    case BM_F_ReadMsrdVals:
    case BM_F_ReadSetVals:
      sprintf(message, ":R%02i=%i,%i,%i,\r\n", command, address, checksum(parameter), parameter);
    break;
    default:
      sprintf(message, ":W%02i=%i,%i,%i,\r\n", command, address, checksum(parameter), parameter);
  }
  debug("sendMessage\nrequest:");
  //sprintf(message, ":%c%02i=%i,%i,%i,\r\n", dir, command, address, checksum(parameter), parameter);
  debug(message);
  #ifdef DEBUG2
    char printstring[80];
    sprintf(printstring, "Sending Message: _%s_", message);
    Serial.print(printstring);
  #endif
  digitalWrite (pinRSE, HIGH); // MP set input/output enable to OUTPUT
  delayMicroseconds(5);
  bm_serial->print(message);
  bm_serial->flush();       // ensure that program flow does not continue before serial data have been sent
  delayMicroseconds(5);
  digitalWrite (pinRSE, LOW);  // MP set input/output enable to INPUT
}

String BatteryMonitor::readMessage(){   
    String _message, printstring;
    msgState_t _msgState;
    char c;
    debug("readMessage");
    if (_msgState==crlf){
      //Serial.println(".....new message");
      _message="";
      _msgState=reading;
    }
    
    if (sizeof(_message)==0) Serial.print("[start]");
    int t_start=millis();
    while(!bm_serial->available() && millis()-t_start < SERIAL_TIMEOUT);
    while(bm_serial->available() && _msgState != crlf){
        c=bm_serial->read();
        debug(c);
        _message=_message+String(c);
        //Serial.print(c);
        if (c=='\r') {
          _msgState=cr;
          //Serial.print("{cr}");
        }
        if (_msgState==cr && c=='\n') _msgState=crlf;
        
        //if (_msgState==reading) Serial.print("[r]");
        //if (_msgState==cr) Serial.print("[CR]");
        //if (_msgState==crlf) Serial.println("[CRLF]");
    }
    #ifdef DEBUG2
      printstring = "Received Message: _" + _message + "_\n";
      Serial.print(printstring);
    #endif
    debug("finished read message\nmessage:");
    debug(_message);
    return _message;
}
void BatteryMonitor::setCacheTime(int cTime){
  cacheTime=cTime;
  }
  
int BatteryMonitor::getCacheTime(){
  return cacheTime;
}

bool BatteryMonitor::checkCache(){
  return(millis()-measuredValues.lastReadTime<cacheTime);
}

void BatteryMonitor::debug(const char msg[]){
    #ifdef DEBUG
    Serial.println(msg);
    #endif
    }

void BatteryMonitor::debug(char msg[]){
    #ifdef DEBUG
    Serial.println(msg);
    #endif
    }
void BatteryMonitor::debug(char c){
    #ifdef DEBUG
    Serial.print(c);
    #endif
    }
void BatteryMonitor::debug(String &msg){
    #ifdef DEBUG
    Serial.println(msg);
    #endif
    }
void BatteryMonitor::debug(int i){
    #ifdef DEBUG
    Serial.println(i);
    #endif
    }