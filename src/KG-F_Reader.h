#ifndef KG_F_READER_H
#define KG_F_READER_H

#ifndef _BATTERYMONITORH_
  #include "JuncTek_BatteryMonitor.h"
#endif

#ifndef WiFi_h
  #include <WiFi.h>               // wifi stuff
#endif  
#ifndef _WIFICLIENT_H_
  #include <WiFiClient.h>
#endif
#ifndef SIMPLETIMER_H
  #include <SimpleTimer.h>        // Simple Timer
#endif  
#ifndef esp_task_wdt_h
  #include <esp_task_wdt.h>       // Load Watchdog-Library
#endif 

//MP define pins for reading,writing and enable output pin RS485
#define TXD2 17
#define RXD2 16
#define RSE 15  // pin for write enable on Max 3485. to be integrated in BatteryMonitor::sendMessage

  // output for ve.direct emulator
#ifdef isVEDIRECT  
  #define RXBAT 21
  #define TXBAT 22
#endif  

#ifdef isOneDS18B20
  // Include file for DS18B20
  #ifndef OneWire_h
    #include <OneWire.h>
  #endif
  #ifndef DallasTemperature_h
    #include <DallasTemperature.h>
  #endif
  // 1-Wire connection for temperature (Dallas DS18B20) is plugged into GPIO13 on the ESP32
  #define ONE_WIRE_BUS 13
  // power for One Wire Bus via GPIO 32
  #define POWER_ONEWIRE_BUS 32    

  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS);
  // Pass our oneWire reference to Dallas Temperature.
  DallasTemperature sensors(&oneWire);
  
  // more flexible in Array: to store DS18B20 Temperatures
  #define MAX_NO_DS18B20 10
  int noDS18B20Connected = 0;       // number of sensors that are actually connected
  volatile double DS18B20Temperature[MAX_NO_DS18B20] = {-111.11, -111.11,-111.11,-111.11,-111.11,
                                            -111.11, -111.11, -111.11, -111.11, -111.11};
  volatile double calDS18B20Temperature[MAX_NO_DS18B20]={-111.11, -111.11,-111.11,-111.11,-111.11,
                                            -111.11, -111.11, -111.11, -111.11, -111.11}; 
  double sum_ThSp_calDS18B20Temperature[MAX_NO_DS18B20]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int n_ThSp_calDS18B20Temperature[MAX_NO_DS18B20]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double sum_MQTT_calDS18B20Temperature[MAX_NO_DS18B20]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int n_MQTT_calDS18B20Temperature[MAX_NO_DS18B20]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

                                                // calibrated temperature values for output
  #define DS18B20RestartLimit 25              // these two to store how long no valid measurements, do restart of above limit
  int noDS18B20Restarts = 0;                  // Counter for DS18B20 Restarts
  
  volatile unsigned int notMeasuredCount = 0; // count for measurements not taken with DS18B20
  volatile unsigned int notChangedCount = 0;  // count for measurements not changed with DS18B20
  unsigned int manualDS18B20Restart = 0;      // flag for manual restart of DS18B20
  
  // address storage added since sometimes one sensor drops out, disturbing the sequence
  volatile uint8_t DS18B20Address[MAX_NO_DS18B20][8]; // my store for the device addresses of the attached sensors
  // Task handle for OneWire read (Core 0 on ESP32)
  TaskHandle_t Task1;
  volatile unsigned long GetOneDS18B20Counter = 0;     // loop counter for detached procedure GetOneDS18B20Temperature
  unsigned long previousGetOneDS18B20Counter=0;         // comparison value for loop counter
  unsigned long notMeasuredDS18B20=0;                   // counter for not measuring DS18B20

  bool stopDS18B20MeasureFlag = false;        // if this flag is set, no measurements are taken

  // timer stuff for DS18B20
  #define ds18b20HandlerInterval 2000L 
  SimpleTimer ds18b20HandlerTimer;
  int ds18B20TimerHandle=1;

  // correction values for sensors
  float corrDS18B20[5]={ 0, 0, 0 };
#endif  

#ifdef isVEDIRECT
  // timer stuff for ve.direct emulation on Serial1
  #define vedirectHandlerInterval 1000L 
  SimpleTimer vedirectHandlerTimer;
  int vedirectTimerHandle=1;
#endif  

#ifdef isMQTT
  // also needs WiFi.h - is always included
  #ifndef PubSubClient_h
    #include <PubSubClient.h>
  #endif  
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);

  // timer stuff
  #define mqttHandlerIntervalQuick 2000L 
  SimpleTimer mqttHandlerTimerQuick;
  int mqttHandlerTimerHandleQuick=1;
  #define mqttHandlerIntervalSlow 30000L 
  SimpleTimer mqttHandlerTimerSlow;
  int mqttHandlerTimerHandleSlow=1;
#endif

// for wifi
char ssid[50];
char pass[50];
// Wifi credentials für frankfurt
 

// credentials for mqtt
#ifdef isMQTT
  char mqttDefaultServer[]    = "192.168.178.64";
  char mqttDefaultUser[]      = "markus";
  char mqttDefaultPaSSWORD[]  = "tirila1";

  char mqttActualServer[50];
  char mqttActualUser[50];
  char mqttActualPaSSWORD[50];
#endif

#define WDT_TIMEOUT_SECONDS 55  // 40 seconds watchdog timeout. Not too short, or the chip is dead!

//Message Severities
#define msgDefault  0
#define msgInfo     1
#define msgWarn     2
#define msgErr      3
#define msgStop     4

// message IDs

#define msgDS18B20NoMeasFlag  1
#define msgDS18B20Info        2
#define msgDS18B20Restart     3
#define msgDS18B20NotMeasuring 4

#define msgStartup            10

#define msgWifiConnected      70
#define msgWifiNotConnected   71
#define msgWiFiRssiInfo       210

#define msgMQTTInfo           220
#define msgMQTTError          221
#define msgMQTTSend           222
#define msgMQTTSendDS10B20    223
#define msgMQTTReceive        225
#define msgMQTTConnect        226
#define msgMQTTSubscribe      227
#define msgMQTTState          228

#define veCreateInfo1         300
#define veCreateInfo2         301
#define veCreateInfo3         302
#define veCreateInfo4         303
#define veCreateInfo5         304
#define veCreateInfo6         305

#define veNoData              310
#define veChecksumMeasured    311
#define veChecksumSetdata     312

#define mqttSensorKGF110ID    "BTG002"
#define mqttKGF110Voltage     "Voltage_V"
#define mqttKGF110Current     "Current_A"
#define mqttKGF110RemCapa     "RemainingCapacity_Ah"
#define mqttKGF110SetCapa     "SetCapacity_Ah"
#define mqttKGF110Temp        "Temperature_C"
#define mqttKGF110Power       "Power_W"
#define mqttKGF110CumulAhOut  "CumulativeAhOut_Ah"
#define mqttKGF110Uptime      "Uptime_sec"
#define mqttKGF110UptimeStr   "Uptime_Str"
#define mqttKGF110LifeLeft    "BatteryLifeLeft_min"
#define mqttKGF110LifeLeftStr  "BatteryLifeLeft_Str"
#define mqttKGF110EnergyIn    "EnergyIn_Wh"
#define mqttKGF110ProtectionTemp "ProtectionTemp_C"
#define mqttKGF110ProtectionRecoveryTime "ProtectionRecoveryTime_s"
#define mqttKGF110ProtectionDelayTime "ProtectionDelayTime_s"
#define mqttKGF110PresetCapacity "PresetCapacity_Ah"
#define mqttKGF110VoltageCalValue "VoltageCalValue_V"
#define mqttKGF110CurrentCalValue "CurrentCalValue_A"
#define mqttKGF110TempCalValue  "TempCalValue_C"
#define mqttKGF110VoltageScale "VoltageScale_V"
#define mqttKGF110CurrentScale "CurrentScale_A"
#define mqttKGF110RelayType "RelayType"
#define mqttKGF110OVPVoltage "OVPVoltage_V"
#define mqttKGF110UVPVoltage "UVPVoltage_V"
#define mqttKGF110OCPForwardCurrent "OCPForwardCurrent_A"
#define mqttKGF110OCPReverseCurrent "OCPReverseCurrent_A"
#define mqttKGF110OPPPower "OPPPower_W"
#define mqttKGF110SOC "SOC"
#define mqttKGF110CE "CE"

#define mqttSensorDS18B20     "DS18B20"
#define mqttDS18B20Temperature "DSTemperature"
#define mqttDS18B20Temperature1 "DSTemperature1"
#define mqttDS18B20Temperature2 "DSTemperature2"
#define mqttDS18B20Temperature3 "DSTemperature3"

// class for all kgf relevant information
class kgfData
{
  public:
  kgfData();    // declare parameter lessconstructor
  ~kgfData();   // declare parameterless destructor
  bool DataValid;  
  float Voltage, Current, Power, RemCapa, SetCapa, CumulAhOut, EnergyIn, 
    OVPVoltage, UVPVoltage, OCPForwardCurrent, OCPReverseCurrent, OPPPower;
  long  Uptime, LifeLeft, ProtectionTemp, ProtectionRecoveryTime, ProtectionDelayTime, 
    VoltageCalValue, CurrentCalValue, TempCalValue, VoltageScale, CurrentScale, 
    RelayType, Temp;
  char UptimeString[40], LifeLeftString[40];
};

#endif // define guardian