# KGF-Reader
MQTT and ve.direct Bridge for JuncTek KGF-110 (and compatible) battery monitor

I had started to use openDTU-onBattery https://github.com/helgeerbe/OpenDTU-OnBattery 
and wanted to use my JuncTek KGF-110 battery monitor to provide data to it, to enable better control 
of the battery charging and discharging.

Unfortunatly openDTU-onBattery does not (yet) include an interface to provide the battery data via MQTT, so I decided to make an attempt to send these data via the ve.dierct interface that is present for a Victron SmartShunt. 

KGF-Reader does just this: it collects data from a JuncTek KGF-110 battery monitor via RS-485 interface and sends them out to a serial port in ve.direct format. These data are accepted by openDTU-onBattery as Victron SmartShunt data

## MQTT
KGH-Reader also sends the battery monitor data to a MQTT server.

### Credentials
Wifi credentials and MQTT server credentials have to be provided in "credentials.h". Only a template for this file is included in this repository, you will have to fill in your own information before compiling, and rename the file to "credentials.h":
// Wifi credentials
char default_ssid[] = "My-Wifi";
char default_pass[] = "my-password";
 
// credentials for MQTT server
char mqttDefaultServer[]    = "192.168.178.100";
char mqttDefaultUser[]      = "my-username";
char mqttDefaultPaSSWORD[]  = "my-password";

### MQTT topics
The base topic is "esp32/<mqttDeviceString>/<mqttSensorKGF110ID>/.."
mqttDeviceString and mqttSensorKGF110ID are static and set in "KG-F_Reader.h"
Example:
  static char mqttSensorKGF110ID[] = "BTG002";
  static char mqttDeviceString[]   = "KG-F110";
results in a MQTT base string of:
  esp32/KG-F110/BTG002/

The following topics are available:
Measurement values - updated every 2 seconds
 "Voltage_V"
"Current_A"
"Power_W"
"RemainingCapacity_Ah"
"SetCapacity_Ah"
"Temperature_C"
"CumulativeAhOut_Ah"
"Uptime_sec"
"Uptime_Str"
"BatteryLifeLeft_min"
"BatteryLifeLeft_Str"
"EnergyIn_Wh"
"ProtectionTemp_C"
"SOC"
"CE"
MeasValSentChecksum
MeasValTstChecksum

Set values - updated every 30 seconds
SetCapacity_Ah
ProtectionTemp_C
"ProtectionRecoveryTime_s"
"ProtectionDelayTime_s"
"VoltageCalValue_V"
"CurrentCalValue_A"
"TempCalValue_C"
"VoltageScale_V"
"CurrentScale_A"
"RelayType"
"OVPVoltage_V"
"UVPVoltage_V"
"OCPForwardCurrent_A"
"OCPReverseCurrent_A"
SetValSentChecksum
SetValTstChecksum

### MQTT Logging
If "isMQTTLog" is defined, logging to MQTT is enabled. Should be disabled for productive use
#define isMQTTLog   // logging to MQTT, topic esp/mqttDeviceString/log

## Credits
Extracting information from the JuncTek battery monitor is based on the work done by PLJaKobs:
https://github.com/pljakobs/JuncTek_Batterymonitor 
I have expanded on his library a bit:
- Adapt to PlatformIO
- Make it compatible to ESP32 hardware serial
- added code to ensure that the RSE pin of a SP3485 based board (e.g. Waveshare RS485 Board (3.3V) ) is used
