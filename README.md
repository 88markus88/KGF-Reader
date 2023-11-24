# KGF-Reader 
MQTT and ve.direct Bridge for JuncTek KGF-110 (and compatible) battery monitor, based on ESP32. Project is built using Platformio and Arduino framework.

I had started to use openDTU-onBattery <br>
https://github.com/helgeerbe/OpenDTU-OnBattery <br>
and wanted to use my JuncTek KGF-110 battery monitor to provide data to it, to enable better control 
of the battery charging and discharging.

Unfortunatly openDTU-onBattery does not (yet) include an interface to provide the battery data via MQTT, so I decided to make an attempt to send these data via the ve.dierct interface that is present for a Victron SmartShunt. 

KGF-Reader does just this: it collects data from a JuncTek KGF-110 battery monitor via RS-485 interface and sends them out to a serial port in ve.direct format. These data are accepted by openDTU-onBattery as Victron SmartShunt data

## Hardware
The software runs on an Expressiv ESP32. It has been tested on a DevKitC by AzDelivery.
RS-485 interface used is a Waveshare RS485 Board, using 3.3V logic: <br>https://www.waveshare.com/rs485-board-3.3v.htm <br>
Up to 3 temperature sensors DS18B20 are also included. Additional sensors can simply be added in parallel to the one shown in the schematic.

The schematic is simplified by just showing those pins of the <i>openDTU-onBattery</i> that should be used for connection to <i>KGF-Reader</i> 
Since I could not find the correct R485 module as Fritzing file, the wires to the RS485 breakout are not shown in the right order. Also, the Waveshare breakout has only one RSE  (receive/send enable) pin, instead of the separate RE (Receive enable) and DE (Data enable) Pins shown here. For the Waveshare board there is only one wire {white], and one pin on the breakout board.

![Schematic](https://github.com/88markus88/KGf-Reader/blob/main/Pictures/KGf-Reader_Steckplatine.png)

## MQTT
KGH-Reader also sends the battery monitor data to a MQTT server.

### Credentials
Wifi credentials and MQTT server credentials have to be provided in <i>"credentials.h"</i>. Only a template for this file is included in this repository, you will have to fill in your own information before compiling, and rename the file to <i>"credentials.h"</i>:<br>
```
// Wifi credentials
char default_ssid[] = "My-Wifi";
char default_pass[] = "my-password";

// credentials for MQTT server
char mqttDefaultServer[]    = "192.168.178.100";
char mqttDefaultUser[]      = "my-username";
char mqttDefaultPaSSWORD[]  = "my-password";
```

### MQTT topics
The base topic is <i><b>esp32/< mqttDeviceString >/< mqttSensorKGF110ID >/.. </b></i><br>
mqttDeviceString and mqttSensorKGF110ID are static and set in <i>"KG-F_Reader.h"</i><br><br>
Example:
```
  static char mqttSensorKGF110ID[] = "BTG002";
  static char mqttDeviceString[]   = "KG-F110";
```
results in a MQTT base string of:
```
  esp32/KG-F110/BTG002/
```
The following topics are available:<br>
Measurement values - updated every 2 seconds<br>
```
Voltage_V:            Voltage [V]
Current_A:            Current [A]
Power_W:              Power [W]
RemainingCapacity_Ah: Remaining battery capacity in [Ah]
SetCapacity_Ah:       Total (Set) battery capacity [Ah]
Temperature_C:        Temperature measured by KGH-110 battery sensor [°C]
CumulativeAhOut_Ah:   Sum of Ampere hours discharged [Ah]
Uptime_sec:           Uptime of the battery monitor [sed]
Uptime_Str:           Uptime of the battery monitor as string  [day, hour:min:sec]
BatteryLifeLeft_min:  Remaining time until battery is full or empty [min]
BatteryLifeLeft_Str:  Remaining time until battery is full or empty as string [day, hour:min:sec]
EnergyIn_Wh:          Energy charged into the battery [Watt hours]
ProtectionTemp_C:     Protection Temperature: relay is switched if this temperature is exceeded
SOC:                  State of charge [‰]
CE:                   Charge exhausted [mAh]
MeasValSentChecksum:  Checksum of the "measured values" data block as sent by KGH-110
MeasValTstChecksum:   Checksum of the "measured values" data block as calculated from the actual data
```
Set values - updated every 30 seconds
```
SetCapacity_Ah:    Total capacity of the battery, set by user [Ah]
ProtectionTemp_C
ProtectionRecoveryTime_s
ProtectionDelayTime_s
VoltageCalValue_V
CurrentCalValue_A
TempCalValue_C
VoltageScale_V
CurrentScale_A
RelayType
OVPVoltage_V
UVPVoltage_V
OCPForwardCurrent_A
OCPReverseCurrent_A
SetValSentChecksum: Checksum of the "set values" data block as sent by KGH-110
SetValTstChecksum:  Checksum of the "set values" data block as calculated from the actual data
```
DS19B20 Temperature data. Zero or upt to 3 temperature sensors are automatically detected and the temperatures in °C are sent to the MQTT server with the following topics:
```
esp32/KG-F110/DS18B20/DSTemperature1
esp32/KG-F110/DS18B20/DSTemperature2
esp32/KG-F110/DS18B20/DSTemperature3
```
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
