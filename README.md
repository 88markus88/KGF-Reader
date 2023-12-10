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

## ve.Direct
KGF-Reader emulates the data sent by a Victron SmartShunt and sends them via serial interface.
The data are sent according to the Victron ve-Direct manual, and in the same format as observed a SmartShunt.

The Victron SmartShunt sends the data in text format in two blocks, here an example as logged by openDTU-onBattery:
Block 1<br>
```
20:03:39.756 > [Victron SmartShunt] Text Event PID: Value: 0XA389 <br>
20:03:39.766 > [Victron SmartShunt] Text Event V: Value: 50998 <br>
20:03:39.774 > [Victron SmartShunt] Text Event I: Value: -8990 <br>
20:03:39.782 > [Victron SmartShunt] Text Event P: Value: -458 <br>
20:03:39.790 > [Victron SmartShunt] Text Event CE: Value: -70658 <br>
20:03:39.798 > [Victron SmartShunt] Text Event SOC: Value: 256 <br>
20:03:39.809 > [Victron SmartShunt] Text Event TTG: Value: 31 <br>
20:03:39.816 > [Victron SmartShunt] Text Event ALARM: Value: OFF <br>
20:03:39.826 > [Victron SmartShunt] Text Event AR: Value: 0 <br>
20:03:39.836 > [Victron SmartShunt] Text Event BMV: Value: SMARTSHUNT 500A/50MV <br>
20:03:39.845 > [Victron SmartShunt] Text Event FW: Value: 0414 <br>
20:03:39.854 > [Victron SmartShunt] Text Event MON: Value: 0 <br>
```
Binary format:<br>
```
20:03:39.877 > [VE.Direct] serial input (137 Bytes): <br>
20:03:39.884 > [VE.Direct] 0d 0a 50 49 44 09 30 78 41 33 38 39 0d 0a 56 09  <CR><LF>PID<t>0xA389<CR><LF>V<t><br>
20:03:39.892 > [VE.Direct] 35 30 39 39 38 0d 0a 49 09 2d 38 39 39 30 0d 0a  50998<CR><LF>I<t>-8990<CR><LF><br>
20:03:39.901 > [VE.Direct] 50 09 2d 34 35 38 0d 0a 43 45 09 2d 37 30 36 35  P<t>-458<CR><LF>CE<t>-7065<br>
20:03:39.909 > [VE.Direct] 38 0d 0a 53 4f 43 09 32 35 36 0d 0a 54 54 47 09  8<CR><LF>SOC<t>256<CR><LF>TTG<t><br>
20:03:39.917 > [VE.Direct] 33 31 0d 0a 41 6c 61 72 6d 09 4f 46 46 0d 0a 41  31<CR><LF>Alarm<t>OFF<CR><LF>A<br>
20:03:39.927 > [VE.Direct] 52 09 30 0d 0a 42 4d 56 09 53 6d 61 72 74 53 68  R<t>0<CR><LF>BMV SmartSh<br>
20:03:39.936 > [VE.Direct] 75 6e 74 20 35 30 30 41 2f 35 30 6d 56 0d 0a 46  unt 500A/50mV<CR><LF>F<br>
20:03:39.944 > [VE.Direct] 57 09 30 34 31 34 0d 0a 4d 4f 4e 09 30 0d 0a 43  W<t>0414<CR><LF>MON<t>0<CR><LF>C<br>
20:03:39.969 > [VE.Direct] 68 65 63 6b 73 75 6d 09 06                       hecksum<t><0x06><br>
```
Block 2<br>
```
20:03:40.133 > [Victron SmartShunt] Text Event H1: Value: -81084 <br>
20:03:40.142 > [Victron SmartShunt] Text Event H2: Value: -81084 <br>
20:03:40.150 > [Victron SmartShunt] Text Event H3: Value: -47599 <br>
20:03:40.157 > [Victron SmartShunt] Text Event H4: Value: 5 <br>
20:03:40.166 > [Victron SmartShunt] Text Event H5: Value: 0 <br>
20:03:40.173 > [Victron SmartShunt] Text Event H6: Value: -1429486 <br>
20:03:40.181 > [Victron SmartShunt] Text Event H7: Value: 12012 <br>
20:03:40.255 > [Victron SmartShunt] Text Event H8: Value: 58696 <br>
20:03:40.265 > [Victron SmartShunt] Text Event H9: Value: 3904892 <br>
20:03:40.277 > [Victron SmartShunt] Text Event H10: Value: 2 <br>
20:03:40.284 > [Victron SmartShunt] Text Event H11: Value: 0 <br>
20:03:40.292 > [Victron SmartShunt] Text Event H12: Value: 0 <br>
20:03:40.299 > [Victron SmartShunt] Text Event H15: Value: 0 <br>
20:03:40.311 > [Victron SmartShunt] Text Event H16: Value: 0 <br>
20:03:40.319 > [Victron SmartShunt] Text Event H17: Value: 7420 <br>
20:03:40.328 > [Victron SmartShunt] Text Event H18: Value: 7501<br>
```
Binary Format:<br>
```
20:03:40.351 > [VE.Direct] serial input (157 Bytes): <br>
20:03:40.360 > [VE.Direct] 0d 0a 48 31 09 2d 38 31 30 38 34 0d 0a 48 32 09 <br>
20:03:40.371 > [VE.Direct] 2d 38 31 30 38 34 0d 0a 48 33 09 2d 34 37 35 39 <br>
20:03:40.377 > [VE.Direct] 39 0d 0a 48 34 09 35 0d 0a 48 35 09 30 0d 0a 48 <br>
20:03:40.384 > [VE.Direct] 36 09 2d 31 34 32 39 34 38 36 0d 0a 48 37 09 31 <br>
20:03:40.392 > [VE.Direct] 32 30 31 32 0d 0a 48 38 09 35 38 36 39 36 0d 0a <br>
20:03:40.404 > [VE.Direct] 48 39 09 33 39 30 34 38 39 32 0d 0a 48 31 30 09 <br>
20:03:40.408 > [VE.Direct] 32 0d 0a 48 31 31 09 30 0d 0a 48 31 32 09 30 0d <br>
20:03:40.418 > [VE.Direct] 0a 48 31 35 09 30 0d 0a 48 31 36 09 30 0d 0a 48 <br>
20:03:40.428 > [VE.Direct] 31 37 09 37 34 32 30 0d 0a 48 31 38 09 37 35 30 <br>
20:03:40.437 > [VE.Direct] 31 0d 0a 43 68 65 63 6b 73 75 6d 09 de<br>
```

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

For timer functions the SIMPLETIMER library is used. See http://playground.arduino.cc/Code/SimpleTimer

