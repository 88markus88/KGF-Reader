/**************************************************************
KG-F_Reader: Reads data via MQTT from KG-F 110 battery monitor
Uses a MQTT board with 3.3V , e.g. Waveshare RS485 communication board, SP3485 on board, 3.3V
https://www.waveshare.com/rs485-board-3.3v.htm
This board uses TX, RX and RSE: High vor Driver enable (TX) and Low for Receiver enable (RX)
Pins used: TXD2 = GPIO 17, RXD2= GPIO 16, RSE= GPIO 21
Attention: when sending RSE needs to be set to HIGH for the duration of the asynchronous sending, and then back to LOW. 
To ensure that RSE remains HIGH for the duration of the sending, Serial.flush() must be used after the serial.print
**************************************************************/

#define getNTPTIME       // get time from NTP server

#define isOTA     // enable OTA Update functionality
#define isOneDS18B20 // include DS18B20
  #define noDS18B20Sensors 3  // number of DS18B20 expected
  #define minDiffDS18B20      0.05 // min data difference from previous value to start sending info

#define isSyncBattery // sync Battery function enabled

#define isMQTT    // include mqtt functionality
#define isMQTTLog   // logging to MQTT, topic esp/mqttDeviceString/log

#define isVEDIRECT // ve.direct emulation on Serial1

#define logSerial // logging to serial

#ifndef _BATTERYMONITORH_
  #include "JuncTek_BatteryMonitor.h"
#endif  
#ifndef  KG_F_READER_H
  #include "KG-F_Reader.h"
#endif

#ifndef time_h
  #include "time.h"
#endif  

#ifndef HardwareSerial_h
  #include <HardwareSerial.h>
#endif  

#ifndef CREDENTIALS_H
  #include "credentials.h"
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
// includes for OTA over the air Updates 
#ifdef isOTA
  #ifndef WiFi_h
    #include <WiFi.h>
  #endif
  #ifndef ESP32MDNS_H
    #include <ESPmDNS.h>
  #endif  
   #ifndef _WIFIUDP_H_
    #include <WiFiUdp.h>
  #endif  
  #ifndef __ARDUINO_OTA_H
    #include <ArduinoOTA.h>
  #endif  
#endif

//HardwareSerial battMon(19,18); 
//use hardware serial port 1 (0 is connected to USB normally, so will be used for debug here)
BatteryMonitor bm1;  
/* this is just calling the empty constructor so the object is ready. I didn't want to use a 
/ parametrized constructor and opted for a begin() function instead as it seems more in line 
/ with Arduino practice
*/

// RS485 setup with ESP32       
// SoftwareSerial SerialSW(RXSW, TXSW); // RX=26 , TX =27
#define maxPRINTSTRINGLEN 500
char printstring[maxPRINTSTRINGLEN];  // for logging

// constructor for class kgfData
kgfData::kgfData():
  DataValid{false},   // initialisierer  
  Voltage{0.0}, Current{0.0}, Power{0.0}, RemCapa{0.0}, SetCapa{0.0}, CumulAhOut{0.0}, EnergyIn{0.0},
  OVPVoltage{0.0}, UVPVoltage{0.0}, OCPForwardCurrent{0.0}, OCPReverseCurrent{0.0}, OPPPower{0.0},
  Uptime{0}, LifeLeft{0}, ProtectionTemp{0}, ProtectionRecoveryTime{0}, ProtectionDelayTime{0}, 
  VoltageCalValue{0}, CurrentCalValue{0}, TempCalValue{0}, VoltageScale{0}, CurrentScale{0}, RelayType{0}, Temp{0},
  UptimeString{""}, LifeLeftString{""} 
 {}

// destructor for class kgfData
kgfData::~kgfData(){}

// global instance of kgfdata class
kgfData kgf{};

/*
bool kgfDataValid;  
float KGF110Voltage, KGF110Current, KGF110Power, KGF110RemCapa, KGF110SetCapa,
  KGF110CumulAhOut,  KGF110EnergyIn;
long  KGF110Temp,  KGF110Uptime, KGF110LifeLeft;
char KGF110UptimeString[40], KGF110LifeLeftString[40];
long KGF110ProtectionTemp, KGF110ProtectionRecoveryTime, KGF110ProtectionDelayTime, 
  KGF110VoltageCalValue, KGF110CurrentCalValue, KGF110TempCalValue, KGF110VoltageScale, KGF110CurrentScale, 
  KGF110RelayType;
float KGF110OVPVoltage, KGF110UVPVoltage, KGF110OCPForwardCurrent, KGF110OCPReverseCurrent, KGF110OPPPower;
*/

// local function prototypes
String toStringIp(IPAddress ip);
bool getKGF110Data();
void restartDS18B20MeasurementFunction();
void vedirectHandler();
void batteryPercentHandler();
void printLocalTime(char* printstring, int mode);
void mqttHandlerQuick();
void mqttHandlerSlow();


/**************************************************!
     @brief    function to convert an IP address into a string
    @details  Bluetooth connection is opened by caller. Then listens for ssid:[ID] and pass:[pw], and returns these
              can also use a scan, but since that makes wifi unreliable on ESP32: better not.
              Not included in #defines for Captive portal, since also used without it.
    @param IPAddress ip : Object containing the IP address
    @return String containing the IP address
 ***************************************************/

  String toStringIp(IPAddress ip)
  {
    String res = "";
    for (int i = 0; i < 3; i++)
    {
      res += String((ip >> (8 * i)) & 0xFF) + ".";
    }
    res += String(((ip >> 8 * 3)) & 0xFF);
    return res;
  }

/******************************************************************************************/

// function to handle all logging output. To serial, into file on SD, to Blynk terminal
void logOut(int logLevel, char* printstring, unsigned int MsgID, unsigned int MsgSeverity)
{
  if(logLevel >= logLEVEL) 
  {  
    char timestring[50]="";      
    char outstring[maxPRINTSTRINGLEN + 50];

    if((strlen(printstring)<1) || (strlen(printstring)>maxPRINTSTRINGLEN))
    {
      strcpy(printstring,"invalid log string");
      MsgID = msgLogError;
      MsgSeverity = msgErr;
    }

    #ifdef isLEDHeartbeat
      heartbeatStatus = !heartbeatStatus;
      // digitalWrite (HEARTBEATPIN, heartbeatStatus);
      digitalWrite (HEARTBEATPIN, HIGH);
    #endif

    #ifdef getNTPTIME
      if(TimeIsInitialized)
      {
        printLocalTime(timestring, 5);
      }  
    #endif 

    #ifdef logSerial
      Serial.print(printstring);
    #endif

    #if defined logSD && defined isSD      
      strcpy(outstring, timestring);
      strcat(outstring, printstring);      
      logSDCard(outstring);
    #endif

    #ifdef isLEDHeartbeat
      heartbeatStatus = !heartbeatStatus;
      digitalWrite (HEARTBEATPIN, LOW);
    #endif

    #ifdef isSyslog
      if(WiFi.status() == WL_CONNECTED)
      {
        sprintf(outstring, "%d %d %s", MsgID, MsgSeverity, printstring);
        syslog.log(LOG_INFO | LOG_USER, outstring);
        // Log message can be formated like with printf function.
        // syslog.logf(LOG_ERR,  "This is error message no. %d", iteration);  
        // Log Levels: LOG_INFO, LOG_ERR, LOG_DAEMON 
      }
    #endif

    #ifdef isMQTTLog
      char topicStr[200];
      sprintf(topicStr,"esp32/%s/log/%d/%d",mqttDeviceString, MsgID, MsgSeverity);
      strcpy(outstring, timestring);
      strcat(outstring, printstring); 

      //sprintf(printstring, "test logOut 2. Topic: _%s_ _%s_ %d",topicStr, outstring, strlen(outstring));
      //Serial.print(printstring);
      if(mqttClient.connected())
      //mqttClient.publish(topicStr, outstring); //payload: outstring
      mqttClient.publish(topicStr, outstring, strlen(outstring)); //payload: outstring
      //sprintf(printstring, "test logOut 2");  
      //Serial.print(printstring);
    #endif
  }
}


 /**************************************************!
  @brief    Connect or reconnect to WiFi
  @details  gets ssid and password for the network to connect to.
            function attempts 6 times to connect to the network
            If not successfull after the sixth time, returns false.
  @param    char* ssid  - SSID of the network to connect to
  @param    char* pass  - password of the network to connect to
  @param    int noRetries - number of retries in connecting. Default is 7.
  @return   bool : true if connected, false if not.
  ***************************************************/
// Connect or reconnect to WiFi
bool connectToWiFi(char* ssid, char* pass, int noRetries)
{
  int i=0;
  long starttime, endtime;
  WiFi.disconnect();   
  delay(500);  
  // if((!wifiClient.connected()) || (WiFi.status() != WL_CONNECTED))
  //{
    sprintf(printstring,"Attempting to connect to SSID: _%s_ PASS: _%s_\n", ssid, pass);
    Serial.println(printstring);

    #ifdef isDisplay
      display.clearDisplay(); 
      display.setTextSize(1);
      sprintf(printstring,"Connecting to SSID: ");
      display.setCursor(0, 0);
      display.println(printstring);
      sprintf(printstring,"%s ", ssid);
      display.setCursor(0, 12);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif 

    // 0 = WL_IDLE_STATUS
    // 1 = WL_NO_SSID_AVAIL 
    // 2 = WL_SCAN_COMPLETED
    // 3 = WL_CONNECTED 
    // 4 = WL_CONNECT_FAILED 
    // 5 = WL_CONNECTION_LOST
    // 6 = WL_DISCONNECTED 

    // see following link for discussion of connection issues
    // https://github.com/espressif/arduino-esp32/issues/2501
    // der entscheidende Punkt, war nach dem WiFi.begin() lange genug zu warten
    // da hilft die wartende Funktion WiFi.waitForConnectResult();

    starttime = millis();
    // WiFi.mode(WIFI_STA);             // Config module as station only.
    int status = WiFi.status();
    while((status != WL_CONNECTED) && (i<=noRetries))
    {
      i++;
      if(status == WL_CONNECT_FAILED)
      {
        WiFi.disconnect(true);      // new 11.10.21: disconnect and connect in the loop. New 19.11.: only if connect failed
        delay(500);
      }  
      esp_task_wdt_reset();   // keep watchdog happy

      WiFi.begin(ssid, pass);
      delay(i*500);  
      // status = WiFi.status();      // this call returns the result right away
      status = WiFi.waitForConnectResult(); // this call may wait a long time

      sprintf(printstring,".%d ",status);
      Serial.print(printstring);
       #ifdef isDisplay
        display.setCursor(15*(i-1), 24);
        display.println(printstring);
        display.display();          // transfer buffer content to display
      #endif  
    } 
    esp_task_wdt_reset();   // keep watchdog happy
  //}
  endtime = millis();
  if(WiFi.status() == WL_CONNECTED)
  {
    sprintf(printstring,"Connected after %5.2f sec IP: %s\n",(float)(endtime-starttime)/1000, toStringIp(WiFi.localIP()).c_str());
    Serial.println(printstring);
    logOut(2,printstring, msgWifiConnected, msgInfo);
    #ifdef isDisplay
      sprintf(printstring,"Connected in %5.2f s ",(float)(endtime-starttime)/1000);
      display.setCursor(0, 36);
      display.println(printstring);
      sprintf(printstring,"IP: %s ",toStringIp(WiFi.localIP()).c_str());
      display.setCursor(0, 48);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif  
    return(true);
  }  
  else
  {
    sprintf(printstring,"NOT Connected after %5.2f sec",(float)(endtime-starttime)/1000);
    Serial.println(printstring);
    logOut(2,printstring, msgWifiConnected, msgInfo);
    #ifdef isDisplay
      display.setCursor(0, 48);
      display.println(printstring);
      display.display();          // transfer buffer content to display
    #endif  
    return(false);
  }  
}  

#ifdef getNTPTIME
  /**************************************************!
  @brief    get local system date/time and convert it to proper string format
  @details  gets local time, and converts it to strings using "strftime"
  @details  http://www.cplusplus.com/reference/ctime/strftime/
  @param    timestring returned string with the date/time info
  @param    mode  deterines format to be returned. 1: only print
  @return   void
  ***************************************************/
  void printLocalTime(char* timestring, int mode)
  {
    char timeHour[3];
    char timeMinute[3];
    char timeSecond[3];
    char timeWeekDay[10];
    char timeDay[3];
    char timeMonth[7];
    char timeMonthShort[5];
    char timeYear[5];
    char timeYearShort[3];
    char timeISODate[12];
    char timeISOTime[10];


    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println(F("Failed to obtain time"));
      strcpy(timestring,"<?time?>");
      return;
    }

    strftime(timeHour,3, "%H", &timeinfo);  
    strftime(timeMinute,3, "%M", &timeinfo);  
    strftime(timeSecond,3, "%S", &timeinfo);  
    strftime(timeWeekDay,10, "%A", &timeinfo);
    strftime(timeDay,3, "%d", &timeinfo);  
    strftime(timeMonth,4, "%b", &timeinfo);  
    strftime(timeMonthShort,3, "%m", &timeinfo);  
    strftime(timeYear,5, "%Y", &timeinfo);  
    strftime(timeYearShort,5, "%y", &timeinfo);  
    strftime(timeISODate,12, "%F", &timeinfo);  
    strftime(timeISOTime,10, "%T", &timeinfo); 

    // format modiefies for strftime()
    // http://www.cplusplus.com/reference/ctime/strftime/ 
    switch(mode)
    {
      // output time and date to timestring and / or serial port
      case 1: 
        Serial.println(&timeinfo,"%e.%m.%G - %H:%M:%S"); 
        break;
      case 2: 
        sprintf(timestring,"%s.%s.%s - %s:%s:%s\n",
          timeDay,timeMonth,timeYear,timeHour,timeMinute,timeSecond);
        // Serial.print(timestring);
      case 3: 
        sprintf(timestring,"%s - %s\n", timeISODate, timeISOTime);
        // Serial.print(timestring); 
        break;
      case 4: 
        sprintf(timestring,"%s.%s.%s-%s:%s:%s ", 
          timeDay,timeMonthShort,timeYearShort,timeHour,timeMinute,timeSecond);
      case 5: 
        // with daylight saving time indicator "timeinfo.tm_isdst"
        // sprintf(timestring,"%d %s%s%s-%s:%s:%s ", timeinfo.tm_isdst,
        //  timeDay,timeMonth,timeYearShort,timeHour,timeMinute,timeSecond);    
        sprintf(timestring,"%s%s%s-%s:%s:%s ", 
          timeDay,timeMonth,timeYearShort,timeHour,timeMinute,timeSecond);    
        // Serial.print(timestring); 
        break;  
      case 6: 
        sprintf(timestring,"%s:%s:%s ",timeHour,timeMinute,timeSecond);    
        // Serial.print(timestring); 
        break; 
      case 7: 
        sprintf(timestring,"%s%s%s-%s:%s:%s ", 
          timeDay,timeMonth,timeYearShort,timeHour,timeMinute,timeSecond);    
        // Serial.print(timestring); 
        break;    
      case 8: 
        sprintf(timestring,"%s",timeISODate);    
        // Serial.print(printstring); 
        break; 
        
      default: Serial.println("Invalid time print mode");
    }
    
    /*
    Serial.print("Day of week: ");
    Serial.println(&timeinfo, "%A");
    Serial.print("Month: ");
    Serial.println(&timeinfo, "%B");
    Serial.print("Day of Month: ");
    Serial.println(&timeinfo, "%d");
    Serial.print("Year: ");
    Serial.println(&timeinfo, "%Y");
    Serial.print("Hour: ");
    Serial.println(&timeinfo, "%H");
    Serial.print("Hour (12 hour format): ");
    Serial.println(&timeinfo, "%I");
    Serial.print("Minute: ");
    Serial.println(&timeinfo, "%M");
    Serial.print("Second: ");
    Serial.println(&timeinfo, "%S");

    Serial.println("Time variables");
    char timeHour[3];
    strftime(timeHour,3, "%H", &timeinfo);
    Serial.println(timeHour);
    char timeWeekDay[10];
    strftime(timeWeekDay,10, "%A", &timeinfo);
    Serial.println(timeWeekDay);
    Serial.println();
    */
  }

  /**************************************************!
  @brief    getNTPTime: start WLAN and get time from NTP server
  @details  routine to get time from an network time server via NTP protocol. Set system time.
  @details  https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
  @param    none
  @return   void
  ***************************************************/
 // Issue: incorrect setting of day when switching to daylight saving time
 // See: https://github.com/espressif/arduino-esp32/issues/3797

  void getNTPTime()
  {
    // char printstring2[80];
    // int tryCount = 0;

    Serial.print(F("Connecting to "));
    Serial.println(ssid);

    // new 22.11.21
    if(WiFi.status() != WL_CONNECTED)
      connectToWiFi(ssid, pass, 7);

    esp_task_wdt_reset();   // keep watchdog happy

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println(F("WiFi connected."));
      esp_task_wdt_reset();   // keep watchdog happy  
      
      struct tm timeinfo;
      // Init and get the time. try all 3 time servers in sequence, if first not successful
      // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, ntpServer2, ntpServer3);
      // works for one server:
      // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

      // improved for setting to the correct time zone directly
      // https://github.com/espressif/arduino-esp32/issues/3797 
      // https://remotemonitoringsystems.ca/time-zone-abbreviations.php
      configTzTime( defaultTimezone, ntpServer); //sets TZ and starts NTP sync

      if(getLocalTime(&timeinfo))
      {
        TimeIsInitialized = true;
        sprintf(printstring,"Successfully obtained time from first server %s",ntpServer);
        Serial.println(printstring);  
      }  
      else{  
        sprintf(printstring,"Failed to obtain time from first server %s",ntpServer);  
        Serial.println(printstring);   
        // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer2);
        configTzTime( defaultTimezone, ntpServer2); //sets TZ and starts NTP sync
        if(getLocalTime(&timeinfo)){
          TimeIsInitialized = true;
          sprintf(printstring,"Successfully obtained time from 2nd server %s",ntpServer2);
          Serial.println(printstring);  
        }  
        else{
          sprintf(printstring,"Failed to obtain time from 2nd server %s",ntpServer2);  
          Serial.println(printstring);  
          // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer3);
          configTzTime( defaultTimezone, ntpServer3); //sets TZ and starts NTP sync
          if(getLocalTime(&timeinfo)){
            sprintf(printstring,"Successfully obtained time from 3rd server %s",ntpServer3);
            Serial.println(printstring);  
            TimeIsInitialized = true;
          } 
          else{
            sprintf(printstring,"Failed to obtain time from 3rd server %s",ntpServer3);  
            Serial.println(printstring);  
            TimeIsInitialized = false;  
          }
        }
      }

      if(TimeIsInitialized){
        printLocalTime(printstring, 3);
        logOut(2,printstring, msgTimeInitialized, msgInfo);
      }  
      //disconnect WiFi as it's no longer needed
      //in EnvMonitor we do not disconnect
      // WiFi.disconnect(true);
      // WiFi.mode(WIFI_OFF);
    }
    else
    {
      Serial.println(F("WiFi NOT connected - time server connection not possible."));
      TimeIsInitialized = false;
    }  
  }
#endif


#ifdef isOneDS18B20

  volatile unsigned long LastMeasTimer;
  const long ds18b20MeasInterval = 1000; // measure every xxx ms

  // mutex could be needed for critical section, not implemented
  // static portMUX_TYPE my_mutex = portMUX_INITIALIZER_UNLOCKED;

  /**************************************************!
  @brief    Function to measure DS18B20 temperatures in endless loop. Designed to run in parallel
  @details  This task runs isolated on core 0 because sensors.requestTemperatures() is slow and blocking for about 750 ms
            gets temperature from DS18B20 via OneWire
            Temperature is measured by address, not by indes. This ensures that there is no mismatch, if a sensor
            goes temporarily offline (as fake sensors occasionally do)
            Does some checking. Sensor values indicate errors if out of range:
              -127: bad reading. 85: no measurement yet (to early after a restart)
            Uses global variables:
            - measuringInfactoryOngoing : If this flag is set, no measurements are taken (to not disturb timing)
            - stopDS18B20MeasureFlag : If this flag is set, no measurements are taken (to not disturb timing)
            - noDS18B20Connected : Number of DS18B20 which are connected
            - DS18B20Address[i][j] :  Address of Sensor i, each 8 bytes long (j= 0..7)
            Returns global variables
            - DS18B20Temperature[i] : raw measured temperature in °C for each sensor
            - LastMeasTimer : timer mark when last measurement has been done
            - notChangedCount : global counter for not changed values. Used to determine if measurements stopped
            - GetOneDS18B20Counter : global counter for cycles through this procedure. Used to determine if measurements stopped
  @param    parameter   Task input parameter, not used. Required for starting function
  @return   void
  ***************************************************/
  void GetOneDS18B20Temperature( void * parameter) 
  {
    float tmp1 = 0;
    // float tmp2 = 0;
    static int i,j;
    int localInfactoryFlag = false;
    uint8_t addr[8];          // uint8_t = unsigned char, 8 bit integer
    char printstring[40];

    #ifdef isInfactory433
      localInfactoryFlag = measuringInfactoryOngoing; // if infactory sensor connected, use global flag - otherwise always false
    #endif

    for (;;) {   // Endless loop
      if(millis()-LastMeasTimer > ds18b20MeasInterval)
      // there should be noDS18B20Connected sensors attached
      {
        if(localInfactoryFlag == false && stopDS18B20MeasureFlag == false)   // do not take measurements if infactory 433 MHz sensor is queried
        {
          sensors.requestTemperatures(); // Send the command to get temperatures
          // vTaskDelay(100);
          LastMeasTimer = millis();
          for(i=0;i<noDS18B20Connected;i++)
          {
            for(j=0;j<8;j++){
              addr[j] = DS18B20Address[i][j];
              // if(addr[j]<16) Serial.print("0");
              // Serial.print(addr[j],HEX);
            }  
            // Serial.println("");

            //*** alternative: by index: tmp1 = sensors.getTempCByIndex(i);
            tmp1 = sensors.getTempC(addr); 
            if(abs(DS18B20Temperature[i] - tmp1) < 0.01) // check: did the value change?
              notChangedCount++;              // global counter for not changed values
            else
              notChangedCount = 0;  
            if (tmp1 != -127 && tmp1 != 85)   //-127: bad reading. 85: no measurement yet 
            {
              DS18B20Temperature[i] = tmp1;
              sprintf(printstring,"%d %f\n", i, tmp1);
              Serial.print(printstring);
              Serial.print(i); 
              notMeasuredCount = 0;     // note that a measurement has been taken, set counter to zero
            }  else
            {
              DS18B20Temperature[i] = -111.11;
              Serial.print(":"); 
              LastMeasTimer = 0;
              notMeasuredCount++;       // note that measurements was not possible, increase counter
            }
          }
        }  
        else
        {
          sprintf(printstring,"No DS18B20 Measurement due to flag %d %d\n",
            localInfactoryFlag,stopDS18B20MeasureFlag);
          logOut(1,printstring, msgDS18B20NoMeasFlag, msgWarn);  
        }
      }
      else
      {
        vTaskDelay(200 / portTICK_PERIOD_MS); // delay for 200 ms
        // sprintf(printstring,"W %ld %ld ", millis(),LastMeasTimer);    
        // logOut(2,printstring, msgDefaultID, msgDefault);
        GetOneDS18B20Counter ++;  
      }  
      sprintf(printstring,".");
      logOut(1,printstring, msgDS18B20NoMeasFlag, msgWarn);
    }
  }

  //---- the following procedure determines the number of DS18B20, sets their precision to 12 bit and stores their addresses
  DeviceAddress tempDeviceAddress;
  #define TEMPERATURE_PRECISION 12   // precision 9..12 Bit

  /**************************************************!
  @brief    Determine number of DS18B20 sensors on 1Wire Bus and determine their addresses
  @details  First attempts sensors.getDeviceCount() to determine the number of devices, which does not work presently
            Then sets temperature precision to 12 bits for 3 devices 
            Finally uses sensors.getAddress() to count how many devices actually present.
            Fills the following global variables:
            noDS18B20Connected                      : Number of DS18B20 actually present
            DS18B20Address[noDS18B20Connected][i]   : their addresses
  @param    none
  @return   void
  ***************************************************/
   void adresseAusgeben(void) {
    byte i, j, k;
    uint8_t addr[8];          // uint8_t = unsigned char, 8 bit integer
    int numberOfDevices, noRepeats;
    char ps[80];
    long checksum[5], sum1, sum2;         // address checksums
    bool check, crcCheck;

    // loop to determine the addresses of the DS18B20 sensors. 
    // Repeat in case of crc check failure or duplicate addresses
    sprintf(printstring,"adresseAusgeben - looking for DS18B20 sensors \n");
    logOut(2,printstring, msgDS18B20Info, msgInfo);
    noRepeats = 0;
    do{
      esp_task_wdt_reset();   // keep watchdog happy
      // code 1: devices finden.
      // this method does not work for ESP32
      delay(1000);
      oneWire.reset_search(); // reset search of oneWire devices
      numberOfDevices = sensors.getDeviceCount();
  
      sprintf(printstring,"sensors.getDeviceCount found %d Devices \n", numberOfDevices);
      logOut(2,printstring, msgDS18B20Info, msgInfo);

      numberOfDevices = noDS18B20Sensors;   // number of DS18B20 expected; /// temporary
      // Setzen der Genauigkeit
      for(i=0; i<numberOfDevices; i++) {
        if(sensors.getAddress(tempDeviceAddress, i)) {
          sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
          Serial.print(F("Sensor "));
          Serial.print(i);
          Serial.print(F(" had a resolution of "));
          Serial.println(sensors.getResolution(tempDeviceAddress), DEC);
        }
      }
      Serial.println("");
      numberOfDevices = sensors.getDeviceCount();   // does not function with OneWire library 2.3.5 (claimed to be ok with 2.3.3)
      sprintf(printstring,"Found %d sensors\n", numberOfDevices);
      logOut(2,printstring, msgDS18B20Info, msgInfo);
      esp_task_wdt_reset();   // keep watchdog happy

      // code 2: find devices by searching on the onewire bus across all addresses. 
      // workaround, works apparently for ESP32

      sprintf(printstring,"Searching 1-Wire-Devices...\n\r");// "\n\r" is NewLine
      logOut(2,printstring, msgDS18B20Info, msgInfo);
      
      //critical section. Could help with incorrect sensor detection
      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html#critical-sections-disabling-interrupts
      //portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
      //portENTER_CRITICAL(&myMutex);

      crcCheck = true;  // reset crc Check flag
      for(j=1; j<=20 ; j++)
      {
        noDS18B20Connected = 0;
        oneWire.reset_search(); // reset search of oneWire devices
        delay(100);
        while(sensors.getAddress(addr, noDS18B20Connected)) {  

          sprintf(printstring,"1-Wire-Device %d found with Adress: ", noDS18B20Connected);
          // logOut(2,printstring, msgDS18B20Info, msgInfo);
          esp_task_wdt_reset();   // keep watchdog happy

          // Fletcher checksum algorithm https://de.wikipedia.org/wiki/Fletcher%E2%80%99s_Checksum 
          checksum[noDS18B20Connected] = 0;
          sum1=0;
          sum2=0;
          for( i = 0; i < 8; i++) {
            DS18B20Address[noDS18B20Connected][i] = addr[i];
            sum1 = (sum1 + addr[i]) % 255;
            sum2 = (sum2 + sum1) % 255;
            //checksum[noDS18B20Connected] += (i+1)*addr[i];
            // Serial.print("0x");
            /*
            if (addr[i] < 16) {
            strcat(printstring,"0");
            }
            sprintf(printstring,"%s%d",printstring, addr[i]);
            if (i < 7) {
              strcat(printstring," ");
            }
            */
          }
          checksum[noDS18B20Connected] = 256 * sum1 + sum2; // checksum is two sub-sums combined
          // strcat(printstring,"\n");
          //logOut(2,printstring, msgDefaultID, msgInfo);
          if ( OneWire::crc8( addr, 7) != addr[7]) {
            // sprintf(printstring,"CRC is not valid!\n\r");
            // logOut(2,printstring, msgDS18B20Info, msgWarn);
            crcCheck = false;
            //return;
          }
          noDS18B20Connected ++;    // one more device found
        } // while

        // check if no two addresses are identical
        check = true;
        /*
        for(i = 1; i<noDS18B20Connected; i++){
          if(checksum[i] == checksum[i-1])
            check = false; 
        }
        */
        for(i = 1; i<noDS18B20Connected; i++)
          for(k = 0; k<i; k++)
            if(checksum[i] == checksum[k])
              check = false; 

        if ((noDS18B20Connected >= noDS18B20Sensors) && (check == true)) // exit for loop if expected number of sensors has been found
          break;
        delay(j * 100);
      } // for j
      #ifdef isBLYNK
        Blynk.connect();
        delay(600);
      #endif  
      //portEXIT_CRITICAL(&myMutex); // exit critical section
      noRepeats++;
    }while((check==false || crcCheck==false) && noRepeats < 3); // repeat until the check is true

    sprintf(printstring,"Check: %d \n", check);
    logOut(2,printstring, msgDS18B20Info, msgInfo);
    sprintf(printstring,"Found 1-Wire-Devices: %d in %d loop runs\n", noDS18B20Connected, j);
    logOut(2,printstring, msgDS18B20Info, msgInfo);

    for(j=0 ; j< noDS18B20Connected; j++) 
    {
      sprintf(printstring, "Addr [%d] (Checksum: %ld): ", j, checksum[j]);
      for( i = 0; i < 8; i++) {
        // Serial.print(DS18B20Address[noDS18B20Connected][i], HEX);
        // Serial.print(" ");
        itoa(DS18B20Address[j][i], ps, 16);
        strcat(printstring, ps); 
        strcat(printstring," ");
      }
      strcat(printstring,"\n");
      logOut(2,printstring, msgDS18B20Info, msgInfo);
    }  
    oneWire.reset_search(); // reset search of oneWire devices
    
    return;
  }

  /**************************************************!
  @brief    Complete restart of the DS18B20 measurement function, incl. parallel running meas. function
  @details  Terminates the detached task (Task1) running to do DS10B20 measurements in parallel
            Switches power for the 1Wire bus off for 2 seconds, then on again.
            Then determines the number and addresses of DS18B20 sensors
            And finally restarts the measuring function running in parallel as Task1 (GetOneDS18B20Temperature)
  @param    none
  @return   void
  ***************************************************/
  void restartDS18B20MeasurementFunction()
  {
    noDS18B20Restarts++;
    sprintf(printstring,"XXXX Restarting DS18B20 measuring function XXXX !!! %d \n", noDS18B20Restarts);
    logOut(2,printstring, msgDS18B20Restart, msgWarn);  
    stopDS18B20MeasureFlag = true;   // use this flag to stop measurements with DS18B20
    esp_task_wdt_reset();   // keep watchdog happy

    vTaskDelete(Task1);              // delete the measurement task

    // switch off Power (via GPIO 32)
    pinMode(POWER_ONEWIRE_BUS, OUTPUT);
    digitalWrite(POWER_ONEWIRE_BUS, LOW);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay for 2000 ms
    esp_task_wdt_reset();   // keep watchdog happy

    // switch on Power (via GPIO 32)
    pinMode(POWER_ONEWIRE_BUS, OUTPUT);
    digitalWrite(POWER_ONEWIRE_BUS, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS); // delay for 1000 ms
    // Start OneWire for DS18B20
    sensors.begin();
    vTaskDelay(700 / portTICK_PERIOD_MS); // delay for 1000 ms

    adresseAusgeben();       // find addresses of DS18B20 devices
    esp_task_wdt_reset();    // keep watchdog happy

    // Create GetTemperature task for core 0, loop() runs on core 1
    xTaskCreatePinnedToCore(
      GetOneDS18B20Temperature, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task. higher number is higher priority, https://esp32.com/viewtopic.php?t=10629 */
      &Task1,  /* Task handle. */
      1); /* Core where the task should run */
    stopDS18B20MeasureFlag = false; // reset flag to start measurements
  }


/**************************************************!
    @brief    send one DS18B20 Temperature, called from MQTT handler
    @details  New 2022-12-18
    @param int sNo: Number of sensor, 0..2
    @return   void
  ***************************************************/
 // helper function to check if data are equal within limit
  bool isEqual(double a, double b, double limit)
  {
    if(abs(a-b) < limit)
      return (true);
    else
      return (false);  
  }

  void mqttSendDS18B20(int sNo)
  {
    char payloadStr[100]; //50
    char topicStr[100];   //50
    char printstring2[250]; //180
    // DS18B20 data 
    double temp;
    double limit = -110.0;
    static double last_DSTemp[MAX_NO_DS18B20] = {-111, -111, -111, -111, -111, -111, -111, -111, -111, -111};
    float time_sec;
    static float last_TimeDSSent[MAX_NO_DS18B20] = {0, 0, 0, 0, 0,  0, 0, 0, 0, 0};

    time_sec = (float)millis()/1000;

    sprintf(printstring,"DS18B20[%d] sum: %3.2f n: %d \n",sNo, sum_MQTT_calDS18B20Temperature[sNo],n_MQTT_calDS18B20Temperature[sNo]);
    logOut(2,printstring,msgMQTTSendDS10B20, msgInfo);
    if(n_MQTT_calDS18B20Temperature[sNo] > 0)
      temp = sum_MQTT_calDS18B20Temperature[sNo] / n_MQTT_calDS18B20Temperature[sNo];
    else
      temp = -111.11;  
    if( 
        (!isEqual(temp,last_DSTemp[sNo],minDiffDS18B20)   // sufficiently large change
        || (time_sec > last_TimeDSSent[sNo] + 120))           // enough time elapsed
        && (temp > limit)                                 // and valid data
      )
      {    
        // insert here if large temp jump: send last_temperature again to avoid unrealistical curve form
        if(!isEqual(temp,last_DSTemp[sNo],minDiffDS18B20*10) && (last_DSTemp[sNo] > -110)) // if jump larger than 10 x minimum recognized temp difference
        {
          sprintf(topicStr,"esp32/%s/%s/%s%d",mqttDeviceString, mqttSensorDS18B20, mqttDS18B20Temperature, sNo+1);
          sprintf(payloadStr,"%3.2f",last_DSTemp[sNo]);
          // caller! mqttSendItemCounter++;
          sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topicStr, payloadStr);
          logOut(2,printstring, msgMQTTSendDS10B20, msgInfo);
          mqttClient.publish(topicStr, payloadStr);

          sprintf(printstring2," MQTT Sent DS18B20 %d temperature % 4.1f before strong rise \n", sNo, last_DSTemp[sNo]);
          strcat(printstring, printstring2);
          logOut(2,printstring, msgMQTTSendDS10B20, msgInfo);          
        }
        sprintf(printstring2," Tmp%d: toMQTT cal: %5.2f last: %5.2f act: %5.2f sum: %5.2f n: %d\n",
          sNo, calDS18B20Temperature[0], last_DSTemp[sNo], temp, sum_MQTT_calDS18B20Temperature[0], n_MQTT_calDS18B20Temperature[0]);
        strcat(printstring, printstring2);
        logOut(2,printstring, msgMQTTSendDS10B20, msgInfo);

        last_DSTemp[sNo] = temp;
        sum_MQTT_calDS18B20Temperature[sNo] = 0;
        n_MQTT_calDS18B20Temperature[sNo] = 0;

        sprintf(topicStr,"esp32/%s/%s/%s%d",mqttDeviceString, mqttSensorDS18B20, mqttDS18B20Temperature,sNo+1);
        sprintf(payloadStr,"%3.2f",temp);
        last_TimeDSSent[sNo] = time_sec; 
        // caller! mqttSendItemCounter++;
        sprintf(printstring,"Strings to MQTT: [%s] [%s] at time %5.2f\n", topicStr, payloadStr, time_sec);
        logOut(2,printstring, msgMQTTSendDS10B20, msgInfo);
        mqttClient.publish(topicStr, payloadStr);
      }  
      else{
        sprintf(printstring2,"\n Tmp%d: notMQTT cal: %5.2f last: %5.2f act: %5.2f sum: %5.2f n: %d last sent:%5.3f[s] now:%5.3f[s]",
          sNo,calDS18B20Temperature[0], last_DSTemp[sNo], temp, sum_MQTT_calDS18B20Temperature[0], n_MQTT_calDS18B20Temperature[0], last_DSTemp[sNo], time_sec);
        strcat(printstring, printstring2);
        logOut(2,printstring, msgMQTTSendDS10B20, msgWarn);
        sum_MQTT_calDS18B20Temperature[sNo] = 0;
        n_MQTT_calDS18B20Temperature[sNo] = 0;
      }
  }
#endif  // DS18B20

#ifdef isMQTT
  /**************************************************!
    @brief    MQTT connect / reconned function. 
    @details  New 2022-11-12.
    @details  https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
    @details  API description: https://pubsubclient.knolleary.net/api
    @details  pubsubclient library is used
    @return   void
  ***************************************************/
  void mqttReconnect() 
  { int i=0;
    char topic1[50], topic2[50];
    // Loop until we're reconnected
    while (!mqttClient.connected() && i<2) {
      sprintf(printstring, "Attempting MQTT connection...");
      logOut(2,printstring, msgMQTTConnect, msgInfo);

      // Attempt to connect
      if (mqttClient.connect(mqttDeviceString, mqttDefaultUser, mqttDefaultPaSSWORD)) {
        sprintf(topic1,"esp/%s",mqttDeviceString);
        // sprintf(topic1,"esp32/KombiExt");
        sprintf(topic2,"esp/cmnd/%s",mqttDeviceString);
        sprintf(printstring, "MQTT connected, subscribed: %s %s\n",topic1, topic2);
        logOut(2,printstring, msgMQTTSubscribe, msgInfo);
        
        // Subscribe. Multiple topics are possible.
        mqttClient.subscribe("esp32/KombiExt");
        mqttClient.loop();
        mqttClient.subscribe(topic2);
        mqttClient.loop();
      } 
      else 
      {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 2 seconds");
        sprintf(printstring, "MQTT connection failed. Retry in 2 seconds");
        logOut(2,printstring, msgMQTTError, msgWarn);

        // Wait 2 seconds before retrying
        delay(2000);
      }
      i++;
    } // while
  }

  /**************************************************!
    @brief    MQTT callback function. 
    @details  New 2022-11-12.
    @details  called when a MQTT message arrives, and is parsed here.
    @return   void
  ***************************************************/
  void mqttCallbackFunction(char* topic, byte* message, unsigned int length) 
  {
    sprintf(printstring, "MQTT msg received: [%s] [%s]",topic, message);
    logOut(2,printstring, msgMQTTReceive, msgInfo);

    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    
    String messageTemp;
    char myTopic [50];
    
    sprintf(myTopic,"esp32/%s",mqttDeviceString);
    //sprintf(myTopic,"esp32/RedBoxYeBtn/output");

    for (int i = 0; i < length; i++) {
      Serial.print((char)message[i]);
      messageTemp += (char)message[i];
    }
    Serial.println();

    // If a message is received on the subscribed topics 
    // Do what is needed
    // if (String(topic) == "esp32/output") 
    if(strstr(topic,myTopic))  
    {
      sprintf(printstring,"action: esp32 %s receivedMQTT %s",mqttDeviceString, message);
      logOut(2,printstring, msgMQTTReceive, msgInfo);
    }
  } // mqttCallbackFunction


  /**************************************************!
    @brief    MQTT send function
    @details  publishes the topic and payload it receives
    @return   void
  ***************************************************/
  void mqttSend(char *topic, char* payload)
  {
    sprintf(printstring,"Strings to MQTT: [%s] [%s]\n", topic, payload);
    logOut(1, printstring, msgMQTTInfo, msgInfo);
    mqttClient.publish(topic, payload);   
  }

  /**************************************************!
    @brief    MQTT handler for quick updates handles sending of data to MQTT broker, called via timer
    @details  New 2023-11-06, from EnvMonitor
    @details  called via timer, does measurements and output for all sensors actually present
    @details  uses global variables:
    @details  char msg[50];
    @details  int value = 0;
    @return   void
  ***************************************************/
  void mqttHandlerQuick()
  {
    char payloadStr[100];
    char topicStr[150];

    static long mqttSendItemCounter = 0, mqttCallCounter = 0;
    int i;

    mqttCallCounter++;      // counter for how often this function has been called
    mqttSendItemCounter = 0; // counter for the number of items to be sent during this call

    #ifdef isOneDS18B20
      // DS18B20 data 
      for(i = 0; i < noDS18B20Connected; i++)
        mqttSendDS18B20(i);
    #endif // isOneDS18B20     

    //kgfDataValid = getKGF110Data();   // get the data

    int tst = mqttClient.connected();
    if (!tst)
    {
      sprintf(printstring,"MQTT client not connected, return: %d State: %d\n",tst, mqttClient.state());
      logOut(2,printstring, msgMQTTConnect, msgWarn);
      mqttReconnect();
    }  
    else{
      sprintf(printstring,"MQTT client is still connected, return: %d\n",tst);
      logOut(2,printstring, msgMQTTConnect, msgWarn);
    }
    sprintf(printstring,"MQTT client after attempt to connect, State: %d\n", mqttClient.state());
    logOut(2,printstring, msgMQTTConnect, msgWarn);
    
    if (kgf.DataValid && mqttClient.connected()) 
    {
      //mqttClient.loop();
      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110Voltage);
      sprintf(payloadStr,"%3.2f",kgf.Voltage);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);     

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110Current);
      sprintf(payloadStr,"%3.2f",kgf.Current);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);  
      
      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110Power);
      sprintf(payloadStr,"%3.2f",kgf.Power);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110RemCapa);
      sprintf(payloadStr,"%4.3f",kgf.RemCapa);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 
 
      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110Temp);
      sprintf(payloadStr,"%d",kgf.Temp);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110CumulAhOut);
      sprintf(payloadStr,"%4.3f",kgf.CumulAhOut);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110Uptime);
      sprintf(payloadStr,"%d",kgf.Uptime);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110UptimeStr);
      sprintf(payloadStr,"%s",kgf.UptimeString);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 
      
      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110LifeLeft);
      sprintf(payloadStr,"%d",kgf.LifeLeft);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110LifeLeftStr);
      sprintf(payloadStr,"%s",kgf.LifeLeftString);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110EnergyIn);
      sprintf(payloadStr,"%4.3f",kgf.EnergyIn);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

       // SOC: State of charge in promille
      if(kgf.SetCapa > 0.01 && kgf.RemCapa > 0.01) // prevent div by zero
      {
        int soc = int(0.5 + 1000 * kgf.RemCapa / kgf.SetCapa);
        if(soc >= 0 && soc <= 1000)
        {
          sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110SOC);
          sprintf(payloadStr,"%4d",soc);
          mqttSendItemCounter++;
          mqttSend(topicStr, payloadStr); 
        }
        // CE: Consumed amp hours in mAh, related to 100% charge. CE=10Ah => 10 Ah entnommen
        int CE = int(0.5 + 1000 * (kgf.SetCapa - kgf.RemCapa));
        if(CE > 0 && CE < 1000000)
        {
          sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110CE);
          sprintf(payloadStr,"%d",CE);
          mqttSendItemCounter++;
          mqttSend(topicStr, payloadStr); 
        }  
      }

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110MeasValSntChksum);
      sprintf(payloadStr,"%d",bm1.getMeasuredValuesChecksum());
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110MeasValTstChksum);
      sprintf(payloadStr,"%d",bm1.getMeasuredValuesTstChecksum());
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);
    } // if mqttClient connected && kgfDataValid
    else
    {
      sprintf(printstring,"no valid data %d or MQTT client % d not connected\n",kgf.DataValid,  mqttClient.connected());
      logOut(2,printstring,  msgMQTTError, msgErr);
    }
  } // mqttHandlerQuick

  /**************************************************!
    @brief    MQTT handler for slow updates handles sending of data to MQTT broker, called via timer
    @details  New 2023-11-06, from EnvMonitor
    @details  called via timer, does measurements and output for all sensors actually present
    @details  uses global variables:
    @details  char msg[50];
    @details  int value = 0;
    @return   void
  ***************************************************/
  void mqttHandlerSlow()
  {
    char payloadStr[100];
    char topicStr[100];

    static long mqttSendItemCounter = 0, mqttCallCounter = 0;
    int i;

    mqttCallCounter++;      // counter for how often this function has been called
    mqttSendItemCounter = 0; // counter for the number of items to be sent during this call

    bm1.getSetValues(); // update the set values 
    delay(500); // wait for serial port
    kgf.DataValid = getKGF110Data();   // get the data

    int tst = mqttClient.connected();
    if (!tst)
    {
      sprintf(printstring,"MQTT client not connected, return: %d State: %d\n",tst, mqttClient.state());
      logOut(2,printstring, msgMQTTConnect, msgWarn);
      mqttReconnect();
    }  
    else{
      sprintf(printstring,"MQTT client is still connected, return: %d\n",tst);
      logOut(2,printstring, msgMQTTConnect, msgWarn);
    }
    sprintf(printstring,"MQTT client after attempt to connect, State: %d\n", mqttClient.state());
    logOut(2,printstring, msgMQTTConnect, msgWarn);
    
    if (kgf.DataValid && mqttClient.connected()) 
    {
      //mqttClient.loop();

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110SetCapa);
      sprintf(payloadStr,"%3.2f",kgf.SetCapa);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110ProtectionTemp);
      sprintf(payloadStr,"%d",kgf.ProtectionTemp);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110ProtectionRecoveryTime);
      sprintf(payloadStr,"%d",kgf.ProtectionRecoveryTime);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110ProtectionDelayTime);
      sprintf(payloadStr,"%d",kgf.ProtectionDelayTime);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 
      /*
      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110PresetCapacity);
      sprintf(payloadStr,"%d",kgf.PresetCapacity);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 
      */
      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110VoltageCalValue);
      sprintf(payloadStr,"%d",kgf.VoltageCalValue);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110CurrentCalValue);
      sprintf(payloadStr,"%d",kgf.CurrentCalValue);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110TempCalValue);
      sprintf(payloadStr,"%d",kgf.TempCalValue);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);      

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110VoltageScale);
      sprintf(payloadStr,"%d",kgf.VoltageScale);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110CurrentScale);
      sprintf(payloadStr,"%d",kgf.CurrentScale);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110RelayType);
      sprintf(payloadStr,"%d",kgf.RelayType);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110OVPVoltage);
      sprintf(payloadStr,"%3.2f",kgf.OVPVoltage);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110UVPVoltage);
      sprintf(payloadStr,"%3.2f",kgf.UVPVoltage);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110OCPForwardCurrent);
      sprintf(payloadStr,"%3.2f",kgf.OCPForwardCurrent);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr); 

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110OCPReverseCurrent);
      sprintf(payloadStr,"%3.2f",kgf.OCPReverseCurrent);
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);       

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110SetValSntChksum);
      sprintf(payloadStr,"%d",bm1.getSetValuesChecksum());
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);

      sprintf(topicStr,"esp32/%s/%s/%s",mqttDeviceString, mqttSensorKGF110ID, mqttKGF110SetValTstChksum);
      sprintf(payloadStr,"%d",bm1.getSetValuesTstChecksum());
      mqttSendItemCounter++;
      mqttSend(topicStr, payloadStr);
    } // if mqttClient connected && kgfDataValid
    else
    {
      sprintf(printstring,"no valid data %d or MQTT client not connected %d", kgf.DataValid,  mqttClient.connected());
      logOut(2,printstring, msgMQTTError, msgErr);
    }
  } // mqttHandlerSlow
#endif // isMQTT

/**************************************************!
  @brief    handler for DS18B20 data, called via timer
  @details  New 2023-11-17, from EnvMonitor
  @details  uses global variables:
  @return   void
***************************************************/
// DS18B20 Data are received in parallel running procedure "GetOneDS18B20Temperature()"
#ifdef isOneDS18B20
  void ds18b20Handler()
  {
    // check state of parallel task
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html#_CPPv46eReady
    eTaskState state;
    state = eTaskGetState(Task1); // eRunning=0, eReady=1, eBlocked=2, eSuspended=3, eDeleted=4, eInvalid
    // GetOneDS18B20Counter is incremented every time the loop runs to get DS18B20

    // correct with compensation factors which are specific to each sensor module, defined near auth codes
    for(int iii=0; iii<noDS18B20Connected; iii++)
      calDS18B20Temperature[iii] = DS18B20Temperature[iii] + corrDS18B20[iii];
    /*  
    sprintf(printstring,"BaseDS18B20 Tmp1 %3.2f Tmp2 %3.2f Tmp3 %3.2f %d %d %d - %d %ld\n", 
      DS18B20Temperature[0], DS18B20Temperature[1], DS18B20Temperature[2], 
      notMeasuredCount, notChangedCount, noDS18B20Restarts, state, GetOneDS18B20Counter);
     logOut(2,printstring, msgDS18B20Info, msgInfo);
    */ 
     
    /*
    sprintf(printstring,"CalDS18B20 Tmp1  %3.2f Tmp2 %3.2f Tmp3 %3.2f\n", 
      calDS18B20Temperature[0], calDS18B20Temperature[1], calDS18B20Temperature[2]);
    logOut(2,printstring, msgDS18B20Info, msgInfo);
    sprintf(printstring,"%d %d %d - %d %ld\n", 
      notMeasuredCount, notChangedCount, noDS18B20Restarts, state, GetOneDS18B20Counter);
    logOut(2,printstring, msgDS18B20Info, msgInfo);
    */
    if(GetOneDS18B20Counter <= previousGetOneDS18B20Counter)  // DS18B20 routine not counting
    {
      notMeasuredDS18B20 ++;
      sprintf(printstring,"!!!! DS18B20 not measuring !!! %ld %ld %ld \n",
        GetOneDS18B20Counter, previousGetOneDS18B20Counter, notMeasuredDS18B20);
      logOut(2,printstring, msgDS18B20NotMeasuring, msgWarn);  
      vTaskDelay(notMeasuredDS18B20 * 1000 / portTICK_PERIOD_MS); // progressive delay to give the measuring routine more time
    }  
    else
      notMeasuredDS18B20 = 0;
    //if (notMeasuredDS18B20 > 5)
    //  restartDS18B20MeasurementFunction();

    previousGetOneDS18B20Counter = GetOneDS18B20Counter;

  //  sprintf(printstring,"Cal.DS18B20 Tmp1 %3.1f Tmp2 %3.1f Tmp3 %3.1f \n", 
  //     calDS18B20Temperature[0], calDS18B20Temperature[1], calDS18B20Temperature[2]);
  //  logOut(2,printstring, msgDS18B20Info, msgInfo);
    
    double limit = -110.0; // EXPDis
  
    #ifdef isMQTT
      int i;
      for(i=0;i<3;i++)
      {
       if((calDS18B20Temperature[i]) > (limit)){
         sum_MQTT_calDS18B20Temperature[i] += calDS18B20Temperature[i];
         n_MQTT_calDS18B20Temperature[i] += 1;
       }
      } 
    #endif

    // checks for problems with measurements of DS18B20
    if(notMeasuredCount > DS18B20RestartLimit || notChangedCount > 3*DS18B20RestartLimit || // in GetOneDS18B20Temperature this count is handled if faulty checksum
      manualDS18B20Restart >= 1 ||  // manual restart via Blynk app
      notMeasuredDS18B20 > 5)       // in GetOneDS18B20Temperature this count isf function is running
    {
      sprintf(printstring,"\nRestarting DS18B20. No measurement taken in %d %d cycles - counter not incr: %ld Manual: %d\n",
        notMeasuredCount, notChangedCount, notMeasuredDS18B20, manualDS18B20Restart);
      logOut(2,printstring, msgDS18B20Restart, msgWarn);
      restartDS18B20MeasurementFunction();
      notMeasuredCount = 0; // reset the counter
      notChangedCount = 0;  // reset the counter
      manualDS18B20Restart = 0; // reset the manual switch
      notMeasuredDS18B20 = 0; // reset the counter for activity of the detached, parallel measurement function
    }
 
  } // ds18b20Handler
#endif  // isOneDS18B20

#ifdef isVEDIRECT  
/**************************************************!
    @brief    createVEdirectMessage1
    @details  erzeugt die 1. VE.direct message um einen Smartshunt
    @details  von Victron zu simulieren
    @details  enthält PID, V, I, P, CE, SOC, TTG, ALARM, AR, BMV, FW, MON
    @details  Struktur der Message ist im Victron Manual zu lesen
    @details  "VE.Direct-Protocol-3.33.pdf"
    @param    char* message: the message to be built
    @return   int: length of message or 0 if none built
  ***************************************************/
int createVEdirectMessage1(char* message)
  {
    int checksum;
    char hstring[50];
    static int lastCE = 0; 
    static float lastSOC = 0.0;

    strcpy(message,""); // empty message

    // Escape sequences: \0x0D = \r  0x0A = \n   \0x09 = \t
    // hex does not work, since more than 3 digits allowed. Oct would work (max 3 chars are specified)
    
    // fake product ID: 0xA389 = Smart Shunt
    sprintf(hstring,"\x0D\x0APID\t0xA389");
    strcat(message,hstring);

    // V: Voltage in mV
    int v = int(0.5 +1000 * kgf.Voltage);
    sprintf(hstring,"\x0D\x0AV\t%d",v);
    strcat(message,hstring);

    // I: Current in mA
    int i = int(0.5 +1000 * kgf.Current);
    sprintf(hstring,"\x0D\x0AI\t%d",i);
    strcat(message,hstring);

    // P: Power in Watt
    int p = int(0.5 + kgf.Power);
    sprintf(hstring,"\x0D\x0AP\t%d",p);
    strcat(message,hstring);

    int CE; 
    float soc;
    if(kgf.SetCapa > 0.01 && kgf.RemCapa > 0.01) // prevent div by zero
    {
      // CE: Consumed amp hours in mAh, related to 100% charge. CE=10Ah => 10 Ah entnommen
      CE = int(0.5 + 1000 * (kgf.SetCapa - kgf.RemCapa));
      if(CE > 0 && CE < 1000000) // 2nd plausibility check
      {
        lastCE = CE;
      } 
      else  // 2nd check failed, use previous values
        CE = lastCE;
      // SOC: State of charge in promille
      soc =  1000 * kgf.RemCapa / kgf.SetCapa;
      if(soc >= 0 && soc <= 1000) // 2nd plausibility check
      {
        lastSOC = soc;
      }
      else // 2nd check failed, use previous values
        soc = lastSOC;
    }
    else // div by zero, use previous values
    {
      CE = lastCE;
      soc = lastSOC;
    }
    sprintf(hstring,"\x0D\nCE\t%d",CE);
    strcat(message,hstring);  

    sprintf(hstring,"\x0D\x0ASOC\t%3.2f",soc);
    strcat(message,hstring);  

    // TTG: Time to go in minutes
    int TTG = int(kgf.LifeLeft);
    sprintf(hstring,"\x0D\x0ATTG\t%d",TTG);
    strcat(message,hstring);  

    // Alarm
    sprintf(hstring,"\x0D\nALARM\tOFF");
    strcat(message,hstring);

    // Alarm Reason
    sprintf(hstring,"\x0D\nAR\t0");
    strcat(message,hstring);

    // fake Model Description: KGF-110
    sprintf(hstring,"\x0D\nBMV\tKGF110");
    strcat(message,hstring);

    // fake Firmware Version: 208
    sprintf(hstring,"\x0D\nFW\t208");
    strcat(message,hstring);

    // fake: MON
    sprintf(hstring,"\x0D\nMON\t0");
    strcat(message,hstring);

    // Add word "Checksum"
    sprintf(hstring,"\x0D\nCHECKSUM\t");
    strcat(message,hstring);

    // sum up
    int sum = 0;
   
    for(int i=0; i< strlen(message); i++)
      sum = (sum + message[i]) & 255; // modulo sum of all bytes in message
    int corr = 256 - sum;

    sprintf(printstring,"message1 w/o checksum len: %d sum: %d corr: %d \n", strlen(message), sum, corr);
    logOut(1,printstring, veCreateInfo1, msgInfo);
    // Add checksum to message
    int len=strlen(message);
    //strcat(message,(char)corr);
    message[len] = (byte)corr;
    message[len+1] = '\0';

    //------------- check checksum;
    checksum = 0;
    for(int i=0; i< strlen(message); i++)
      checksum = (checksum + message[i]) & 255; 
   
    sprintf(printstring,"%s \n ", message);
    logOut(1,printstring,  veCreateInfo2, msgInfo);
    sprintf(printstring," message len: %d sum: %d corr: %d Checksum: %d\n", strlen(message), sum, corr, checksum);
    logOut(1,printstring, veCreateInfo3, msgInfo);

    return(strlen(message));
 } //createVEdirectMessage1


/**************************************************!
    @brief    createVEdirectMessage2
    @details  erzeugt die 2. VE.direct message um einen Smartshunt
    @details  von Victron zu simulieren
    @details  enthält H1..H18 (alle)
    @details  Struktur der Message ist im Victron Manual zu lesen
    @details  "VE.Direct-Protocol-3.33.pdf"
    @param    char* message: the message to be built
    @return   int: length of message or 0 if none built
  ***************************************************/
  int createVEdirectMessage2(char* message)
  {
    int checksum;
    char hstring[50];
    static int H7=10000000, H8=0; // to remember min and max voltages

    strcpy(message,""); // empty message

    // Escape sequences: \0x0D = \r  0x0A = \n   \0x09 = \t
    // hex does not work, since more than 3 digits allowed. Oct would work (max 3 chars are specified)
    
    // fake H1: Deepest Discharge in mAh
    sprintf(hstring,"\x0D\nH1\t-87654");
    strcat(message,hstring);

    //fake H2: Last Discharge in mAh
    sprintf(hstring,"\x0D\nH2\t-65432");
    strcat(message,hstring);

    //fake H3: Average Discharge in mAh
    sprintf(hstring,"\x0D\nH3\t-54321");
    strcat(message,hstring);    

    //fake H4: No. of Cycles
    /*
    int H4 = (int)(3);
    sprintf(hstring,"\x0D\nH4\t%d",H4);
    strcat(message,hstring);
    */
    int H4; 
    if(kgf.SetCapa > 0.01)
      H4 = (int)(0.5 + kgf.CumulAhOut / kgf.SetCapa);
    else 
      H4 = 0;  
    H4 = min(H4, 99999);
    H4 = max(H4, 0);    
    sprintf(hstring,"\x0D\nH4\t%d",H4);
    strcat(message,hstring);  

    //fake H5: number of full discharges
    int H5; 
    if(kgf.SetCapa > 0.01)
      H5 = (int)(0.5+ kgf.CumulAhOut / kgf.SetCapa);
    else 
      H5 = 0;  
    H5 = min(H5, 99999);
    H5 = max(H5, 0);  
     sprintf(hstring,"\x0D\nH5\t%d",H5);
    strcat(message,hstring);    

    // H6: cumulative Amp hours Drawn in mAh
    int H6= -1000 * kgf.CumulAhOut;
    sprintf(hstring,"\x0D\nH6\t%d",H6);
    strcat(message,hstring);    

    //fake H7: minimal battery voltage. this returns min voltage during present run of KGF-Reader
    if(1000*kgf.Voltage < H7)
      H7 = 1000*kgf.Voltage;
    // int H7= 1000 * 20.0;
    sprintf(hstring,"\x0D\nH7\t%d",H7);
    strcat(message,hstring);

    //fake H8: maxmal battery voltage. this returns max voltage during present run of KGF-Reader
    if(1000*kgf.Voltage > H8)
      H8 = 1000*kgf.Voltage;
    //int H8= 1000 * 27.0;
    sprintf(hstring,"\x0D\nH8\t%d",H8);
    strcat(message,hstring);

    //fake H9: seconds since last full charge
    int H9= 1111111;
    sprintf(hstring,"\x0D\nH9\t%d",H9);
    strcat(message,hstring);

    //fake H10: number of automatic synchronizations
    int H10= 2;
    sprintf(hstring,"\x0D\nH10\t%d",H10);
    strcat(message,hstring);

    //fake H11: No. of low main voltage alarms
    int H11= 0;
    sprintf(hstring,"\x0D\nH11\t%d",H11);
    strcat(message,hstring);

    //fake H12: No. of high main voltage alarms
    int H12= 0;
    sprintf(hstring,"\x0D\nH12\t%d",H12);
    strcat(message,hstring);

    //fake H13: No. of low aux voltage alarms
    int H13= 0;
    sprintf(hstring,"\x0D\nH13\t%d",H13);
    strcat(message,hstring);

    //fake H14: No. of high aux voltage alarms
    int H14= 0;
    sprintf(hstring,"\x0D\nH14\t%d",H14);
    strcat(message,hstring);

    //fake H15: min aux battery voltage
    int H15= 0;
    sprintf(hstring,"\x0D\nH15\t%d",H15);
    strcat(message,hstring);

    //fake H16: max aux battery voltage
    int H16= 0;
    sprintf(hstring,"\x0D\nH16\t%d",H16);
    strcat(message,hstring);

    // H17: Discharged energy in 0.01 KWh
    // kgf.Voltage in V, kgf.CumulAhOut in Ah. Multiplied = Energy in WH, /1000 => KWh
    int H17 = (int)(0.5 + 100 * kgf.Voltage * kgf.CumulAhOut / 1000);
    sprintf(hstring,"\x0D\nH17\t%d",H17);
    strcat(message,hstring);      

    // H18: Charged energy in 0.01 KWh 
    int H18 = (int)(0.5 + kgf.EnergyIn);
    sprintf(hstring,"\x0D\nH18\t%d",H18);
    strcat(message,hstring);

    // Add word "Checksum"
    sprintf(hstring,"\x0D\nCHECKSUM\t");
    strcat(message,hstring);

    // sum up
    int sum = 0;
   
    for(int i=0; i< strlen(message); i++)
      sum = (sum + message[i]) & 255; // modulo sum of all bytes in message
    int corr = 256 - sum;

    sprintf(printstring,"message2 w/o checksum len: %d sum: %d corr: %d \n", strlen(message), sum, corr);
    logOut(1,printstring, veCreateInfo4, msgInfo);
    // Add checksum to message
    int len=strlen(message);
    //strcat(message,(char)corr);
    message[len] = (byte)corr;
    message[len+1] = '\0';

    //------------- check checksum;
    checksum = 0;
    for(int i=0; i< strlen(message); i++)
      checksum = (checksum + message[i]) & 255; 
   
    sprintf(printstring,"%s \n ", message);
    logOut(1,printstring, veCreateInfo5, msgInfo);
    sprintf(printstring," message len: %d sum: %d corr: %d Checksum: %d\n", strlen(message), sum, corr, checksum);
    logOut(1,printstring,  veCreateInfo6, msgInfo);

    return(strlen(message));
 } //createVEdirectMessage

  /**************************************************!
    @brief    serial handler for ve.direct updates to Serial1, called via timer
    @details  New 2023-11-18, from EnvMonitor
    @details  called via timer, does measurements and output for all sensors actually present
    @details  uses global variables for voltage, current etc (KGF...)
    @return   void
  ***************************************************/
  void vedirectHandler()
  {
    char msg[400];
    int ret;
    if(kgf.DataValid)
    {
      ret = createVEdirectMessage1(msg); // create message1 from global variables KGF...
      Serial1.print(msg);
      Serial1.flush();
      delayMicroseconds(150);
      ret = createVEdirectMessage2(msg); // create message1 from global variables KGF...
      Serial1.print(msg);
    }
    else
    {
      sprintf(printstring,"no valid data %d for ve.direct emulator\n", kgf.DataValid);
      logOut(2,printstring,  veNoData, msgErr);
    }
  }

#endif //isVEDIRECT

/**************************************************!
    @brief    setup function
    @details  ---
    @return   void
  ***************************************************/
void setup()
{
  Serial.begin(115200); // port for debug, usb serial with 115200, normal serial with 9600
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // this is the RS485 port

  // Set Watchdog
  pinMode(0, INPUT_PULLUP); // Digital-Pin 0 as input
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS,true); //Init Watchdog with 40 seconds timeout and panic (hardware rest if watchdog acts)
  esp_task_wdt_add(NULL); //No special task needed

  // output for ve.direct emulator
  #ifdef isVEDIRECT
    Serial1.begin(19200, SERIAL_8N1, RXBAT, TXBAT);
  #endif  

  // SerialSW.begin(115200); //, SERIAL_8N1, RXSW, TXSW);

  sprintf(printstring,"setup() before connecting to wifi\n");
  logOut(2,printstring,msgStartup,msgInfo);
  // connect to wifi network
  bool connectPossible = connectToWiFi(default_ssid, default_pass, 7);

  // get time


  pinMode(RSE, OUTPUT);  // set Output enable pin to output mode
  /* you can use any other serial port that your platform uses as long as it can handle 115200 baud
  /
  / initialize the BatteryMonitor object with address and the serial port. Since 485 is a bus, 
  / multiple BatteryMonitor objects can use the same serial device, but be aware that, at least
  / currently, no demuxing is done within the library. Instead, reading and writing are mostly 
  / built as a single operation. You might run into problems if you run two bm objects in 
  / different threads on FreeRTOS or so
  */
  bm1.begin(1, Serial2, RSE);
  // bm1.begin(1, SerialSW);
  bm1.getSetValues(); // update the set values .. that could really be part of begin()
  sprintf(printstring,"setup() before starting OTA\n");
  logOut(2,printstring,msgStartup, msgInfo);

  #ifdef isOTA
  ArduinoOTA.setHostname("KGF_Reader");
  // handle OTA over the air Updates 
  // Added 2022-11-12.
  // details Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
          Serial.println("Start OTA updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

      ArduinoOTA.begin();
  #endif // isOTA

  sprintf(printstring,"setup() before starting MQTT\n");
  logOut(2,printstring,msgStartup, msgInfo);

  // https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
  #ifdef isMQTT // library dafür: pubsubclient
     // check wifi status and connect if not yet done
    if(WiFi.status() != WL_CONNECTED)
      connectToWiFi(ssid, pass, 7);

    // create the timers for mqtt handling
    mqttHandlerTimerHandleQuick = mqttHandlerTimerQuick.setInterval(mqttHandlerIntervalQuick, mqttHandlerQuick);
    mqttHandlerTimerHandleSlow = mqttHandlerTimerSlow.setInterval(mqttHandlerIntervalSlow, mqttHandlerSlow);
    
    // set MQTT server and MQTT callback function
    strcpy(mqttActualServer, mqttDefaultServer);
    mqttClient.setServer(mqttActualServer, 1883);
    mqttClient.setCallback(mqttCallbackFunction);
    mqttClient.setKeepAlive(120); // keep the client alive for 120 sec if no action occurs
    mqttClient.setBufferSize(1024); // increase buffer size from standard 256 to 1024 Bytes
  #endif //isMQTT

  #ifdef isVEDIRECT
    // create the timers for ve.direct handling
    vedirectTimerHandle = vedirectHandlerTimer.setInterval(vedirectHandlerInterval, vedirectHandler);
  #endif //isVEDIRECT

  sprintf(printstring,"setup() before starting DS18B20\n");
  logOut(2,printstring,msgStartup, msgInfo);

  esp_task_wdt_reset();   // keep watchdog happy

  #ifdef isOneDS18B20
    sprintf(printstring,"starting DS18B20\n");
    logOut(2,printstring,msgStartup, msgInfo);

    // switch on Power (via GPIO 32)
    pinMode(POWER_ONEWIRE_BUS, OUTPUT);
    digitalWrite(POWER_ONEWIRE_BUS, HIGH);
    // Start OneWire for DS18B20
    sensors.begin();
    delay(1000);
      // Create GetTemperature task for core 0, loop() runs on core 1
    adresseAusgeben();    // adressen onewire devices ausgeben, devices finden

    // create the timers for DS18B20 handling
    ds18B20TimerHandle = ds18b20HandlerTimer.setInterval(ds18b20HandlerInterval, ds18b20Handler);
  
    // start measurement task on parallel core for DS10B20
    xTaskCreatePinnedToCore(
      GetOneDS18B20Temperature, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task. higher number is higher priority, https://esp32.com/viewtopic.php?t=10629 */
      &Task1,  /* Task handle. */
      1); /* Core where the task should run */
  #endif

  #ifdef isSyncBattery
    sprintf(printstring,"starting handler for synchBattery()\n");
    logOut(2,printstring,msgStartup, msgInfo);    
    syncBatteryHandle = syncBatteryHandlerTimer.setInterval(syncBatteryHandlerInterval, batteryPercentHandler);
  #endif

  #ifdef getNTPTIME
    // get time from NTP server
    // https://randomnerdtutorials.com/esp32-date-time-ntp-client-server-arduino/
    getNTPTime();
  #endif  

  #ifdef isMQTT
    // get initial data
    getKGF110Data();
    // call handlers once to send initial data via MQTT
    mqttHandlerQuick();
    mqttHandlerSlow();
  #endif
}  // setup



/**************************************************!
    @brief    convert input in seconds to "DDDd hh:mm:ss" string
    @details  New 2023-11-12
    @return   void
  ***************************************************/
 void longToDateString1(long input, char* datestring)
 {
  int sec, min, hour, day, tot_min, tot_hour;

  sec = input % 60;
  tot_min = input / 60;    // integer div with seconds per min
  min     = tot_min % 60;  // modulo with sec per min
  tot_hour= tot_min / 60;  // integer div with minutes per hour
  hour = tot_hour % 24;    // modulo with hours per day
  day = tot_hour / 24;     // integer div with hours per day

  // %02d: 2 digit integer with "0" padding
  sprintf(datestring,"%dd %02d:%02d:%02d",day,hour,min, sec);
 }

/**************************************************!
    @brief    convert input in minutes to "DDDd hh:mm:ss" string
    @details  New 2023-11-12
    @return   void
  ***************************************************/
 void longToDateString2(long input, char* datestring)
 {
  int sec, min, hour, day, tot_min, tot_hour;

  tot_min = input;  
  min     = tot_min % 60;  // modulo with sec per min
  tot_hour= tot_min / 60;  // integer div with minutes per hour
  hour = tot_hour % 24;    // modulo with hours per day
  day = tot_hour / 24;     // integer div with hours per day

  // %02d: 2 digit integer with "0" padding
  sprintf(datestring,"%dd %02d:%02d",day,hour,min);
 }
/**************************************************!
    @brief    Get the data from battery monitor and store them in global variables
    @details  New 2023-11-12, out of loop()
    @return   bool: true if data are valid, false if not
  ***************************************************/
bool getKGF110Data()
{
  char charging;
  float current_direction;
  static long lastUptime=0, lastTemp=-200, lastSetCapa =-1;
  int actUptime, actTemp; 
  float actSetCapa;
  //Serial.print("G1 ");

  esp_task_wdt_reset();   // keep watchdog happy

  int checksumM     = bm1.getMeasuredValuesChecksum();
  int tst_checksumM = bm1.getMeasuredValuesTstChecksum();
  sprintf(printstring,"KFG sent measured values checksum: %d calc checksum: %d",
    checksumM, tst_checksumM);
  logOut(2,printstring, veChecksumMeasured, msgDefault);

  esp_task_wdt_reset();   // keep watchdog happy

  int checksumS     = bm1.getSetValuesChecksum();
  int tst_checksumS = bm1.getSetValuesTstChecksum();
  sprintf(printstring,"KFG sent set values checksum: %d calc checksum: %d",
    checksumM, tst_checksumM);
  logOut(2,printstring, veChecksumSetdata, msgDefault);

  // check for valid data. return and do not populate data if no valid input  
  actUptime = bm1.getUptime();
  //Serial.print("G1a ");
  actTemp   = bm1.getTemperature();
  //Serial.print("G1b ");
  actSetCapa = bm1.getCapacity();
  
  if((actUptime < lastUptime)||(actTemp < -90) || (actTemp > 500)
  //  || (actSetCapa < 1) 
  )
  {
    sprintf(printstring,"getKGF110Data inval data: UpT: %d %d T: %d actSetC: %3.2f",
       actUptime, lastUptime, actTemp, actSetCapa);
    logOut(2,printstring, veNoData, msgWarn);
    lastUptime = actUptime;
    return(false);
  }
  else 
  {
    sprintf(printstring,"getKGF110Data valid data: UpT: %d %d T: %d actSetC: %3.2", 
      actUptime, lastUptime, actTemp, actSetCapa);
    logOut(2,printstring, veNoData, msgWarn);
    lastUptime = actUptime;
    lastSetCapa = actSetCapa;
  }  
  //Serial.print("G2 ");

  (bm1.getCurrentDirection()==0)?current_direction=-1:current_direction=1;

  kgf.Voltage                 = bm1.getVoltage();
  kgf.Current                 = current_direction * bm1.getCurrent();
  kgf.Power                   = current_direction * bm1.getPower();
  kgf.RemCapa                 = bm1.getRemainingCapacity();
  kgf.SetCapa                 = bm1.getCapacity();
  kgf.Temp                    = bm1.getTemperature();
  kgf.CumulAhOut              = bm1.getCumulativeCapacity();
  kgf.Uptime                  = bm1.getUptime();
  kgf.LifeLeft                = bm1.getBatteryLifeLeft();
  kgf.EnergyIn                = bm1.getWattHours();

  longToDateString1(kgf.Uptime, kgf.UptimeString);
  longToDateString2(kgf.LifeLeft, kgf.LifeLeftString);

  //Serial.print("G3 ");
  (bm1.getCurrentDirection()==0)?charging='-':charging='+';

  sprintf(printstring,"\nBatt:\n\tVolt: %4.2fV\t Curr: %4.2fA\tRemCapa: %5.3fAh\t SetCapa: %5.2fAh\tTemp: %ld°C\n Power: %3.2fW\t CumulAhOut %5.3fAh\t Uptime %ldsec\t LifeLeft: %ldmin\t WattHrsIn: %5.3fWh\n", 
    kgf.Voltage,kgf.Current,kgf.RemCapa,kgf.SetCapa,kgf.Temp,
    kgf.Power, kgf.CumulAhOut,kgf.Uptime, kgf.LifeLeft, kgf.EnergyIn);
  Serial.print(printstring);

  kgf.ProtectionTemp          = bm1.getProtectionTemperature();
  kgf.ProtectionRecoveryTime  = bm1.getProtectionRecoveryTime();
  kgf.ProtectionDelayTime     = bm1.getProtectionDelayTime();
  // kgf.PresetCapacity          = bm1.getCapacity();
  kgf.VoltageCalValue         = bm1.getVoltageCalibration();
  kgf.CurrentCalValue         = bm1.getCurrentCalibration();
  kgf.TempCalValue            = bm1.getTemperatureCalibration();
  kgf.VoltageScale            = bm1.getVoltageScale();
  kgf.CurrentScale            = bm1.getCurrentScale();
  kgf.RelayType               = bm1.getRelayType();
  sprintf(printstring,"\nData:\n\tProtT: %ld°C\t ProtRecTime %lds\tProtDelayTime: %lds\t PresetCapa: %ldAh\tVoltageCal: %ld\n CurrentCal: %ld\t TempCal %ld\t VoltScale %ld\t CurrentScale: %ld\t RelayType: %ld\n", 
    kgf.ProtectionTemp, kgf.ProtectionRecoveryTime, kgf.ProtectionDelayTime, 
    kgf.VoltageCalValue, kgf.CurrentCalValue, kgf.TempCalValue, kgf.VoltageScale, kgf.CurrentScale, 
    kgf.RelayType);
  Serial.print(printstring);
  //Serial.print("G4 ");

  kgf.OVPVoltage              = bm1.getOverVoltageProtectionVoltage();
  kgf.UVPVoltage              = bm1.getUnderVoltageProtectionVoltage();
  kgf.OCPForwardCurrent       = bm1.getOverCurrentProtectionForwardCurrent();
  kgf.OCPReverseCurrent       = bm1.getOverCurrentProtectionReverseCurrent();
  kgf.OPPPower                = bm1.getOverPowerProtectionPower();
  sprintf(printstring,"Data2:\n\tOVPVolt: %5.2fV\t UVPVolt %5.2fV\tOCPForwC: %5.2fA\tOCPRevC: %5.2fA\tOPPPwr: %5.2fW\n\n", 
    kgf.OVPVoltage, kgf.UVPVoltage, kgf.OCPForwardCurrent, kgf.OCPReverseCurrent, kgf.OPPPower);
  Serial.print(printstring);

  //Serial.print("G5 ");
  return(true); // valid data
}

/**************************************************!
    @brief    subtractBatteryDailyConsumption()
    @details  New 2024-01-28, via timer out of loop()
    @details  Checks if a day has elapsed (date change at midnight)
    @details  and subtracts a set percent value off the actual battery capacity in %
    @details  to take into account the consumption by the BM, ESPs etc (too low to register by BM)
    @return   none
  ***************************************************/

// calc. normalized voltage for multiple batteries
float normalizedVoltage(float voltage)
{
  if(voltage < 15)    // voltage < 15 V: take unchanged
    return(voltage);
  if(voltage < 30)    // voltage < 30 V: 2 batteries, normalized is half value
    return(voltage/2);  
  if(voltage < 45)    // voltage < 45 V: 3 batteries, normalized is 1/3 value
    return(voltage/3);
  if(voltage < 60)
    return(voltage/4);// voltage < 60 V: 3 batteries, normalized is 1/4 value   
  return(-1); // error  
}

#define batterySubtractValue 2   // this is the % value that needs to be subtracted from percentage at midnight every day
void subtractBatteryDailyConsumption()
{
  char ISODate[50];
  static char lastISODate[50];
  float RemCapaPercent;
  int newPercentValue;
  bool ret;

  // get the time
  #ifdef getNTPTIME
   if (TimeIsInitialized == true) {
       printLocalTime(printstring, 2);
       printLocalTime(ISODate, 8);
       if(strlen(lastISODate)<1)      // ensure proper initialization
        strcpy(lastISODate, ISODate);
     }   
   else
    sprintf(printstring," Time not available - do nothing");  
    logOut(2,printstring, msgTimeInfo, msgInfo);
    return;
  #endif 

  if(kgf.SetCapa > 0.01 && kgf.RemCapa > 0.01) // prevent div by zero
    RemCapaPercent = 100*kgf.RemCapa / kgf.SetCapa;
  else{   
    RemCapaPercent = 10;
    return; // do nothing if proper value for remaining capacity could not be determined
  }  

  sprintf(printstring,"subtractBatteryDaily: Date %s lastDate: %s (V: %3.1f RemC%%:%3.1f RemC: %3.1f SetC %3.1f)", 
       ISODate, lastISODate, kgf.Voltage, RemCapaPercent, kgf.RemCapa, kgf.SetCapa);
    logOut(2,printstring, kgfSubtrDailyValues1, msgInfo);  

  if(0 != strcmp(ISODate, lastISODate)) // date has changed, strings not equal (equal returns 0)
  {
    newPercentValue = (int)(RemCapaPercent - batterySubtractValue);
    ret=bm1.setBatteryPercent(newPercentValue);
    strcpy(lastISODate, ISODate);
    sprintf(printstring,"subtractBatteryDaily BatPercent set to: %d returned: %d (V: %3.1f RemC: %3.1f SetC %3.1f)", 
      newPercentValue, ret, kgf.Voltage, kgf.RemCapa, kgf.SetCapa);
    logOut(2,printstring, kgfSubtrDailyCorrectedPercent, msgInfo);
  }
  else{
      sprintf(printstring,"subtractBatteryDaily BatPercent unchanged (V: %3.1f RemC: %3.1f SetC %3.1f)", 
         kgf.Voltage, kgf.RemCapa, kgf.SetCapa);
    logOut(2,printstring, kgfSubtrDailyValues2, msgInfo);  
  }
}

/**************************************************!
    @brief    syncBatteryPercent()
    @details  New 2024-01-28, via timer out of loop()
    @details  Checks battery capacity versus battery voltage. 
    @details  Corrects if necessary according to preset values
    @details  may also have to take into account the current:
    @details  corrV = actV +(actPower[W] * corrFactor)
    @details  corrFactor = 0.0013
    @return   none
  ***************************************************/

#define voltageThreshold           13.1 // 13.0
#define capacityThresholdPercent   40   // 35
#define correctCapacityPercent     35
#define corrFactor                 0.0015
#define noBelowBeforeCorr          5

void syncBatteryPercent()
{
  bool ret;
  float RemCapaPercent, corrVoltage, normalVoltage;
  static int count = 0;

  // check for need to synchronize battery charge percentage
  if(kgf.SetCapa > 0.01 && kgf.RemCapa > 0.01) // prevent div by zero
    RemCapaPercent = 100*kgf.RemCapa / kgf.SetCapa;
  else   
    RemCapaPercent = 1;

  normalVoltage =   normalizedVoltage(kgf.Voltage);

  // correct voltage to take care of power consumption, which causes voltage to bee 
  // smaller than in rest. 
  // example: normalVoltage is 13.16, power is 100 W => 
  // corrVoltage = 13.16+(-(-100)*0.0015) = 13.16 + 0.15 = 13.29 V
  if(kgf.Power < 0) // current correction: higher actual value if larger power out of battery
    corrVoltage = normalVoltage + (-kgf.Power * corrFactor); // kgf.Power is negative when current out of battery 
  else
    corrVoltage = normalVoltage;  

  // Count: at least 
  if((corrVoltage < voltageThreshold) && (RemCapaPercent > capacityThresholdPercent))
  {
    count++;
    if(count > noBelowBeforeCorr)
    {
      ret = bm1.setBatteryPercent(correctCapacityPercent);
      sprintf(printstring,"Battery Percent charged set to: %d returned: %d (V: %3.3f (n:%3.3f c:%3.3f) RemC: %3.1f SetV %3.1f)", 
        correctCapacityPercent, ret, kgf.Voltage, normalVoltage, corrVoltage, kgf.RemCapa, kgf.SetCapa);
      logOut(2,printstring, kgfSyncBatCorrPercent, msgWarn);  
      count = 0;
    }    
    else{
      sprintf(printstring,"Battery Percent not yet changed. Count: %d (V: %3.3f (n:%3.3f c:%3.3f) RemC: %3.1f SetV %3.1f)", 
        count, kgf.Voltage, normalVoltage, corrVoltage, kgf.RemCapa, kgf.SetCapa);

      logOut(2,printstring, kgfSyncBatCorrPercWaiting, msgInfo);
    }  
  }  
  else{
    count = 0;
    sprintf(printstring,"Battery Percent unchanged. (V: %3.3f (n:%3.3f c:%3.3f) C%% %3.1f (%3.1f%%)). IP: %s", 
      kgf.Voltage, normalVoltage, corrVoltage, kgf.RemCapa, RemCapaPercent, toStringIp(WiFi.localIP()).c_str());
    logOut(2,printstring, kgfSyncBatUnchgPercent, msgInfo);  
  }
}

// handler called by timer
void batteryPercentHandler()
{
  // test
  // bm1.setBatteryPercent(40);

  syncBatteryPercent();

  subtractBatteryDailyConsumption();  // check if a day has passed, and subtract daily value if necessary
}

void loop()
{  
  char datestring[40];
  //Serial.print("L3 ");

  esp_task_wdt_reset();   // keep watchdog happy

  kgf.DataValid = getKGF110Data(); // get data from battery monitor and store them in global variables
  sprintf(printstring,"return from getKGFData: %d", kgf.DataValid);
  logOut(2,printstring, veCreateInfo3, msgInfo);
  //Serial.print("L3a ");
  longToDateString1(kgf.LifeLeft, datestring);
  //Serial.print("L3b ");
  sprintf(printstring,"converted date: %s",datestring);
  //Serial.print("L3c ");
  Serial.print(printstring);
  //Serial.print("L5 ");

  #ifdef isOneDS18B20
    ds18b20HandlerTimer.run();
  #endif  
  //Serial.print("L6 ");
  #ifdef isMQTT
    mqttClient.loop();
    mqttHandlerTimerQuick.run(); // simple timer for MQTT data handler
    mqttHandlerTimerSlow.run(); // simple timer for MQTT data handler
  #endif

  //Serial.print("L7 ");
  // bm1.getSetValues(); // update the set values 

  //Serial.print("L8 ");
  #ifdef isVEDIRECT
    vedirectHandlerTimer.run(); // simple timer for ve.direct emulation
  #endif

  #ifdef isSyncBattery
    syncBatteryHandlerTimer.run();  // simple timer for battery sync handler
  #endif

  //Serial.print("L9 ");
  // handle OTA over the air Updates 
   #ifdef isOTA
    ArduinoOTA.handle();
  #endif
  //Serial.print("L10 ");
  delay(300);
}