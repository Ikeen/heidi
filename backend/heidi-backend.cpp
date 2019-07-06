
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <esp_deep_sleep.h>
#include <driver/adc.h>
#include <HardwareSerial.h>
#include "heidi-backend.h"
#include "TinyGPS++.h"
#include "images.h"
#ifdef SIM_MODULE
#define TINY_GSM_MODEM_SIM800 // define modem (SIM800L)
#include <TinyGsmClient.h>
#endif
#ifdef TEMP_SENSOR
#include "OneWire.h"
#include "DallasTemperature.h"
#endif
#ifdef OLED_DISPLAY
#include "SSD1306.h"
#endif
#include <sys/time.h>

//#define RECEIVER
//#define SENDER

#ifdef RECEIVER
#define USE_LORA
#endif
#ifdef SENDER
#define USE_LORA
#endif
int analog_value = 0;

#ifdef OLED_DISPLAY
SSD1306 display(0x3c, 4, 15);
String rssi = "RSSI --";
String packSize = "--";
String packet ;
#endif

//String voltage = "0.0 V";
//int counter;
//int GpsCnt;
//int loopCnt;
//int bufcount = 0;
//bool GPSfirstLockDone;

RTC_DATA_ATTR t_SendData sendData1;
RTC_DATA_ATTR t_SendData sendData2;
RTC_DATA_ATTR t_SendData sendData3;
RTC_DATA_ATTR t_SendData sendData4;
RTC_DATA_ATTR t_SendData sendData5;
RTC_DATA_ATTR t_SendData sendData6;
RTC_DATA_ATTR t_SendData sendData7;
RTC_DATA_ATTR t_SendData sendData8;
RTC_DATA_ATTR int8_t     bootCount       = START_FROM_RESET;
RTC_DATA_ATTR int32_t    lastTimeDiffMs  = 0;
RTC_DATA_ATTR uint32_t   errorCode       = 0;

t_SendData* currentDataSet;
t_SendData* availableDataSet[8];

// The TinyGPS++ object
HardwareSerial SerialGPS(GPS_UART_NO);
TinyGPSPlus gps;
#ifdef SIM_MODULE
HardwareSerial SerialGSM(GSM_UART_NO);
TinyGsm modemGSM(SerialGSM);
#endif

#ifdef TEMP_SENSOR
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
#endif

tm bootTime;
tm expBootTime;
int bootTimeStampMs    = INVALID_TIME_VALUE;
int expectedBootTimeMs = INVALID_TIME_VALUE;
int currentTimeDiffMs  = 0;
int msStartTime = 0;

void setup()
{
  msStartTime = millis();
  uint16_t init_cycle  = (herdeID() % 10) * 6;

  Serial.begin(19200); //9600);
#ifdef GPS_MODULE
  GPS_off();
#endif
#ifdef SIM_MODULE
  GSM_off();
#endif
#ifdef USE_LORA
  SetupLoRa();
#endif
  //testMeasure();

  pinMode(LED,OUTPUT);
  digitalWrite(LED, LED_OFF);
  DebugPrintln("Boot number: " + String(bootCount), DEBUG_LEVEL_1);
  checkWakeUpReason();

#ifdef OLED_DISPLAY
  void SetupDisplay();
#endif

  initGlobalVar();
  
#ifdef GPS_MODULE
  GPS_on();
  //find boot time
  struct tm timeinfo;
  if((!getLocalTime(&timeinfo)) || (bootCount <= REFETCH_SYS_TIME)){
    lastTimeDiffMs = 0; //not useful anymore
	delay(5000);
    DebugPrintln("Boot: Get time from GPS", DEBUG_LEVEL_1);
    if (!SetSysToGPSTime()){ //sets boot time if succeed
      GPS_off();
      goto_sleep(SLEEP_DURATION_MSEC);
    }
    getLocalTime(&timeinfo);
  } else {
    int bootMs = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
    bootMs *= 1000;
    bootMs -= millis();
    SetBootTimeFromMs(bootMs);
  }
  #if DEBUG_LEVEL > 0
  Serial.println(&timeinfo, "Current sys time: %B %d %Y %H:%M:%S");
  Serial.println(&bootTime, "sys. boot time: %H:%M:%S");
  #endif
  
  //find expected boot time
  if(!isInCycle(init_cycle)){ //calculates expected boot time
    DebugPrintln("Boot: not in cycle.", DEBUG_LEVEL_1);
    #if DEBUG_LEVEL >= DEBUG_LEVEL_2
    DebugPrintln("Sleep for : " + String(currentTimeDiffMs - millis()), DEBUG_LEVEL_2);
    delay(100);
    #endif
    GPS_off();
	lastTimeDiffMs = 0; //not useful since time was not correct
	if (currentTimeDiffMs <= 0) { goto_sleep(SLEEP_DURATION_MSEC); } //something went wrong
	if ((currentTimeDiffMs - millis()) < 1000) { goto_sleep(1000); } //just to be sure
    goto_sleep(currentTimeDiffMs - millis());
  }
  currentDataSet = availableDataSet[bootCount];
  initDataSet(currentDataSet);
  if (GPSGetPosition(currentDataSet, 10, 40000) == 0){
    DebugPrintln("GPS: Unable to fetch position.", DEBUG_LEVEL_1);
  }
  DebugPrintln("GPS done: " + String(millis() - msStartTime), DEBUG_LEVEL_2);
  GPS_off();

  #if DEBUG_LEVEL > 0
  Serial.println(&bootTime,    "cor. boot time: %H:%M:%S");
  Serial.println(&expBootTime, "exp. boot time: %H:%M:%S");
  #endif

#endif
#ifdef TEMP_SENSOR
  tempSensor.begin();
  tempSensor.requestTemperaturesByIndex(0);
  currentDataSet->temperature = (int16_t)round(tempSensor.getTempCByIndex(0) * 100);
  DebugPrint("Temperature: ", DEBUG_LEVEL_1);
  DebugPrint(currentDataSet->temperature / 100, DEBUG_LEVEL_1);
  DebugPrintln(" C", DEBUG_LEVEL_1);
#endif
  currentDataSet->errCode = errorCode;
#ifdef SIM_MODULE
  int  HTTPrc = 0;
  if (bootCount == BOOT_CYCLES - 1){
    #if DEBUG_LEVEL >= DEBUG_LEVEL_1
    for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
      t_SendData* DataSet = availableDataSet[i];
      Serial.print("Dataset " + String(i) + ": ");
      Serial.print(LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))));
      Serial.println("; " + String(DataSet->errCode, HEX));
    }
    #endif
	bool GSMfailure = true;
    String sendLine = generateMultiSendLine(0, BOOT_CYCLES - 1, DATA_SET_BACKUPS);
    DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_3);
    if (sendLine.length() > 0){
      /*for (int i=0; i<2; i++)*/{
        GSM_on();
        if (GSMsetup()){
          DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_1);
          int HTTPtimeOut = sendLine.length() * 20;
          HTTPrc = GSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data.php",
                             "application/x-www-form-urlencoded",
                              sendLine,
							  HTTPtimeOut,
							  HTTPtimeOut);
          if (HTTPrc == 200){
            GSMfailure = false;
            //i = 2;
          }
          GSMshutDown();
        }
        GSM_off();
        //if (i<1) { delay(1000); }
      }
    }
    if (GSMfailure){
      DebugPrintln("GSM transmission failed: " + String(HTTPrc), DEBUG_LEVEL_1);
    }
    for(int i=0; i<(BOOT_CYCLES * DATA_SET_BACKUPS); i++){
      if ((GSMfailure)  && (i < BOOT_CYCLES)) { availableDataSet[i]->errCode |= GSM_CONNECTION_FAILED; }
      if (!GSMfailure)  { availableDataSet[i]->errCode &= ~GSM_CONNECTION_FAILED; }
      copyDataSet(availableDataSet[i], availableDataSet[i + BOOT_CYCLES]);
      initDataSet(availableDataSet[i]);
    }
  }
#endif

//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
#ifdef USE_LORA
  LoRa.end();
  LoRa.sleep();
  delay(100);
#endif

  bootCount++;
  if (bootCount >= BOOT_CYCLES) { bootCount = 0; }

  int diffTimeFinalMs = lastTimeDiffMs + currentTimeDiffMs;
  if ((SLEEP_DURATION_MSEC - millis() + diffTimeFinalMs) < 1000){
    #if DEBUG_LEVEL >= DEBUG_LEVEL_2
    DebugPrintln("Sleep for : 1000", DEBUG_LEVEL_2);
    delay(100);
    #endif
	goto_sleep(1000);
  } 
  if (diffTimeFinalMs < SLEEP_DURATION_MSEC) { 
    #if DEBUG_LEVEL >= DEBUG_LEVEL_2
    DebugPrintln("Sleep for : " + String(SLEEP_DURATION_MSEC + diffTimeFinalMs - millis()), DEBUG_LEVEL_2);
    delay(100);
    #endif
    goto_sleep(SLEEP_DURATION_MSEC + diffTimeFinalMs - millis());
  }
  // just to be sure to ware up
  #if DEBUG_LEVEL >= DEBUG_LEVEL_2
  DebugPrintln("Sleep for : " + String(SLEEP_DURATION_MSEC), DEBUG_LEVEL_2);
  delay(100);
  #endif
  lastTimeDiffMs   = 0;
  goto_sleep(SLEEP_DURATION_MSEC);
}

void loop()
{
  #ifdef OLED_DISPLAY
  display.clear();
  display.drawString(4 , 1 , "GPS waiting\nfor 1st lock\n" + String(float(analogRead(ANALOG_PIN_0))/4095*6.6, 2) + " V\n");
  display.display();
  delay(1000);
  #endif
}

#ifdef GPS_MODULE
int GPSGetPosition(t_SendData* DataSet, int averages, int timeoutms){
  uint32_t analog_value = 0;
  uint32_t a_measures   = 0;
  int32_t  measures     = 0;
  double a_lng = 0.0;
  double a_lat = 0.0;
  double a_alt = 0.0;
  int  gps_sat   = 0;
  int  gps_day   = 0;
  int  gps_month = 0;
  int  gps_year  = 0;
  bool timestamp = false;
  uint32_t startMs = millis();

  errorCode |= (COULD_NOT_FETCH_GPS_1 << bootCount);
  errorCode |= (WRONG_GPS_VALUES_1 << bootCount);
  errorCode |= COULD_NOT_FETCH_GPS_TIME;

  pinMode(BATTERY_ANALOG_ENABLE,OUTPUT);
  digitalWrite(BATTERY_ANALOG_ENABLE, LOW);
  DebugPrintln("GPS: Acquire position", DEBUG_LEVEL_1);
  while(((measures < averages) || (gps_year == 0)) && ((millis() - startMs) < timeoutms)){
    while ((SerialGPS.available() == 0) && ((millis() - startMs) < timeoutms)) {
      analog_value += analogRead(BATTERY_ANALOG_PIN);
      a_measures++;
      delay(1);
    }
    while ((SerialGPS.available() > 0) && ((measures < averages) || (gps_year == 0))) {
      analog_value += analogRead(BATTERY_ANALOG_PIN);
      a_measures++;
      gps.encode(SerialGPS.read());
      if (   gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated()
    	  && gps.date.isUpdated() && gps.date.isValid() && gps.time.isUpdated() && gps.time.isValid()){
        measures++;
        if ((!timestamp) && ((gps.location.age() <= 1) || ((millis() - startMs - timeoutms) < 1500))){
          if (SetSysToGPS()){ //resets boot time corrected by GPS time
            if (calcCurrentTimeDiff()){
              DebugPrintln("GPS time vs. internal timer: " + String(currentTimeDiffMs), DEBUG_LEVEL_1);
              timestamp = true;
            } else { restartCycling(); }
          } else { restartCycling(); }
        }
        if ((gps_day == 0) and (gps.date.day() > 0)){ gps_day = gps.date.day(); }
        if ((gps_month == 0) and (gps.date.month() > 0)){ gps_month = gps.date.month(); }
        if ((gps_year == 0) and (gps.date.year() > 2000)){ gps_year = gps.date.year(); }
        if (gps_sat < gps.satellites.value()) { gps_sat = gps.satellites.value(); }
        #if DEBUG_LEVEL >= DEBUG_LEVEL_1
        DebugPrint("Latitude= ", DEBUG_LEVEL_3);
        DebugPrint(gps.location.lat(), 6, DEBUG_LEVEL_3);
        DebugPrint(" Longitude= ", DEBUG_LEVEL_3);
        DebugPrint(gps.location.lng(), 6, DEBUG_LEVEL_3);
        DebugPrint(" Altitude= ", DEBUG_LEVEL_3);
        DebugPrint(gps.altitude.meters(), 6, DEBUG_LEVEL_3);
        DebugPrint(" Date= ", DEBUG_LEVEL_3);
        DebugPrint(String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()), DEBUG_LEVEL_3);
        DebugPrint(" Time= ", DEBUG_LEVEL_3);
        DebugPrint(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3);
        DebugPrint(" Age= ", DEBUG_LEVEL_3);
        DebugPrint(gps.location.age(), DEBUG_LEVEL_3);
        DebugPrint("ms", DEBUG_LEVEL_3);
        DebugPrint(" Sat= ", DEBUG_LEVEL_3);
        DebugPrintln(gps.satellites.value(), DEBUG_LEVEL_3);
        #endif
        a_lng += gps.location.lng();
        a_lat += gps.location.lat();
        a_alt += gps.altitude.meters();
      }
    }
  }
  if(measures > 0){
    a_lng = a_lng / measures;
    a_lat = a_lat / measures;
    a_alt = a_alt / measures;
	errorCode &= ~(COULD_NOT_FETCH_GPS_1 << bootCount);
  }
  if(a_measures > 0){
    analog_value = analog_value / a_measures;
  }
  #if DEBUG_LEVEL >= DEBUG_LEVEL_1
  DebugPrint("Latitude= ", DEBUG_LEVEL_2);
  DebugPrint(a_lat, 6, DEBUG_LEVEL_2);
  DebugPrint(" Longitude= ", DEBUG_LEVEL_2);
  DebugPrint(a_lng, 6, DEBUG_LEVEL_2);
  DebugPrint(" Altitude= ", DEBUG_LEVEL_2);
  DebugPrint(a_alt, 6, DEBUG_LEVEL_2);
  DebugPrint(" Battery= ", DEBUG_LEVEL_2);
  DebugPrint((double(analog_value + 166) / 605), 2, DEBUG_LEVEL_2);
  DebugPrint(" BatteryAverages= ", DEBUG_LEVEL_2);
  DebugPrintln(a_measures, DEBUG_LEVEL_2);
  #endif
  DataSet->latitude = (int32_t)(a_lat * 1000000);
  DataSet->longitude = (int32_t)(a_lng * 1000000);
  DataSet->altitude = (uint16_t)a_alt;
  DataSet->date = dosDate(gps_year, gps_month, gps_day);
  DataSet->time = dosTime(bootTime.tm_hour, bootTime.tm_min, bootTime.tm_sec);
  DataSet->battery = (uint16_t)analog_value;
  DataSet->secdiff = (int8_t)(currentTimeDiffMs / 1000);
  if (!((measures > 0) && (DataSet->latitude == 0) && (DataSet->longitude == 0))){ //0.0, 0.0 must be wrong (or a fish)
    errorCode &= ~(WRONG_GPS_VALUES_1 << bootCount);
  }
  if (gps_day != 0){
    errorCode &= ~COULD_NOT_FETCH_GPS_TIME;
  }
  //DataSet->satellites = gps_sat;
  pinMode(BATTERY_ANALOG_ENABLE,INPUT);
  return measures;
}

bool SetSysToGPSTime()
{
  int startMs = millis();
  uint16_t year = 0;
  uint8_t  mon  = 0;
  uint8_t  day  = 0;
  uint32_t timeoutms = WAIT_FOR_GPS_TIME;
  errorCode |= COULD_NOT_FETCH_GPS_TIME;
  while(((year <= 2000) || (mon == 0) || (day == 0)) && ((millis() - startMs) < timeoutms)){
    while ((SerialGPS.available() == 0) && ((millis() - startMs) < timeoutms)) {
      delay(10);
    }
    while (SerialGPS.available() > 0) {
     gps.encode(SerialGPS.read());
     if (gps.date.isUpdated() && gps.date.isValid() && gps.time.isUpdated() && gps.time.isValid()){
        year = gps.date.year();
        mon  = gps.date.month();
        day  = gps.date.day();
        if ((year > 2000) && (mon > 0) && (day > 0)){
          bool result = SetSysToGPS();
		  #if DEBUG_LEVEL >= DEBUG_LEVEL_3
          DebugPrint("Set sys-time to GPS time Date= ", DEBUG_LEVEL_3);
          DebugPrint(String(year) + "-" + String(mon) + "-" + String(day), DEBUG_LEVEL_3);
          DebugPrint(" Time= ", DEBUG_LEVEL_3);
          DebugPrintln(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3);
		  #endif
		  return result;
        }
	  }
    }
  }
  return false;
}
bool SetSysToGPS(){
  tm time;
  int startms = millis();
  time.tm_year = gps.date.year()-1900;
  time.tm_mon  = gps.date.month();
  time.tm_mday = gps.date.day();
  time.tm_hour = gps.time.hour();
  time.tm_min  = gps.time.minute();
  time.tm_sec  = gps.time.second();
  if (gps.time.centisecond() > 50) { time.tm_sec++; }
  time_t t = mktime(&time);
  struct timeval now = { .tv_sec = t };
  if (settimeofday(&now, NULL) == 0){
	int stampms = gps.time.second() + gps.time.minute() * 60 + gps.time.hour() * 3600;
	stampms *= 1000;
	stampms += gps.time.centisecond() * 10;
	SetBootTimeFromMs(stampms - startms);
	errorCode &= ~COULD_NOT_FETCH_GPS_TIME;
	return true;
  }
  return false;
}
void GPS_on(){
  pinMode(GPS,OUTPUT);
  digitalWrite(GPS, GPS_ON);
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX, false);
  while(!SerialGPS) { Serial.print("."); }
  DebugPrintln("GPS on", DEBUG_LEVEL_1);
}
void GPS_off(){
  SerialGPS.end();
  pinMode(GPS,OUTPUT);
  digitalWrite(GPS, GPS_OFF);
  DebugPrintln("GPS off", DEBUG_LEVEL_1);
}
#endif //GPS_MODULE

#ifdef SIM_MODULE
void GSM_on(){
  pinMode(GSM,OUTPUT);
  pinMode(GSM_RST,OUTPUT);
  digitalWrite(GSM_RST, LOW);
  for(int i=0; i<1000; i++){
    digitalWrite(GSM, GSM_OFF);
    delayMicroseconds(470);
    digitalWrite(GSM, GSM_ON);
    delayMicroseconds(10);
  }
  #if DEBUG_LEVEL >= DEBUG_LEVEL_2
	//digitalWrite(GSM, GSM_OFF);
    //DebugPrintln("Step 1 survived", DEBUG_LEVEL_2);
  #endif
  for(int i=0; i<1000; i++){
    digitalWrite(GSM, GSM_OFF);
    delayMicroseconds(100);
	digitalWrite(GSM, GSM_ON);
    delayMicroseconds(50);
  }
  #if DEBUG_LEVEL >= DEBUG_LEVEL_2
	//digitalWrite(GSM, GSM_OFF);
    //DebugPrintln("Step 2 survived", DEBUG_LEVEL_2);
  #endif
  for(int i=0; i<1000; i++){
    digitalWrite(GSM, GSM_OFF);
    delayMicroseconds(50);
    digitalWrite(GSM, GSM_ON);
    delayMicroseconds(50);
  }
  #if DEBUG_LEVEL >= DEBUG_LEVEL_2
    //DebugPrintln("Step 3 survived", DEBUG_LEVEL_2);
  #endif
  delay(200);
  digitalWrite(GSM_RST, HIGH);
  delay(300);
  DebugPrintln("GSM on", DEBUG_LEVEL_1);
}

void GSM_off(){
  SerialGSM.end();
  pinMode(GSM,OUTPUT);
  digitalWrite(GSM, GPS_OFF);
  DebugPrintln("GSM off", DEBUG_LEVEL_1);
}

bool GSMsetup()
{
  String response = "";
  String resp = "";

  DebugPrintln("Setup GSM", DEBUG_LEVEL_1);
  // init serial SIM800L
  SerialGSM.begin(38400, SERIAL_8N1, GSM_RXD, GSM_TXD, false);
  /*
  AT
  >> AT // This should come back. SIM900 default is to echo back commands you enter
  >> OK // This string should tell you all is well
  */
#if 0
  int i=0;
  while (i<100){
	response = GSMsendCommand("AT");
    if(response.indexOf("OK") == -1) {
      delay(10);
      if ( i>= 100){
        DebugPrintln("SIM800L : Modem is not responding.", DEBUG_LEVEL_1);
        delay(200);
        return false;
      }
    } else {
      break; //modem is up
    }
    i++;
  }
  DebugPrintln("SIM800L : Modem is up", DEBUG_LEVEL_1);
  /*
  AT+CPIN? // This is to check if SIM is unlocked.
  >> +CPIN: SIM PIN // pin codes need to be entered
  >> +CPIN: READY // If your response contains this, then it means SIM is unlocked and ready
  >> OK
  */
  response = GSMsendCommand("AT+CPIN?");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("+CPIN: SIM PIN") != -1) {
	//PIN needed
	response = GSMsendCommand("AT+CPIN=\"0354\"");
    if(response.indexOf("OK") == -1) {
      DebugPrintln("SIM800L : Cannot unlock SIM.", DEBUG_LEVEL_1);
      delay(200);
      return false;
    }
    response = GSMsendCommand("AT+CPIN?");
  }
  if(response.indexOf("+CPIN: READY") == -1) {
    DebugPrintln("SIM800L : Cannot unlock SIM.", DEBUG_LEVEL_1);
    delay(200);
    return false;
  }
  DebugPrintln("SIM800L : SIM ready", DEBUG_LEVEL_1);

  /*
  AT+CREG? // This checks if SIM is registered or not
  >> +CREG: 0,1 // This string in the response indicates SIM is registered (2=registering, 5=roaming)
  */
  //if (!GSMsendCommand2("AT+CREG?", response)) { DebugPrintln("SIM800L : AT+CREG? command failed.", DEBUG_LEVEL_1); delay(200); return false; }
  response = GSMsendCommand("AT+CREG?");
  DebugPrintln(response, DEBUG_LEVEL_3);
  int p = response.indexOf("+CREG:");
  if(p != -1) {
	int startMS = millis();
	//check status on second result position
	while((response.substring(p+9, p+10) == "2") && (millis() - startMS < 10000)){
	  do{
	    delay(1000);
	    response = GSMsendCommand("AT+CREG?");
	    p = response.indexOf("+CREG:");
	  }while((p == -1) && (millis() - startMS < 10000));
	  if (p == -1) { break; }
	}
	if (p != -1){
	  if((response.substring(p+9, p+10) != "1") && (response.substring(p+9, p+10) != "5")){
	    //we need status "1" or "5"
	    p = -1;
	  }
      #if DEBUG_LEVEL >= DEBUG_LEVEL_1
	  if (response.substring(p+9, p+10) == "5"){
	    DebugPrintln("SIM800L : Use roaming.", DEBUG_LEVEL_1);
	  }
      #endif
	}
  }
  if(p == -1) {
	DebugPrintln("SIM800L : Cannot register SIM.", DEBUG_LEVEL_1);
	delay(200);
	return false;
  }
  DebugPrintln("SIM800L : SIM registered", DEBUG_LEVEL_1);

  /*
  AT+CGATT? // Check if GPRS is attached or not
  >> +CGATT: 1 // A response containing this string indicates GPRS is attached
  */
  response = GSMsendCommand("AT+CGATT?");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("+CGATT: 1") != -1){
	//GPRS is already attached
	resp = GSMsendCommand("AT+CIPSHUT"); // Reset the IP session if any
    DebugPrintln(resp, DEBUG_LEVEL_3);
	if(resp.indexOf("SHUT OK") == -1){ // This string in the response represents all IP sessions shutdown.
	  DebugPrintln("SIM800L : Cannot shut old IP sessions.", DEBUG_LEVEL_1);
	  delay(200);
	  return false;
	}
  } else if(response.indexOf("+CGATT: 0") != -1){
	delay(500); // wait a short time
	resp = GSMsendCommand("AT+CGATT=1"); // attach GPRS
	DebugPrintln(resp, DEBUG_LEVEL_3);
	if(resp.indexOf("OK") == -1){
      delay(1000);
	  resp = GSMsendCommand("AT+CGATT=1"); // attach GPRS 2nd
	  DebugPrintln(resp, DEBUG_LEVEL_3);
	  if(resp.indexOf("OK") == -1){
        DebugPrintln("SIM800L : Cannot attach GPRS.", DEBUG_LEVEL_1);
	    delay(200);
	    return false;
	  }
	}
	response = GSMsendCommand("AT+CGATT?");
    DebugPrintln(response, DEBUG_LEVEL_3);
  }
  if(response.indexOf("+CGATT: 1") == -1){
	DebugPrintln("SIM800L : Cannot attach GPRS.", DEBUG_LEVEL_1);
	delay(200);
    return false;
  }
  DebugPrintln("SIM800L : GPRS attached", DEBUG_LEVEL_1);

  /*
  AT+CIPSTATUS // Check if the IP stack is initialized
  >> STATE: IP INITIAL // This string in the response indicates IP stack is initialized
  */
  response = GSMsendCommand("AT+CIPSTATUS");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("STATE: IP INITIAL") == -1){
	DebugPrintln("SIM800L : unexpected IP status: " + response, DEBUG_LEVEL_1);
	delay(200);
	return false;
  }
  DebugPrintln("SIM800L : IP status ok.", DEBUG_LEVEL_1);

  /*
  AT+CIPMUX=0 // To keep things simple, I’m setting up a single connection mode
  >> OK // This string indicates single connection mode set successfully at SIM 900
  */
  response = GSMsendCommand("AT+CIPMUX=0");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1){
	DebugPrintln("SIM800L : unable to set CIPMUX to zero.", DEBUG_LEVEL_1);
	delay(200);
	//return false;
  }
  DebugPrintln("SIM800L : CIPMUX off.", DEBUG_LEVEL_1);

  /*
  AT+CSTT= "APN", "UNAME", "PWD" // Start the task, based on the SIM card you are using, you need to know the APN, username and password for your service provider
  >> OK // This response indicates task started successfully
  */
  response = GSMsendCommand("AT+CSTT=\"web.vodafone.de\",\"\",\"\"");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1){
	DebugPrintln("SIM800L : unable to set APN.", DEBUG_LEVEL_1);
	delay(200);
	return false;
  }
  DebugPrintln("SIM800L : APN set.", DEBUG_LEVEL_1);

  /*
  AT+CIICR // Now bring up the wireless. Please note, the response to this might take some time
  >> OK // This text in response string indicates wireless is up
  */
  response = GSMsendCommand("AT+CIICR");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1){
	DebugPrintln("SIM800L : unable to bring up the wireless.", DEBUG_LEVEL_1);
	delay(200);
	//return false;
  }
  DebugPrintln("SIM800L : wireless up.", DEBUG_LEVEL_1);

  /*
  AT+CIFSR // Get the local IP address. Some people say that this step is not required, but if I do not issue this, it was not working for my case. So I made this mandatory, no harm.
  >> xxx.xxx.xxx.xxx //
  */
  response = GSMsendCommand("AT+CIFSR");
  DebugPrintln(response, DEBUG_LEVEL_1);
  if(response.indexOf("OK") == -1){
	DebugPrintln("SIM800L : unable to get IP address.", DEBUG_LEVEL_1);
	delay(200);
	//return false;
  }
  #if DEBUG_LEVEL > 0
  int x = response.indexOf((char)0x0A)+1;
  DebugPrintln("SIM800L : IP: " + response.substring(x, response.indexOf((char)0x0C, x)), DEBUG_LEVEL_1);
  #endif
#endif

  // get info
  if (DEBUG_LEVEL > 0){
    Serial.println(modemGSM.getModemInfo());
  }
  // init modem
  if (!modemGSM.restart())
  {
    DebugPrintln("Restarting GSM\nModem failed", DEBUG_LEVEL_1);
    delay(1000);
    return false;
  }
  DebugPrintln("Modem restart OK", DEBUG_LEVEL_3);
  //unlock SIM
  if (!modemGSM.simUnlock("0354"))
  {
      DebugPrintln("Failed to SIM unlock", DEBUG_LEVEL_1);
      delay(1000);
      return false;
  }
  DebugPrintln("SIM unlock OK", DEBUG_LEVEL_3);
  // connect to network
  if (!modemGSM.waitForNetwork())
  {
    DebugPrintln("Failed to connect to network", DEBUG_LEVEL_1);
    delay(1000);
    return false;
  }
  DebugPrintln("Modem network OK", DEBUG_LEVEL_3);

  //if (!modemGSM.sendSMS("01522xxxxxxx", "Heidi-Tracker hat soeben das erste mal erfolgreich Daten in die Datenbank gebracht. War gar nicht so einfach. :-)"))
  //{
  //  Serial.println("Failed to send SMS");
  //  delay(1000);
  //  return;
  //}
  //Serial.println("SMS sent!");

  // connect GPRS
  //  modemGSM.gprsConnect(APN,USER,PASSWORD))
  if(!modemGSM.gprsConnect("web.vodafone.de","",""))
  {
    DebugPrintln("GPRS Connection\nFailed", DEBUG_LEVEL_1);
    delay(1000);
    return false;
  }
  DebugPrintln("GPRS Connect OK", DEBUG_LEVEL_3);

  
  //reset DNS to vodafone DNS
  response = GSMsendCommand("AT+CDNSCFG=\"139.007.030.125\",\"139.007.030.126\"");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1) {
    DebugPrintln("SIM800L : doPost() - Unable to define vodafone DNS", DEBUG_LEVEL_1);
  }
  DebugPrintln("GPRS Setup done: " + String(millis() - msStartTime), DEBUG_LEVEL_2);
  //delay(500);
  return true;
}

bool GSMshutDown()
{
  String resp = GSMsendCommand("AT+CIPSHUT"); // Reset the IP session if any
  if(resp.indexOf("SHUT OK") == -1){ // This string in the response represents all IP sessions shutdown.
	DebugPrintln("SIM800L : Cannot shut IP sessions.", DEBUG_LEVEL_1);
	delay(200);
	return false;
  }
  resp = GSMsendCommand("AT+CGATT=0"); // detach GPRS
  if(resp.indexOf("OK") == -1){
  	DebugPrintln("SIM800L : Cannot detach GPRS.", DEBUG_LEVEL_1);
  	delay(200);
  	return false;
  }
  resp = GSMsendCommand("AT+CREG=0");
  if(resp.indexOf("OK") == -1){
  	DebugPrintln("SIM800L : Cannot unregister SIM.", DEBUG_LEVEL_1);
  	delay(200);
  	return false;
  }
  SerialGSM.end();
  return true;
}
/**
 * Do HTTP/S POST to a specific URL
 */
int GSMdoPost(String url, String contentType, String payload, unsigned int clientWriteTimeoutMs, unsigned int serverReadTimeoutMs) {
  String response ="";
  // Initiate HTTP/S session with the module
  int initRC = GSMinitiateHTTP(url);
  if(initRC > 0) { return initRC; }
  // Define the content type
  DebugPrint("AT+HTTPPARA=\"CONTENT\",\"" + contentType + "\"", DEBUG_LEVEL_3);
  response = GSMsendCommand("AT+HTTPPARA=\"CONTENT\",\"" + contentType + "\"");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1) {
    DebugPrintln("SIM800L : doPost() - Unable to define the content type", DEBUG_LEVEL_1);
    return 702;
  }

  //for(int k=0; k<5; k++){

  int httpRC;
  int l = payload.length();
  // Prepare to send the payload
  DebugPrint("AT+HTTPDATA=" + String(l) + "," + String(clientWriteTimeoutMs), DEBUG_LEVEL_3);
  response = GSMsendCommand("AT+HTTPDATA=" + String(payload.length()) + "," + String(clientWriteTimeoutMs));
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("DOWNLOAD") == -1) {
    DebugPrintln("SIM800L : doPost() - Unable to send payload to module", DEBUG_LEVEL_1);
    return 707;
  }
  // Write the payload on the module
  DebugPrintln("SIM800L : doPost() - send " + String(l) + " bytes payload", DEBUG_LEVEL_3);
  for (int i=0; i<l; i++){
    if  (SerialGSM.write(payload.charAt(i)) == 0){
        DebugPrintln("SIM800L : send Payload -could not write char " + String(i), DEBUG_LEVEL_1);
        delay(10);
    }
  }
  SerialGSM.println("");
  response = SerialGSM.readString();
  DebugPrintln(response, DEBUG_LEVEL_3);

  // Start HTTP POST action
  DebugPrint("AT+HTTPACTION=1", DEBUG_LEVEL_3);
  response = GSMsendCommand("AT+HTTPACTION=1", 30000);
  DebugPrint(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1) {
    DebugPrintln("SIM800L : doPost() - Unable to initiate POST action", DEBUG_LEVEL_1);
    return 703;
  }

  // Wait answer from the server
  int i=0;
  while(i < serverReadTimeoutMs){
    delay(1);
    i++;
    response = SerialGSM.readString();
    if (response.length() > 0) { DebugPrintln(response, DEBUG_LEVEL_3); }
    if(response.indexOf("+HTTPACTION: 1,") > -1){
      i = serverReadTimeoutMs;
    }
  }
  if(response == ""){
    DebugPrintln("SIM800L : doPost() - Server timeout", DEBUG_LEVEL_1);
    return 408;
  }
  i = response.indexOf("+HTTPACTION: 1,");
  if(i < 0) {
    DebugPrintln("SIM800L : doPost() - Invalid answer on HTTP POST", DEBUG_LEVEL_1);
    return 703;
  }

  // Get the HTTP return code
  httpRC = response.substring(i+15,i+18).toInt();
  int dataSize = response.substring(i+19,response.length()-i).toInt();

  DebugPrintln("SIM800L : doPost() - HTTP status " + String(httpRC) + ", " + String(dataSize) + " bytes to read ", DEBUG_LEVEL_3);
  if(httpRC == 200) {

    // Ask for reading and detect the start of the reading...
    response = GSMsendCommand("AT+HTTPREAD");
    DebugPrintln("SIM800L : doPost() - response to \"AT+HTTPREAD\": " + response, DEBUG_LEVEL_1);
    i = response.indexOf("+HTTPREAD: ");
    if(i == -1){
       //DebugPrintln("SIM800L : doPost() - Invalid response to \"AT+HTTPREAD\": " + response, DEBUG_LEVEL_1);
       return 705;
    }
    int startPayload = i + 11 + 2 + String(dataSize).length();
    // extract number of bytes defined in the dataSize
    String httpData = response.substring(startPayload, startPayload + dataSize);
    // We are expecting a final OK
    if(response.substring(startPayload + dataSize, response.length()).indexOf("OK") == -1){
      DebugPrintln("SIM800L : doPost() - Invalid end of data while reading HTTP result from the module", DEBUG_LEVEL_1);
      return 705;
    }
    DebugPrintln("SIM800L : doPost() - Received from HTTP GET : \n"  + httpData, DEBUG_LEVEL_3);
	if(httpData.indexOf("OK") == -1){
        DebugPrintln("SIM800L : doPost() - server does not accept data", DEBUG_LEVEL_1);
		httpRC = 406;
	}
  }
  // Terminate HTTP/S session
  int termRC = GSMterminateHTTP();
  if(termRC > 0) {
    return termRC;
  }
  DebugPrintln("GPRS send done: " + String(millis() - msStartTime), DEBUG_LEVEL_2);
  return httpRC;
}

/**
 * Do HTTP/S GET on a specific URL
 */
int GSMdoGet(const char* url, unsigned int serverReadTimeoutMs) {

  // Initiate HTTP/S session
  int initRC = GSMinitiateHTTP(url);
  if(initRC > 0) {
    return initRC;
  }

  // Start HTTP GET action
  Serial.println("AT+HTTPACTION=0");
  String response = GSMsendCommand("AT+HTTPACTION=0");
  Serial.println(response);
  if(response.indexOf("OK") == -1){
    Serial.println("SIM800L : doGet() - Unable to initiate GET action");
    return 703;
  }

  // Wait answer from the server
  int i=0;
  while(i<serverReadTimeoutMs){
    delay(1);
    i++;
    response = SerialGSM.readString();
    Serial.println(response);
    if(response.indexOf("+HTTPACTION: 0,") > -1){
      i = serverReadTimeoutMs;
    }
  }
  if(response == ""){
    Serial.println("SIM800L : doPost() - Server timeout");
    return 408;
  }
  i = response.indexOf("+HTTPACTION: 0,");
  if(i < 0) {
    Serial.println("SIM800L : doPost() - Invalid answer on HTTP POST");
    return 703;
  }

  // Get the HTTP return code
  int httpRC = response.substring(i+15,i+18).toInt();
  int dataSize = response.substring(i+19,response.length()-i).toInt();

  //if(enableDebug) {
  Serial.println("SIM800L : doPost() - HTTP status " + String(httpRC) + ", " + String(dataSize) + " bytes to read");
  //  }

  if(httpRC == 200) {

    Serial.println("SIM800L : " + String(dataSize) + " bytes to read");

    // Ask for reading and detect the start of the reading...
    response = GSMsendCommand("AT+HTTPREAD");
    i = response.indexOf("+HTTPREAD: ");
    if(i == -1){
      return 705;
    }
    int startPayload = i + 11 + 2 + String(dataSize).length();
    // extract number of bytes defined in the dataSize
    String httpData = response.substring(startPayload, startPayload + dataSize);
    // We are expecting a final OK
    if(response.substring(startPayload + dataSize, response.length()).indexOf("OK") == -1){
      Serial.println("SIM800L : doGet() - Invalid end of data while reading HTTP result from the module");
      return 705;
    }

    Serial.println("SIM800L : doGet() - Received from HTTP GET : \n"  + httpData);
  }

  // Terminate HTTP/S session
  int termRC = GSMterminateHTTP();
  if(termRC > 0) {
    return termRC;
  }

  return httpRC;
}

int GSMinitiateHTTP(String url) {
  String response = "";
  // Init HTTP connection
  DebugPrint("AT+HTTPINIT", DEBUG_LEVEL_3);
  response = GSMsendCommand("AT+HTTPINIT");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == 0) {
    DebugPrintln("SIM800L : initiateHTTP() - Unable to init HTTP", DEBUG_LEVEL_1);
    return 701;
  }
  // Use the GPRS bearer
  DebugPrint("AT+HTTPPARA=\"CID\",1", DEBUG_LEVEL_3);
  response = GSMsendCommand("AT+HTTPPARA=\"CID\",1");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1) {
    DebugPrintln("SIM800L : initiateHTTP() - Unable to define bearer", DEBUG_LEVEL_1);
    return 702;
  }
  // Define URL to look for
  DebugPrint("AT+HTTPPARA=\"URL\",\"" + url + "\"", DEBUG_LEVEL_3);
  response = GSMsendCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  DebugPrintln(response, DEBUG_LEVEL_3);
  if(response.indexOf("OK") == -1) {
    DebugPrintln("SIM800L : initiateHTTP() - Unable to define the URL", DEBUG_LEVEL_1);
    return 702;
  }
  // HTTP or HTTPS
  if(url.indexOf("https://") == 0) {
    DebugPrint("AT+HTTPSSL=1", DEBUG_LEVEL_3);
    response = GSMsendCommand("AT+HTTPSSL=1");
    DebugPrintln(response, DEBUG_LEVEL_3);
    if(response.indexOf("OK") == -1) {
      DebugPrintln("SIM800L : initiateHTTP() - Unable to switch to HTTPS", DEBUG_LEVEL_1);
      return 702;
    }
  } else {
    DebugPrint("AT+HTTPSSL=0", DEBUG_LEVEL_3);
    response = GSMsendCommand("AT+HTTPSSL=0");
    DebugPrintln(response, DEBUG_LEVEL_3);
    if(response.indexOf("OK") == -1) {
      DebugPrintln("SIM800L : initiateHTTP() - Unable to switch to HTTP", DEBUG_LEVEL_1);
      return 702;
    }
  }
  return 0;
}
int GSMterminateHTTP() {
  // Close HTTP connection
  String response = GSMsendCommand("AT+HTTPTERM");
  if(response.indexOf("OK") == -1)  {
    DebugPrintln("SIM800L : terminateHTTP() - Unable to close HTTP session", DEBUG_LEVEL_1);
    return 706;
  }
  return 0;
}

String GSMsendCommand(const String command, int timeoutMs /* = 5000 */)
{
  SerialGSM.println(command);
  int msStart = millis();
  while((!SerialGSM.available()) && ((millis() - msStart) < timeoutMs)) { delay(1); };
  if ((millis() - msStart) >= timeoutMs) { return ""; }
  return SerialGSM.readString();
}
#endif //SIM_MODULE

#ifdef USE_LORA
void SetupLoRa(){
  DebugPrint("LoRa init .. ", DEBUG_LEVEL_1);
  while(!Serial) { Serial.print("."); }
  DebugPrintln(" done", DEBUG_LEVEL_3);

  DebugPrint("Starting LoRa at ", DEBUG_LEVEL_3);
  DebugPrint(String(BAND/1E6,2), DEBUG_LEVEL_3);
  DebugPrint("MHz ", DEBUG_LEVEL_3);
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    DebugPrintln(" failed!", DEBUG_LEVEL_3);
    while (1);
  }
  DebugPrintln(".. done", DEBUG_LEVEL_3);
  DebugPrintln("Configuring LoRa", DEBUG_LEVEL_3);
  DebugPrint(" - spreading factor: ", DEBUG_LEVEL_3);
  DebugPrintln(spreadingFactor, DEBUG_LEVEL_3);
  LoRa.setSpreadingFactor(spreadingFactor);
  DebugPrint(" - signal bandwidth: ", DEBUG_LEVEL_3);
  DebugPrintln(SignalBandwidth, DEBUG_LEVEL_3);
  LoRa.setSignalBandwidth(SignalBandwidth);
  DebugPrint(" - code rate: 4/", DEBUG_LEVEL_3);
  DebugPrintln(codingRateDenominator, DEBUG_LEVEL_3);
  LoRa.setCodingRate4(codingRateDenominator);
  DebugPrint(" - preamble length: ", DEBUG_LEVEL_3);
  DebugPrintln(preambleLength, DEBUG_LEVEL_3);
  LoRa.setPreambleLength(preambleLength);
  DebugPrintln("done", DEBUG_LEVEL_1);
  //LoRa.setTxPower(17);
  //LoRa.receive();
}
#endif //USE_LORA
#ifdef OLED_DISPLAY
void SetupDisplay(){
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}
#endif

String generateSendLine(t_SendData* DataSet){
  //"TrackerID=0002.1235&Longitude=-13.344855&Latitude=51.006709&Altitude=305&Date=2019-06-09&Time=17:20:00&Battery=4.02",
  String result = "";
  if (!emptyDataSet(DataSet)){
    result = "TrackerID=";
    int8_t hour_shift = GetLocalTimeHourShift(); //local time setting to be done
    int l = String(herdeID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(herdeID()) + ".";
    l = String(animalID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(animalID());
    result += "&Longitude=" + String(double(DataSet->longitude) / 1000000.0, 6);
    result += "&Latitude=" + String(double(DataSet->latitude) / 1000000.0, 6);
    result += "&Altitude=" + String(DataSet->altitude);
    result += "&Date=" + String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date)));
    result += "&Time=" + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time)));
    result += "&Battery=" + String((double(DataSet->battery + 166) / 605), 2);
    result += "&FreeValue1=" + String((int)DataSet->secdiff);
    result += "&FreeValue2="  + String((float)DataSet->temperature / 100, 2);
    result += "&FreeValue3="  + String(DataSet->errCode, HEX);
  }
  return result;
}
String generateMultiSendLine(int first, int last, int backups){
  //short notation with multiple data sets
  //php.ini: max_input_var = 1000 by default (1 data set = 7..12 vars) 
  //php.ini: post_max_size = 8M by default - SIM800 312k only! (1 data set = 90..150 byte)
  //->up 80 data sets possible 
  //"ID1=0002.1235&Lo1=-13.344855&La1=51.006709&Al1=305&Da1=2019-06-09&Ti1=17:20:00&Ba1=4.02&ID1=0002.1235&Lo1=...",
  t_SendData* DataSet;
  String result = "";
  int k = 0;
  int i = 0;
  for (int b = backups; b >= 0; b--){
    for (int a=first; a<=last; a++){
      i = a + b * BOOT_CYCLES;
      DataSet = availableDataSet[i];
      if (emptyDataSet(DataSet)) { continue; }
      if ((i < BOOT_CYCLES) || ((DataSet->errCode & GSM_CONNECTION_FAILED) == GSM_CONNECTION_FAILED)) {
        k++;
	    if (k > 1) { result += "&"; }
	    result += "ID" + String(k) + "=";
        int8_t hour_shift = GetLocalTimeHourShift(); //local time setting to be done
        int l = String(herdeID()).length();
        for (int j=0; j<(4-l); j++){ result += "0";}
        result += String(herdeID()) + ".";
        l = String(animalID()).length();
        for (int j=0; j<(4-l); j++){ result += "0";}
        result += String(animalID());
        result += "&Lo" + String(k) + "=" + String(double(DataSet->longitude) / 1000000.0, 6);
        result += "&La" + String(k) + "=" + String(double(DataSet->latitude) / 1000000.0, 6);
        result += "&Al" + String(k) + "=" + String(DataSet->altitude);
        result += "&Da" + String(k) + "=" + String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date)));
        result += "&Ti" + String(k) + "=" + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time)));
        result += "&Ba" + String(k) + "=" + String((double(DataSet->battery + 166) / 605), 2);
        result += "&F1" + String(k) + "=" + String((int)DataSet->secdiff);
        result += "&F2" + String(k) + "=" + String((float)DataSet->temperature / 100, 2);
        result += "&F3" + String(k) + "=" + String(DataSet->errCode, HEX);
      }
    }
  }
  return result;
}
uint16_t herdeID(){
  return 1;
}
uint16_t animalID(){
  return 1;
}
bool isInCycle(int firstCycleInHour){
  if (bootTimeStampMs == INVALID_TIME_VALUE) { return false; } // should never happen
  bool inCycle = false;
  int i = 0;
  int t = firstCycleInHour; //minute of 1st cycle within a new hour
  while (t < (60 + firstCycleInHour)){ 
    if (isInTime(t, bootTime.tm_min, bootTime.tm_sec)){
      inCycle = true;
   	  if (bootCount == i) {
        DebugPrintln("Boot: regular cycle " + String(i) + " boot", DEBUG_LEVEL_1);
      } else {
        DebugPrintln("Boot: need to set boot count to " + String(i), DEBUG_LEVEL_1);
		lastTimeDiffMs = 0; //not useful out of regular cycles
        bootCount = i;
      }
      break;
    } else {
      if (bootTime.tm_min >= firstCycleInHour) {
		if(t > bootTime.tm_min) { break; } 
	  } else {
		if(t > (bootTime.tm_min + 60)) { break; } 
	  }
    }
    t += CYCLE_DURATION_MIN;
    i++;
    if (i == BOOT_CYCLES) { i = 0; }
  }
  if (t >=60 ) { t -= 60; }
  //now we have the current (if in cycle) or the next (if out of cycle) boot minute -> t
  //time for calculations - 1st: expected boot time
  int x = (bootTime.tm_hour * 3600 + t * 60) * 1000; //ms time stamp for the next / current cycle so far
  if((x - bootTimeStampMs) < -SLEEP_DURATION_MSEC){
	//woken up early (maybe woken up yesterday, but exp. today) 
	if ((bootTime.tm_hour + 1) > 23) { expBootTime.tm_hour = 0; } else { expBootTime.tm_hour = bootTime.tm_hour + 1; } 
  } else if ((x - bootTimeStampMs) > SLEEP_DURATION_MSEC){
	//woken up late (maybe woken up today, but exp. yesterday) 
	if ((bootTime.tm_hour - 1) < 0) { expBootTime.tm_hour = 23; } else { expBootTime.tm_hour = bootTime.tm_hour - 1; } 
  } else {
    expBootTime.tm_hour = bootTime.tm_hour;
  }
  expBootTime.tm_min = t;
  expBootTime.tm_sec = 0;
  expectedBootTimeMs = expBootTime.tm_hour * 3600 + t * 60;
  expectedBootTimeMs *= 1000;
  //2nd time diff 
  if (!calcCurrentTimeDiff()) { return false; };
  return inCycle;
}
bool calcCurrentTimeDiff(){
  if ((expectedBootTimeMs == INVALID_TIME_VALUE) || (bootTimeStampMs == INVALID_TIME_VALUE)) { 
    currentTimeDiffMs = 0; 
	return false; 
  }
  int y = expectedBootTimeMs - bootTimeStampMs;
  if(y < (SLEEP_DURATION_MSEC - MS_PER_DAY)){
    currentTimeDiffMs = y + MS_PER_DAY;
  } else if (y > (MS_PER_DAY - SLEEP_DURATION_MSEC)){
	currentTimeDiffMs = y - MS_PER_DAY;
  } else {
    currentTimeDiffMs = y;
  }
  return true;
}

bool isInTime(const int target_m, const int current_m, const int current_s){
  int current = current_m;
  int target  = target_m;
  int wrap_border = 3600 - (SLEEP_DURATION_MSEC / 1000);

  if (target > 60)  { target -= 60; }
  current *= 60;
  current += current_s;
  target  *= 60;
  int diffToRegularS = target - current;
  if (diffToRegularS > wrap_border) { diffToRegularS -= 3600; }
  else if (diffToRegularS < (wrap_border * -1)) { diffToRegularS += 3600; }
  #if DEBUG_LEVEL > 0
  if ((diffToRegularS >= -SLEEP_MAX_SHIFT_S) && (diffToRegularS <= SLEEP_MAX_SHIFT_S)) {
    DebugPrintln("InTime: target: " + String(target) + "; current: " + String(current) + "diff: " + String(diffToRegularS) + "max_diff: " + String(SLEEP_MAX_SHIFT_S), DEBUG_LEVEL_1);
  }
  #endif
  return ((diffToRegularS >= -SLEEP_MAX_SHIFT_S) && (diffToRegularS <= SLEEP_MAX_SHIFT_S));
}

void SetBootTimeFromMs(int timeStampMs){
  int bootms = timeStampMs;
  if (bootms < 0) { bootms += MS_PER_DAY; }
  bootTimeStampMs = bootms;
  bootTime.tm_hour = (int)(bootms / 3600000);
  bootms -= bootTime.tm_hour * 3600000;
  bootTime.tm_min = (int)(bootms / 60000);
  bootms -= bootTime.tm_min * 60000;
  bootTime.tm_sec = bootms / 1000;
}

int8_t  GetLocalTimeHourShift(){
  return 2;
}
uint16_t measurePin(const uint8_t pin){
  unsigned long analog_value = 0;

  for(int i=0; i<2000; i++){
    analog_value += analogRead(pin);
    delay(1);
  }
  analog_value = analog_value / 2000;
  return (uint16_t)analog_value;
}
//uint16_t date; // 0-4 Day of the month / 5-8 Month /  9-15 Year offset from 1980
uint16_t dosDate(const uint8_t year, const uint8_t month, const uint8_t day){
  uint16_t date_val = day & 0x1F;
  date_val += (month & 0x0F) << 5;
  date_val += (year-1980) << 9;
  return date_val;
}
uint16_t dosYear(const uint16_t date){
  return (date >> 9) + 1980;
}
uint8_t dosMonth(const uint16_t date){
  return (date >> 5) & 0x0F;
}
uint8_t dosDay(const uint16_t date){
  return date & 0x1F;
}
//uint16_t time; // 0-4 Second divided by 2 / 5-10 Minute / 11-15 Hour
uint16_t dosTime(const uint8_t hour, const uint8_t minute, const uint8_t second){
  uint16_t time_val = second >> 1;
  time_val += (minute & 0x3F) << 5;
  time_val += (hour & 0x1F) << 11;
  return time_val;
}
uint8_t dosHour(const uint16_t time){
  return (time >> 11);
}
uint8_t dosMinute(const uint16_t time){
  return (time >> 5) & 0x3F;;
}
uint8_t dosSecond(const uint16_t time){
  return (time & 0x1F) << 1;
}
void initDataSet(t_SendData* DataSet){
  DataSet->longitude   = 0;
  DataSet->latitude    = 0;
  DataSet->altitude    = 0;
  DataSet->date        = 0;
  DataSet->time        = 0;
  DataSet->battery     = 0;
  DataSet->secdiff     = 0;
  DataSet->temperature = NO_TEMPERATURE; //-127;
  DataSet->errCode     = 0;
  DataSet->satellites  = 0;
}
bool emptyDataSet(t_SendData* DataSet){
  return (DataSet->date == 0);
}
void copyDataSet(t_SendData* _from, t_SendData* _to){	
  _to->longitude  = _from->longitude ;
  _to->latitude   = _from->latitude  ;
  _to->altitude   = _from->altitude  ;
  _to->date       = _from->date      ;
  _to->time       = _from->time      ;
  _to->battery    = _from->battery   ;
  _to->temperature= _from->temperature;
  _to->satellites = _from->satellites;
  _to->secdiff    = _from->secdiff   ;
  _to->errCode    = _from->errCode   ;
 }
String LenTwo(const String No){
  if (No.length() == 1) { return "0" + No; }
  return No;
}
void initGlobalVar(){
  bootTime.tm_hour    = INVALID_TIME_VALUE;
  bootTime.tm_min     = INVALID_TIME_VALUE;
  bootTime.tm_sec     = INVALID_TIME_VALUE;
  expBootTime.tm_hour = INVALID_TIME_VALUE;
  expBootTime.tm_min  = INVALID_TIME_VALUE;
  expBootTime.tm_sec  = INVALID_TIME_VALUE;
  sendData1.id = 0;
  sendData2.id = 1;
  sendData3.id = 2;
  sendData4.id = 3;
  sendData5.id = 4;
  sendData6.id = 5;
  sendData7.id = 6;
  sendData8.id = (DATA_SET_BACKUPS + 1) * BOOT_CYCLES - 1;

  availableDataSet[0] = &sendData1;
  availableDataSet[1] = &sendData2;
  availableDataSet[2] = &sendData3;
  availableDataSet[3] = &sendData4;
  availableDataSet[4] = &sendData5;
  availableDataSet[5] = &sendData6;
  availableDataSet[6] = &sendData7;
  availableDataSet[7] = &sendData8;
  DebugPrintln("Boot number: " + String(bootCount), DEBUG_LEVEL_1);
  if (bootCount == START_FROM_RESET){
	DebugPrintln("Clear all data sets", DEBUG_LEVEL_1);
	for(int i=0; i<MAX_DATA_SETS; i++){
      initDataSet(availableDataSet[i]);
	}
	bootCount = REFETCH_SYS_TIME;
  }
}
void restartCycling(){
  bootCount = REFETCH_SYS_TIME;
  goto_sleep(SLEEP_DURATION_MSEC);
}

void DebugPrint(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.print(text); }}
void DebugPrintln(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.println(text); }}
void DebugPrint(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number, digits); }}
void DebugPrintln(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number, digits); }}
void DebugPrint(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}
void DebugPrint(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}

void goto_sleep(int mseconds){
  int ms = millis();
  pinMode(5,INPUT);
  pinMode(14,INPUT);
  pinMode(15,INPUT);
  pinMode(16,INPUT);
  pinMode(17,INPUT);
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  pinMode(26,INPUT);
  pinMode(27,INPUT);
  pinMode(GSM,INPUT);
  pinMode(GPS,INPUT);
  delay(100);
  esp_sleep_enable_timer_wakeup((mseconds - (millis() - ms)) * uS_TO_mS_FACTOR);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}

void checkWakeUpReason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  uint32_t ebuffer = 0;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  #if DEBUG_LEVEL >= DEBUG_LEVEL_1
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : DebugPrintln("Wakeup caused by external signal using RTC_IO", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_EXT1 : DebugPrintln("Wakeup caused by external signal using RTC_CNTL", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_TIMER : DebugPrintln("Wakeup caused by timer", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : DebugPrintln("Wakeup caused by touchpad", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_ULP : DebugPrintln("Wakeup caused by ULP program", DEBUG_LEVEL_1); break;
    default : DebugPrint("Wakeup was not caused by deep sleep: ", DEBUG_LEVEL_1); DebugPrintln(String(wakeup_reason), DEBUG_LEVEL_1); break;
  }
  #endif
  ebuffer = errorCode & WRONG_BOOT_REASON_MASK;
  ebuffer = ebuffer << 1;
  if(wakeup_reason != ESP_SLEEP_WAKEUP_TIMER){
	  ebuffer |= WRONG_BOOT_REASON;
  }
  errorCode &= ~WRONG_BOOT_REASON_MASK;
  errorCode |= ebuffer;
}


#ifdef OLED_DISPLAY
void displayLocationData(int cnt, double latt, double lngg, int volt)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  String DisplayData="Digits: " + String(cnt) + ",\n U:" + String(float(volt)/4095*3.3, 2) + "\n";
  //String DisplayData="P: " + String(cnt) + ", U:" + String(float(volt)/4095*3.3, 2) + " V\nLat: " + String(latt, 6) + "\nLon: " + String(lngg, 6) + "\n";
  display.drawString(4 , 1 , DisplayData);
  display.display();
}
#endif

void testMeasure(){
  uint32_t a_measures   = 0;
  int32_t  measures     = 0;
  pinMode(BATTERY_ANALOG_ENABLE,OUTPUT);
  digitalWrite(BATTERY_ANALOG_ENABLE, LOW);
  while(true){
	for(int i=1; i<1000;i++){
	  analog_value += analogRead(BATTERY_ANALOG_PIN);
	  a_measures++;
	  delay(1);
	}
	if(a_measures > 0){
	  analog_value = analog_value / a_measures;
	}
	Serial.print("Measured Value: ");
	Serial.println(analog_value);
	a_measures   = 0;
	measures     = 0;
  }
  pinMode(BATTERY_ANALOG_ENABLE,INPUT);
}
