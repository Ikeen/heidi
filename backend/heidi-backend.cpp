/*
 * nächste Aufgaben:
 *   - 2 Verbindungsversuche = doppelte Werte-Übertragung, wenn es beim 2. mal klappt?
 *   - Energiesparmodi
 *   - Nacht - weniger Werte / Übertragungen
 *   - Providermanagement
 *   - Hook beim Reset oder Power-Cut
 *   - einstellbarer Vorzugsprovider
 *   - Neue Provider
 *   - Neue Karten-Daten
 *   - Provider - Scans
 *   -
 * - Erweitertes Powermanagement (kein Durchbooten bei < 3.6V)
 * - Dichtere Übertragungsversuche (RTC-Memory? 4*32*50 = 6400 Bytes)
 *
 */

#include "heidi-backend.h"
#include "TinyGPS++.h"
#include "images.h"
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

/*
RTC_DATA_ATTR t_SendData sendData1; //
RTC_DATA_ATTR t_SendData sendData2; //8k haben wir insgesamt
RTC_DATA_ATTR t_SendData sendData3;
RTC_DATA_ATTR t_SendData sendData4;
RTC_DATA_ATTR t_SendData sendData5;
RTC_DATA_ATTR t_SendData sendData6;
RTC_DATA_ATTR t_SendData sendData7;
RTC_DATA_ATTR t_SendData sendData8;
*/
RTC_DATA_ATTR int8_t     bootCount            = START_FROM_RESET;
RTC_DATA_ATTR int8_t     lastWrongResetReason = 0;
RTC_DATA_ATTR int32_t    lastTimeDiffMs       = 0;
RTC_DATA_ATTR uint32_t   errorCode            = 0;

t_SendData* currentDataSet;

// The TinyGPS++ object
HardwareSerial SerialGPS(GPS_UART_NO);
TinyGPSPlus gps;

#ifdef TEMP_SENSOR
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
#endif

tm bootTime;
tm expBootTime;
esp_timer_handle_t watchd; //the watchdog
int bootTimeStampMs    = INVALID_TIME_VALUE;
int expectedBootTimeMs = INVALID_TIME_VALUE;
int currentTimeDiffMs  = 0;

int msStartTime = 0;
double volt;

void setup()
{
  tm timeinfo;
  msStartTime = millis();
  uint16_t init_cycle  = (herdeID() % 10) * 6;
  Serial.begin(115200); //9600);
#ifdef GSM_MODULE
  GSM_off();
#endif
#ifdef USE_LORA
  SetupLoRa();
#endif
  pinMode(LED,OUTPUT);
  digitalWrite(LED, LED_OFF);
  _D(DebugPrintln("Boot number: " + String(bootCount), DEBUG_LEVEL_1));
  checkWakeUpReason();
  setupWatchDog();
#ifdef OLED_DISPLAY
  void SetupDisplay();
#endif

  Measures_On();
  delay(10);
  volt = GetVoltage();
  _D(DebugPrintln("Battery: " + String(volt, 2), DEBUG_LEVEL_1));
  if (volt <= 3.5){
    _D(DebugPrintln("Battery too low.", DEBUG_LEVEL_1));
	delay(100);
    goto_sleep(SLEEP_DURATION_MSEC - millis());
  }

  initGlobalVar();
#ifdef GPS_MODULE
  if((!GetSysTime(&timeinfo)) || (bootCount <= REFETCH_SYS_TIME)){
    lastTimeDiffMs = 0; //not useful anymore
	delay(3000);
    _D(DebugPrintln("Boot: Get time from GPS", DEBUG_LEVEL_1));
    if (!SetSysToGPSTime()){ //sets boot time if succeed
      Measures_Off();
      goto_sleep(SLEEP_DUR_NOTIME);
    }
    GetSysTime(&timeinfo);
  } else {
    int bootMs = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
    bootMs *= 1000;
    bootMs -= millis();
    SetBootTimeFromMs(bootMs);
  }
  bootTime.tm_wday = timeinfo.tm_wday;
  bootTime.tm_mday = timeinfo.tm_mday;
  bootTime.tm_yday = timeinfo.tm_yday;
  bootTime.tm_mon  = timeinfo.tm_mon;
  bootTime.tm_year = timeinfo.tm_year;
  _D(DebugPrintln("Current sys time: " + DateString(timeinfo) + " " + TimeString(timeinfo), DEBUG_LEVEL_1));
  _D(DebugPrintln("sys. boot time:   " + DateString(bootTime) + " " + TimeString(bootTime), DEBUG_LEVEL_1));
  _D(DebugPrint("Battery: ", DEBUG_LEVEL_1));
  _D(DebugPrintln(volt, 2, DEBUG_LEVEL_1));
  //find expected boot time
  if(!isInCycle(init_cycle)){ //calculates expected boot time
    Measures_Off();
    #if DEBUG_LEVEL >= DEBUG_LEVEL_0
    _D(DebugPrintln("Boot: not in cycle.", DEBUG_LEVEL_1));
    #ifdef GSM_MODULE
    GSMCheckSignalStrength();
    #endif
    #ifdef TEMP_SENSOR
    tempSensor.begin();
    tempSensor.requestTemperaturesByIndex(0);
    _D(DebugPrintln("Temperature: " + String(tempSensor.getTempCByIndex(0)), DEBUG_LEVEL_1));;
    #endif
    _D(DebugPrintln("Sleep for : " + String(currentTimeDiffMs - millis()), DEBUG_LEVEL_2));
    delay(100);
    #endif
	lastTimeDiffMs = 0; //not useful since time was not correct
	if (currentTimeDiffMs <= 0) { goto_sleep(SLEEP_DURATION_MSEC); } //something went wrong
	if ((currentTimeDiffMs - millis()) < 1000) { goto_sleep(1000); } //just to be sure
    goto_sleep(currentTimeDiffMs - millis());
  }
  currentDataSet = availableDataSet[bootCount];
  initDataSet(currentDataSet);
  if (GPSGetPosition(currentDataSet, 10, 90000) == 0){
    _D(DebugPrintln("GPS: Unable to fetch position.", DEBUG_LEVEL_1));
  }
  _D(DebugPrintln("cor. boot time: " + TimeString(bootTime), DEBUG_LEVEL_1));
  _D(DebugPrintln("exp. boot time: " + TimeString(expBootTime), DEBUG_LEVEL_1));
#else
  GetSysTime(&timeinfo);
  bootTime.tm_sec = timeinfo.tm_sec;
  bootTime.tm_min = timeinfo.tm_min;
  bootTime.tm_hour = timeinfo.tm_hour;
  bootTime.tm_wday = timeinfo.tm_wday;
  bootTime.tm_mday = timeinfo.tm_mday;
  bootTime.tm_yday = timeinfo.tm_yday;
  bootTime.tm_mon  = timeinfo.tm_mon;
  bootTime.tm_year = timeinfo.tm_year;
#endif
#ifdef TEMP_SENSOR
  tempSensor.begin();
  tempSensor.requestTemperaturesByIndex(0);
  currentDataSet->temperature = (int16_t)round(tempSensor.getTempCByIndex(0) * 100);
  _D(DebugPrint("Temperature: ", DEBUG_LEVEL_1));
  _D(DebugPrint(currentDataSet->temperature / 100, DEBUG_LEVEL_1));
  _D(DebugPrintln(" C", DEBUG_LEVEL_1));
#endif
  Measures_Off();
  currentDataSet->errCode = errorCode;
#ifdef GSM_MODULE
  int  HTTPrc = 0;
  bool GSMfailure = false;
  if (bootCount == BOOT_CYCLES - 1){
    #if DEBUG_LEVEL >= DEBUG_LEVEL_1
    for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
      t_SendData* DataSet = availableDataSet[i];
      Serial.print("Dataset " + String(i) + ": ");
      Serial.print(LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))));
      Serial.println("; " + String(DataSet->errCode, HEX));
    }
    #endif
	GSMfailure = true;
    String sendLine = generateMultiSendLine(0, BOOT_CYCLES - 1, DATA_SET_BACKUPS);
    if ((sendLine.length() > 0) && (volt >= 3.5)){
      _D(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_3));
      /*for (int i=0; i<2; i++)*/{
        GSM_on();
        if (GSMsetup()){
          _D(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_1));
          int HTTPtimeOut = sendLine.length() * 20;
          if (HTTPtimeOut < 10000) {HTTPtimeOut = 10000;}
          HTTPrc = GSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data.php",
                             "application/x-www-form-urlencoded",
                              sendLine,
                              HTTPtimeOut,
					          HTTPtimeOut);
          if (HTTPrc == 200){
            GSMfailure = false;
            if ((errorCode & WRONG_BOOT_REASON) == 0) { lastWrongResetReason = 0; }
            //i = 2;
          }
        }else{
          _D(DebugPrintln("GPRS check OK.", DEBUG_LEVEL_1));
        }
        GSMshutDown();
        GSM_off();
        //mark transmission result
        for(int i=(BOOT_CYCLES * DATA_SET_BACKUPS); i >= 0; i--){
          if (!emptyDataSet(availableDataSet[i])){
            if (GSMfailure)  { availableDataSet[i]->errCode |= GSM_TRANSMISSION_FAILED; }
            else { availableDataSet[i]->errCode &= ~GSM_TRANSMISSION_FAILED; }
          }
        }
        //delete all transmitted data
        for(int i=(BOOT_CYCLES * DATA_SET_BACKUPS); i >= 0; i--){
          if(availableDataSet[i]->errCode & GSM_TRANSMISSION_FAILED == 0){
        	initDataSet(availableDataSet[i]);
          }
        }
        //concentrate
        int k = 0;
        for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
          if(!emptyDataSet(availableDataSet[i])){
            if (k != i){
        	  copyDataSet(availableDataSet[i], availableDataSet[k]);
              initDataSet(availableDataSet[i]);
            }
            k++;
          }
        }
        //if (i<1) { delay(1000); }
      }
    }
    #if DEBUG_LEVEL >= DEBUG_LEVEL_0
    if (GSMfailure){
	  if(volt < 3.5){
	    _D(DebugPrintln("GSM: low battery.", DEBUG_LEVEL_1));
	  } else {
        _D(DebugPrintln("GSM transmission failed: " + String(HTTPrc), DEBUG_LEVEL_1));
	  }
    }
    #endif
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
  int sleeptime = SLEEP_DURATION_MSEC + diffTimeFinalMs - millis();
  if (sleeptime <= 0) { goto_sleep(SLEEP_DURATION_MSEC); } //something went wrong
  if (sleeptime < 10000){
    #if DEBUG_LEVEL >= DEBUG_LEVEL_2
    _D(DebugPrintln("Sleep for : 1000", DEBUG_LEVEL_2));
    delay(100);
	goto_sleep(9900);
    #else
	goto_sleep(10000);
    #endif
  } else {
    #if DEBUG_LEVEL >= DEBUG_LEVEL_2
    _D(DebugPrintln("Sleep for : " + String(sleeptime), DEBUG_LEVEL_2));
    delay(100);
	goto_sleep(sleeptime-100);
    #else
    goto_sleep(sleeptime);
    #endif
  }
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
  bool timestamp = false;
  uint32_t startMs = millis();

  errorCode |= (COULD_NOT_FETCH_GPS_1 << bootCount);
  errorCode |= (WRONG_GPS_VALUES_1 << bootCount);
  errorCode |= COULD_NOT_FETCH_GPS_TIME;

  //pinMode(BATTERY_ANALOG_ENABLE,OUTPUT);
  //digitalWrite(BATTERY_ANALOG_ENABLE, LOW);
  _D(DebugPrintln("GPS: Acquire position", DEBUG_LEVEL_1));
  while(((measures < averages) || (gps_day == 0)) && ((millis() - startMs) < timeoutms)){
    while ((SerialGPS.available() == 0) && ((millis() - startMs) < timeoutms)) {
      analog_value += analogRead(BATTERY_ANALOG_PIN);
      a_measures++;
      delay(1);
    }
    while ((SerialGPS.available() > 0) && ((measures < averages) || (gps_day == 0))) {
      analog_value += analogRead(BATTERY_ANALOG_PIN);
      a_measures++;
      gps.encode(SerialGPS.read());
      if (   gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated()
    	  && gps.date.isUpdated() && gps.date.isValid() && gps.time.isUpdated() && gps.time.isValid()){
        measures++;
        if ((!timestamp) && ((gps.location.age() <= 1) || ((millis() - startMs - timeoutms) < 1500))){
          if (SetSysToGPS()){ //resets boot time corrected by GPS time
            if (calcCurrentTimeDiff()){
              _D(DebugPrintln("GPS time vs. internal timer: " + String(currentTimeDiffMs), DEBUG_LEVEL_1));
              timestamp = true;
            } else { restartCycling(); }
          } else { restartCycling(); }
        }
        if ((gps_day == 0) and (gps.date.day() > 0)){ gps_day = gps.date.day(); }
        if (gps_sat < gps.satellites.value()) { gps_sat = gps.satellites.value(); }
        #if DEBUG_LEVEL >= DEBUG_LEVEL_1
        _D(DebugPrint("Latitude= ", DEBUG_LEVEL_3));
        _D(DebugPrint(gps.location.lat(), 6, DEBUG_LEVEL_3));
        _D(DebugPrint(" Longitude= ", DEBUG_LEVEL_3));
        _D(DebugPrint(gps.location.lng(), 6, DEBUG_LEVEL_3));
        _D(DebugPrint(" Altitude= ", DEBUG_LEVEL_3));
        _D(DebugPrint(gps.altitude.meters(), 6, DEBUG_LEVEL_3));
        _D(DebugPrint(" Date= ", DEBUG_LEVEL_3));
        _D(DebugPrint(String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()), DEBUG_LEVEL_3));
        _D(DebugPrint(" Time= ", DEBUG_LEVEL_3));
        _D(DebugPrint(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3));
        _D(DebugPrint(" Age= ", DEBUG_LEVEL_3));
        _D(DebugPrint(gps.location.age(), DEBUG_LEVEL_3));
        _D(DebugPrint("ms", DEBUG_LEVEL_3));
        _D(DebugPrint(" Sat= ", DEBUG_LEVEL_3));
        _D(DebugPrintln(gps.satellites.value(), DEBUG_LEVEL_3));
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
  DataSet->latitude = (int32_t)(a_lat * 1000000);
  DataSet->longitude = (int32_t)(a_lng * 1000000);
  DataSet->altitude = (uint16_t)a_alt;
  DataSet->date = dosDate(bootTime.tm_year, bootTime.tm_mon, bootTime.tm_mday);
  DataSet->time = dosTime(bootTime.tm_hour, bootTime.tm_min, bootTime.tm_sec);
  DataSet->battery = (uint16_t)analog_value;
  DataSet->secGPS = (int8_t)((millis() - startMs)/1000);
  if (!((measures > 0) && (DataSet->latitude == 0) && (DataSet->longitude == 0))){ //0.0, 0.0 must be wrong (or a fish)
    errorCode &= ~(WRONG_GPS_VALUES_1 << bootCount);
  }
  if (gps_day != 0){
    errorCode &= ~COULD_NOT_FETCH_GPS_TIME;
  }
  DataSet->satellites = gps_sat;
  _D(DebugPrint("DataSet Latitude= ", DEBUG_LEVEL_2));
  _D(DebugPrint(DataSet->latitude, DEBUG_LEVEL_2));
  _D(DebugPrint(" Longitude= ", DEBUG_LEVEL_2));
  _D(DebugPrint(DataSet->longitude, DEBUG_LEVEL_2));
  _D(DebugPrint(" Altitude= ", DEBUG_LEVEL_2));
  _D(DebugPrint(DataSet->altitude, DEBUG_LEVEL_2));
  _D(DebugPrint(" TimeStamp= ", DEBUG_LEVEL_2));
  _D(DebugPrint(String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date))), DEBUG_LEVEL_2));
  _D(DebugPrint(" " + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))), DEBUG_LEVEL_2));
  _D(DebugPrint(" Battery= ", DEBUG_LEVEL_2));
  _D(DebugPrint((double(DataSet->battery + ANALOG_MEASURE_OFFSET) / ANALOG_MEASURE_DIVIDER), 2, DEBUG_LEVEL_2));
  _D(DebugPrint(" Satellites= ", DEBUG_LEVEL_2));
  _D(DebugPrintln(DataSet->satellites, DEBUG_LEVEL_2));
  _D(DebugPrintln("GPS done: " + String(DataSet->secGPS), DEBUG_LEVEL_2));
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
          _D(DebugPrint("Set sys-time to GPS time Date= ", DEBUG_LEVEL_3));
          _D(DebugPrint(String(year) + "-" + String(mon) + "-" + String(day), DEBUG_LEVEL_3));
          _D(DebugPrint(" Time= ", DEBUG_LEVEL_3));
          _D(DebugPrintln(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3));
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
  time.tm_mon  = gps.date.month()-1;
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
void Measures_On(){
  pinMode(GPS,OUTPUT);
  digitalWrite(GPS, GPS_ON);
  _D(DebugPrintln("Measures on", DEBUG_LEVEL_1));
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX, false);
  while(!SerialGPS) { Serial.print("."); }
  _D(DebugPrintln("GPS on", DEBUG_LEVEL_1));
}
void Measures_Off(){
  SerialGPS.end();
  pinMode(GPS,OUTPUT);
  digitalWrite(GPS, GPS_OFF);
  _D(DebugPrintln("GPS off", DEBUG_LEVEL_1));
  delay(100);
}
#endif //GPS_MODULE




#ifdef USE_LORA
void SetupLoRa(){
  _D(DebugPrint("LoRa init .. ", DEBUG_LEVEL_1));
  while(!Serial) { Serial.print("."); }
  _D(DebugPrintln(" done", DEBUG_LEVEL_3));

  _D(DebugPrint("Starting LoRa at ", DEBUG_LEVEL_3));
  _D(DebugPrint(String(BAND/1E6,2), DEBUG_LEVEL_3));
  _D(DebugPrint("MHz ", DEBUG_LEVEL_3));
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    _D(DebugPrintln(" failed!", DEBUG_LEVEL_3));
    while (1);
  }
  _D(DebugPrintln(".. done", DEBUG_LEVEL_3));
  _D(DebugPrintln("Configuring LoRa", DEBUG_LEVEL_3));
  _D(DebugPrint(" - spreading factor: ", DEBUG_LEVEL_3));
  _D(DebugPrintln(spreadingFactor, DEBUG_LEVEL_3));
  LoRa.setSpreadingFactor(spreadingFactor);
  _D(DebugPrint(" - signal bandwidth: ", DEBUG_LEVEL_3));
  _D(DebugPrintln(SignalBandwidth, DEBUG_LEVEL_3));
  LoRa.setSignalBandwidth(SignalBandwidth);
  _D(DebugPrint(" - code rate: 4/", DEBUG_LEVEL_3));
  _D(DebugPrintln(codingRateDenominator, DEBUG_LEVEL_3));
  LoRa.setCodingRate4(codingRateDenominator);
  _D(DebugPrint(" - preamble length: ", DEBUG_LEVEL_3));
  _D(DebugPrintln(preambleLength, DEBUG_LEVEL_3));
  LoRa.setPreambleLength(preambleLength);
  _D(DebugPrintln("done", DEBUG_LEVEL_1));
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
        _D(DebugPrintln("Boot: regular cycle " + String(i) + " boot", DEBUG_LEVEL_1));
      } else {
        _D(DebugPrintln("Boot: need to set boot count to " + String(i), DEBUG_LEVEL_1));
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
    _D(DebugPrintln("InTime: target: " + String(target) + "; current: " + String(current) + "diff: " + String(diffToRegularS) + "max_diff: " + String(SLEEP_MAX_SHIFT_S), DEBUG_LEVEL_1));
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

double GetVoltage(){
  int analog_value;
  //pinMode(BATTERY_ANALOG_ENABLE,OUTPUT);
  //digitalWrite(BATTERY_ANALOG_ENABLE, LOW);
  analog_value = 0;
  for(int i=0; i<1000; i++){ analog_value += analogRead(BATTERY_ANALOG_PIN); }
  analog_value /= 1000;
  return((double)(analog_value + ANALOG_MEASURE_OFFSET) / ANALOG_MEASURE_DIVIDER);
}

int8_t  GetLocalTimeHourShift(){
  /*
  tm cur, summer, winter;
  if (!GetSysTime(&cur)){ return 0; }
  summer.tm_year = cur.tm_year;
  summer.tm_mon  = 3;
  summer.tm_mday = 31;
  */
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
bool GetSysTime(tm *info){
  uint32_t count = 500;
  time_t now;
  do{
    time(&now);
    localtime_r(&now, info);
    if(info->tm_year > (2016 - 1900)){ info->tm_year += 1900; info->tm_mon++; return true; }
    delay(10);
  }while(count--);
  return false;
}

void initGlobalVar(){
  bootTime.tm_hour    = INVALID_TIME_VALUE;
  bootTime.tm_min     = INVALID_TIME_VALUE;
  bootTime.tm_sec     = INVALID_TIME_VALUE;
  expBootTime.tm_hour = INVALID_TIME_VALUE;
  expBootTime.tm_min  = INVALID_TIME_VALUE;
  expBootTime.tm_sec  = INVALID_TIME_VALUE;
  initDataSets();
  _D(DebugPrintln("Boot number: " + String(bootCount), DEBUG_LEVEL_1));
  if (bootCount == START_FROM_RESET){
	_D(DebugPrintln("Clear all data sets", DEBUG_LEVEL_1));
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

void goto_sleep(int mseconds){
  esp_timer_delete(watchd);
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
  esp_sleep_enable_timer_wakeup(mseconds * uS_TO_mS_FACTOR);
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
    case ESP_SLEEP_WAKEUP_EXT0 : _D(DebugPrintln("Wakeup caused by external signal using RTC_IO", DEBUG_LEVEL_1)); break;
    case ESP_SLEEP_WAKEUP_EXT1 : _D(DebugPrintln("Wakeup caused by external signal using RTC_CNTL", DEBUG_LEVEL_1)); break;
    case ESP_SLEEP_WAKEUP_TIMER : _D(DebugPrintln("Wakeup caused by timer", DEBUG_LEVEL_1)); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : _D(DebugPrintln("Wakeup caused by touchpad", DEBUG_LEVEL_1)); break;
    case ESP_SLEEP_WAKEUP_ULP : _D(DebugPrintln("Wakeup caused by ULP program", DEBUG_LEVEL_1)); break;
    default : _D(DebugPrint("Wakeup was not caused by deep sleep: ", DEBUG_LEVEL_1)); _D(DebugPrintln(String(wakeup_reason), DEBUG_LEVEL_1)); break;

  }
  #endif
  ebuffer =  errorCode & WRONG_BOOT_REASON_MASK;
  ebuffer =  ebuffer << 1;
  ebuffer &= WRONG_BOOT_REASON_MASK;
  if(wakeup_reason != ESP_SLEEP_WAKEUP_TIMER){
	  ebuffer |= WRONG_BOOT_REASON;
	  lastWrongResetReason = (int8_t)rtc_get_reset_reason(0);
  }
  errorCode &= ~WRONG_BOOT_REASON_MASK;
  errorCode |= ebuffer;
  errorCode &= ~WRONG_RESET_REASON_MASK;
  errorCode |= (lastWrongResetReason << 24) & WRONG_RESET_REASON_MASK;
  _D(DebugPrintln("Error Code: " +  String(errorCode, HEX), DEBUG_LEVEL_1));
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
  int analog_value      = 0;
  //pinMode(BATTERY_ANALOG_ENABLE,OUTPUT);
  //digitalWrite(BATTERY_ANALOG_ENABLE, LOW);
  while(true){
	for(int i=1; i<1000;i++){
	  analog_value += analogRead(BATTERY_ANALOG_PIN);
	  a_measures++;
	  delay(1);
	}
	if(a_measures > 0){
	  analog_value /= a_measures;
	}
	Serial.print("Measured Value: ");
	Serial.println(analog_value);
	a_measures   = 0;
	measures     = 0;
  }
  //pinMode(BATTERY_ANALOG_ENABLE,INPUT);
}

static void watchDog(void* arg)
{
	_D(DebugPrintln("This is the watchDog - Howdy?! :-D",DEBUG_LEVEL_1));
	delay(10);
	bootCount = REFETCH_SYS_TIME;
    goto_sleep(10000);
}

void setupWatchDog(void){
    const esp_timer_create_args_t timargs = {
            .callback = &watchDog
    //        .name = "takecare"
    };
    esp_timer_create(&timargs, &watchd);
    esp_timer_start_once(watchd, 300000000); //cut all after 5 minutes
}

