/*
 * nächste Aufgaben:
 *   - 2 Verbindungsversuche = doppelte Werte-Übertragung, wenn es beim 2. mal klappt?
 *   - Energiesparmodi - Tiefschlaf bei U < 3,4 - Diode von Stützakku durch Draht ersetzen?
 *   - Datenmenge begrenzen - base64 Übertragung
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
#include "images.h"
#ifdef OLED_DISPLAY
#include "SSD1306.h"
#endif
#include <sys/time.h>
#include "heidi-measures.h"
#include "heidi-sys.h"

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

RTC_DATA_ATTR int8_t     bootCount            = START_FROM_RESET;
RTC_DATA_ATTR int8_t     lastWrongResetReason = 0;
RTC_DATA_ATTR int32_t    lastTimeDiffMs       = 0;

t_SendData* currentDataSet;

tm bootTime;
tm expBootTime;
esp_timer_handle_t watchd; //the watchdog

int currentTimeDiffMs  = 0;

static int msStartTime        = 0;
static uint16_t init_cycle    = (herdeID() % 10) * 6;
static double volt;


void setup()
{
  tm timeinfo;
  msStartTime = millis();
  #ifdef GSM_MODULE
    GSM_off();
  #endif
  LED_off();
  setupWatchDog();
  Serial.begin(115200); //9600);
  /**** check voltage ****/
  MeasuresOn();  //enable all measures - switch off in goto_sleep
  volt = MeasureVoltage();
  _D(DebugPrintln("Battery: " + String(volt, 2), DEBUG_LEVEL_1));
  if (volt <= 3.0){ // low Battery?
    _D(DebugPrintln("Battery much too low.", DEBUG_LEVEL_1));
    _D(delay(100));
    goto_sleep(3600000);
  }
  if (volt <= 3.3){ // low Battery?
    _D(DebugPrintln("Battery too low.", DEBUG_LEVEL_1));
    _D(delay(100));
    goto_sleep(SLEEP_DURATION_MSEC - millis());
  }
  _D(DebugPrintln("Boot number: " + String(bootCount), DEBUG_LEVEL_1));
  /**** setup ****/
#ifdef OLED_DISPLAY
  void SetupDisplay();
#endif
#ifdef USE_LORA
  SetupLoRa();
#endif
  initGlobalVar();
  initError();

  checkWakeUpReason();
  if (lastWrongResetReason != 0){
    setError(WRONG_BOOT_REASON);
  }
#ifdef GPS_MODULE
  /**** check sys-time and cycle ****/
  openGPS();
  if((!GetSysTime(&timeinfo)) || (bootCount <= REFETCH_SYS_TIME)){
    lastTimeDiffMs = 0; //not useful anymore
	  delay(3000);
    _D(DebugPrintln("Boot: Get time from GPS", DEBUG_LEVEL_1));
    if (!SetSysToGPSTime()){ //sets boot time if succeed
      closeGPS();
      goto_sleep(SLEEP_DUR_NOTIME);
    } else {
      if (!calcCurrentTimeDiff()){
        closeGPS();
        bootCount = REFETCH_SYS_TIME;
        goto_sleep(SLEEP_DURATION_MSEC);
      }
    }
    _D(DebugPrintln("GPS time vs. internal timer: " + String(currentTimeDiffMs), DEBUG_LEVEL_1));
    GetSysTime(&timeinfo);
  } else {
    int bootMs = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
    bootMs *= 1000;
    bootMs -= millis();
    SetBootTimeFromMs(bootMs);
  }
  _copyDate(&timeinfo, &bootTime);
  _D(DebugPrintln("Current sys time: " + DateString(timeinfo) + " " + TimeString(timeinfo), DEBUG_LEVEL_1));
  _D(DebugPrintln("sys. boot time:   " + DateString(bootTime) + " " + TimeString(bootTime), DEBUG_LEVEL_1));
  _D(DebugPrint("Battery: ", DEBUG_LEVEL_1));
  _D(DebugPrintln(volt, 2, DEBUG_LEVEL_1));
  //find expected boot time
  if(!isInCycle(init_cycle, &bootCount)){ //calculates expected boot time
	lastTimeDiffMs = 0;       //not useful out of regular cycles
    _D(doResetTests());
    if (currentTimeDiffMs <= 0) { goto_sleep(SLEEP_DURATION_MSEC); } //something went wrong
	if ((currentTimeDiffMs - millis()) < 1000) { goto_sleep(1000); } //just to be sure
    closeGPS();
    goto_sleep(currentTimeDiffMs - millis());
  }
  lastWrongResetReason = 0;
  currentDataSet = availableDataSet[bootCount];

  /**** measure ****/
  initDataSet(currentDataSet);
  currentDataSet->errCode = getErrorCode(); //errors until now
  if (GPSGetPosition(currentDataSet, 10, 90000) == 0){
    _D(DebugPrintln("GPS: Unable to fetch position.", DEBUG_LEVEL_1));
  }
  closeGPS();
  _D(DebugPrintln("cor. boot time: " + TimeString(bootTime), DEBUG_LEVEL_1));
  _D(DebugPrintln("exp. boot time: " + TimeString(expBootTime), DEBUG_LEVEL_1));
#else
  GetSysTime(&timeinfo);
  _copyTime(&timeinfo, &bootTime);
  _copyDate(&timeinfo, &bootTime);
#endif
#ifdef TEMP_SENSOR
  currentDataSet->temperature = (int16_t)round(MeasureTemperature() * 100);
  _D(DebugPrint("Temperature: ", DEBUG_LEVEL_1));
  _D(DebugPrint(currentDataSet->temperature / 100, DEBUG_LEVEL_1));
  _D(DebugPrintln(" C", DEBUG_LEVEL_1));
#endif
  currentDataSet->date    = dosDate(bootTime.tm_year, bootTime.tm_mon, bootTime.tm_mday);
  currentDataSet->time    = dosTime(bootTime.tm_hour, bootTime.tm_min, bootTime.tm_sec);
  currentDataSet->battery = (uint16_t)(round(volt * 1000));
  _D(_PrintDataSet(currentDataSet));
#if DEBUG_LEVEL >= DEBUG_LEVEL_2
  {
    // why that complicated and not just memcopy?
    //push_data.phtml expects 8 bit count of values, 16 bit ID, 2x32bit coordinates
    // and than count-3 16 bit values - always 16 bit
    byte hexbuffer[64];
    String b64u = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";   // base64url dictionary
    hexbuffer[0] = 11; //count of data values
    hexbuffer[1] = herdeID();
    hexbuffer[2] = animalID();
    #define HEX_BUFFER_OFFSET 3
    _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET +  0, currentDataSet->latitude);
    _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET +  4, currentDataSet->longitude);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET +  8, currentDataSet->altitude);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 10, currentDataSet->date);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 12, currentDataSet->time);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 14, currentDataSet->battery);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 16, currentDataSet->temperature);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 18, currentDataSet->errCode);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 20, currentDataSet->secGPS);
    _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 22, currentDataSet->satellites);
    #define HEX_BUFFER_LEN (HEX_BUFFER_OFFSET + 24)
    String HexStr = "";
    for(int i=0; i<HEX_BUFFER_LEN; i++){
      String _hex = String(hexbuffer[i], HEX);
      if (_hex.length() < 2) {_hex = "0" + _hex;}
      HexStr = HexStr + _hex;
    }
    _D(DebugPrintln("bin: " + HexStr, DEBUG_LEVEL_2));
    String Base64Str = "";
    for (int i=0; i<=(HEX_BUFFER_LEN-3); i+=3){
      Base64Str += b64u.charAt(hexbuffer[i] >> 2);
      Base64Str += b64u.charAt(((hexbuffer[i]   & 3)  << 4) | (hexbuffer[i+1] >> 4));
      Base64Str += b64u.charAt(((hexbuffer[i+1] & 15) << 2) | (hexbuffer[i+2] >> 6));
      Base64Str += b64u.charAt(hexbuffer[i+2]   & 63);
    }
    if (HEX_BUFFER_LEN % 3 == 2){
      Base64Str += b64u.charAt(hexbuffer[HEX_BUFFER_LEN-2] >> 2);
      Base64Str += b64u.charAt(((hexbuffer[HEX_BUFFER_LEN-2] & 3)<< 4) | (hexbuffer[HEX_BUFFER_LEN-1] >> 4));
      Base64Str += b64u.charAt((hexbuffer[HEX_BUFFER_LEN-1] & 15) << 2);
    }
    else if (HEX_BUFFER_LEN % 3 == 1){
      Base64Str += b64u.charAt(hexbuffer[HEX_BUFFER_LEN-1] >> 2);
      Base64Str += b64u.charAt((hexbuffer[HEX_BUFFER_LEN-1] & 3)<< 4);
    }
    _D(DebugPrintln("bas: " + Base64Str, DEBUG_LEVEL_2));
  }
#endif
  MeasuresOff();

#ifdef GSM_MODULE
  if (doDataTransmission(bootCount)){
	  int HTTPrc = 0;
	  bool GSMfailure = true;
	  _D(_PrintShortSummary());
    String sendLine = generateMultiSendLine(0, BOOT_CYCLES - 1, DATA_SET_BACKUPS);
    if ((sendLine.length() > 0) && (volt >= 3.5)){
      _D(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_3));
      /*for (int i=0; i<2; i++)*/{
        GSM_on();
        if (GSMsetup()){
          _D(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_1));
          int HTTPtimeOut = sendLine.length() * 20 + 1000;
          if (HTTPtimeOut < 10000) {HTTPtimeOut = 10000;}
          HTTPrc = GSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data2.php",
                             "application/x-www-form-urlencoded",
                              sendLine,
                              HTTPtimeOut,
                              HTTPtimeOut);
          if (HTTPrc == 200){
            GSMfailure = false;
          }
        }else{
          _D(DebugPrintln("GPRS check OK.", DEBUG_LEVEL_1));
        }
        GSMshutDown();
        GSM_off();
      }//*for (int i=0; i<2; i++)*/
      cleanUpDataSets(GSMfailure);
    }
    _D(DebugPrintln("GPRS send done: " + String(millis() - msStartTime), DEBUG_LEVEL_2));
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

void goto_sleep(uint32_t mseconds){
  esp_timer_delete(watchd);
  forceMeasuresOff();
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
  pinMode(MEASURES_ENABLE_PIN,INPUT); // HIGH-Z = off
  #if DEBUG_LEVEL >= DEBUG_LEVEL_2
  _D(DebugPrintln("Sleep for : " + String(uint32_t(mseconds/1000)) + " seconds", DEBUG_LEVEL_2));
  delay(100);
  #endif
  esp_sleep_enable_timer_wakeup(uint64_t(mseconds) * uint64_t(uS_TO_mS_FACTOR));
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}

void checkWakeUpReason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
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
  if(wakeup_reason != ESP_SLEEP_WAKEUP_TIMER){
	  lastWrongResetReason = 1;
  }
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
#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void doResetTests(){
  _D(DebugPrintln("Boot: not in cycle.", DEBUG_LEVEL_1));
  #ifdef GSM_MODULE
  if (volt >= 3.6){ GSMCheckSignalStrength(); }
  #endif
  #ifdef TEMP_SENSOR
  _D(DebugPrintln("Temperature: " + String(MeasureTemperature()), DEBUG_LEVEL_1));;
  #endif
  _D(DebugPrintln("Sleep for : " + String(currentTimeDiffMs - millis()), DEBUG_LEVEL_2));
  delay(100);
}
void _PrintDataSet(t_SendData* DataSet){
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
  _D(DebugPrint(String(double(DataSet->battery)/1000, 2), DEBUG_LEVEL_2));
  _D(DebugPrint(" Satellites= ", DEBUG_LEVEL_2));
  _D(DebugPrintln(DataSet->satellites, DEBUG_LEVEL_2));
}
void _PrintShortSummary(){
  for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
    t_SendData* DataSet = availableDataSet[i];
    _D(DebugPrint("Dataset " + String(i) + ": ", DEBUG_LEVEL_3));
    _D(DebugPrint(LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))), DEBUG_LEVEL_3));
    _D(DebugPrintln("; " + String(DataSet->errCode, HEX), DEBUG_LEVEL_3));
  }
}
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
#endif
