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
#include <sys/time.h>
#include "heidi-measures.h"
#include "heidi-sys.h"

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
  hGSM_off();
  #endif
  LED_off();
  setupWatchDog();
  Serial.begin(115200); //9600);
  /**** check voltage ****/
  #ifdef CHECK_BATTERY
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
  #else
  volt = 4.0;
  #endif
  _D(DebugPrintln("Boot number: " + String(bootCount), DEBUG_LEVEL_1));
  /**** setup ****/
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
    doResetInits();
    #ifndef TEST_ON_BOOT
    if (currentTimeDiffMs <= 0) { goto_sleep(SLEEP_DURATION_MSEC); } //something went wrong
	  if ((currentTimeDiffMs - millis()) < 1000) { goto_sleep(1000); } //just to be sure
    closeGPS();
    goto_sleep(currentTimeDiffMs - millis());
    #else
    bootCount = 0;
    #endif
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
  currentTimeDiffMs = SLEEP_DUR_NOTIME;
  _D(doResetTests());
  if (GetSysTime(&timeinfo)){
    _copyTime(&timeinfo, &bootTime);
    _copyDate(&timeinfo, &bootTime);
  } else {
    MeasuresOff();
    goto_sleep(SLEEP_DUR_NOTIME);
  }
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
  _D(_PrintDataSet(currentDataSet, DEBUG_LEVEL_3));
  MeasuresOff();

#ifdef GSM_MODULE
  if (doDataTransmission(bootCount)){
	  int HTTPrc = 0;
	  bool GSMfailure = true;
	  _D(_PrintShortSummary(DEBUG_LEVEL_3));
    String sendLine = generateMulti64SendLine(0, BOOT_CYCLES - 1, DATA_SET_BACKUPS);
    if ((sendLine.length() > 0) && (volt >= GSM_MINIMUM_VOLTAGE)){
      {
        hGSM_on();
        if (hGSMsetup()){
          _D(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_3));
          int HTTPtimeOut = sendLine.length() * 20 + 1000;
          if (HTTPtimeOut < 10000) {HTTPtimeOut = 10000;}
          HTTPrc = hGSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data64.php",
                             "application/x-www-form-urlencoded",
                              sendLine,
                              HTTPtimeOut,
                              HTTPtimeOut);
          if (HTTPrc == 200){
            _D(DebugPrintln("HTTP response: " + hGSMGetLastResponse(), DEBUG_LEVEL_1));
            GSMfailure = !setFenceFromHTTPresponse(hGSMGetLastResponse());
          }
        }else{
          _D(DebugPrintln("GPRS check OK.", DEBUG_LEVEL_1));
        }
        hGSMshutDown();
        hGSM_off();
      }
      cleanUpDataSets(GSMfailure);
    }
    _D(DebugPrintln("GPRS send done: " + String(millis() - msStartTime), DEBUG_LEVEL_2));
    #if DEBUG_LEVEL >= DEBUG_LEVEL_0
    if (GSMfailure){
	    if(volt < GSM_MINIMUM_VOLTAGE){
	      _D(DebugPrintln("GSM: low battery.", DEBUG_LEVEL_1));
	    } else {
        _D(DebugPrintln("GSM result: " + String(HTTPrc), DEBUG_LEVEL_1));
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
{}

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

void initGlobalVar(){
  bootTime.tm_hour    = INVALID_TIME_VALUE;
  bootTime.tm_min     = INVALID_TIME_VALUE;
  bootTime.tm_sec     = INVALID_TIME_VALUE;
  expBootTime.tm_hour = INVALID_TIME_VALUE;
  expBootTime.tm_min  = INVALID_TIME_VALUE;
  expBootTime.tm_sec  = INVALID_TIME_VALUE;
  initRTCData();
  if (bootCount == START_FROM_RESET){
    _D(DebugPrintln("Clear all data sets", DEBUG_LEVEL_1));
    for(int i=0; i<MAX_DATA_SETS; i++){
      initDataSet(availableDataSet[i]);
    }
    _D(DebugPrintln("Clear fence", DEBUG_LEVEL_1));
    void clearFence();
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
  //testGeoFencing();
  //testData();
  #ifdef GSM_MODULE
  //if (volt >= GSM_MINIMUM_VOLTAGE){ hGSMCheckSignalStrength(); }
  #endif
  #ifdef GPS_MODULE
  //testGPS();
  #endif
  #ifdef TEMP_SENSOR
  _D(DebugPrintln("Temperature: " + String(MeasureTemperature()), DEBUG_LEVEL_1));
  #endif
}
void doResetInits(){
  #ifdef GSM_MODULE
  _D(DebugPrintln("Get new fence - send ID", DEBUG_LEVEL_1));
  if (volt >= GSM_MINIMUM_VOLTAGE){
    hGSM_on();
    if (hGSMsetup()){
      int HTTPtimeOut = 10000;
      clearFence();
      for (int i = 0; i<3; i++){
        int HTTPrc = hGSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data64.php",
                                "application/x-www-form-urlencoded",
                                "ID=" + _herdeID(),
                                HTTPtimeOut,
                                HTTPtimeOut);
        _D(DebugPrintln("HTTP result: " + String(HTTPrc), DEBUG_LEVEL_1));
        if (HTTPrc == 200){
          _D(DebugPrintln("HTTP response: " + hGSMGetLastResponse(), DEBUG_LEVEL_2));
          if (setFenceFromHTTPresponse(hGSMGetLastResponse())) { break; }
        } _D( else { DebugPrintln("GSM HTTP error: " + String(HTTPrc), DEBUG_LEVEL_1); } )
      }
      //TODO: SMS alert if fails
      hGSMshutDown();
    }
    hGSM_off();
  }
  #endif
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
