/*
 * nächste Aufgaben:
 *   - Providermanagement
 *   - Hook beim Reset oder Power-Cut
 *   - einstellbarer Vorzugsprovider
 *   - Neue Provider
 *   - Neue Karten-Daten
 *   - Provider - Scans
 *   -
 *   - Erweitertes Powermanagement (kein Durchbooten bei < 3.6V)
 *   - Dichtere Übertragungsversuche (RTC-Memory? 4*32*50 = 6400 Bytes)
 *
 */
#include "heidi-backend.h"
#include "heidi-measures.h"
#include "heidi-sys.h"
#ifdef TEST_ACC
#include "heidi-acc.h"
#endif

tm bootTime;
//tm sysTime;
tm expBootTime;
esp_timer_handle_t watchd = NULL; //the watchdog
int currentTimeDiffMs  = 0;
static double volt;
bool powerOnReset;

void setup()
{

  t_SendData* currentDataSet;
  int  startMS = millis();
  int  timeOut = MAX_AWAKE_TIME_TIMER;

  setCpuFrequencyMhz(80); //save power

  powerOnReset = wasPowerOnReset();
  if (wasbrownOut()) { setState(RESET_INITS); doSleepRTCon(MAX_CYCLE_DURATION_MSEC); }
  _D(setupDebug(startMS, powerOnReset);)
  setupFlashData(powerOnReset);
  setupError();
  setupData(powerOnReset);
  setupCycleTable();

  _D(debugHeidiState(powerOnReset);)
  if(heidiConfig->bootCount <= REFETCH_SYS_TIME) { timeOut = MAX_AWAKE_TIME_POWER; }
  setupWatchDog(timeOut);

  enableControls();
  /*check battery status and goto sleep, if too low (disables measuring and controls too)*/
  #ifdef USE_VOLTAGE_MEAS_PIN
  /* best is to check battery before enabling measures...*/
  volt = checkBattery();
  enableMeasures();
  #else
  /*... otherwise we need to enable measures before, what may lead into brown-outs */
  enableMeasures();
  volt = checkBattery();
  #endif

  #ifdef PRE_MEASURE_HANDLING
  if(!powerOnReset){ handlePreMeasuring(); }
  #endif

  //prepare free data set
  freeFirstDataSet();
  currentDataSet = availableDataSet[0];

  #ifdef USE_LORA
  SetupLoRa();
  #endif

#ifdef HEIDI_CONFIG_TEST
  doTests(currentDataSet);
#endif


  /*setupSystemDateTime opens / checks GPS*/
  if (!setupSystemBootTime(&bootTime, timeOut)) {
    heidiConfig->bootCount = REFETCH_SYS_TIME;
    if(GPSalert()){ gotoSleep(SLEEP_DUR_ALERT); } else { gotoSleep(SLEEP_DUR_NOTIME); }
  }
  _D(DebugPrintln("System boot time: " + DateString(&bootTime) + " " + TimeString(&bootTime), DEBUG_LEVEL_1);)

  _D(
  PRINT_CYCLE_STATUS
  #ifdef TEST_RTC
  testRTC(currentDataSet, &bootTime);
  #endif
  )

  if(!GPSalert()){ checkCycle(); }

  // test alerts: comment in next line
  //_D(clrState(NEW_FENCE); DebugPrintln("!!!!!! NEW FENCE off !!!!!", DEBUG_LEVEL_1);) //!!!!!!!!

  checkGPSposition(currentDataSet, timeOut, powerOnReset); //closes GPS
  finalizeDataSet(currentDataSet); //does last measurements
  disableMeasures();

#ifdef GSM_MODULE
  _D(PRINT_ALERT_STATUS)
  if (getState(TRSMT_DATA_ERROR)){ //there was an error on last received data
    currentDataSet->errCode |= E_RECEIVE_DATA_ERROR;
    clrState(TRSMT_DATA_ERROR);
  }
  if (doDataTransmission()){
    transmitData(currentDataSet);
  } else {
    currentDataSet->errCode |= getErrorCode(); //error codes from doDataTransmission !!!!!!!!!!!!!!!!!!
    _D( if(getState(POWER_SAVE_2)){ DebugPrintln("GSM: low battery.", DEBUG_LEVEL_1); })
  }
#endif

//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
#ifdef USE_LORA
  LoRa.end();
  LoRa.sleep();
  delay(100);
#endif

  finalizeHeidiStatus(powerOnReset);

  _DD(DebugPrintln("Last diff: " + String(heidiConfig->lastTimeDiffMs) + " / current diff : " + String(currentTimeDiffMs), DEBUG_LEVEL_3);)
  _D(DebugPrintln("SLEEP: heidi state: 0x" + String(heidiConfig->status, HEX) ,DEBUG_LEVEL_1);)
  int diffTimeFinalMs = heidiConfig->lastTimeDiffMs + currentTimeDiffMs;
  heidiConfig->lastTimeDiffMs = diffTimeFinalMs;
  gotoSleep(timeToNextBootMS() + diffTimeFinalMs - millis());
}

void loop()
{
  /* usually this should never happen */
  heidiConfig->bootCount = REFETCH_SYS_TIME;
  doSleepRTCon(DEFAULT_CYCLE_DURATION_MSEC);
}


void finalizeDataSet(t_SendData* currentDataSet){
  #ifdef TEMP_SENSOR
  currentDataSet->temperature = (int16_t)(measureTemperature()*100.0);
  _D(DebugPrintln("Temperature: " + String(((float)currentDataSet->temperature / 100)) + "C", DEBUG_LEVEL_1);)
  #endif
  #ifdef ACCELEROMETER
  currentDataSet->accThresCnt1 = get_accel_excnt1_ULP();
  set_accel_excnt1_ULP(0);
  currentDataSet->accThresCnt2 = get_accel_excnt2_ULP();
  set_accel_excnt2_ULP(0);
  _D(DebugPrintln("accel threshold 1 count: " + String(currentDataSet->accThresCnt1), DEBUG_LEVEL_2);
     DebugPrintln("accel threshold 2 count: " + String(currentDataSet->accThresCnt2), DEBUG_LEVEL_2);)
  #else
    #ifdef TRACK_HEIDI_STATE
    _D(currentDataSet->accThres1 = heidiConfig->status;
       DebugPrintln("set accThres1 to heidi status: 0x" + String(currentDataSet->accThres1, HEX), DEBUG_LEVEL_2);)
    #endif
  #endif
  currentDataSet->date     = dosDate(bootTime.tm_year, bootTime.tm_mon+1, bootTime.tm_mday);
  currentDataSet->time     = dosTime(bootTime.tm_hour, bootTime.tm_min, bootTime.tm_sec);
  currentDataSet->battery  = (uint16_t)(round(volt * 1000));
  currentDataSet->errCode |= getErrorCode();
  _D(
    #ifdef MEAS_ACQUIRNG_TIME
    if (millis() > 254500) { currentDataSet->GPShdop = 255; }
    else { currentDataSet->GPShdop = (uint8_t)(millis() / 1000); }
    DebugPrintln("set GPShdop to acquiring time: " + String(currentDataSet->GPShdop) + " ms", DEBUG_LEVEL_2);
    #endif
    #ifdef TRACK_HEIDI_STATE
    currentDataSet->accThresCnt2 = heidiConfig->status;
    DebugPrintln("set accThresCnt2 to heidi status: 0x" + String(currentDataSet->accThresCnt2, HEX), DEBUG_LEVEL_2);
    #endif
    _DD(_PrintDataSet(currentDataSet, DEBUG_LEVEL_3);)
  )
}

void finalizeHeidiStatus(bool powerOnReset){

  if (getState(NEW_SETTINGS)){ //NEW_SETTINGS state is set only when cycle data has changed
    _D( DebugPrintln("calculate new cycle table due to changed cycle data.", DEBUG_LEVEL_1); )
    setupCycleTable();
    if(!isInCycle(&(heidiConfig->bootCount))){ setState(NOT_IN_CYCLE); }
    clrState(NEW_SETTINGS);
  }
  #ifdef ACCELEROMETER
  if (getState(FROM_PWR_SAVE_SLEEP) || powerOnReset){
    wake_config_ADXL345();
  }
  #endif
  #ifdef USE_ULP
  if (getState(NEW_ACC_DATA | FROM_PWR_SAVE_SLEEP) || powerOnReset || (!getState(ULP_RUNNING))){
    disableULP();
    init_accel_ULP(ULP_INTERVALL_US);
    clrState(NEW_ACC_DATA);
  }
  #endif

  if (GPSalert() && getState(NOT_IN_CYCLE)){
    heidiConfig->bootCount = prevBootCycleNo();
  }
  heidiConfig->bootCount++;
  if (heidiConfig->bootCount >= _currentCycles()) { heidiConfig->bootCount = 0; }

  if (getState(NOT_IN_CYCLE)){
    heidiConfig->lastTimeDiffMs = 0;       //not useful out of regular cycles
    currentTimeDiffMs = 0;
    clrState(NOT_IN_CYCLE);
  }

}

#ifdef PRE_MEASURE_HANDLING
void handlePreMeasuring(void){
  if(getState(PRE_MEAS_STATE)){
    clrState(PRE_MEAS_STATE);
    _D(DebugPrintln("Pre-Measure running.", DEBUG_LEVEL_1); delay(50);)
    #ifdef GPS_MODULE
    openGPS();
    #endif
    enableHoldPin(MEASURES_ENABLE_PIN);
    _D(DebugPrintln("Sleep for : " + String(uint32_t((PRE_CYCLE_TIME - millis())/1000)) + " seconds", DEBUG_LEVEL_2); delay(50);)
    doSleepRTCon(PRE_CYCLE_TIME - millis());
  }
  _D(DebugPrintln("Pre-Measure off.", DEBUG_LEVEL_1);)
}
#endif
#ifdef GSM_MODULE
void transmitData(t_SendData* currentDataSet){
  int HTTPrc = 0;
  bool GSMfailure = true;
  currentDataSet->errCode |= getErrorCode(); //error codes from doDataTransmission !!!!!!!!!!!!!!!!!!
  _DD(_PrintShortSummary(DEBUG_LEVEL_3));
  String sendLine = "";
  sendLine = generateMulti64SendLine(0, (MAX_DATA_SETS-1));
  if ((sendLine.length() == 0)){ sendLine = "ID=" + _herdeID(); } //just get settings
  if(openGSM()){
    if (GSMsetup()){
      _DD(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_3);)
      int HTTPtimeOut = sendLine.length() * 20 + 1000;
      if (HTTPtimeOut < 10000) {HTTPtimeOut = 10000;}
      HTTPrc = GSMdoPost(HEIDI_SERVER_PUSH_URL,
                          "application/x-www-form-urlencoded",
                           sendLine,
                           HTTPtimeOut,
                           HTTPtimeOut);
      if (HTTPrc == 200){
        _D(DebugPrintln("HTTP send Line OK.", DEBUG_LEVEL_1);)
        _D(DebugPrintln("HTTP response: " + GSMGetLastResponse(), DEBUG_LEVEL_2);)
        GSMfailure = false;
        if (    !setFenceFromHTTPresponse(GSMGetLastResponse())
             || !setSettingsFromHTTPresponse(GSMGetLastResponse())
             || !setTelNoFromHTTPresponse(GSMGetLastResponse())){
          setState(TRSMT_DATA_ERROR);
        }
        clrState(RESET_INITS);
      }
    }
    GSMshutDown();
    closeGSM();
  }
  cleanUpDataSets(GSMfailure);
  _D(
    DebugPrintln("GPRS send done: " + String(millis()), DEBUG_LEVEL_2);
    if (GSMfailure){ DebugPrintln("GSM result: " + String(HTTPrc), DEBUG_LEVEL_1); }
  )
}
#endif

void checkGPSalert(t_SendData* currentDataSet){
  if (currentDataSet->metersOut > heidiConfig->distAlertThres) {
    if (!getState(NEW_FENCE)){
      if (getState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 ) && getState(GPS_ALERT_PSD)) { clrState(GPS_ALERT_PSD); } //stop pausing alerts
      if (getState(PRE_GPS_ALERT)){ clrState(PRE_GPS_ALERT); setState(GPS_ALERT_1); _D(DebugPrintln("GPS ALERT: 1", DEBUG_LEVEL_2);)}
      if (!getState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 | GPS_ALERT_PSD)){ setState(PRE_GPS_ALERT); _D(DebugPrintln("GPS ALERT: pre", DEBUG_LEVEL_2);)}
    } else { currentDataSet->errCode |= E_NEW_FENCE_STATE; _D(DebugPrintln("GPS ALERT: distance too big, but new fence.", DEBUG_LEVEL_2);)}
  } else { if (!getState(NEW_FENCE) && GPSalert()){ setState(GPS_ALERT_PSD); } } //pause alerting

  if (currentDataSet->metersOut <= 5){ //disable alerting
    _D(
       if (GPSalert()) { DebugPrintln("GPS ALERT off", DEBUG_LEVEL_2); }
       if (getState(NEW_FENCE)) { DebugPrintln("new fence active", DEBUG_LEVEL_2); };
    )
    clrState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 | GPS_ALERT_PSD | NEW_FENCE);
  }
}

void checkGPSposition(t_SendData* currentDataSet, int timeOut, bool force){
  if (!getState(NOT_IN_CYCLE) || GPSalert() || (timeOut == 0) || force){
    #ifdef GPS_MODULE
    if (GPSGetPosition(currentDataSet, 10, timeOut) > 0){
      checkGPSalert(currentDataSet);
    } else {
      _D(DebugPrintln("GPS: Unable to fetch position.", DEBUG_LEVEL_1);)
    }
    if(getState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 | GPS_ALERT_PSD)) {
      setError(E_GPS_ALERT);
    }
    if(getState( GPS_ALERT_1 | GPS_ALERT_2 ) && !getState(GPS_ALERT_PSD)) {
      setError(E_GPS_ALERT_SMS);
    }
    #else
    clrState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 | GPS_ALERT_PSD | NEW_FENCE);
    setError(E_COULD_NOT_FETCH_GPS);
    setError(E_COULD_NOT_FETCH_GPS_TIME);
    #endif
  }
  #ifdef GPS_MODULE
  closeGPS();
  #endif
}

void checkCycle(void){
  //find expected boot time
  if(!isInCycle(&(heidiConfig->bootCount))){ //calculates expected boot time
    setState(NOT_IN_CYCLE);
  } else {
    clrState(NOT_IN_CYCLE);
    _D(DebugPrintln("Expected boot time: " + TimeString(&expBootTime), DEBUG_LEVEL_1);)
    calcCurrentTimeDiff();
    _D(DebugPrintln("Expected boot time vs. real boot time: " + String(currentTimeDiffMs), DEBUG_LEVEL_1);)
  }
}

double checkBattery(void){
  double val;
  int   _dval;
  /**** check voltage does not need any inits ****/
  _dval = MeasureVoltage(BATTERY_MEASURE_PIN);
  val = (double)(_dval + ANALOG_MEASURE_OFFSET) / ANALOG_MEASURE_DIVIDER;
  _D(DebugPrintln("Battery: " + String(val, 2) + "V [" + String(_dval) + "]", DEBUG_LEVEL_1);)
#ifdef CHECK_BATTERY
  if (val <= GSM_POWER_SAVE_3_VOLTAGE){
    setState(FROM_PWR_SAVE_SLEEP);
    #ifdef ACCELEROMETER
    sleep_ADXL345();
    #endif
    #ifdef USE_ULP
    disableULP();
    #endif
    disableControls(true);
     if (val <= GSM_POWER_SAVE_4_VOLTAGE){ // low Battery?
      _D(DebugPrintln("Battery much too low.", DEBUG_LEVEL_1); delay(100);)
      doSleepRTCon(MAX_CYCLE_DURATION_MSEC);
    }
    if (val <= GSM_POWER_SAVE_3_VOLTAGE){ // low Battery?
      _D(DebugPrintln("Battery too low.", DEBUG_LEVEL_1); delay(100);)
      int32_t sleeptime = getCycleTimeMS();
      doSleepRTCon(sleeptime - millis());
    }
  }
  if (val <= GSM_POWER_SAVE_2_VOLTAGE){
    setState(POWER_SAVE_2);
    _DD(DebugPrintln("Battery: POWER_SAVE_2", DEBUG_LEVEL_3);)
  } else if (val <= GSM_POWER_SAVE_1_VOLTAGE){
    setState(POWER_SAVE_1);
    clrState(POWER_SAVE_2);
    setError(E_POWER_SAVE_1);
    _DD(DebugPrintln("Battery: POWER_SAVE_1", DEBUG_LEVEL_3);)
  }  else {
    clrState(POWER_SAVE_1);
    clrState(POWER_SAVE_2);
    _DD(DebugPrintln("Battery: no POWER_SAVE", DEBUG_LEVEL_3);)
  }
  #else
  val = 4.0;
  clrState(POWER_SAVE_1);
  clrState(POWER_SAVE_2);
  #endif
  return val;
}

#ifdef USE_LORA
void SetupLoRa(){
  _D(DebugPrint("LoRa init .. ", DEBUG_LEVEL_1);)
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
  _D(DebugPrintln("done", DEBUG_LEVEL_1);)
  //LoRa.setTxPower(17);
  //LoRa.receive();
}
#endif //USE_LORA

void setupData(bool powerOnReset){
  initBootTimeMS();
  bootTime.tm_hour    = INVALID_TIME_VALUE;
  bootTime.tm_min     = INVALID_TIME_VALUE;
  bootTime.tm_sec     = INVALID_TIME_VALUE;
  expBootTime.tm_hour = INVALID_TIME_VALUE;
  expBootTime.tm_min  = INVALID_TIME_VALUE;
  expBootTime.tm_sec  = INVALID_TIME_VALUE;
  initRTCData(powerOnReset);
  if(!wasRegularWakeUp()){ setError(E_WRONG_BOOT_REASON); }
}

void restartCycling(){
  heidiConfig->bootCount = REFETCH_SYS_TIME;
  gotoSleep(getCycleTimeMS());
}

bool setupSystemBootTime(tm* bootTime, int timeOut){
  #ifdef GPS_MODULE
  _D(DebugPrintln("Get current time from GPS", DEBUG_LEVEL_1);)
  if (!openGPS()) {
    if (heidiConfig->bootCount <= REFETCH_SYS_TIME){ return false; }
    return false;
  }
  if (!setBootTimeFromGPSTime(bootTime, timeOut)){
    closeGPS();
    _D(DebugPrintln("Unable to get time from GPS", DEBUG_LEVEL_1);)
    if (heidiConfig->bootCount <= REFETCH_SYS_TIME){ return false; }
  }
  #else
  tm* sysTime;
  time_t now;
  time(&now);
  sysTime = gmtime(&now);

  DebugPrint("-->" + DateString(sysTime), DEBUG_LEVEL_1);
  DebugPrintln(" " + TimeString(sysTime), DEBUG_LEVEL_1);

  setBootTimeFromCurrentTime(sysTime, bootTime);
  #endif
  _D(DebugPrintln("Got Time after: " + String(millis() / 1000) + "s", DEBUG_LEVEL_2);)
  return true;
}

void gotoSleep(int32_t mseconds){
  if (watchd != NULL) { esp_timer_delete(watchd); }
  disableControls(true);
  disableGPIOs();
  #ifdef USE_RTC_FAST_MEM
  RTCfastMemWrite();
  #endif

  int32_t sleeptime = mseconds;
  _DD(DebugPrintln("Sleep requested for : " + String(uint32_t(sleeptime/1000)) + " seconds", DEBUG_LEVEL_3); delay(50);)
  if (sleeptime > MAX_CYCLE_DURATION_MSEC) { sleeptime = MAX_CYCLE_DURATION_MSEC; }
  #ifdef PRE_MEASURE_HANDLING
  if (sleeptime < (PRE_CYCLE_TIME + MIN_SLEEP_TIME_MS)){
    clrState(PRE_MEAS_STATE);
    _D(DebugPrintln("Pre-Measure off - time's too short.", DEBUG_LEVEL_1); delay(50);)
  } else {
    setState(PRE_MEAS_STATE);
    _D(DebugPrintln("Pre-Measure on.", DEBUG_LEVEL_1); delay(50);)
    sleeptime -= PRE_CYCLE_TIME;
    disableHoldPin(MEASURES_ENABLE_PIN);
  }
  #endif
  _D(DebugPrintln("Sleep for : " + String(uint32_t(sleeptime/1000)) + " seconds", DEBUG_LEVEL_2); delay(50);)
  doSleepRTCon(sleeptime);
}
void IRAM_ATTR doSleepRTCon(int32_t ms){
  int32_t mSeconds = ms;
  if (mSeconds < MIN_SLEEP_TIME_MS) { mSeconds = MIN_SLEEP_TIME_MS; }
  if (mSeconds > MAX_SLEEP_TIME_MS) { mSeconds = MAX_SLEEP_TIME_MS; }
  esp_sleep_enable_timer_wakeup(uint64_t(mSeconds) * uint64_t(uS_TO_mS_FACTOR));
  #ifdef USE_ULP
  esp_sleep_enable_ulp_wakeup();
  #endif
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);  //ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_deep_sleep_start();
}

bool wasPowerOnReset(void){
  _DD(DebugPrintln("RESET reason " + String(rtc_get_reset_reason(0)), DEBUG_LEVEL_3);)
  return rtc_get_reset_reason(0) == POWERON_RESET;
}
bool wasbrownOut(void){
  return (rtc_get_reset_reason(0) == RTCWDT_BROWN_OUT_RESET) || (rtc_get_reset_reason(0) == SW_CPU_RESET) ;
}

bool wasRegularWakeUp(void){
  bool rst = rtc_get_reset_reason(0) == DEEPSLEEP_RESET;
  bool tmr = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
  bool ulp = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP;
  return (rst & (tmr | ulp));
}
bool wasULPWakeUp(void){
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP;
}
static void watchDog(void* arg)
{
	_D(DebugPrintln("This is the watchDog - Howdy?! :-D",DEBUG_LEVEL_1); delay(10); )
	heidiConfig->bootCount = REFETCH_SYS_TIME;
  gotoSleep(10000);
}

void setupWatchDog(uint32_t timeOutMs){
  if (timeOutMs > 0){
    const esp_timer_create_args_t timargs = {
            .callback = &watchDog
    //        .name = "takecare"
    };
    esp_timer_create(&timargs, &watchd);
    esp_timer_start_once(watchd, (timeOutMs + 60000) * 1000); //cut 1 minute after timeout
  }
}


#ifdef HEIDI_CONFIG_TEST
void doTests(t_SendData* currentDataSet){
  _D(DebugPrintln("Boot: not in cycle.", DEBUG_LEVEL_1);)
  //testGeoFencing();
  //testData();
  #ifdef GSM_MODULE
  //if (volt >= GSM_MINIMUM_VOLTAGE){ GSMCheckSignalStrength(); }
  #endif
  #ifdef GPS_MODULE
  //testGPS();
  #endif
  #ifdef TEMP_SENSOR
  //_D(DebugPrintln("Temperature: " + String(MeasureTemperature()), DEBUG_LEVEL_1);)
  #endif
  #ifdef TEST_ACC
  TEST_ACC_MACRO
  #endif

  /*
   * now just testing code...
   */

      _D(DebugPrintln("Alles krass neu!!!!! " + String (millis()), DEBUG_LEVEL_1);)
      #ifdef ACCELEROMETER
      if(powerOnReset){init_accel_ULP(ULP_INTERVALL_US);} else {
        _D(DebugPrintln("accel threshold 1 count: " + String(get_accel_excnt1_ULP()), DEBUG_LEVEL_2);
           DebugPrintln("accel threshold 2 count: " + String(get_accel_excnt2_ULP()), DEBUG_LEVEL_2);)
        set_accel_excnt1_ULP(0);
        set_accel_excnt2_ULP(0);
      }
      #endif
      //if (setupFastMemAccess()) {
        RTCfastMemRead();
        for (int i=1; i<RTC_FAST_MEM_SIZE_32-1; i++){ fastMemBuffer[i] = 0; }
        fastMemBuffer[0]++;
        fastMemBuffer[RTC_FAST_MEM_SIZE_32-1]++;
        RTCfastMemWrite();
      //}

      openGPS();
      GPSGetPosition(currentDataSet, 10000, 3600000);
      SPI.begin(SCK,MISO,MOSI,SS);
      LoRa.setPins(SS,RST,DI0);
      LoRa.begin(BAND);
      LoRa.end();
      LoRa.sleep();
      _D(DebugPrintln("CPU Freq: " + String(getCpuFrequencyMhz()) + " Core: " + String(xPortGetCoreID()), DEBUG_LEVEL_1);)
      delay(500);
      disableMeasures();
      disableControls(true);
      gotoSleep(10000);
}

#endif
