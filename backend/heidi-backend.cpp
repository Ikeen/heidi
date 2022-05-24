#include <Arduino.h>
#include "heidi-backend.h"
#include "heidi-measures.h"
#include "heidi-sys.h"
#ifdef TEST_ACC
#include "heidi-acc.h"
#endif
#include "heidi-defines.h"
#include "heidi-data.h"
#include "heidi-debug.h"
#include "heidi-gsm.h"
#include "heidi-gps.h"
#include "heidi-fence.h"
#include "heidi-error.h"
#include "heidi-flash.h"
#include "heidi-lora.h"

tm bootTime;
//tm sysTime;
tm expBootTime;
esp_timer_handle_t watchd = NULL; //the watchdog
int currentTimeDiffMs  = 0;
static double volt;
bool powerOnReset;
int  timeOut;

HardwareSerial HeidiDBSerial(1);

void setup() {
  t_SendData* currentDataSet;
  int  startMS = millis();
  int  gotTimeStamp = 0;
  timeOut = MAX_AWAKE_TIME_TIMER_MSEC;
  setCpuFrequencyMhz(80); //save power
  //never change the following 3 lines in their position of code!
  powerOnReset = wasPowerOnReset(); //add validation of setup data
  setupDebug(startMS, powerOnReset);
  setupData(powerOnReset); //the first thing we have to do before accessing any config data!!

  if (sysWasbrownOut()) { setState(RESET_INITS); doSleepRTCon(MAX_SLEEP_TIME_MS); }

  setupError();
  //setupFlashData(powerOnReset);
  setupCycleTable();

  _D(debugHeidiState(powerOnReset);)
  if(heidiConfig->bootNumber <= REFETCH_SYS_TIME) { timeOut = MAX_AWAKE_TIME_POWER_MSEC; }
  #ifdef GPS_MODULE
  if(heidiConfig->gpsStatus < GPS_GOT_3D_LOCK) { timeOut = MAX_AWAKE_TIME_POWER_MSEC; _DD(DebugPrintln("extended meas. time.", DEBUG_LEVEL_3);)}
  #endif

  if (isTransmissionCycle() || powerOnReset) {setupWatchDog(timeOut + AWAKE_TM_TRANSMIT_MSEC_OFFS); }
  else { setupWatchDog(timeOut); }

  #ifndef USE_NO_MEASURES
  enableControls();
  /*check battery status and goto sleep, if too low (disables measuring and controls too)*/
  #ifdef USE_VOLT_MEAS_EN_PIN
  /* best is to check battery before enabling measures...*/
  volt = checkBattery();
  enableMeasures();
  #else
  /*... otherwise we need to enable measures before, what may lead into brown-outs */
  enableMeasures();
  volt = checkBattery();
  #endif
  #endif//USE_NO_MEASURES

  #ifdef PRE_MEASURE_HANDLING
  if(!powerOnReset){ handlePreMeasuring(); }
  #endif

  //prepare free data set
  if (!freeDataSet(0)){ _D(DebugPrintln("BOOT: unable to free 1st data set", DEBUG_LEVEL_1);) }
  currentDataSet = availableDataSet[0];
  #ifdef HEIDI_CONFIG_TEST
  #ifndef NO_TESTS
  doTests(currentDataSet);
  #endif
  #endif

  #ifdef ACCELEROMETER
  checkAccData(currentDataSet);
  #endif

  #ifdef USE_LORA
  if(heidiConfig->bootNumber >= FIRST_REGULAR_CYCLE){ setupLoraTask(); }
  #endif

  /*setupSystemDateTime opens / checks GPS*/
  if (!setupSystemBootTime(&bootTime, timeOut)) {
    heidiConfig->bootNumber = REFETCH_SYS_TIME;
    if(GPSalert()){ gotoSleep(SLEEP_DUR_ALERT); } else { gotoSleep(SLEEP_DUR_NOTIME); }
  }
  _D( DebugPrintln("System boot time: " + DateString(&bootTime) + " " + TimeString(&bootTime), DEBUG_LEVEL_1); PRINT_CYCLE_STATUS )
  gotTimeStamp = millis();

  if(!GPSalert()){ checkCycle(); }
  // test alerts: comment in next line
  //_D(clrState(NEW_FENCE); DebugPrintln("!!!!!! NEW FENCE off !!!!!", DEBUG_LEVEL_1);) //!!!!!!!!
  #ifndef USE_NO_POSITION
  checkGPSposition(currentDataSet, timeOut, powerOnReset); //closes GPS
  #endif
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
  #else
  #ifdef HEIDI_GATEWAY
  _D(_PrintNotDoneGSMHandling(DEBUG_LEVEL_1);)
  #endif
  #endif

  finalizeHeidiStatus(powerOnReset);
  #ifdef USE_LORA
  closeLoraTask();
  #endif

  _DD(DebugPrintln("Last diff: " + String(heidiConfig->lastTimeDiffMs) + " / current diff : " + String(currentTimeDiffMs), DEBUG_LEVEL_3);)
  _D(DebugPrintln("SLEEP: heidi state: 0x" + String(heidiConfig->status, HEX) ,DEBUG_LEVEL_1);)
  if (heidiConfig->lastTimeDiffMs > 5000) { heidiConfig->lastTimeDiffMs = 0; }
  int diffTimeFinalMs = heidiConfig->lastTimeDiffMs + currentTimeDiffMs;
  heidiConfig->lastTimeDiffMs = diffTimeFinalMs;

  if((millis() - gotTimeStamp) > timeToNextBootMS()) {
    _DD(DebugPrintln("Goto over next cycle ", DEBUG_LEVEL_3);)
    setToOverNextBoot();
  }
  gotoSleep(timeToNextBootMS() + diffTimeFinalMs - millis());
}

void loop()
{
  /* usually this should never happen */
  heidiConfig->bootNumber = REFETCH_SYS_TIME;
  doSleepRTCon(DEFAULT_CYCLE_DURATION_MSEC);
}


void finalizeDataSet(t_SendData* currentDataSet){
  #ifdef TEMP_SENSOR
  currentDataSet->temperature = (int8_t)round(measureTemperature());
  _D(DebugPrintln("Temperature: " + String(currentDataSet->temperature) + " C", DEBUG_LEVEL_1);)
  #endif
  #ifndef ACCELEROMETER
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
    if (millis() > 254500) { currentDataSet->GPSpDOP = 255; }
    else { currentDataSet->GPSpDOP = (uint8_t)(millis() / 1000); }
    DebugPrintln("set GPSpDOP to acquiring time: " + String(currentDataSet->GPSpDOP) + " s", DEBUG_LEVEL_2);
    #endif
    #ifdef TRACK_HEIDI_STATE
    currentDataSet->accThresCnt2 = heidiConfig->status;
    DebugPrintln("set accThresCnt2 to heidi status: 0x" + String(currentDataSet->accThresCnt2, HEX), DEBUG_LEVEL_2);
    #endif
  )
  currentDataSet->animalID = animalID(); //needs to be the last value set
  _DD(_PrintDataSet(currentDataSet, DEBUG_LEVEL_3);)
}
/*
 *  finalizes the status of Heidi before going to sleep
 *  the boot number is set to the next cycle number, so on boot, before we get the time from GPS
 *  some tasks can be done
 */
void finalizeHeidiStatus(bool powerOnReset){

  if (getState(NEW_SETTINGS)){ //NEW_SETTINGS state is set only when cycle data has changed
    _D( DebugPrintln("calculate new cycle table due to changed cycle data.", DEBUG_LEVEL_1); )
    setupCycleTable();
    if(!isInCycle()){ setState(NOT_IN_CYCLE); }
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
    heidiConfig->bootNumber = prevBootCycleNo();
  }
  heidiConfig->bootNumber++; //number for the next cycle
  if (heidiConfig->bootNumber >= bootTableLength()) { heidiConfig->bootNumber = 0; }

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
    _D(DebugPrintln("Pre-Measure running.", DEBUG_LEVEL_1); pause(50);)
    #ifdef GPS_MODULE
    openGPS();
    #endif
    enableHoldPin(MEASURES_ENABLE_PIN);
    _D(DebugPrintln("Sleep for : " + String(uint32_t((PRE_CYCLE_TIME - millis())/1000)) + " seconds", DEBUG_LEVEL_2); pause(50);)
    doSleepRTCon(PRE_CYCLE_TIME - millis());
  }
  _D(DebugPrintln("Pre-Measure off.", DEBUG_LEVEL_1);)
}
#endif

#ifdef GSM_MODULE
void transmitData(t_SendData* currentDataSet){
  int HTTPrc = 0, setsSent = 0;
  int maxLoops = 5;
  String sendLine = "";
  _D(DebugPrintln("GPRS send data", DEBUG_LEVEL_2); int cnt = 0;)
  _DD(_PrintShortSummary(DEBUG_LEVEL_3));
  #ifdef USE_ULP
  getULPLock(); //GSM antenna activity generates fake measures on ADXL345
  #endif
  if(openGSM()){
    if (GSMsetup()){
      if (GSMopenHTTPconnection(HEIDI_SERVER_PUSH_URL)){
        int buffersize = (MAX_DATA_SETS_PER_SEND_LINE * sizeof(t_SendData));
        t_SendData* buffer = (t_SendData*)malloc(buffersize);
        memset(buffer, 0, buffersize);
        if (buffer != NULL){
          setsSent = MAX_DATA_SETS_PER_SEND_LINE;
          while (setsSent > 0){
            setsSent = getNextnDataSets(MAX_DATA_SETS_PER_SEND_LINE, buffer, buffersize, &buffer[setsSent-1]);
            if(setsSent > 0){
              sendLine = generateMulti64SendLine(buffer, setsSent);
              if(GSMsendLine(sendLine)){
                eraseDataSets(buffer, setsSent);
                _D(cnt += setsSent;)
              } else {
                setErrorToDataSets(buffer, setsSent, E_GSM_TRANSMISSION_FAILED);
              }
            }
            if(maxLoops-- <= 0){ break; }
          }
          free(buffer);
        }
        _D(DebugPrintln("GSM: " + String(cnt) + " data sets sent successfully", DEBUG_LEVEL_1);)
        #ifdef CHANGE_HERDE_ID_TO
        sendLine = "ID=" + intString4(CHANGE_HERDE_ID_TO); //get settings
        #else
        sendLine = "ID=" + _herdeID(); //get settings
        #endif
        if(GSMsendLine(sendLine)){
          String httpResponse = GSMGetLastResponse();
          if (    !setFenceFromHTTPresponse(httpResponse)
               || !setSettingsFromHTTPresponse(httpResponse)
               || !setTelNoFromHTTPresponse(httpResponse)){
            setState(TRSMT_DATA_ERROR);
          }
          clrState(RESET_INITS);
        } _D( else {_D(DebugPrintln("GSM: config data transmission failed", DEBUG_LEVEL_1);)})
        GSMcloseHTTPconnection();
      }
      GSMshutDown();
    }
    closeGSM();
  }
  #ifdef USE_ULP
  freeULP();
  #endif
  packUpDataSets();
  _D(DebugPrintln("GPRS send done: " + (String((int)(millis()/1000)) + " s"), DEBUG_LEVEL_2);)
}
#endif

void checkGPSalert(t_SendData* currentDataSet){
  if (currentDataSet->metersOut > heidiConfig->c.distAlertThres) {
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
    if (GPSGetPosition(currentDataSet, SUFFICIENT_DOP_VALUE, 10, timeOut) > 0){
      checkGPSalert(currentDataSet);
    } else {
      _D(DebugPrintln("GPS: Unable to fetch position.", DEBUG_LEVEL_1);)
    }
    if(getState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 | GPS_ALERT_PSD)) {
      setError(E_GPS_ALERT);
    }
    if(getState( GPS_ALERT_1 | GPS_ALERT_2 ) && !getState(GPS_ALERT_PSD)) {
      setError(E_ALERT_SMS);
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
  if(!isInCycle()){ //calculates expected boot time
    setState(NOT_IN_CYCLE);
  } else {
    clrState(NOT_IN_CYCLE);
    _D(DebugPrintln("Expected boot time: " + TimeString(&expBootTime), DEBUG_LEVEL_1);)
    calcCurrentTimeDiff();
    _D(DebugPrintln("Expected boot time vs. real boot time: " + String(currentTimeDiffMs), DEBUG_LEVEL_1);)
  }
}

double checkBattery(void){
  float val;
  int   _dval;
  /**** check voltage does not need any inits ****/
  _dval = MeasureVoltage(BATTERY_MEASURE_PIN);
  val = CALCULATE_VOLTAGE(_dval);
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
      _D(DebugPrintln("Battery much too low.", DEBUG_LEVEL_1); pause(100);)
      doSleepRTCon(MAX_SLEEP_TIME_MS);
    }
    if (val <= GSM_POWER_SAVE_3_VOLTAGE){ // low Battery?
      _D(DebugPrintln("Battery too low.", DEBUG_LEVEL_1); pause(100);)
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
#ifdef ACCELEROMETER
void checkAccData(t_SendData* currDataSet){
  //need to set this here because getting position needs various time
  currDataSet->accThresCnt1 = get_accel_excnt1_ULP();
  currDataSet->accThCnt1Spr = get_accel_ct_spreading(1);
  currDataSet->accThresCnt2 = get_accel_excnt2_ULP();
  currDataSet->accThCnt2Spr = get_accel_ct_spreading(2);
  init_accel_data_ULP(ULP_INTERVALL_US, _currentCyleLen_m());
  _D(DebugPrintln("accel. threshold 1 count: " + String(currDataSet->accThresCnt1), DEBUG_LEVEL_2);
     DebugPrintln("accel. threshold 2 count: " + String(currDataSet->accThresCnt2), DEBUG_LEVEL_2);)
}
#endif

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
  heidiConfig->bootNumber = REFETCH_SYS_TIME;
  gotoSleep(getCycleTimeMS());
}

bool setupSystemBootTime(tm* bootTime, int timeOut){
  #ifdef GPS_MODULE
  _D(DebugPrintln("Get current time from GPS", DEBUG_LEVEL_1);)
  if (!openGPS()) {
    if (heidiConfig->bootNumber <= REFETCH_SYS_TIME){ return false; }
    return false;
  }
  if (!setBootTimeFromGPSTime(bootTime, timeOut)){
    closeGPS();
    _D(DebugPrintln("Unable to get time from GPS", DEBUG_LEVEL_1);)
    if (heidiConfig->bootNumber <= REFETCH_SYS_TIME){ return false; }
  }
  #else
  tm* sysTime;
  time_t now;
  time(&now);
  sysTime = gmtime(&now);

  setBootTimeFromCurrentTime(sysTime, bootTime);
  #endif
  _D(DebugPrintln("Got Time after: " + String(millis() / 1000) + "s", DEBUG_LEVEL_2);)
  return true;
}

void gotoSleep(int32_t mseconds){
  if (watchd != NULL) { esp_timer_delete(watchd); }

  #ifndef USE_NO_MEASURES
  disableControls(true);
  disableGPIOs();
  #endif
  #ifdef USE_LORA
  closeLoraTask(); //if the cycle was not ended regular, we need to do this here. Doing it twice is no problem.
  #endif
  #ifdef USE_RTC_FAST_MEM
  RTCfastMemWrite();
  #endif

  int32_t sleeptime = mseconds;
  _DD(DebugPrintln("Sleep requested for : " + String(uint32_t(sleeptime/1000)) + " seconds", DEBUG_LEVEL_3);)
  #ifdef PRE_MEASURE_HANDLING
  if (sleeptime < (PRE_CYCLE_TIME + MIN_SLEEP_TIME_MS)){
    clrState(PRE_MEAS_STATE);
    _D(DebugPrintln("Pre-Measure off - time's too short.", DEBUG_LEVEL_1); pause(50);)
  } else {
    setState(PRE_MEAS_STATE);
    _D(DebugPrintln("Pre-Measure on.", DEBUG_LEVEL_1); pause(50);)
    sleeptime -= PRE_CYCLE_TIME;
    disableHoldPin(MEASURES_ENABLE_PIN);
  }
  #endif
  _D(DebugPrintln("Heap water mark: " + String(uxTaskGetStackHighWaterMark(NULL)),  DEBUG_LEVEL_1);
     DebugPrintln("Sleep for: " + String(uint32_t(sleeptime/1000)) + " seconds", DEBUG_LEVEL_1); pause(50);)
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

static void watchDog(void* arg)
{
  _D(DebugPrintln("This is the watchDog - Howdy?! :-D",DEBUG_LEVEL_1); pause(10); )
  heidiConfig->bootNumber = REFETCH_SYS_TIME;
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
//extern uint8_t bootTimeTable[MAX_CYCLES_PER_DAY][3];

void doTests(t_SendData* currentDataSet){
  //disable watchdog
  if (watchd != NULL) {
    esp_timer_stop(watchd);
    esp_timer_delete(watchd);
    watchd = NULL;
  }
  #ifndef HEIDI_GATEWAY
  //if(powerOnReset){
    //prepare data
    loadTestData();
  //}
  #endif
  //testGeoFencing();
  #ifdef TEST_CYCLES
  testCycles(powerOnReset);
  #endif
  #ifdef TEST_DATA
  testData();
  #endif
  #ifdef TEST_RTC
  testRTC(currentDataSet, &bootTime);
  #endif
  #ifdef TEST_GSM
  testGSM();
  #endif
  #ifdef TEST_GPS
  testGPS(powerOnReset ? 180000 : 60000);
  #endif
  #ifdef TEST_TEMP
  testTemp();
  #endif
  #ifdef TEST_VOLT
  testVolt();
  #endif
  #ifdef TEST_ACC
  testAcc(powerOnReset);
  #endif
  #ifdef TEST_LORA
  TestLoRa(90000);
  #endif
  /*
   * now just testing code...
   */
#if 0
  _D(DebugPrintln("setup_size = " + String(sizeof(_t_ConfigData)), DEBUG_LEVEL_1);)
#endif
  //return;
//#ifdef HEIDI_GATEWAY
//  init_accel_ULP(ULP_INTERVALL_US);
//  gotoSleep(120000);
//#else
  gotoSleep(1000);
//#endif
}

#endif
