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
#include <driver/rtc_io.h>
#include "heidi-measures.h"
#include "heidi-sys.h"
#ifdef TEST_ACC
#include "heidi-acc.h"
#endif
/*
#include "SIM800L.h"
#define SIM800_RST_PIN GSM_RST
HardwareSerial SerialSIM800L(GPS_UART_NO);
SIM800L* sim800l;
*/

tm bootTime;
tm sysTime;
tm expBootTime;
esp_timer_handle_t watchd = NULL; //the watchdog
int currentTimeDiffMs  = 0;
static double volt;


void setup()
{
  t_SendData* currentDataSet;
  int  startMS = millis();
  int  timeOut = MAX_AWAKE_TIME_TIMER;
  _D(Serial.begin(115200); checkWakeUpReason(); )
  bool powerOnReset = wasPowerOnReset();
  _D(if(powerOnReset){ delay(3000); }) //enable COM terminal
  _D(DebugPrintln("Enter Main Loop. " + String(startMS), DEBUG_LEVEL_1); delay(50);)

  #ifdef USE_ULP
  stopULP(); //anyway
  #endif

  initError();
  initGlobalVar(powerOnReset);

  /* enable controls before brown-out check because disabling them also disables GPS- and GSM-power */
  enableControls(); // !!!!! may fail
  #ifdef USE_ULP
  sleep_ADXL345(); //keep silence during measuring, or sleep is was brown-out or we have low battery
  #endif
  /* this is for testing purposes - comment it out... */
  //_D(if (powerOnReset) { volt = 4.0; getSystemSettings(); }) //!!!!!

  if (wasbrownOut()) {
    _D(DebugPrintln("BOOT: was brown-out reset",DEBUG_LEVEL_1); delay(100);)
    /* usually this should not happen - if so, we need to avoid endless booting */
    disableControls(true);
    setState(RESET_INITS);
    doSleepRTCon(900000);
  }
  /*check battery status and goto sleep, if too low (disables measuring and controls too)*/
  #ifndef USE_VOLTAGE_MEAS_PIN
  /*if we need to enable measures for voltage measuring we do it first ...*/
  openMeasures();
  volt = CheckBattery();
  #else
  volt = CheckBattery();
  {
  int t = millis();
  /*... otherwise we do it afterwards, what saves power in case of low battery */
  openMeasures();
  _D(DebugPrintln("Open Measures takes " + String(millis() - t)+ "ms", DEBUG_LEVEL_3); )
  }
  #endif

  #ifdef PRE_MEASURE_HANDLING
  initConfig(false);
  if(!powerOnReset){
    if(getState(PRE_MEAS_STATE)){
      clrState(PRE_MEAS_STATE);
      _D(DebugPrintln("Pre-Measure running.", DEBUG_LEVEL_1); delay(50);)
      #ifdef USE_ULP
      enableULP();
      esp_sleep_enable_ulp_wakeup();
      #endif
      doSleepRTCon(PRE_CYCLE_TIME - millis());
    }
  }
  _D(DebugPrintln("Pre-Measure off.", DEBUG_LEVEL_1);)
  #endif

  calcCycleTable();
  setupWatchDog();

  _D(
    if(powerOnReset) { DebugPrintln("BOOT: was power on reset", DEBUG_LEVEL_1); }
    else { DebugPrintln("BOOT: cycle number: " + String(heidiConfig->bootCount), DEBUG_LEVEL_1); }
    DebugPrintln("BOOT: Heidi state: 0x" + String(heidiConfig->status, HEX) ,DEBUG_LEVEL_2);
  )

#ifdef USE_ULP
#ifdef TEST_ACC
  _D(DebugPrintln("accel measurement count: " + String(get_accel_meas_cnt_ULP()), DEBUG_LEVEL_2);
     DebugPrintln("transmission result x: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_2);
     DebugPrintln("transmission result y: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_2);
     DebugPrintln("transmission result z: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_2);
     set_accel_excnt1_ULP(0);
     set_accel_excnt2_ULP(0);
     delay(10000);
     goto_sleep(30000);
    )
#endif
#endif
  //prepare free data set
  freeFirstDataSet();
  currentDataSet = availableDataSet[0];

#ifdef USE_LORA
  SetupLoRa();
#endif

#ifdef TEMP_SENSOR
  currentDataSet->temperature = (int16_t)(MeasureTemperature()*100);
  _D(DebugPrintln("Temperature: " + String(((float)currentDataSet->temperature / 100)) + "C", DEBUG_LEVEL_1);)
#endif
#ifdef USE_ULP
  currentDataSet->accThres1 = get_accel_excnt1_ULP();
  currentDataSet->accThres2 = get_accel_excnt2_ULP();
  _D(DebugPrintln("accel threshold 1 count: " + String(currentDataSet->accThres1), DEBUG_LEVEL_2);)
  _D(DebugPrintln("accel threshold 2 count: " + String(currentDataSet->accThres2), DEBUG_LEVEL_2);)
#else
#ifdef TRACK_HEIDI_STATE
  _D(
    currentDataSet->accThres1 = heidiConfig->status;
    DebugPrintln("set accThres1 to heidi status: 0x" + String(currentDataSet->accThres1, HEX), DEBUG_LEVEL_2);
   )
#endif
#endif

  if(heidiConfig->bootCount <= REFETCH_SYS_TIME) { timeOut = MAX_AWAKE_TIME_POWER; }
  /*setupSystemDateTime opens / checks GPS*/
  if (!setupSystemDateTime(&sysTime, timeOut)) { heidiConfig->bootCount = REFETCH_SYS_TIME; goto_sleep(SLEEP_DUR_NOTIME); }
  _D(DebugPrintln("Current system time: " + DateString(&sysTime) + " " + TimeString(&sysTime), DEBUG_LEVEL_1);)
  setBootTimeFromCurrentTime(&sysTime, &bootTime);

  _D(DebugPrintln("Boot time:   " + DateString(&bootTime) + " " + TimeString(&bootTime), DEBUG_LEVEL_1);)
  _DD(
    DebugPrintln("bootCycles day: " + String(heidiConfig->bootCycles) + ", night: " + String(heidiConfig->nightBootCycles), DEBUG_LEVEL_3);
    DebugPrintln("_currentCycles: " + String(__currentCyclesD(_night())) + "( == " + String(_currentCycles()) + ")", DEBUG_LEVEL_3);
    DebugPrintln("night from: " + String(heidiConfig->nightHourStart) + " to: " + String(heidiConfig->nightHourEnd), DEBUG_LEVEL_3);
    if (_night()) {DebugPrintln("_night = true", DEBUG_LEVEL_2);} else {DebugPrintln("_night = false", DEBUG_LEVEL_3);}
    if (getState(POWER_SAVE_1)) {DebugPrintln("POWER_SAVE_1 = true", DEBUG_LEVEL_2);} else {DebugPrintln("POWER_SAVE_1 = false", DEBUG_LEVEL_3);}
  )

  //_D(if(_night()) { setState(GOOD_NIGHT); } )

#ifdef TEST_RTC
  _D(testRTC(currentDataSet, &bootTime);)
#endif

  if (!GPSalert()){
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
#ifdef GPS_MODULE
  //test alerts
  //_D(clrState(NEW_FENCE); DebugPrintln("!!!!!! NEW FENCE off !!!!!", DEBUG_LEVEL_1);) //!!!!!!!!
  if (!getState(NOT_IN_CYCLE) || GPSalert()){
    if (GPSGetPosition(currentDataSet, 10, timeOut) == 0){
      _D(DebugPrintln("GPS: Unable to fetch position.", DEBUG_LEVEL_1);)
    } else {
      if (currentDataSet->metersOut > heidiConfig->distAlertThres) {
        if (!getState(NEW_FENCE)){
          if (GPSalert() && getState(GPS_ALERT_PSD)) { clrState(GPS_ALERT_PSD); } //stop pausing alerts
          if (getState(GPS_ALERT_2)){ clrState(GPS_ALERT_2); setState(GPS_ALERT_PSD); _D(DebugPrintln("GPS ALERT: silent", DEBUG_LEVEL_2);)}
          else if (getState(GPS_ALERT_1)){ clrState(GPS_ALERT_1); setState(GPS_ALERT_2); _D(DebugPrintln("GPS ALERT: 2", DEBUG_LEVEL_2);)}
          else if (getState(PRE_GPS_ALERT)){ clrState(PRE_GPS_ALERT); setState(GPS_ALERT_1); _D(DebugPrintln("GPS ALERT: 1", DEBUG_LEVEL_2);)}
          else if (!getState(GPS_ALERT_PSD)){ setState(PRE_GPS_ALERT); _D(DebugPrintln("GPS ALERT: pre", DEBUG_LEVEL_2);)}
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
    if(getState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 | GPS_ALERT_PSD)) {
      currentDataSet->errCode |= E_GPS_ALERT;
    };
    if(getState( GPS_ALERT_1 | GPS_ALERT_2 ) && !getState(GPS_ALERT_PSD)) {
      currentDataSet->errCode |= E_GPS_ALERT_SMS;
    };
#else
    currentDataSet->errCode |= COULD_NOT_FETCH_GPS;
    currentDataSet->errCode |= COULD_NOT_FETCH_GPS_TIME;
#endif
    //finalize data set
    currentDataSet->date     = dosDate(bootTime.tm_year, bootTime.tm_mon, bootTime.tm_mday);
    currentDataSet->time     = dosTime(bootTime.tm_hour, bootTime.tm_min, bootTime.tm_sec);
    currentDataSet->battery  = (uint16_t)(round(volt * 1000));
    currentDataSet->errCode |= getErrorCode(); //add system errors
    _D(
      #ifdef MEAS_ACQUIRNG_TIME
      currentDataSet->GPShdop = (uint8_t)(millis() / 1000);
      DebugPrintln("set GPShdop to acquiring time: " + String(currentDataSet->GPShdop) + " ms", DEBUG_LEVEL_2);
      #endif
      #ifdef TRACK_HEIDI_STATE
      currentDataSet->accThres2 = heidiConfig->status;
      DebugPrintln("set accThres2 to heidi status: 0x" + String(currentDataSet->accThres2, HEX), DEBUG_LEVEL_2);
      #endif
    )
    _DD(_PrintDataSet(currentDataSet, DEBUG_LEVEL_3);)

  } else {
    initDataSet(currentDataSet);
  }
#ifdef GPS_MODULE
  closeGPS();
#endif
#ifdef USE_ULP
  wake_config_ADXL345();
#endif
  closeMeasures();

#ifdef GSM_MODULE
  _DD(
    DebugPrintln("heidiConfig->bootCount: " + String(heidiConfig->bootCount), DEBUG_LEVEL_3);
    if (GPSalert()) {DebugPrintln("GPSalert = true", DEBUG_LEVEL_3);} else {DebugPrintln("GPSalert = false", DEBUG_LEVEL_3);}
    if (doDataTransmission()) {DebugPrintln("doDataTransmission = true", DEBUG_LEVEL_3);} else {DebugPrintln("doDataTransmission = false", DEBUG_LEVEL_3);}
    if (getState(PRE_GPS_ALERT)) {DebugPrintln("PRE_GPS_ALERT = true", DEBUG_LEVEL_3);} else {DebugPrintln("PRE_GPS_ALERT = false", DEBUG_LEVEL_3);}
    if (getState(GPS_ALERT_1)) {DebugPrintln("GPS_ALERT_1 = true", DEBUG_LEVEL_3);} else {DebugPrintln("GPS_ALERT_1 = false", DEBUG_LEVEL_3);}
    if (getState(GPS_ALERT_2)) {DebugPrintln("GPS_ALERT_2 = true", DEBUG_LEVEL_3);} else {DebugPrintln("GPS_ALERT_2 = false", DEBUG_LEVEL_3);}
    if (getState(POWER_SAVE_2)) {DebugPrintln("POWER_SAVE_2 = true", DEBUG_LEVEL_3);} else {DebugPrintln("POWER_SAVE_2 = false", DEBUG_LEVEL_3);}
  )
  if (doDataTransmission()){
	  int HTTPrc = 0;
	  bool GSMfailure = true;
	  _DD(_PrintShortSummary(DEBUG_LEVEL_3));
    String sendLine = "";
    if ((heidiConfig->bootCount >= _currentCycles()) || GPSalert()) { sendLine = generateMulti64SendLine(0, (MAX_DATA_SETS-1)); }
    if ((sendLine.length() == 0)){ sendLine = "ID=" + _herdeID(); } //just get settings
    if(openGSM()){
      if (hGSMsetup()){
        _DD(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_3);)
        int HTTPtimeOut = sendLine.length() * 20 + 1000;
        if (HTTPtimeOut < 10000) {HTTPtimeOut = 10000;}
        HTTPrc = hGSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data64.php",
                            "application/x-www-form-urlencoded",
                             sendLine,
                             HTTPtimeOut,
                             HTTPtimeOut);
        if (HTTPrc == 200){
          _D(DebugPrintln("HTTP send Line OK.", DEBUG_LEVEL_1);)
          _D(DebugPrintln("HTTP response: " + hGSMGetLastResponse(), DEBUG_LEVEL_2);)
          GSMfailure = false;
          setFenceFromHTTPresponse(hGSMGetLastResponse());
          setSettingsFromHTTPresponse(hGSMGetLastResponse());
          setTelNoFromHTTPresponse(hGSMGetLastResponse());
          clrState(RESET_INITS);
        }
      }
      hGSMshutDown();
      closeGSM();
    }
    cleanUpDataSets(GSMfailure);
    _D(
      DebugPrintln("GPRS send done: " + String(millis()), DEBUG_LEVEL_2);
      if (GSMfailure){
	      if(!getState(POWER_SAVE_2)){
	        DebugPrintln("GSM: low battery.", DEBUG_LEVEL_1);
	      } else {
          DebugPrintln("GSM result: " + String(HTTPrc), DEBUG_LEVEL_1);
	      }
      }
    )
    if (GSMfailure && GPSalert()){
      if (getState(GPS_ALERT_1)) { clrState(GPS_ALERT_1); setState(PRE_GPS_ALERT); }
      if (getState(GPS_ALERT_2)) { clrState(GPS_ALERT_2); setState(GPS_ALERT_1); }
    }
  }
#endif

//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
#ifdef USE_LORA
  LoRa.end();
  LoRa.sleep();
  delay(100);
#endif

  heidiConfig->bootCount++;
  if (heidiConfig->bootCount >= _currentCycles()) { heidiConfig->bootCount = 0; }

  _DD(DebugPrintln("Last diff: " + String(heidiConfig->lastTimeDiffMs) + " / current diff : " + String(currentTimeDiffMs), DEBUG_LEVEL_3);)
  _D(DebugPrintln("SLEEP: heidi state: " + String(heidiConfig->status) ,DEBUG_LEVEL_1);)
  if (getState(NOT_IN_CYCLE)){
    clrState(NOT_IN_CYCLE);
    heidiConfig->lastTimeDiffMs = 0;       //not useful out of regular cycles
    currentTimeDiffMs = 0;
  }
  int diffTimeFinalMs = heidiConfig->lastTimeDiffMs + currentTimeDiffMs;
  heidiConfig->lastTimeDiffMs = diffTimeFinalMs;
  goto_sleep(timeToNextBootMS() + diffTimeFinalMs - millis());
}

void loop()
{}

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

void initGlobalVar(bool powerOnReset){
  initSysTimeMS();
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
  goto_sleep(getCycleTimeMS());
}

bool setupSystemDateTime(tm* systime, int timeOut){
  #ifdef GPS_MODULE
  _D(DebugPrintln("Get system time from GPS", DEBUG_LEVEL_1);)
  if (!openGPS()) {
    if (heidiConfig->bootCount <= REFETCH_SYS_TIME){ return false; }
    return getSysTime(systime);
  }
  if (!setSysTimeToGPSTime(timeOut)){
    closeGPS();
    if (heidiConfig->bootCount <= REFETCH_SYS_TIME){ return false; }
  }
  #endif
  return getSysTime(systime);
}

void goto_sleep(int32_t mseconds){
  if (watchd != NULL) { esp_timer_delete(watchd); }
  disableControls(true);
  pinMode(GPIO_NUM_4,  INPUT); //SCL
  pinMode(GPIO_NUM_5,  INPUT);
  pinMode(GPIO_NUM_13, INPUT); //GSM_ENABLE_PIN; SDA
  pinMode(GPIO_NUM_14, INPUT);
  pinMode(GPIO_NUM_15, INPUT);
  pinMode(GPIO_NUM_16, INPUT); //RXD
  pinMode(GPIO_NUM_17, INPUT); //TXD
  pinMode(GPIO_NUM_18, INPUT);
  pinMode(GPIO_NUM_19, INPUT);
  pinMode(GPIO_NUM_21, INPUT); //VOLT_ENABLE_PIN, GSM_RST; TXD
  pinMode(GPIO_NUM_22, INPUT); //TEMP_SENSOR_PIN
  pinMode(GPIO_NUM_23, INPUT); //RXD
  pinMode(GPIO_NUM_25, INPUT); //MEASURES_ENABLE_PIN
  pinMode(GPIO_NUM_26, INPUT);
  pinMode(GPIO_NUM_27, INPUT);

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
  }
  #endif
  _D(DebugPrintln("Sleep for : " + String(uint32_t(sleeptime/1000)) + " seconds", DEBUG_LEVEL_2); delay(50);)
  #ifdef USE_ULP
  init_accel_ULP(ULP_INTERVALL_US);
  esp_sleep_enable_ulp_wakeup();
  #endif
  doSleepRTCon(sleeptime);
}
void doSleepRTCon(int32_t ms){
  int32_t mSeconds = ms;
  if (mSeconds < MIN_SLEEP_TIME_MS) { mSeconds = MIN_SLEEP_TIME_MS; }
  esp_sleep_enable_timer_wakeup(uint64_t(mSeconds) * uint64_t(uS_TO_mS_FACTOR));
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_deep_sleep_start();
}

#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void checkWakeUpReason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : _D(DebugPrintln("Wakeup caused by external signal using RTC_IO", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_EXT1 : _D(DebugPrintln("Wakeup caused by external signal using RTC_CNTL", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_TIMER : _D(DebugPrintln("Wakeup caused by timer", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : _D(DebugPrintln("Wakeup caused by touchpad", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_ULP : _D(DebugPrintln("Wakeup caused by ULP program", DEBUG_LEVEL_1);) break;
    default : _D(DebugPrint("Wakeup was not caused by deep sleep: ", DEBUG_LEVEL_1);) _D(DebugPrintln(String(wakeup_reason), DEBUG_LEVEL_1);) break;

  }
}
#endif

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
	_D(DebugPrintln("This is the watchDog - Howdy?! :-D",DEBUG_LEVEL_1);)
	delay(10);
	heidiConfig->bootCount = REFETCH_SYS_TIME;
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
  _D(DebugPrintln("Boot: not in cycle.", DEBUG_LEVEL_1);)
  //testGeoFencing();
  //testData();
  #ifdef GSM_MODULE
  //if (volt >= GSM_MINIMUM_VOLTAGE){ hGSMCheckSignalStrength(); }
  #endif
  #ifdef GPS_MODULE
  //testGPS();
  #endif
  #ifdef TEMP_SENSOR
  //_D(DebugPrintln("Temperature: " + String(MeasureTemperature()), DEBUG_LEVEL_1);)
  #endif
}

double CheckBattery(void){
  double val;
  /**** check voltage does not need any inits ****/
  #ifdef CHECK_BATTERY
  val = MeasureVoltage(BATTERY_MEASURE_PIN);
  _D(DebugPrintln("Battery: " + String(val, 2), DEBUG_LEVEL_1); delay(100);)
  if (val <= GSM_POWER_SAVE_4_VOLTAGE){ // low Battery?
    _D(DebugPrintln("Battery much too low.", DEBUG_LEVEL_1); delay(100);)
    disableControls(true);
    doSleepRTCon(MAX_CYCLE_DURATION_MSEC);
  }
  if (val <= GSM_POWER_SAVE_3_VOLTAGE){ // low Battery?
    _D(DebugPrintln("Battery too low.", DEBUG_LEVEL_1); delay(100);)
    //esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ULP);
    int32_t sleeptime = getCycleTimeMS();
    if (sleeptime > MAX_CYCLE_DURATION_MSEC) { sleeptime = MAX_CYCLE_DURATION_MSEC; }
    disableControls(true);
    doSleepRTCon(sleeptime - millis());
  }
  if (val <= GSM_POWER_SAVE_2_VOLTAGE){
    setState(POWER_SAVE_2);
    setError(E_POWER_SAVE_2);
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


void testMeasure(){
  uint32_t a_measures   = 0;
  int32_t  measures     = 0;
  int analog_value      = 0;
  while(true){
    for(int i=1; i<1000;i++){
	    analog_value += analogRead(BATTERY_MEASURE_PIN);
	    a_measures++;
	    delay(1);
    }
	  if(a_measures > 0){
	    analog_value /= a_measures;
    }
	  _D(DebugPrintln("Measured Value: " + String(analog_value), DEBUG_LEVEL_1);)
	  a_measures   = 0;
	  measures     = 0;
  }
}

byte hexbuffer[35] = { 0x0e, 0x01, 0x01, 0xa7, 0x55, 0x0a, 0x03, 0xe5, 0x97, 0xcb, 0x00, 0x1c, 0x01, 0xa7, 0x52, 0x7d, 0x8a, 0x4b, 0x0f, 0xdd, 0x09, 0x00, 0x00, 0x05, 0x00, 0x44, 0x00, 0x9c, 0x01, 0x20, 0x01, 0x00, 0x00, 0x2b, 0x40 };
void CheckGSM(void){
  if(openGSM()){
    if (hGSMsetup()){
      String sendLine = "X01=" + b64Encode(hexbuffer, 35);
      _D(DebugPrintln("SEND: " + sendLine, DEBUG_LEVEL_1);)
      int HTTPtimeOut = sendLine.length() * 20 + 1000;
      if (HTTPtimeOut < 10000) {HTTPtimeOut = 10000;}
      int HTTPrc = hGSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data64.php",
                          "application/x-www-form-urlencoded",
                           sendLine,
                           HTTPtimeOut,
                           HTTPtimeOut);
      if (HTTPrc == 200){
        _D(DebugPrintln("HTTP send Line OK.", DEBUG_LEVEL_1);)
        _D(DebugPrintln("HTTP response: " + hGSMGetLastResponse(), DEBUG_LEVEL_1);)
        setFenceFromHTTPresponse(hGSMGetLastResponse());
        setSettingsFromHTTPresponse(hGSMGetLastResponse());
        setTelNoFromHTTPresponse(hGSMGetLastResponse());
      }
    }
    hGSMshutDown();
    closeGSM();
  }
}

#endif
