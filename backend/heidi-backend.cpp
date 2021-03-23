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
tm expBootTime;
esp_timer_handle_t watchd; //the watchdog

int currentTimeDiffMs  = 0;

static int msStartTime        = 0;
static uint16_t init_cycle    = (herdeID() % 10) * 6 - 1; //!!!!!!!!!!!!
static double volt;


void setup()
{
  t_SendData* currentDataSet;

  msStartTime = millis();
  #ifdef USE_ULP
  stopULP(); //anyway
  #endif
  _D(Serial.begin(115200);
     checkWakeUpReason();
  )

  //check battery status and goto sleep, if too low
  volt = CheckBattery();

  initError();
  initGlobalVar(wasPowerOnReset());
  setupWatchDog();
  enableControls(); // !!!!! may fail

  _D(if(wasPowerOnReset()) { DebugPrintln("power on reset", DEBUG_LEVEL_1); } else {
       DebugPrintln("Boot number: " + String(heidiConfig->bootCount), DEBUG_LEVEL_1); } delay(50);)
#ifdef TEST_ACC
  _D()
  _D(DebugPrintln("accel measurement count: " + String(get_accel_meas_cnt_ULP()), DEBUG_LEVEL_2);)
  _D(DebugPrintln("accel thresholt 1 count: " + String(get_accel_excnt1_ULP()), DEBUG_LEVEL_2);)
  _D(DebugPrintln("accel thresholt 2 count: " + String(get_accel_excnt2_ULP()), DEBUG_LEVEL_2);)
  _D(DebugPrintln("transmission result x: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_2);)
  _D(DebugPrintln("transmission result y: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_2);)
  _D(DebugPrintln("transmission result z: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_2);)
  _D(set_accel_excnt1_ULP(0);)
  _D(set_accel_excnt2_ULP(0);)
  delay(10000);
  goto_sleep(30000);
#endif

  //prepare free data set
  freeFirstDataSet();
  currentDataSet = availableDataSet[0];

  openMeasures();   //this here to prevent measures from multiple switching on/off

#ifdef USE_LORA
  SetupLoRa();
#endif

#ifdef TEMP_SENSOR
  currentDataSet->temperature = (int16_t)(MeasureTemperature()*100);
  _D(DebugPrintln("Temperature: " + String(((float)currentDataSet->temperature / 100)) + "C", DEBUG_LEVEL_1);)
#endif

  setupSystemDateTime();
  _D(if(_night()) {})

#ifdef TEST_RTC
  _D(testRTC(currentDataSet, &bootTime);)
#endif

  //find expected boot time
  if(!isInCycle(init_cycle, &(heidiConfig->bootCount))){ //calculates expected boot time
    heidiConfig->lastTimeDiffMs = 0;       //not useful out of regular cycles
    #ifdef GPS_MODULE
    closeGPS();
    #endif
    closeMeasures();
    doResetInits();

    if (currentTimeDiffMs <= 0) { goto_sleep(getCycleTimeMS()); } //something went wrong
    if ((currentTimeDiffMs - millis()) < 1000) { goto_sleep(1000); } //just to be sure
    goto_sleep(currentTimeDiffMs - millis());
  }
  currentDataSet->errCode = getErrorCode(); //errors until now

#ifdef GPS_MODULE
  if (GPSGetPosition(currentDataSet, 10, 90000) == 0){
    _D(DebugPrintln("GPS: Unable to fetch position.", DEBUG_LEVEL_1);)
  }
  _D(DebugPrintln("cor. boot time: " + TimeString(bootTime), DEBUG_LEVEL_1);)
  _D(DebugPrintln("exp. boot time: " + TimeString(expBootTime), DEBUG_LEVEL_1);)
  closeGPS();
#else
  currentDataSet->errCode |= COULD_NOT_FETCH_GPS;
  currentDataSet->errCode |= COULD_NOT_FETCH_GPS_TIME;
#endif

#ifdef USE_ULP
  currentDataSet->accThres1 = get_accel_excnt1_ULP();
  currentDataSet->accThres2 = get_accel_excnt2_ULP();
  _D(DebugPrintln("accel thresholt 1 count: " + String(currentDataSet->accThres1), DEBUG_LEVEL_2);)
  _D(DebugPrintln("accel thresholt 2 count: " + String(currentDataSet->accThres2), DEBUG_LEVEL_2);)
#endif
  //finalize data set
  currentDataSet->date    = dosDate(bootTime.tm_year, bootTime.tm_mon, bootTime.tm_mday);
  currentDataSet->time    = dosTime(bootTime.tm_hour, bootTime.tm_min, bootTime.tm_sec);
  currentDataSet->battery = (uint16_t)(round(volt * 1000));
  _DD(_PrintDataSet(currentDataSet, DEBUG_LEVEL_3);)

  closeMeasures();
#ifdef GSM_MODULE
  if (doDataTransmission(heidiConfig->bootCount)){
	  int HTTPrc = 0;
	  bool GSMfailure = true;
	  _DD(_PrintShortSummary(DEBUG_LEVEL_3));
    String sendLine = generateMulti64SendLine(0, (MAX_DATA_SETS-1));
    if ((sendLine.length() > 0) && (volt >= GSM_MINIMUM_VOLTAGE)){
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
            _D(DebugPrintln("HTTP response: " + hGSMGetLastResponse(), DEBUG_LEVEL_1);)
            GSMfailure = false;
            setFenceFromHTTPresponse(hGSMGetLastResponse());
            setSettingsFromHTTPresponse(hGSMGetLastResponse());
          }
        }else{
          _D(DebugPrintln("GPRS check OK.", DEBUG_LEVEL_1);)
        }
        hGSMshutDown();
        closeGSM();
      }
      cleanUpDataSets(GSMfailure);
    }
    _D(DebugPrintln("GPRS send done: " + String(millis() - msStartTime), DEBUG_LEVEL_2));
    #if DEBUG_LEVEL >= DEBUG_LEVEL_0
    if (GSMfailure){
	    if(volt < GSM_MINIMUM_VOLTAGE){
	      _D(DebugPrintln("GSM: low battery.", DEBUG_LEVEL_1);)
	    } else {
        _D(DebugPrintln("GSM result: " + String(HTTPrc), DEBUG_LEVEL_1);)
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

  heidiConfig->bootCount++;
  if (heidiConfig->bootCount >= getBootCycles()) { heidiConfig->bootCount = 0; }

  int diffTimeFinalMs = heidiConfig->lastTimeDiffMs + currentTimeDiffMs;
  _DD(DebugPrintln("getCycleTimeMS: " + String(getCycleTimeMS()), DEBUG_LEVEL_3);)
  goto_sleep(getCycleTimeMS() + diffTimeFinalMs - millis());
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
  bootTime.tm_hour    = INVALID_TIME_VALUE;
  bootTime.tm_min     = INVALID_TIME_VALUE;
  bootTime.tm_sec     = INVALID_TIME_VALUE;
  expBootTime.tm_hour = INVALID_TIME_VALUE;
  expBootTime.tm_min  = INVALID_TIME_VALUE;
  expBootTime.tm_sec  = INVALID_TIME_VALUE;
  initRTCData(powerOnReset);
  if(!wasRegularWakeUp()){ setError(WRONG_BOOT_REASON); }
}

void restartCycling(){
  heidiConfig->bootCount = REFETCH_SYS_TIME;
  goto_sleep(getCycleTimeMS());
}

void setupSystemDateTime(){
  tm timeinfo;
  #ifdef GPS_MODULE
  /**** check sys-time and cycle ****/
  if (openGPS()){
    if((!GetSysTime(&timeinfo)) || (heidiConfig->bootCount <= REFETCH_SYS_TIME)){
      heidiConfig->lastTimeDiffMs = 0; //not useful anymore
      delay(3000);
      _D(DebugPrintln("Boot: Get time from GPS", DEBUG_LEVEL_1);)
      if (!SetSysToGPSTime()){ //sets boot time if succeed
        closeGPS();
        goto_sleep(SLEEP_DUR_NOTIME);
      } else {
        if (!calcCurrentTimeDiff()){
          closeGPS();
          heidiConfig->bootCount = REFETCH_SYS_TIME;
          goto_sleep(getCycleTimeMS());
        }
      }
      _D(DebugPrintln("GPS time vs. internal timer: " + String(currentTimeDiffMs), DEBUG_LEVEL_1);)
      GetSysTime(&timeinfo);
    } else {
      int bootMs = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
      bootMs *= 1000;
      bootMs -= millis();
      SetBootTimeFromMs(bootMs);
    }
    _copyDate(&timeinfo, &bootTime);
  }
  #else
  currentTimeDiffMs = SLEEP_DUR_NOTIME;
  if (GetSysTime(&timeinfo)){
    _copyTime(&timeinfo, &bootTime);
    _copyDate(&timeinfo, &bootTime);
  } else {
    goto_sleep(SLEEP_DUR_NOTIME);
  }
  #endif
  _D(DebugPrintln("Current sys time: " + DateString(timeinfo) + " " + TimeString(timeinfo), DEBUG_LEVEL_1);)
  _D(DebugPrintln("sys. boot time:   " + DateString(bootTime) + " " + TimeString(bootTime), DEBUG_LEVEL_1);)
}

void goto_sleep(int32_t mseconds){
  esp_timer_delete(watchd);
  disableControls();
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
  if (sleeptime > MAX_SLEEP_TIME_MS) { sleeptime = MAX_SLEEP_TIME_MS; }
  if (sleeptime < MIN_SLEEP_TIME_MS) { sleeptime = MIN_SLEEP_TIME_MS; }
  _D(DebugPrintln("Sleep for : " + String(uint32_t(sleeptime/1000)) + " seconds", DEBUG_LEVEL_2); delay(50);)
  #ifdef USE_ULP
  //if (wasRegularWakeUp()) { enableULP(); }
  //else
  { init_accel_ULP(ULP_INTERVALL_US); }
  esp_sleep_enable_ulp_wakeup();
  #endif
  esp_sleep_enable_timer_wakeup(uint64_t(sleeptime) * uint64_t(uS_TO_mS_FACTOR));
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
  return rtc_get_reset_reason(0) == POWERON_RESET;
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
void doResetInits(){
  #ifdef GSM_MODULE
  _D(DebugPrintln("Get new fence - send ID", DEBUG_LEVEL_1);)
  if (volt >= GSM_MINIMUM_VOLTAGE){

    if (openGSM()){

      if (hGSMsetup()){
        int HTTPtimeOut = 10000;
        for (int i = 0; i<1; i++){
          int HTTPrc = hGSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data64.php",
                                  "application/x-www-form-urlencoded",
                                  "ID=" + _herdeID(),
                                  HTTPtimeOut,
                                  HTTPtimeOut);
          _D(DebugPrintln("HTTP result: " + String(HTTPrc), DEBUG_LEVEL_1);)
          if (HTTPrc == 200){
            _D(DebugPrintln("HTTP response: " + hGSMGetLastResponse(), DEBUG_LEVEL_2));
            if ((setFenceFromHTTPresponse(hGSMGetLastResponse())) && (setSettingsFromHTTPresponse(hGSMGetLastResponse()))) { break; }
          } _D( else { DebugPrintln("GSM HTTP error: " + String(HTTPrc), DEBUG_LEVEL_1); } )
        }
        //TODO: SMS alert if fails
        hGSMshutDown();
      }

      closeGSM();
    }
  }
  #endif
}

double CheckBattery(void){
  double val;
  /**** check voltage does not need any inits ****/
  #ifdef CHECK_BATTERY
  val = MeasureVoltage(BATTERY_MEASURE_PIN);
  _D(DebugPrintln("Battery: " + String(val, 2), DEBUG_LEVEL_1);)
  if (val <= 3.0){ // low Battery?
    _D(DebugPrintln("Battery much too low.", DEBUG_LEVEL_1);)
    _D(delay(100));
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ULP);
    esp_sleep_enable_timer_wakeup(uint64_t(3600000) * uint64_t(uS_TO_mS_FACTOR)); //sleep for 1h
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    esp_deep_sleep_start();
  }
  if (val <= 3.3){ // low Battery?
    _D(DebugPrintln("Battery too low.", DEBUG_LEVEL_1);)
    _D(delay(100));
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ULP);
    int32_t sleeptime = getCycleTimeMS();
    if (sleeptime > MAX_SLEEP_TIME_MS) { sleeptime = MAX_SLEEP_TIME_MS; }
    if ((sleeptime - millis()) < MIN_SLEEP_TIME_MS) { sleeptime = MIN_SLEEP_TIME_MS + millis(); }
    esp_sleep_enable_timer_wakeup(uint64_t(sleeptime - millis()) * uint64_t(uS_TO_mS_FACTOR));
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    esp_deep_sleep_start();
  }
  #else
  val = 4.0;
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

#endif
