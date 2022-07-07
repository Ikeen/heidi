/*
 * heidi-gsm.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 *
 *  note: This code is intended for using SIM800L with GPRS data transmission only - no more no less.
 *  Other modems may work but also may not. Especially the booting sequence, which in this case puts out
 *  data depending on the current status of e.g. SIM card, may vary from modem to modem. We tried modem
 *  drivers from other too, but they lack of reliability or setup-speed. In fact most of them just wait
 *  a save amount of time after power up, and do lots of unnecessary configurations afterwards, but SIM800L
 *  for example, starts up with a useful default setup. So we have to wait a little bit after power up and
 *  do some settings - that cuts the setup time from 40 to 20 seconds. Waiting-time eats up power...
 *
 *  If you intend to use any other module, do some tests with DEBUG_LEVEL_3 and testGSM();
 *
 */

#include "heidi-defines.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp32-hal-gpio.h>
#include "heidi-data.h"
#include "heidi-sys.h"
#include "heidi-gsm.h"
#include "heidi-debug.h"
#include "heidi-error.h"
#include "heidi-fence.h"
#include "heidi-measures.h"
#include "heidi-secrets.h"
#include <HardwareSerial.h>
bool GSMenabled = false;
#ifdef GSM_MODULE

#ifdef COMMON_SERIAL
extern HardwareSerial heidiSerial1;
#define SerialGSM heidiSerial1
#else
extern HardwareSerial heidiSerial2;
#define SerialGSM heidiSerial2
#endif
String httpResponseData;
String ATresponse;

#ifdef TEST_GSM
#define WAIT_FOR_MODEM_TIMEOUT 10000
#define WAIT_FOR_NETWORK_TIMEOUT 60000
#define WAIT_FOR_SIM_TIMEOUT 15000
#else
#define WAIT_FOR_MODEM_TIMEOUT 10000
#define WAIT_FOR_NETWORK_TIMEOUT 30000
#define WAIT_FOR_SIM_TIMEOUT 15000
#endif

bool openGSM(){
  if (!CTLenabled){
    _D(DebugPrintln("controls not enabled - cannot enable GSM", DEBUG_LEVEL_1); pause(50););
    return false;
  }
  if (MEASenabled){
    _D(DebugPrintln("MEAS still enabled - cannot open GSM", DEBUG_LEVEL_1); pause(50);)
    return false;
  }
  if (!GSMenabled){
    GSMenabled = true;
    if (!openUart(GSM_UART_NO, GSM_BAUD, INPUT_PULLDOWN)){_D(DebugPrintln("GSM: could not open UART", DEBUG_LEVEL_1);) return false; }
    SerialGSM.setTimeout(READ_STRING_TIMEOUT);
    GSMOn();
    pause(3000); //GSM boot time
    SerialGSM.flush();
    _D(DebugPrintln("GSM open", DEBUG_LEVEL_2);)
  } _D(else { DebugPrintln("GSM already open", DEBUG_LEVEL_2); })
  return true;
}
void closeGSM(){
  if (GSMenabled){
    while(SerialGSM.available() > 0){ SerialGSM.read(); }
    //never use: SerialGSM.end();
    GSMOff();
    _D(DebugPrintln("GSM closed", DEBUG_LEVEL_2);)
  } _D(else { DebugPrintln("GSM already closed", DEBUG_LEVEL_2); })
  GSMenabled = false;
}


bool GSMsetup()
{
  _D(DebugPrintln("Setup GSM", DEBUG_LEVEL_1));
  // wait for modem
  if (!GSMwaitForModem(WAIT_FOR_MODEM_TIMEOUT)) { _D(DebugPrintln("GSM: Wait for modem failed", DEBUG_LEVEL_1)); return false; }
  _D(DebugPrintln("Modem OK", DEBUG_LEVEL_2));
  //wait for SIM
  pause(2000);
  //unlock SIM
  if(!GSMsimUnlock(HEIDI_SIM_PIN)) { _D(DebugPrintln("GSM: SIM unlock failed", DEBUG_LEVEL_1));  return false; };
  _D(DebugPrintln("SIM unlock OK", DEBUG_LEVEL_2));
  // connect to network
  if (!GSMwaitForNetwork(WAIT_FOR_NETWORK_TIMEOUT)){ _D(DebugPrintln("GSM: Wait for network failed", DEBUG_LEVEL_1)); return false; }
  _D(DebugPrintln("Modem network OK", DEBUG_LEVEL_2);)
  //need to handle SMS alerts before setup GPRS (?)
  if(!GSMhandleAlerts()) { _D(DebugPrintln("GSM - Handle Alerts failed!", DEBUG_LEVEL_1);) }

  if(!GSMsetupGPRS(HEIDI_MOBILE_APN, HEIDI_MOBILE_USER, HEIDI_MOBILE_PASSWD)){
	_D(DebugPrintln("GPRS Connection\nFailed", DEBUG_LEVEL_1));
	return false;
  }
  _DD(DebugPrintln("GPRS Connect OK", DEBUG_LEVEL_2));
  return true;
}

bool GSMsendLine(const String line, const String url){
  if(line == ""){ return true; }
  int initRC = GSMinitiateHTTP(url);
  if(initRC != 0) {
    _D(DebugPrintln("HTTP init connection fails with: " + String(initRC), DEBUG_LEVEL_1);)
    return false;
  }
  _DD(DebugPrintln("SEND: " + line, DEBUG_LEVEL_3);)
  int HTTPtimeOut = line.length() * 10 + 5000;
  if (HTTPtimeOut < 20000) {HTTPtimeOut = 20000;}
  int HTTPrc = GSMdoPost("application/x-www-form-urlencoded",
                         line,
                         HTTPtimeOut,
                         HTTPtimeOut);
  _D(
    String httpResponse = GSMGetLastResponse();
    if (HTTPrc == 200){
      DebugPrintln("HTTP send Line OK.", DEBUG_LEVEL_2);
      DebugPrintln("HTTP response: " + httpResponse.substring(0, 10) + (httpResponse.length()>10?"..":""), DEBUG_LEVEL_3);
    } else {
      DebugPrintln("HTTP send Line failed!. Error code: " + String(HTTPrc), DEBUG_LEVEL_1);
      DebugPrintln("HTTP response: " + httpResponse, DEBUG_LEVEL_1);
    }
   )
  GSMterminateHTTP();
  return (HTTPrc == 200);
}


bool GSMhandleAlerts(void){
  //alerts
  if (getState(GPS_ALERT_1 | GPS_ALERT_2) && !getState(GPS_ALERT_PSD)) {
    bool fail = true;
    int emptyTelNo = 0;
    for(int i=0; i<TEL_NO_CNT; i++){
      String TelNo = getTelNo(i);
      if (TelNo != ""){
        #ifdef SEND_ALERT_SMS
        if (GSMsendSMS(TelNo, "Heidi-Tracker: Positionsalarm Herde " + _herdeID()))
        #endif
        {
          fail = false;
          _D(DebugPrintln("GPS_ALERT sent to " + TelNo, DEBUG_LEVEL_2));
        }
      } else { emptyTelNo++; }
    }
    if (emptyTelNo == TEL_NO_CNT) { fail = false; }
    if(fail){
      heidiConfig->alertFailCount++;
      if (heidiConfig->alertFailCount >= MAX_FAILED_ALERTS) {
        clrState(GPS_ALERT_2 | GPS_ALERT_1 | PRE_GPS_ALERT);
        setState(GPS_ALERT_PSD);
        heidiConfig->alertFailCount = 0;
        _D(DebugPrintln("abort GPS ALERT", DEBUG_LEVEL_2);)
      }
      return false;
    }
    else {
      if (getState(GPS_ALERT_2)){ clrState(GPS_ALERT_2 | GPS_ALERT_1 | PRE_GPS_ALERT); setState(GPS_ALERT_PSD); _D(DebugPrintln("GPS ALERT: silent", DEBUG_LEVEL_2);)}
      if (getState(GPS_ALERT_1)){ clrState(GPS_ALERT_1 | PRE_GPS_ALERT); setState(GPS_ALERT_2); _D(DebugPrintln("GPS ALERT: 2", DEBUG_LEVEL_2);)}
      heidiConfig->alertFailCount = 0;
    }
  }
  return true;
}

/**
 * Do HTTP/S POST to a specific URL
 */
int GSMdoPost(String contentType, String payload, unsigned int clientWriteTimeoutMs, unsigned int serverReadTimeoutMs) {
  String response ="";
  unsigned int t = millis();
  // Define the content type
  if(!GSMsendCommand("AT+HTTPPARA=CONTENT," + contentType)){  return 702; }
  if(!GSMsendCommand("AT+HTTPDATA=" + String(payload.length()) + "," + String(clientWriteTimeoutMs), "DOWNLOAD", 5000)){ return 707; }
  SerialGSM.println(payload); SerialGSM.print(0);
  while((SerialGSM.available() == 0) && (millis()-t < clientWriteTimeoutMs)) { pause(10); }
  response = SerialGSM.readString();
  if(response.indexOf("OK") == -1){ return 703; }
  _DD(DebugPrintln("SIM800L : payload transmitted.", DEBUG_LEVEL_3);)

  // Start HTTP POST action
  if(!GSMsendCommand("AT+HTTPACTION=1", "OK", 30000)){ return 703; }

  // Wait answer from the server
  response = "";
  t = millis();
  while(millis() - t < serverReadTimeoutMs){
    response += SerialGSM.readString();
    if(response.indexOf("+HTTPACTION: 1,") > -1){ break; }
    pause(10);
  }
  if(response == ""){
    _D(DebugPrintln("SIM800L : doPost() - Server timeout", DEBUG_LEVEL_1));
    return 408;
  }
  response = _responseCleanUp(response);
  _DD(DebugPrintln("SIM800L : HTTP post response: " + response, DEBUG_LEVEL_3));
  int i = response.indexOf("+HTTPACTION: 1,");
  if(i < 0) {
    _D(DebugPrintln("SIM800L : doPost() - Invalid answer on HTTP POST", DEBUG_LEVEL_1));
    _DD(DebugPrintln("--> " + response, DEBUG_LEVEL_3));
    return 703;
  }
  // Get the HTTP return code
  int httpRC = _responseGetInt(2, response, -1);
  int dataSize = _responseGetInt(3, response, -1);

  _DD(DebugPrintln("SIM800L : doPost() - HTTP status " + String(httpRC) + ", " + String(dataSize) + " bytes to read ", DEBUG_LEVEL_3));
  if(dataSize >= 2) { //need to find at least servers "ok" inside, so at least 2 Bytes
    // Ask for reading and detect the start of the reading...
    if(!GSMsendCommand("AT+HTTPREAD")){ return 705; }
    int startPayload = ATresponse.indexOf("ok"); //server "ok" is lower cased = start of payload
    if(startPayload == -1){ // is there an "ok" from server?
      _D(DebugPrintln("SIM800L : doPost() - server does not accept data", DEBUG_LEVEL_1));
      return 406;
    }
    // extract number of bytes defined in the dataSize
    httpResponseData = ATresponse.substring(startPayload, startPayload + dataSize);
    httpResponseData.trim();
    _DD(DebugPrintln("SIM800L : doPost() - Received from HTTP GET : \n"  + httpResponseData, DEBUG_LEVEL_3));
  }
  return httpRC;
}

bool GSMsendSMS(String TelNo, String Message){
  if (!GSMsendCommand("AT+CMGF=1")) {return false;}
  if (!GSMsendCommand("AT+CMGS=\"" +  TelNo + "\"", ">", 1000)) {return false;}
  String _M = Message + " ";
  _M.setCharAt(Message.length(), 26);
  SerialGSM.println(_M); SerialGSM.print(0);
  int msStart = millis();
  String response = "";
  while ((millis() - msStart) < 5000){
    while((!SerialGSM.available()) && ((millis() - msStart) < 5000)) { pause(10); };
    if (SerialGSM.available()) { response += SerialGSM.readString(); }
    if (response.indexOf("OK") != -1) {return true;}
  }
  return false;
}

bool GSMshutDown()
{
  if(!GSMsendCommand("AT+CIPSHUT", "SHUT OK", 5000)){ return false;  }
  if(!GSMsendCommand("AT+CGREG=0")){ return false;  }
  if(!GSMsendCommand("AT+CREG=0")){ return false;  }
  //GSMsendCommand("AT+CPOWD=1"); pause(100);
  //never use: SerialGSM.end();
  return true;
}

bool GSMsimUnlock(String pin){
  if(!GSMwaitPIN(WAIT_FOR_SIM_TIMEOUT)) { return false; }
  _D(
    if(ATresponse.indexOf("SIM PUK") != -1){
      DebugPrintln("SIM requests PUK", DEBUG_LEVEL_1);
      return false;
    }
  )
  if(ATresponse.indexOf("+CPIN: SIM PIN") != -1) {
    if(!GSMsendCommand("AT+CPIN=" + pin)) { return false; }
    pause(1000); //some setup time
    if(!GSMsendCommand("AT+CPIN?", "+CPIN:", 5000)) { return false; }
  }
  if(ATresponse.indexOf("+CPIN: READY") == -1) {
    _D(DebugPrintln("SIM800L : Cannot unlock SIM.", DEBUG_LEVEL_1);)
    return false;
  }
  return true;
}

bool GSMwaitForNetwork(uint32_t timeOutMS){
  uint32_t t = millis();
  int status = 0;
  GSMsendCommand("AT+CREG=1");
  GSMsendCommand("AT+CGREG=1");
  while (millis()-t < timeOutMS){
    if (GSMsendCommand("AT+CGREG?")){
      int status = _responseGetIntKeyWord(2, "+CGREG", 2000);
      switch (status) {
        case 1: {
          _DD(DebugPrintln("SIM800L : registered to home network", DEBUG_LEVEL_3);)
          return true;
        }
        case 3: {
          _D(DebugPrintln("SIM800L : Registration denied", DEBUG_LEVEL_1);)
          return false;
        }
        case 4: {
          _D(DebugPrintln("SIM800L : Unknown registration error", DEBUG_LEVEL_1);)
          return false;
        }
        case 5: {
          _DD(DebugPrintln("SIM800L : registered to roaming network", DEBUG_LEVEL_3);)
          return true;
        }
        default: { pause(1000); }
      }
    }
  }
  _D(DebugPrintln("SIM800L : Registration timeout", DEBUG_LEVEL_1);)
  return false;
}

String GSMGetLastResponse()
{ return httpResponseData; }


bool GPRSsetUp = false;
bool GSMsetupGPRS (const String apn, const String user, const String pwd){
  if (!GSMsendCommand("AT+CIPSHUT", "SHUT OK", 5000)) {return false;} // Reset the IP session if any
  if (!GSMsendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"")) {return false;}
  if (!GSMsendCommand("AT+SAPBR=3,1,\"APN\",\"" + apn + "\"")) {return false;}
  if (!GSMsendCommand("AT+SAPBR=3,1,\"USER\",\"" + user + "\"" )) {return false;}
  if (!GSMsendCommand("AT+SAPBR=3,1,\"PWD\",\"" + pwd + "\"")) {return false;}
  GPRSsetUp = true;
  return true;
}

bool GPRSopen = false;
bool GSMopenGPRS(){
  if(GPRSopen) { _DD(DebugPrintln("GSM  : GPRS was already opened", DEBUG_LEVEL_3);) return true; }
  if(!GPRSsetUp) { _D(DebugPrintln("GSM  : GPRS setup was not done", DEBUG_LEVEL_1);) return false; }
  if (!GSMsendCommand("AT+SAPBR=1,1")){ return false; }
  if (!GSMsendCommand("AT+SAPBR=2,1")){ return false; }
  _DD(
      int i=ATresponse.indexOf('\"');
      int j=ATresponse.indexOf('\"', i+1);
      DebugPrintln("SIM800L : IP - " + ATresponse.substring(i+1,j), DEBUG_LEVEL_3);
  )
  pause(2000); //a little setup time
  GPRSopen = true;
  _DD(DebugPrintln("GSM  : GPRS opened", DEBUG_LEVEL_3);)
  return true;
}
int GSMcloseGPRS() {
  GPRSopen = false;
  if (!GSMsendCommand("AT+SAPBR=0,1")) { return 707; }
  return 0;
}

bool HTTPinit = false;
int GSMinitiateHTTP(String url) {
  if(HTTPinit) { _DD(DebugPrintln("GSM  : HTTP init was already done", DEBUG_LEVEL_3);) return 0; }
  if(!GPRSopen) { _D(DebugPrintln("GSM  : GPRS was not opened", DEBUG_LEVEL_1);) return -1; }
  String response = "";
  // Init HTTP connection
  if (!GSMsendCommand("AT+HTTPINIT")) { return 701; }
  if (!GSMsendCommand("AT+HTTPPARA=CID,1")) { return 702; }
  if (!GSMsendCommand("AT+HTTPPARA=URL," + url)) { return 703; }
  // HTTP or HTTPS
  if(url.indexOf("https://") == 0) {
    if (!GSMsendCommand("AT+HTTPSSL=1")) { return 704; }
  } else {
    if (!GSMsendCommand("AT+HTTPSSL=0")) { return 705; }
  }
  HTTPinit = true;
  return 0;
}
int GSMterminateHTTP() {
  HTTPinit = false;
  if (!GSMsendCommand("AT+HTTPTERM")) { return 706; }
  return 0;
}
bool GSMRestartModem(void){
  GPRSsetUp = false;
  GPRSopen = false;
  HTTPinit = false;
  if (!GSMsendCommand("AT+CFUN=0")) { return false; }
  pause(100);
  if (!GSMsendCommand("AT+CFUN=1")) { return false; }
  return true;
}
bool GSMwaitForModem(uint32_t timeOutMS){
  int msStart = millis();
  while((millis() - msStart) < timeOutMS){
    while(SerialGSM.available() > 0){ SerialGSM.read(); }
    SerialGSM.println("AT");
    SerialGSM.print(0);
    for( int i=0; i< 200; i++){
      if(SerialGSM.available()) { break; }
      pause(1);
    }
    if(SerialGSM.available()) {
      String response =  SerialGSM.readString();
      if (response.indexOf("OK") > -1) {
        pause(1000); //some setup time
        GSMsendCommand("ATE0");
        return GSMsetCFUN(5000);
      }
    }
  }
  return false;
}
bool GSMsetCFUN(uint32_t timeOutMS){
  int msStart = millis();
  GSMsendCommand("AT+CFUN=1");
  while((millis() - msStart) < timeOutMS){
    GSMsendCommand("AT+CFUN?");
    if (ATresponse.indexOf("+CFUN: 1") > -1) { return true; }
    pause(500);
  }
  return false;
}
bool GSMwaitPIN(uint32_t timeOutMS){
  int msStart = millis();
  while((millis() - msStart) < timeOutMS){
    GSMsendCommand("AT+CPIN?");
    if (ATresponse.indexOf("+CPIN:") > -1) { return true; }
    pause(1000);
  }
  return false;
}


bool GSMsendCommand(const String command) {return GSMsendCommand(command, "OK", 5000); }
bool GSMsendCommand(const String command, const String okPhrase, int timeOutMs){
  String response = "";
  unsigned int t = millis();
  int localTimeOut = timeOutMs / 2;
  while(SerialGSM.available() > 0){ SerialGSM.read();}
  while((millis() - t < timeOutMs) && (response.indexOf(okPhrase) == -1)){
    _DD(DebugPrintln("SIM800L>: " + command, DEBUG_LEVEL_3); pause(10);)
    SerialGSM.println(command); SerialGSM.print(0);
    while((response.indexOf(okPhrase) == -1) && ((millis() - t) < localTimeOut)) {
      if (SerialGSM.available() > 0) { response += SerialGSM.readString(); }
      else { pause(1); }
    }
    if (millis() - t < timeOutMs) { localTimeOut = timeOutMs; }
  }
  ATresponse = _responseCleanUp(response);
  _DD(
    if(response.length() != ATresponse.length()){
      DebugPrintln("SIM800L<: " + ATresponse + " [c" + String(response.length() - ATresponse.length()) + "]", DEBUG_LEVEL_3); pause(10);
    } else {
      DebugPrintln("SIM800L<: " + ATresponse, DEBUG_LEVEL_3); pause(10);
    }
  )
  ATresponse.trim();
  if(ATresponse.indexOf(okPhrase) == -1){
    _D(DebugPrintln("SIM800L : " + command + " failed", DEBUG_LEVEL_1); pause(10);)
    return false;
  }
  return true;
}

int _responseGetIntKeyWord(int position, String keyWord, int errValue){
  int p1 = ATresponse.indexOf(keyWord);
  if (p1 < 0) { return errValue; }
  int p2 = ATresponse.indexOf(10, p1);
  if (p2 < 0) { p2 = ATresponse.length(); }
  String keyLine = ATresponse.substring(p1, p2+1);
  return _responseGetInt(position, keyLine, errValue);
}

/*
 * reads comma separated values from last HTTP response string beginning from char ':'
 * 1st value = number/position 1
 */
int _responseGetInt(int position, int errValue){
  return _responseGetInt(position, ATresponse, errValue);
}

/*
 * reads comma separated values from string beginning from char ':'
 * 1st value = number/position 1
 */
int _responseGetInt(int position, String response, int errValue){
  String payload = response.substring(response.indexOf(':')+1);
  int p1 = 0;
  int p2 = -1;
  for(int i=0; i<position; i++){
    p1 = p2 + 1;
    p2 = payload.indexOf(',', p1);
    if (p2 == -1) { p2 = payload.length(); }
    if (p2 == p1) { break; }
  }
  if  (p2 > p1){
    payload = payload.substring(p1, p2+1);
    String number = "";
    int l = payload.length();
    for(int i=0; i<l; i++){
      if(((payload.charAt(i)>='0') && (payload.charAt(i)<='9')) || ((number == "") && (payload.charAt(i)=='-'))){
        number.concat(payload.charAt(i));
      }
    }
    if (number == "") { return errValue; }
    return number.toInt();
  }
  return errValue;
}
String _responseCleanUp(const String str){
  String res = "";
  int l = str.length();
  for(int i=0; i<l; i++){
    if((str.charAt(i) <= '~') && (str.charAt(i) > '\0')){
      res.concat(str.charAt(i));
    }
  }
  return res;
}

#ifdef TEST_GSM
_D(
void testGSM(void){
  #ifdef CHANGE_HERDE_ID_TO
  String Sendline = "ID=" + intString4(CHANGE_HERDE_ID_TO);
  #else
  String Sendline = "ID=" + _herdeID();
  #endif
  disableMeasures();
  if(openGSM()){
    if (GSMsetup()){
      GSMCheckSignalStrength();
      if (GSMopenGPRS()){
        int HTTPtimeOut = Sendline.length() * 20 + 1000;
        if (HTTPtimeOut < 10000) {HTTPtimeOut = 10000;}
        if (GSMsendLine(Sendline, HEIDI_SERVER_PUSH_URL)){
          setFenceFromHTTPresponse(GSMGetLastResponse());
          setSettingsFromHTTPresponse(GSMGetLastResponse());
          setTelNoFromHTTPresponse(GSMGetLastResponse());
        }
        Sendline = "TST=TestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTestTest";
        for(int i=0; i<10; i++){
            if(GSMsendLine(Sendline, HEIDI_SERVER_PUSH_URL)){
              String response = GSMGetLastResponse();
              if (response.substring(3)  != Sendline.substring(4)){
                DebugPrintln("************ !! *********************", DEBUG_LEVEL_1);
                DebugPrintln("HTTP : Got : " + response.substring(3), DEBUG_LEVEL_1);
                DebugPrintln("HTTP : Sent: " + Sendline.substring(4), DEBUG_LEVEL_1);
                DebugPrintln("************ !! *********************", DEBUG_LEVEL_1);
              }
          }
          pause(5000);
        }
        GSMcloseGPRS();
      } _D( else {_D(DebugPrintln("GSM: open GPRS failed", DEBUG_LEVEL_1);)})
      GSMshutDown();
    } _D( else {_D(DebugPrintln("GSM: setup GSM failed", DEBUG_LEVEL_1);)})
    closeGSM();
  } _D( else {_D(DebugPrintln("GSM: open GSM failed", DEBUG_LEVEL_1);)})
}

void GSMCheckSignalStrength(){
  //for(int i=0; i<20; i++){
    GSMsendCommand("AT+CCLK?");
    DebugPrint("SIM800L : response to \"AT+CGSN\": " + ATresponse, DEBUG_LEVEL_1);
    GSMsendCommand("AT+CGSN");
    DebugPrint("SIM800L : response to \"AT+CGSN\": " + ATresponse, DEBUG_LEVEL_1);
    GSMsendCommand("AT+CSQ");
    DebugPrint("SIM800L : response to \"AT+CSQ\": " + ATresponse, DEBUG_LEVEL_1);
    pause(1000);
  //}
}
)
#endif //TEST_GSM
#endif //GSM_MODULE
