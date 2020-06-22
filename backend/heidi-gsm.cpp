/*
 * heidi-gsm.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#include "heidi-defines.h"
#ifdef GSM_MODULE

#include <Arduino.h>
//#include <esp32-hal-gpio.h>
#include "heidi-data.h"
#include "heidi-gsm.h"
#include "heidi-debug.h"
#include "heidi-error.h"

#define TINY_GSM_MODEM_SIM800 // define modem (SIM800L)
#include <TinyGsmClient.h>
#include <HardwareSerial.h>
HardwareSerial SerialGSM(GSM_UART_NO);
TinyGsm modemGSM(SerialGSM);

extern int msStartTime;
extern double volt;

void GSM_on(){
  _D(DebugPrint("GSM on...", DEBUG_LEVEL_1));
  delay(300);
  pinMode(GSM,OUTPUT);
  pinMode(GSM_RST,OUTPUT);
  digitalWrite(GSM_RST, LOW);
  for(int i=0; i<2000; i++){
    digitalWrite(GSM, GSM_OFF);
    delayMicroseconds(2010-i);
    digitalWrite(GSM, GSM_ON);
    delayMicroseconds(10+i);
  }
  delay(500);
  digitalWrite(GSM_RST, HIGH);
  delay(1500);
  _D(DebugPrintln("done", DEBUG_LEVEL_1));
}

void GSM_off(){
  SerialGSM.end();
  pinMode(GSM,OUTPUT);
  digitalWrite(GSM, GSM_OFF);
  #if DEBUG_LEVEL >= DEBUG_LEVEL_1
  _D(DebugPrintln("GSM off", DEBUG_LEVEL_1));
  delay(100);
  #endif
}

bool GSMsetup()
{
  String response = "";
  String resp = "";

  _D(DebugPrintln("Setup GSM", DEBUG_LEVEL_1));
  // init serial SIM800L
  SerialGSM.begin(38400, SERIAL_8N1, GSM_RXD, GSM_TXD, false);

  for(int z=0; z<2; z++){
	if (z>0){
	  resp = GSMsendCommand("AT+CPOWD=0");
	  delay(500);
	}
    // get info
    if (DEBUG_LEVEL > 0){
      Serial.println(modemGSM.getModemInfo());
    }
    // init modem
    if (!modemGSM.restart())
    {
      _D(DebugPrintln("Restarting GSM\nModem failed", DEBUG_LEVEL_1));
      continue;
    }
    _D(DebugPrintln("Modem restart OK", DEBUG_LEVEL_3));
    //unlock SIM
    response = GSMsendCommand("AT+CPIN?");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("+CPIN: SIM PIN") != -1) {
      if (!modemGSM.simUnlock("0354"))
      //if (!modemGSM.simUnlock("0041"))
      {
        _D(DebugPrintln("Failed to SIM unlock", DEBUG_LEVEL_1));
        continue;
      }
      response = GSMsendCommand("AT+CPIN?");
    }
    if(response.indexOf("+CPIN: READY") == -1) {
      _D(DebugPrintln("SIM800L : Cannot unlock SIM.", DEBUG_LEVEL_1));
      delay(200);
      continue;
    }
    _D(DebugPrintln("SIM800L : SIM ready", DEBUG_LEVEL_1));
    _D(DebugPrintln("SIM unlock OK", DEBUG_LEVEL_3));
    // connect to network
    if (!modemGSM.waitForNetwork())
    {
      _D(DebugPrintln("Failed to connect to network", DEBUG_LEVEL_1));
      delay(1000);
      continue;
    }
    _D(DebugPrintln("Modem network OK", DEBUG_LEVEL_3));

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
      _D(DebugPrintln("GPRS Connection\nFailed", DEBUG_LEVEL_1));
      delay(1000);
      continue;
    }
    _D(DebugPrintln("GPRS Connect OK", DEBUG_LEVEL_3));


    //reset DNS to vodafone DNS
    response = GSMsendCommand("AT+CDNSCFG=\"139.007.030.125\",\"139.007.030.126\"");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1) {
      _D(DebugPrintln("SIM800L : doPost() - Unable to define vodafone DNS", DEBUG_LEVEL_1));
    }else {
      _D(DebugPrintln("GPRS Setup done: " + String(millis() - msStartTime), DEBUG_LEVEL_2));
      response = GSMsendCommand("AT+CSQ");
	  _D(DebugPrintln("SIM800L : response to \"AT+CSQ\": " + response, DEBUG_LEVEL_1));
      return true;
    }
  } //for z
  return false;
}

bool GSMshutDown()
{
  String resp = GSMsendCommand("AT+CIPSHUT"); // Reset the IP session if any
  if(resp.indexOf("SHUT OK") == -1){ // This string in the response represents all IP sessions shutdown.
	_D(DebugPrintln("SIM800L : Cannot shut IP sessions.", DEBUG_LEVEL_1));
	delay(200);
	return false;
  }
  resp = GSMsendCommand("AT+CGATT=0"); // detach GPRS
  if(resp.indexOf("OK") == -1){
  	_D(DebugPrintln("SIM800L : Cannot detach GPRS.", DEBUG_LEVEL_1));
  	delay(200);
  	return false;
  }
  resp = GSMsendCommand("AT+CREG=0");
  if(resp.indexOf("OK") == -1){
  	_D(DebugPrintln("SIM800L : Cannot unregister SIM.", DEBUG_LEVEL_1));
  	delay(200);
  	return false;
  }
  resp = GSMsendCommand("AT+CPOWD=1");
  delay(100);
  SerialGSM.end();
  return true;
}
/**
 * Do HTTP/S POST to a specific URL
 */
int GSMdoPost(String url, String contentType, String payload, unsigned int clientWriteTimeoutMs, unsigned int serverReadTimeoutMs) {
  String response ="";
  response = GSMsendCommand("AT+CCLK?");
  if(response.indexOf("OK") != -1){
	_D(DebugPrint("GSM time : " + response.substring(7, response.indexOf((char)0x0C)) , DEBUG_LEVEL_1));
  }
  // Initiate HTTP/S session with the module
  int initRC = GSMinitiateHTTP(url);
  if(initRC > 0) { return initRC; }
  // Define the content type
  _D(DebugPrint("AT+HTTPPARA=\"CONTENT\",\"" + contentType + "\"", DEBUG_LEVEL_3));
  response = GSMsendCommand("AT+HTTPPARA=\"CONTENT\",\"" + contentType + "\"");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == -1) {
    _D(DebugPrintln("SIM800L : doPost() - Unable to define the content type", DEBUG_LEVEL_1));
    return 702;
  }

  //for(int k=0; k<5; k++){

  int httpRC;
  int l = payload.length();
  // Prepare to send the payload
  _D(DebugPrint("AT+HTTPDATA=" + String(l) + "," + String(clientWriteTimeoutMs), DEBUG_LEVEL_3));
  response = GSMsendCommand("AT+HTTPDATA=" + String(payload.length()) + "," + String(clientWriteTimeoutMs));
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("DOWNLOAD") == -1) {
    _D(DebugPrintln("SIM800L : doPost() - Unable to send payload to module", DEBUG_LEVEL_1));
    return 707;
  }
  // Write the payload on the module
  _D(DebugPrintln("SIM800L : doPost() - send " + String(l) + " bytes payload", DEBUG_LEVEL_3));
  for (int i=0; i<l; i++){
    if  (SerialGSM.write(payload.charAt(i)) == 0){
        _D(DebugPrintln("SIM800L : send Payload -could not write char " + String(i), DEBUG_LEVEL_1));
        delay(10);
    }
  }
  SerialGSM.println("");
  response = SerialGSM.readString();
  _D(DebugPrintln(response, DEBUG_LEVEL_3));

  // Start HTTP POST action
  _D(DebugPrint("AT+HTTPACTION=1", DEBUG_LEVEL_3));
  response = GSMsendCommand("AT+HTTPACTION=1", 30000);
  _D(DebugPrint(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == -1) {
    _D(DebugPrintln("SIM800L : doPost() - Unable to initiate POST action", DEBUG_LEVEL_1));
    return 703;
  }
  // Wait answer from the server
  unsigned int t=millis();
  while(millis() - t < serverReadTimeoutMs){
    delay(1);
    response = SerialGSM.readString();
    if (response.length() > 0) { _D(DebugPrintln(response, DEBUG_LEVEL_3)); }
    if(response.indexOf("+HTTPACTION: 1,") > -1){
      t = serverReadTimeoutMs + millis();
    }
  }
  if(response == ""){
    _D(DebugPrintln("SIM800L : doPost() - Server timeout", DEBUG_LEVEL_1));
    return 408;
  }
  int i = response.indexOf("+HTTPACTION: 1,");
  if(i < 0) {
    _D(DebugPrintln("SIM800L : doPost() - Invalid answer on HTTP POST", DEBUG_LEVEL_1));
    return 703;
  }
  // Get the HTTP return code
  httpRC = response.substring(i+15,i+18).toInt();
  int dataSize = response.substring(i+19,response.length()-i).toInt();

  _D(DebugPrintln("SIM800L : doPost() - HTTP status " + String(httpRC) + ", " + String(dataSize) + " bytes to read ", DEBUG_LEVEL_3));
  if(dataSize >= 2) { //need to find "OK" inside, so at least 2 Bytes

    // Ask for reading and detect the start of the reading...
    response = GSMsendCommand("AT+HTTPREAD");
    _D(DebugPrintln("SIM800L : doPost() - response to \"AT+HTTPREAD\": " + response, DEBUG_LEVEL_1));
    i = response.indexOf("+HTTPREAD: ");
    if(i == -1){
       _D(DebugPrintln("SIM800L : doPost() - Invalid response to \"AT+HTTPREAD\": " + response, DEBUG_LEVEL_1));
       return 705;
    }
    int startPayload = i + 11 + 2 + String(dataSize).length();
    // extract number of bytes defined in the dataSize
    String httpData = response.substring(startPayload, startPayload + dataSize);
    // We are expecting a final OK (maybe "OK - no valid data")
    if(response.substring(startPayload + dataSize, response.length()).indexOf("OK") == -1){ //now look for everything behind AT - "OK"
      _D(DebugPrintln("SIM800L : doPost() - Invalid end of data while reading HTTP result from the module", DEBUG_LEVEL_1));
      return 705;
    }
    _D(DebugPrintln("SIM800L : doPost() - Received from HTTP GET : \n"  + httpData, DEBUG_LEVEL_3));
	if(httpData.indexOf("OK") == -1){ // is there another "OK" (from server)?
        _D(DebugPrintln("SIM800L : doPost() - server does not accept data", DEBUG_LEVEL_1));
		httpRC = 406;
	}
  }
  // Terminate HTTP/S session
  int termRC = GSMterminateHTTP();
  if(termRC > 0) {
    return termRC;
  }
  _D(DebugPrintln("GPRS send done: " + String(millis() - msStartTime), DEBUG_LEVEL_2));
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
  unsigned int t=millis();
  while(millis() - t < serverReadTimeoutMs){
    delay(1);
    response = SerialGSM.readString();
    Serial.println(response);
    if(response.indexOf("+HTTPACTION: 0,") > -1){
      t = serverReadTimeoutMs + millis();
    }
  }
  if(response == ""){
    Serial.println("SIM800L : doPost() - Server timeout");
    return 408;
  }
  int i = response.indexOf("+HTTPACTION: 0,");
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
  _D(DebugPrint("AT+HTTPINIT", DEBUG_LEVEL_3));
  response = GSMsendCommand("AT+HTTPINIT");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == 0) {
    _D(DebugPrintln("SIM800L : initiateHTTP() - Unable to init HTTP", DEBUG_LEVEL_1));
    return 701;
  }
  // Use the GPRS bearer
  _D(DebugPrint("AT+HTTPPARA=\"CID\",1", DEBUG_LEVEL_3));
  response = GSMsendCommand("AT+HTTPPARA=\"CID\",1");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == -1) {
    _D(DebugPrintln("SIM800L : initiateHTTP() - Unable to define bearer", DEBUG_LEVEL_1));
    return 702;
  }
  // Define URL to look for
  _D(DebugPrint("AT+HTTPPARA=\"URL\",\"" + url + "\"", DEBUG_LEVEL_3));
  response = GSMsendCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == -1) {
    _D(DebugPrintln("SIM800L : initiateHTTP() - Unable to define the URL", DEBUG_LEVEL_1));
    return 702;
  }
  // HTTP or HTTPS
  if(url.indexOf("https://") == 0) {
    _D(DebugPrint("AT+HTTPSSL=1", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+HTTPSSL=1");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1) {
      _D(DebugPrintln("SIM800L : initiateHTTP() - Unable to switch to HTTPS", DEBUG_LEVEL_1));
      return 702;
    }
  } else {
    _D(DebugPrint("AT+HTTPSSL=0", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+HTTPSSL=0");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1) {
      _D(DebugPrintln("SIM800L : initiateHTTP() - Unable to switch to HTTP", DEBUG_LEVEL_1));
      return 702;
    }
  }
  return 0;
}
int GSMterminateHTTP() {
  // Close HTTP connection
  String response = GSMsendCommand("AT+HTTPTERM");
  if(response.indexOf("OK") == -1)  {
    _D(DebugPrintln("SIM800L : terminateHTTP() - Unable to close HTTP session", DEBUG_LEVEL_1));
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
void GSMCheckSignalStrength(){
  if ((errorCode & WRONG_BOOT_REASON) == WRONG_BOOT_REASON){
    if (volt >= 3.6){
      GSM_on();
      if (GSMsetup()){
        String response = "";
	    response = GSMsendCommand("AT+CCLK?");
	    if(response.indexOf("OK") != -1){
	      _D(DebugPrint("GSM time : " + response.substring(7, response.indexOf((char)0x0C)) , DEBUG_LEVEL_1));
        }
	    for(int i=0; i<5; i++){
	      response = GSMsendCommand("AT+CSQ");
	  	  _D(DebugPrint("SIM800L : response to \"AT+CSQ\": " + response, DEBUG_LEVEL_1));
	  	  delay(500);
	    }
	    GSMshutDown();
	  }
	  GSM_off();
    }
  }
}
#endif //GSM_MODULE






/*
 * init step by step - currently not working
 */


#if 0
  /*
  AT
  >> AT // This should come back. SIM900 default is to echo back commands you enter
  >> OK // This string should tell you all is well
  */
  int i=0;
  while (i<100){
	response = GSMsendCommand("AT");
    if(response.indexOf("OK") == -1) {
      delay(10);
      if ( i>= 100){
        _D(DebugPrintln("SIM800L : Modem is not responding.", DEBUG_LEVEL_1));
        delay(200);
        return false;
      }
    } else {
      break; //modem is up
    }
    i++;
  }
  _D(DebugPrintln("SIM800L : Modem is up", DEBUG_LEVEL_1));
  /*
  AT+CPIN? // This is to check if SIM is unlocked.
  >> +CPIN: SIM PIN // pin codes need to be entered
  >> +CPIN: READY // If your response contains this, then it means SIM is unlocked and ready
  >> OK
  */
  response = GSMsendCommand("AT+CPIN?");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("+CPIN: SIM PIN") != -1) {
	//PIN needed
	response = GSMsendCommand("AT+CPIN=\"0354\"");
    if(response.indexOf("OK") == -1) {
      _D(DebugPrintln("SIM800L : Cannot unlock SIM.", DEBUG_LEVEL_1));
      delay(200);
      return false;
    }
    response = GSMsendCommand("AT+CPIN?");
  }
  if(response.indexOf("+CPIN: READY") == -1) {
    _D(DebugPrintln("SIM800L : Cannot unlock SIM.", DEBUG_LEVEL_1));
    delay(200);
    return false;
  }
  _D(DebugPrintln("SIM800L : SIM ready", DEBUG_LEVEL_1));
  /*
  AT+CREG? // This checks if SIM is registered or not
  >> +CREG: 0,1 // This string in the response indicates SIM is registered (2=registering, 5=roaming)
  */
  //if (!GSMsendCommand2("AT+CREG?", response)) { _D(DebugPrintln("SIM800L : AT+CREG? command failed.", DEBUG_LEVEL_1)); delay(200); return false; }
  response = GSMsendCommand("AT+CREG?");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
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
	    _D(DebugPrintln("SIM800L : Use roaming.", DEBUG_LEVEL_1));
	  }
      #endif
	}
  }
  if(p == -1) {
	_D(DebugPrintln("SIM800L : Cannot register SIM.", DEBUG_LEVEL_1));
	delay(200);
	return false;
  }
  _D(DebugPrintln("SIM800L : SIM registered", DEBUG_LEVEL_1));

  /*
  AT+CGATT? // Check if GPRS is attached or not
  >> +CGATT: 1 // A response containing this string indicates GPRS is attached
  */
  response = GSMsendCommand("AT+CGATT?");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("+CGATT: 1") != -1){
	//GPRS is already attached
	resp = GSMsendCommand("AT+CIPSHUT"); // Reset the IP session if any
    _D(DebugPrintln(resp, DEBUG_LEVEL_3));
	if(resp.indexOf("SHUT OK") == -1){ // This string in the response represents all IP sessions shutdown.
	  _D(DebugPrintln("SIM800L : Cannot shut old IP sessions.", DEBUG_LEVEL_1));
	  delay(200);
	  return false;
	}
  } else if(response.indexOf("+CGATT: 0") != -1){
	delay(500); // wait a short time
	resp = GSMsendCommand("AT+CGATT=1"); // attach GPRS
	_D(DebugPrintln(resp, DEBUG_LEVEL_3));
	if(resp.indexOf("OK") == -1){
      delay(1000);
	  resp = GSMsendCommand("AT+CGATT=1"); // attach GPRS 2nd
	  _D(DebugPrintln(resp, DEBUG_LEVEL_3));
	  if(resp.indexOf("OK") == -1){
        _D(DebugPrintln("SIM800L : Cannot attach GPRS.", DEBUG_LEVEL_1));
	    delay(200);
	    return false;
	  }
	}
	response = GSMsendCommand("AT+CGATT?");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
  }
  if(response.indexOf("+CGATT: 1") == -1){
	_D(DebugPrintln("SIM800L : Cannot attach GPRS.", DEBUG_LEVEL_1));
	delay(200);
    return false;
  }
  _D(DebugPrintln("SIM800L : GPRS attached", DEBUG_LEVEL_1));

  /*
  AT+CIPSTATUS // Check if the IP stack is initialized
  >> STATE: IP INITIAL // This string in the response indicates IP stack is initialized
  */
  response = GSMsendCommand("AT+CIPSTATUS");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("STATE: IP INITIAL") == -1){
	_D(DebugPrintln("SIM800L : unexpected IP status: " + response, DEBUG_LEVEL_1));
	delay(200);
	return false;
  }
  _D(DebugPrintln("SIM800L : IP status ok.", DEBUG_LEVEL_1));

  /*
  AT+CIPMUX=0 // To keep things simple, I’m setting up a single connection mode
  >> OK // This string indicates single connection mode set successfully at SIM 900
  */
  response = GSMsendCommand("AT+CIPMUX=0");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == -1){
	_D(DebugPrintln("SIM800L : unable to set CIPMUX to zero.", DEBUG_LEVEL_1));
	delay(200);
	//return false;
  }
  _D(DebugPrintln("SIM800L : CIPMUX off.", DEBUG_LEVEL_1));

  /*
  AT+CSTT= "APN", "UNAME", "PWD" // Start the task, based on the SIM card you are using, you need to know the APN, username and password for your service provider
  >> OK // This response indicates task started successfully
  */
  response = GSMsendCommand("AT+CSTT=\"web.vodafone.de\",\"\",\"\"");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == -1){
	_D(DebugPrintln("SIM800L : unable to set APN.", DEBUG_LEVEL_1));
	delay(200);
	return false;
  }
  _D(DebugPrintln("SIM800L : APN set.", DEBUG_LEVEL_1));

  /*
  AT+CIICR // Now bring up the wireless. Please note, the response to this might take some time
  >> OK // This text in response string indicates wireless is up
  */
  response = GSMsendCommand("AT+CIICR");
  _D(DebugPrintln(response, DEBUG_LEVEL_3));
  if(response.indexOf("OK") == -1){
	_D(DebugPrintln("SIM800L : unable to bring up the wireless.", DEBUG_LEVEL_1));
	delay(200);
	//return false;
  }
  _D(DebugPrintln("SIM800L : wireless up.", DEBUG_LEVEL_1));

  /*
  AT+CIFSR // Get the local IP address. Some people say that this step is not required, but if I do not issue this, it was not working for my case. So I made this mandatory, no harm.
  >> xxx.xxx.xxx.xxx //
  */
  response = GSMsendCommand("AT+CIFSR");
  _D(DebugPrintln(response, DEBUG_LEVEL_1));
  if(response.indexOf("OK") == -1){
	_D(DebugPrintln("SIM800L : unable to get IP address.", DEBUG_LEVEL_1));
	delay(200);
	//return false;
  }
  #if DEBUG_LEVEL > 0
  int x = response.indexOf((char)0x0A)+1;
  _D(DebugPrintln("SIM800L : IP: " + response.substring(x, response.indexOf((char)0x0C, x)), DEBUG_LEVEL_1));
  #endif
#endif //if 0



