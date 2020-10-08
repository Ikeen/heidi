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

#if DEBUG_LEVEL > 0
String LongLine="ID1=0003.0001&Lo1=13.342958&La1=51.008994&Al1=337&Da1=2020-07-01&Ti1=22:05:54&Ba1=4.03&F11=5&F21=16.25&F31=2&F41=21&ID2=0003.0001&Lo2=13.343122&La2=51.008755&Al2=337&Da2=2020-07-01&Ti2=22:20:54&Ba2=4.03&F12=5&F22=15.69&F32=2&F42=22&ID3=0003.0001&Lo3=13.342990&La3=51.009034&Al3=340&Da3=2020-07-01&Ti3=22:35:54&Ba3=4.03&F13=4&F23=15.19&F33=2&F43=21&ID4=0003.0001&Lo4=13.343020&La4=51.008973&Al4=342&Da4=2020-07-01&Ti4=22:50:54&Ba4=4.03&F14=5&F24=14.69&F34=2&F44=41&ID5=0003.0001&Lo5=13.343132&La5=51.008887&Al5=346&Da5=2020-07-01&Ti5=23:05:56&Ba5=4.03&F15=5&F25=14.44&F35=2&F45=16&ID6=0003.0001&Lo6=13.343005&La6=51.008806&Al6=343&Da6=2020-07-01&Ti6=23:20:54&Ba6=4.03&F16=5&F26=14.44&F36=2&F46=12&ID7=0003.0001&Lo7=13.342929&La7=51.008924&Al7=342&Da7=2020-07-01&Ti7=23:35:54&Ba7=4.03&F17=5&F27=14.06&F37=2&F47=16&ID8=0003.0001&Lo8=13.343002&La8=51.008803&Al8=341&Da8=2020-07-01&Ti8=23:50:54&Ba8=4.03&F18=4&F28=13.88&F38=2&F48=21&ID9=0003.0001&Lo9=13.343114&La9=51.008933&Al9=340&Da9=2020-07-02&Ti9=00:05:56&Ba9=4.03&F19=4&F29=13.63&F39=2&F49=20&ID10=0003.0001&Lo10=13.342987&La10=51.008886&Al10=340&Da10=2020-07-02&Ti10=00:20:54&Ba10=4.03&F110=4&F210=13.19&F310=2&F410=16&ID11=0003.0001&Lo11=13.343022&La11=51.008895&Al11=339&Da11=2020-07-02&Ti11=00:35:54&Ba11=4.02&F111=5&F211=12.75&F311=2&F411=44&ID12=0003.0001&Lo12=13.342887&La12=51.008778&Al12=338&Da12=2020-07-02&Ti12=00:50:54&Ba12=4.02&F112=6&F212=12.38&F312=2&F412=18&ID13=0003.0001&Lo13=13.342988&La13=51.008934&Al13=337&Da13=2020-07-02&Ti13=01:05:54&Ba13=4.02&F113=5&F213=12.06&F313=2&F413=15&ID14=0003.0001&Lo14=13.342982&La14=51.009038&Al14=338&Da14=2020-07-02&Ti14=01:20:54&Ba14=4.02&F114=5&F214=11.88&F314=2&F414=21&ID15=0003.0001&Lo15=13.343172&La15=51.008998&Al15=337&Da15=2020-07-02&Ti15=01:35:54&Ba15=4.02&F115=4&F215=12.06&F315=2&F415=21&ID16=0003.0001&Lo16=13.343058&La16=51.008866&Al16=336&Da16=2020-07-02&Ti16=01:50:54&Ba16=4.02&F116=4&F216=12.19&F316=2&F416=21&ID17=0003.0001&Lo17=13.343020&La17=51.008920&Al17=336&Da17=2020-07-02&Ti17=02:05:56&Ba17=4.02&F117=4&F217=12.19&F317=2&F417=56&ID18=0003.0001&Lo18=13.342892&La18=51.008921&Al18=334&Da18=2020-07-02&Ti18=02:20:54&Ba18=4.01&F118=4&F218=12.63&F318=2&F418=27&ID19=0003.0001&Lo19=13.343056&La19=51.008918&Al19=334&Da19=2020-07-02&Ti19=02:35:56&Ba19=4.02&F119=4&F219=13.06&F319=2&F419=20&ID20=0003.0001&Lo20=13.342940&La20=51.008885&Al20=334&Da20=2020-07-02&Ti20=02:50:54&Ba20=4.01&F120=4&F220=13.25&F320=2&F420=22&ID21=0003.0001&Lo21=13.342996&La21=51.008948&Al21=334&Da21=2020-07-02&Ti21=03:05:56&Ba21=4.01&F121=5&F221=13.50&F321=2&F421=42&ID22=0003.0001&Lo22=13.343065&La22=51.009073&Al22=333&Da22=2020-07-02&Ti22=03:20:54&Ba22=4.01&F122=5&F222=13.75&F322=2&F422=13&ID23=0003.0001&Lo23=13.342933&La23=51.008856&Al23=332&Da23=2020-07-02&Ti23=03:35:54&Ba23=4.01&F123=4&F223=13.94&F323=2&F423=21&ID24=0003.0001&Lo24=13.342905&La24=51.009112&Al24=332&Da24=2020-07-02&Ti24=03:50:54&Ba24=4.01&F124=4&F224=14.06&F324=2&F424=21&ID25=0003.0001&Lo25=13.342967&La25=51.008937&Al25=333&Da25=2020-07-02&Ti25=04:05:56&Ba25=4.01&F125=4&F225=14.25&F325=2&F425=42&ID26=0003.0001&Lo26=13.343409&La26=51.008502&Al26=331&Da26=2020-07-02&Ti26=04:20:54&Ba26=4.01&F126=4&F226=14.38&F326=2&F426=23&ID27=0003.0001&Lo27=13.343103&La27=51.008737&Al27=331&Da27=2020-07-02&Ti27=04:35:54&Ba27=4.01&F127=5&F227=14.50&F327=2&F427=73&ID28=0003.0001&Lo28=13.343004&La28=51.008779&Al28=333&Da28=2020-07-02&Ti28=04:50:56&Ba28=4.01&F128=5&F228=14.69&F328=2&F428=20&ID29=0003.0001&Lo29=13.343050&La29=51.008868&Al29=333&Da29=2020-07-02&Ti29=05:05:56&Ba29=4.01&F129=4&F229=14.88&F329=2&F429=20&ID30=0003.0001&Lo30=13.343079&La30=51.008988&Al30=333&Da30=2020-07-02&Ti30=05:20:54&Ba30=4.01&F130=4&F230=15.00&F330=2&F430=21&ID31=0003.0001&Lo31=13.342829&La31=51.008890&Al31=335&Da31=2020-07-02&Ti31=05:35:54&Ba31=4.01&F131=4&F231=15.19&F331=2&F431=21&ID32=0003.0001&Lo32=13.343015&La32=51.008942&Al32=335&Da32=2020-07-02&Ti32=05:50:54&Ba32=4.01&F132=5&F232=15.81&F332=2&F432=44&ID33=0003.0001&Lo33=13.343091&La33=51.008990&Al33=334&Da33=2020-07-02&Ti33=06:05:56&Ba33=4.01&F133=5&F233=16.44&F333=2&F433=15&ID34=0003.0001&Lo34=13.342958&La34=51.008976&Al34=332&Da34=2020-07-02&Ti34=06:20:54&Ba34=4.01&F134=5&F234=16.19&F334=2&F434=19&ID35=0003.0001&Lo35=13.343150&La35=51.008882&Al35=331&Da35=2020-07-02&Ti35=06:35:54&Ba35=4.01&F135=5&F235=15.44&F335=2&F435=16&ID36=0003.0001&Lo36=13.343047&La36=51.008954&Al36=332&Da36=2020-07-02&Ti36=06:50:54&Ba36=4.01&F136=4&F236=15.44&F336=2&F436=22&ID37=0003.0001&Lo37=13.343060&La37=51.008899&Al37=332&Da37=2020-07-02&Ti37=07:05:58&Ba37=4.00&F137=4&F237=16.81&F337=2&F437=18&ID38=0003.0001&Lo38=13.343116&La38=51.008887&Al38=333&Da38=2020-07-02&Ti38=07:20:54&Ba38=4.00&F138=4&F238=17.38&F338=2&F438=52&ID39=0003.0001&Lo39=13.343068&La39=51.008994&Al39=334&Da39=2020-07-02&Ti39=07:35:56&Ba39=4.01&F139=7&F239=18.13&F339=2&F439=43&ID40=0003.0001&Lo40=13.342994&La40=51.008671&Al40=336&Da40=2020-07-02&Ti40=07:50:54&Ba40=4.01&F140=5&F240=18.69&F340=2&F440=19&ID41=0003.0001&Lo41=13.343020&La41=51.008791&Al41=335&Da41=2020-07-02&Ti41=08:05:56&Ba41=4.01&F141=5&F241=19.50&F341=2&F441=16&ID42=0003.0001&Lo42=13.343017&La42=51.008827&Al42=336&Da42=2020-07-02&Ti42=08:20:56&Ba42=4.02&F142=4&F242=20.69&F342=2&F442=20&ID43=0003.0001&Lo43=13.343074&La43=51.008937&Al43=337&Da43=2020-07-02&Ti43=08:35:54&Ba43=4.02&F143=4&F243=22.00&F343=2&F443=27&ID44=0003.0001&Lo44=13.343027&La44=51.008958&Al44=336&Da44=2020-07-02&Ti44=08:50:54&Ba44=4.02&F144=4&F244=21.38&F344=2&F444=45&ID45=0003.0001&Lo45=13.343002&La45=51.008950&Al45=337&Da45=2020-07-02&Ti45=09:05:56&Ba45=4.02&F145=4&F245=20.94&F345=2&F445=20&ID46=0003.0001&Lo46=13.342974&La46=51.008927&Al46=337&Da46=2020-07-02&Ti46=09:20:56&Ba46=4.03&F146=5&F246=23.38&F346=2&F446=42&ID47=0003.0001&Lo47=13.342989&La47=51.008885&Al47=335&Da47=2020-07-02&Ti47=09:35:58&Ba47=4.04&F147=5&F247=27.25&F347=2&F447=16&ID48=0003.0001&Lo48=13.342856&La48=51.008965&Al48=336&Da48=2020-07-02&Ti48=09:50:54&Ba48=4.04&F148=5&F248=28.13&F348=2&F448=17&ID49=0003.0001&Lo49=13.342945&La49=51.008937&Al49=336&Da49=2020-07-02&Ti49=10:05:56&Ba49=4.08&F149=4&F249=30.88&F349=2&F449=18&ID50=0003.0001&Lo50=13.342939&La50=51.008919&Al50=336&Da50=2020-07-02&Ti50=10:20:56&Ba50=4.10&F150=5&F250=32.38&F350=2&F450=18&ID51=0003.0001&Lo51=13.343009&La51=51.008902&Al51=343&Da51=2020-07-02&Ti51=10:35:54&Ba51=4.05&F151=7&F251=31.44&F351=2&F451=43&ID52=0003.0001&Lo52=13.343024&La52=51.008907&Al52=348&Da52=2020-07-02&Ti52=10:50:54&Ba52=4.06&F152=6&F252=32.31&F352=2&F452=11&ID53=0003.0001&Lo53=13.342900&La53=51.008856&Al53=347&Da53=2020-07-02&Ti53=11:05:56&Ba53=4.05&F153=6&F253=30.44&F353=2&F453=12&ID54=0003.0001&Lo54=13.343029&La54=51.008911&Al54=347&Da54=2020-07-02&Ti54=11:21:04&Ba54=4.08&F154=6&F254=38.50&F354=2&F454=12&ID55=0003.0001&Lo55=13.343038&La55=51.008909&Al55=345&Da55=2020-07-02&Ti55=11:35:54&Ba55=4.05&F155=7&F255=36.75&F355=2&F455=34&ID56=0003.0001&Lo56=13.343009&La56=51.008930&Al56=345&Da56=2020-07-02&Ti56=11:50:50&Ba56=4.05&F156=5&F256=32.13&F356=2&F456=9&ID57=0003.0001&Lo57=13.342995&La57=51.008956&Al57=344&Da57=2020-07-02&Ti57=12:05:52&Ba57=4.05&F157=7&F257=30.63&F357=2&F457=11&ID58=0003.0001&Lo58=13.343034&La58=51.008957&Al58=344&Da58=2020-07-02&Ti58=12:20:50&Ba58=4.05&F158=6&F258=28.06&F358=2&F458=14&ID59=0003.0001&Lo59=13.343022&La59=51.008895&Al59=343&Da59=2020-07-02&Ti59=12:35:56&Ba59=4.06&F159=6&F259=33.19&F359=2&F459=34&ID60=0003.0001&Lo60=13.342920&La60=51.008868&Al60=343&Da60=2020-07-02&Ti60=12:51:00&Ba60=4.06&F160=6&F260=35.88&F360=2&F460=9&ID61=0003.0001&Lo61=13.342993&La61=51.008798&Al61=344&Da61=2020-07-02&Ti61=13:05:56&Ba61=4.06&F161=6&F261=38.31&F361=2&F461=15&ID62=0003.0001&Lo62=13.342956&La62=51.008884&Al62=343&Da62=2020-07-02&Ti62=13:20:54&Ba62=4.06&F162=6&F262=35.69&F362=2&F462=33&ID63=0003.0001&Lo63=13.343061&La63=51.008761&Al63=342&Da63=2020-07-02&Ti63=13:35:46&Ba63=4.06&F163=6&F263=31.75&F363=2&F463=9&ID64=0003.0001&Lo64=13.343016&La64=51.008934&Al64=341&Da64=2020-07-02&Ti64=13:50:52&Ba64=4.06&F164=6&F264=29.69&F364=2&F464=34&ID65=0003.0001&Lo65=13.342779&La65=51.009126&Al65=341&Da65=2020-07-02&Ti65=14:05:52&Ba65=4.06&F165=5&F265=29.06&F365=2&F465=9&ID66=0003.0001&Lo66=13.342692&La66=51.009014&Al66=340&Da66=2020-07-02&Ti66=14:20:54&Ba66=4.06&F166=5&F266=28.81&F366=2&F466=14&ID67=0003.0001&Lo67=13.342973&La67=51.008910&Al67=340&Da67=2020-07-02&Ti67=14:35:54&Ba67=4.06&F167=5&F267=30.56&F367=2&F467=34&ID68=0003.0001&Lo68=13.343023&La68=51.008852&Al68=338&Da68=2020-07-02&Ti68=14:50:56&Ba68=4.06&F168=6&F268=31.19&F368=2&F468=10&ID69=0003.0001&Lo69=13.342945&La69=51.008960&Al69=338&Da69=2020-07-02&Ti69=15:05:52&Ba69=4.06&F169=5&F269=28.50&F369=0&F469=17&ID70=0003.0001&Lo70=0.000000&La70=0.000000&Al70=0&Da70=2020-07-02&Ti70=15:20:58&Ba70=4.05&F170=0&F270=25.00&F370=21&F470=90&ID71=0003.0001&Lo71=13.342947&La71=51.008974&Al71=338&Da71=2020-07-02&Ti71=15:35:44&Ba71=4.05&F171=7&F271=23.88&F371=20&F471=42&ID72=0003.0001&Lo72=13.342910&La72=51.009003&Al72=338&Da72=2020-07-02&Ti72=15:50:54&Ba72=4.05&F172=6&F272=23.81&F372=20&F472=10";
String ShortLine="ID1=0003.0001&Lo1=13.342958&La1=51.008994&Al1=337&Da1=2020-07-01&Ti1=22:05:54&Ba1=4.03&F11=5&F21=16.25&F31=2&F41=21&ID2=0003.0001&Lo2=13.343122&La2=51.008755&Al2=337&Da2=2020-07-01&Ti2=22:20:54&Ba2=4.03&F12=5&F22=15.69&F32=2&F42=22";
#endif

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
  SerialGSM.begin(57600, SERIAL_8N1, GSM_RXD, GSM_TXD, false);

  for(int z=0; z<2; z++){
  	if (z>0){
	    resp = GSMsendCommand("AT+CPOWD=0");
	    delay(500);
	  }
    // get info
    _D(DebugPrintln(modemGSM.getModemInfo(), DEBUG_LEVEL_1));
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
    /*
    if (!modemGSM.sendSMS("01522xxxxxxx", "Heidi-Tracker hat soeben das erste mal erfolgreich Daten in die Datenbank gebracht. War gar nicht so einfach. :-)"))
    {
      Serial.println("Failed to send SMS");
      delay(1000);
      return;
    }
    Serial.println("SMS sent!");
    */
    // connect GPRS
    //  modemGSM.gprsConnect(APN,USER,PASSWORD))

    if(!GSMsetupGPRS("web.vodafone.de", "", ""))
    {
      _D(DebugPrintln("GPRS Connection\nFailed", DEBUG_LEVEL_1));
      delay(1000);
      continue;
    }
    _D(DebugPrintln("GPRS Connect OK", DEBUG_LEVEL_3));


    //configure DNS to vodafone DNS
    response = GSMsendCommand("AT+CDNSCFG=\"139.007.030.125\",\"139.007.030.126\"");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1) {
      _D(DebugPrintln("SIM800L : doPost() - Unable to define vodafone DNS", DEBUG_LEVEL_1));
    }else {
      response = GSMsendCommand("AT+CSQ");
	  _D(DebugPrintln("SIM800L : response to \"AT+CSQ\": " + response, DEBUG_LEVEL_1));
      return true;
    }
  } //for z
  return false;
}

bool GSMsetupGPRS(const String apn, const String user, const String pwd)
{
	String response, answer;
    int8_t code;

    _D(DebugPrintln("SIM800L : shut IP sessions.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+CIPSHUT"); // Reset the IP session if any
    if(response.indexOf("SHUT OK") == -1){ // This string in the response represents all IP sessions shutdown.
  	  _D(DebugPrintln("SIM800L : Cannot shut IP sessions.", DEBUG_LEVEL_1));
  	  delay(200);
  	  return false;
    }

    // Set the Bearer for the IP
    _D(DebugPrintln("SIM800L : set the connection type to GPRS.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot set the connection type to GPRS.", DEBUG_LEVEL_1));
  	  delay(200);
  	  return false;
    }

    _D(DebugPrintln("SIM800L : set APN.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+SAPBR=3,1,\"APN\",\"" + apn + "\"");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot set APN.", DEBUG_LEVEL_1));
  	  delay(200);
  	  return false;
    }

    _D(DebugPrintln("SIM800L : set USER.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+SAPBR=3,1,\"USER\",\"" + user + "\"");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot set USER.", DEBUG_LEVEL_1));
  	  delay(200);
    }

    _D(DebugPrintln("SIM800L : set PWD.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+SAPBR=3,1,\"PWD\",\"" + pwd + "\"");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot set PWD.", DEBUG_LEVEL_1));
  	  delay(200);
    }

    // Open the defined GPRS bearer context
    _D(DebugPrintln("SIM800L : open the defined GPRS bearer context.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+SAPBR=1,1");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot open the defined GPRS bearer context.", DEBUG_LEVEL_1));
  	  delay(200);
    }
    /*
    // Query the defined GPRS bearer context
    _D(DebugPrintln("SIM800L : query the GPRS bearer context status.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+SAPBR=2,1");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot query the GPRS bearer context status.", DEBUG_LEVEL_1));
  	  delay(200);
    }
    //Set to multi-IP
    response = GSMsendCommand("+CIPMUX=1");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot set to multi-IP.", DEBUG_LEVEL_1));
  	  delay(200);
  	  //return false;
    }
    */
    // Put in "quick send" mode (thus no extra "Send OK")
    _D(DebugPrintln("SIM800L : enable quick send mode.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+CIPQSEND=1");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot enable quick send mode.", DEBUG_LEVEL_1));
  	  delay(200);
  	  //return false;
    }

    // Set to get data manually
    _D(DebugPrintln("SIM800L : set to get data manually.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+CIPRXGET=1");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot set to get data manually.", DEBUG_LEVEL_1));
  	  delay(200);
  	  //return false;
    }

    // Start Task and Set APN, USER NAME, PASSWORD
    _D(DebugPrintln("SIM800L : start Task and Set APN, USER NAME, PASSWORD.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+CSTT=\"" + apn + "\",\"" + user + "\",\"" + pwd + "\"");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot start Task and Set APN, USER NAME, PASSWORD.", DEBUG_LEVEL_1));
  	  delay(200);
  	  return false;
    }

    // Bring Up Wireless Connection with GPRS or CSD
    _D(DebugPrintln("SIM800L : bring Up Wireless Connection with GPRS.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+CIICR");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot bring Up Wireless Connection with GPRS.", DEBUG_LEVEL_1));
  	  delay(200);
    }

    // Get Local IP Address, only assigned after connection
    _D(DebugPrintln("SIM800L : get Local IP Address.", DEBUG_LEVEL_3));
    response = GSMsendCommand("AT+CIFSR;E0");
    _D(DebugPrintln(response, DEBUG_LEVEL_3));
    if(response.indexOf("OK") == -1){
  	  _D(DebugPrintln("SIM800L : Cannot get Local IP Address.", DEBUG_LEVEL_1));
  	  delay(200);
  	  return false;
   }

   return true;

}
bool GSMshutDown()
{
  String resp = GSMsendCommand("AT+CIPSHUT"); // Reset the IP session if any
  if(resp.indexOf("SHUT OK") == -1){ // This string in the response represents all IP sessions shutdown.
	_D(DebugPrintln("SIM800L : Cannot shut IP sessions.", DEBUG_LEVEL_1));
	delay(200);
	return false;
  }
/*
  resp = GSMsendCommand("AT+CGATT=0"); // detach GPRS
  if(resp.indexOf("OK") == -1){
  	_D(DebugPrintln("SIM800L : Cannot detach GPRS.", DEBUG_LEVEL_1));
  	delay(200);
  	return false;
  }
*/
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
  response = "";
  unsigned int t=millis();
  while(millis() - t < serverReadTimeoutMs){
    delay(1);
    response += SerialGSM.readString();
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
  if (getError(WRONG_BOOT_REASON)){
    GSM_on();
    if (GSMsetup()){
      String response = "";
      response = GSMsendCommand("AT+CCLK?");
	  if(response.indexOf("OK") != -1){
	    _D(DebugPrint("GSM time : " + response.substring(7, response.indexOf((char)0x0C)) , DEBUG_LEVEL_1));
      }
	  response = GSMsendCommand("AT+CGSN");
	  _D(DebugPrint("SIM800L : response to \"AT+CGSN\": " + response, DEBUG_LEVEL_1));

	  //for(int i=0; i<5; i++){
	    response = GSMsendCommand("AT+CSQ");
	    _D(DebugPrint("SIM800L : response to \"AT+CSQ\": " + response, DEBUG_LEVEL_1));
	  	delay(500);
	  //}
    /*
	  int HTTPtimeOut = ShortLine.length() * 5 + 10000;
      _D(DebugPrintln("HTTP Line Length: " + String(ShortLine.length()), DEBUG_LEVEL_3));
      _D(DebugPrintln("HTTP TimeOut    : " + String(HTTPtimeOut), DEBUG_LEVEL_3));
      int HTTPrc = GSMdoPost("https://sx8y7j2yhsg2vejk.myfritz.net:1083/push_data2.php",
	                      "application/x-www-form-urlencoded",
						   ShortLine,
	  	                   HTTPtimeOut,
	  					   HTTPtimeOut);
      _D(DebugPrintln("HTTP Result: " + String(HTTPrc), DEBUG_LEVEL_3));
    */
	  GSMshutDown();
	}
	GSM_off();
  }
}
#endif //GSM_MODULE
