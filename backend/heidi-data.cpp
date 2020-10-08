/*
 * heidi-data.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include <rom/rtc.h>
#include <time.h>
#include "heidi-backend.h"
#include "heidi-measures.h"

RTC_DATA_ATTR uint8_t SendData_Space[DATA_SET_MEM_SPACE]; //bis 32 Byte (derzeit 25) können wir hier 32 Datensätze = 1K verballern
t_SendData* availableDataSet[MAX_DATA_SETS];

void initDataSets(void){
  uint8_t* curSet = SendData_Space;
  for (int i=0; i<MAX_DATA_SETS; i++){
    availableDataSet[i] = (t_SendData*)curSet;
    curSet += DATA_SET_LEN;
  }
}

void initDataSet(t_SendData* DataSet){
  DataSet->longitude   = 0;
  DataSet->latitude    = 0;
  DataSet->altitude    = 0;
  DataSet->date        = 0;
  DataSet->time        = 0;
  DataSet->battery     = 0;
  DataSet->secGPS      = 0;
  DataSet->temperature = NO_TEMPERATURE; //-127;
  DataSet->errCode     = 0;
  DataSet->satellites  = 0;
}
bool emptyDataSet(t_SendData* DataSet){
  return (DataSet->date == 0);
}
void copyDataSet(t_SendData* _from, t_SendData* _to){
  _to->longitude  = _from->longitude ;
  _to->latitude   = _from->latitude  ;
  _to->altitude   = _from->altitude  ;
  _to->date       = _from->date      ;
  _to->time       = _from->time      ;
  _to->battery    = _from->battery   ;
  _to->temperature= _from->temperature;
  _to->satellites = _from->satellites;
  _to->secGPS     = _from->secGPS    ;
  _to->errCode    = _from->errCode   ;
 }

String generateSendLine(t_SendData* DataSet){
  //"TrackerID=0002.1235&Longitude=-13.344855&Latitude=51.006709&Altitude=305&Date=2019-06-09&Time=17:20:00&Battery=4.02",
  String result = "";
  if (!emptyDataSet(DataSet)){
    result = "TrackerID=";
    int8_t hour_shift = GetLocalTimeHourShift(); //local time setting to be done
    int l = String(herdeID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(herdeID()) + ".";
    l = String(animalID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(animalID());
    result += "&Longitude=" + String(double(DataSet->longitude) / 1000000.0, 6);
    result += "&Latitude=" + String(double(DataSet->latitude) / 1000000.0, 6);
    result += "&Altitude=" + String(DataSet->altitude);
    result += "&Date=" + String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date)));
    result += "&Time=" + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time)));
    result += "&Battery=" + String((double(DataSet->battery) / 1000), 2);
    result += "&FreeValue1=" + String((int)DataSet->satellites);
    result += "&FreeValue2="  + String((float)DataSet->temperature / 100, 2);
    result += "&FreeValue3="  + String(DataSet->errCode, HEX);
    result += "&FreeValue4="  + String((int)DataSet->secGPS);
  }
  return result;
}
String generateMultiSendLine(int first, int last, int backups){
  //short notation with multiple data sets
  //php.ini: max_input_var = 1000 by default (1 data set = 7..12 vars)
  //php.ini: post_max_size = 8M by default - SIM800 312k only! (1 data set = 90..150 byte)
  //->up 80 data sets possible
  //don't forget to increase timeouts on web server side
  //"ID1=0002.1235&Lo1=-13.344855&La1=51.006709&Al1=305&Da1=2019-06-09&Ti1=17:20:00&Ba1=4.02&ID1=0002.1235&Lo1=...",
  t_SendData* DataSet;
  String result = "";
  int k = 0;
  int i = 0;
  for (int b = backups; b >= 0; b--){
    for (int a=first; a<=last; a++){
      i = a + b * BOOT_CYCLES;
      DataSet = availableDataSet[i];
      if (emptyDataSet(DataSet)) { continue; }
      if ((i < BOOT_CYCLES) || ((DataSet->errCode & GSM_TRANSMISSION_FAILED) == GSM_TRANSMISSION_FAILED)) {
        k++;
	    if (k > 1) { result += "&"; }
	    result += "ID" + String(k) + "=";
        int8_t hour_shift = GetLocalTimeHourShift(); //local time setting to be done
        int l = String(herdeID()).length();
        for (int j=0; j<(4-l); j++){ result += "0";}
        result += String(herdeID()) + ".";
        l = String(animalID()).length();
        for (int j=0; j<(4-l); j++){ result += "0";}
        result += String(animalID());
        result += "&Lo" + String(k) + "=" + String(double(DataSet->longitude) / 1000000.0, 6);
        result += "&La" + String(k) + "=" + String(double(DataSet->latitude) / 1000000.0, 6);
        result += "&Al" + String(k) + "=" + String(DataSet->altitude);
        result += "&Da" + String(k) + "=" + String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date)));
        result += "&Ti" + String(k) + "=" + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time)));
        result += "&Ba" + String(k) + "=" + String((double(DataSet->battery) / 1000), 2);
        result += "&F1" + String(k) + "=" + String((int)DataSet->satellites);
        result += "&F2" + String(k) + "=" + String((float)DataSet->temperature / 100, 2);
        result += "&F3" + String(k) + "=" + String(DataSet->errCode, HEX);
        result += "&F4" + String(k) + "=" + String((int)DataSet->secGPS);
      }
    }
  }
  return result;
}

void cleanUpDataSets(bool TransmissionFailed){
  //mark transmission result
  for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
    if (!emptyDataSet(availableDataSet[i])){
      if (TransmissionFailed)  { setError(availableDataSet[i], GSM_TRANSMISSION_FAILED); }
      else { rmError(availableDataSet[i], GSM_TRANSMISSION_FAILED); }
    }
  }
  //delete all transmitted data
  for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
    if(getError(availableDataSet[i], GSM_TRANSMISSION_FAILED) == false){
      initDataSet(availableDataSet[i]);
    }
  }
  //concentrate
  int k = 0;
  for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
    if(!emptyDataSet(availableDataSet[i]) && (k < i) && emptyDataSet(availableDataSet[k])){
      copyDataSet(availableDataSet[i], availableDataSet[k]);
      initDataSet(availableDataSet[i]);
    }
    if(!emptyDataSet(availableDataSet[k])){ k++; }
  }
  //prepare next cycle
  for(int i=(BOOT_CYCLES * DATA_SET_BACKUPS); i >= 0; i--){
    if(!emptyDataSet(availableDataSet[i])){
      copyDataSet(availableDataSet[i], availableDataSet[i+BOOT_CYCLES]);
      initDataSet(availableDataSet[i]);
    }
  }
}


void getRTCDataSpace(uint8_t** buffer){
	*buffer = SendData_Space;
}

String DateString(tm timestamp){
  return String(timestamp.tm_year) + "-" + LenTwo(String(timestamp.tm_mon)) + "-" + LenTwo(String(timestamp.tm_mday));
}
String TimeString(tm timestamp){
  return LenTwo(String(timestamp.tm_hour)) + ":" + LenTwo(String(timestamp.tm_min)) + ":"+ LenTwo(String(timestamp.tm_sec));
}
String LenTwo(const String No){
  if (No.length() == 1) { return "0" + No; }
  return No;
}


//uint16_t date; // 0-4 Day of the month / 5-8 Month /  9-15 Year offset from 1980
uint16_t dosDate(const uint8_t year, const uint8_t month, const uint8_t day){
  uint16_t date_val = day & 0x1F;
  date_val += (month & 0x0F) << 5;
  date_val += (year-1980) << 9;
  return date_val;
}
uint16_t dosYear(const uint16_t date){
  return (date >> 9) + 1980;
}
uint8_t dosMonth(const uint16_t date){
  return (date >> 5) & 0x0F;
}
uint8_t dosDay(const uint16_t date){
  return date & 0x1F;
}
//uint16_t time; // 0-4 Second divided by 2 / 5-10 Minute / 11-15 Hour
uint16_t dosTime(const uint8_t hour, const uint8_t minute, const uint8_t second){
  uint16_t time_val = second >> 1;
  time_val += (minute & 0x3F) << 5;
  time_val += (hour & 0x1F) << 11;
  return time_val;
}
uint8_t dosHour(const uint16_t time){
  return (time >> 11);
}
uint8_t dosMinute(const uint16_t time){
  return (time >> 5) & 0x3F;;
}
uint8_t dosSecond(const uint16_t time){
  return (time & 0x1F) << 1;
}
