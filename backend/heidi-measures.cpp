/*
 * heidi-measures.cpp
 *
 *  Created on: 08.07.2020
 *      Author: frank
 */
#include <Arduino.h>
#include "heidi-measures.h"

int running_measures = 0;

#ifdef TEMP_SENSOR
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

float MeasureTemperature(){
  float temp;
  MeasuresOn();
  tempSensor.begin();
  for(int i=0; i<500; i++){
    tempSensor.requestTemperaturesByIndex(0);
    temp = tempSensor.getTempCByIndex(0);
    if(temp != NO_TEMPERATURE) { break; }
  }
  MeasuresOff();
  return temp;
}


#endif

void initTriggeredMeasuring(){
  MeasuresOn();

}
double MeasureVoltage(){
  int analog_value = 0;
  MeasuresOn();
  for(int i=0; i<1000; i++){ analog_value += analogRead(BATTERY_ANALOG_PIN); }
  MeasuresOff();
  analog_value /= 1000;
  return((double)(analog_value + ANALOG_MEASURE_OFFSET) / ANALOG_MEASURE_DIVIDER);
}

void MeasuresOn(){
  if (running_measures == 0){
    pinMode(MEASURES_ENABLE_PIN,OUTPUT);
    digitalWrite(MEASURES_ENABLE_PIN, MEASURES_ON);
    _D(DebugPrintln("Measures on", DEBUG_LEVEL_1));
  }
  running_measures++;
}
void MeasuresOff(){
  running_measures--;
  if (running_measures < 0) { running_measures = 0; }
  else if (running_measures == 0){
    pinMode(MEASURES_ENABLE_PIN,OUTPUT);
    digitalWrite(MEASURES_ENABLE_PIN, MEASURES_OFF);
    _D(DebugPrintln("Measures off", DEBUG_LEVEL_1));
    _D(delay(50));
  }
}
void forceMeasuresOff(){
  running_measures = 1;
  MeasuresOff();
}

void LED_off(){
  pinMode(LED_ENABLE_PIN,OUTPUT);
  digitalWrite(LED_ENABLE_PIN, LED_OFF);
}
