/*
 * heidi-error.cpp
 *
 *  Created on: 10.07.2020
 *      Author: frank
 */
#include "heidi-error.h"

static uint16_t errorCode;

void setupError(){
  errorCode = 0;
}
void setError(uint16_t code){
  errorCode |= code;
}
void rmError(uint16_t code){
  errorCode &= ~code;
}
bool getError(uint16_t code){
  return ((errorCode & code) == code)?true:false;
}
void setError(t_SendData* DataSet, uint16_t code){
  DataSet->errCode |= code;
}
void setError(t_SendData** sets, int first, int last, uint16_t code){
  int current = first;
  while (current <= last){
    t_SendData* set = sets[current];
    setError(set, code);
    current++;
  }
}
void rmError(t_SendData* DataSet, uint16_t code){
  DataSet->errCode &= ~code;
}
bool getError(t_SendData* DataSet, uint16_t code){
  return ((DataSet->errCode & code) == code)?true:false;
}
uint16_t getErrorCode(){
  return errorCode;
}




