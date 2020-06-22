/*
 * heidi-debug.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include <Arduino.h>
#include "heidi-debug.h"

void DebugPrint(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.print(text); }}
void DebugPrintln(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.println(text); }}
void DebugPrint(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number, digits); }}
void DebugPrintln(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number, digits); }}
void DebugPrint(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}
void DebugPrint(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}




