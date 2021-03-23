/*
 * heidi-debug.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_DEBUG_H_
#define HEIDI_DEBUG_H_
#include <WString.h>
#include "heidi-data.h"

typedef enum {
  DEBUG_LEVEL_0,
  DEBUG_LEVEL_1,
  DEBUG_LEVEL_2,
  DEBUG_LEVEL_3
};

#define DEBUG_LEVEL 2 //0 (no prints) .. 3 (all prints)
//#define TEST_ON_BOOT

#if (DEBUG_LEVEL > 0 )
#  define _D(x) x
#else
#  define _D(x)
#endif
#if (DEBUG_LEVEL > 2 )
#  define _DD(x) x
#else
#  define _DD(x)
#endif
#if (DEBUG_LEVEL > 0)
void DebugPrint(String text, int level);
void DebugPrintln(String text, int level);
void DebugPrint(double number, int digits, int level);
void DebugPrintln(double number, int digits, int level);
void DebugPrint(int number, int level);
void DebugPrintln(int number, int level);
void DebugPrint(unsigned int number, int level);
void DebugPrintln(unsigned int number, int level);

void _PrintDataSet(t_SendData* DataSet, int dLevel);
void _PrintShortSummary(int dLevel);
void _PrintFence(int dLevel);
void _PrintHeidiConfig(int dLevel);
void _PrintHeidiAccParams(int dLevel);
#endif
#endif /* HEIDI_DEBUG_H_ */
