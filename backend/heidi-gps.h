/*
 * heidi-gsm.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_GPS_H_
#define HEIDI_GPS_H_
#include "heidi-defines.h"
#ifdef GPS_MODULE
#include <stdint.h>
#include <WString.h>
#include "esp32-hal-gpio.h"
#include "heidi-data.h"

//#define USE_TINY_GPS_LIB

#ifdef USE_TINY_GPS_LIB
#include "TinyGPS++.h"
#endif
#define GPS_RXD   GPIO_NUM_16
#define GPS_TXD   GPIO_NUM_17
#define GPS_UART_NO 1
#define GPS_BAUD 9600

#define WAIT_FOR_GPS_TIME 180000

#ifdef  USE_TINY_GPS_LIB
    #define GPS_GET_SAT gps.satellites.value()
    #define GPS_GET_LON gps.location.lng()
    #define GPS_GET_LAT gps.location.lat()
    #define GPS_GET_ALT gps.altitude.meters()
    #define GPS_GET_DOP gps.hdop.value()
    #define GPS_GET_ACC 0
    #define GPS_GET_YR  gps.date.year()
    #define GPS_GET_MO  gps.date.month()
    #define GPS_GET_DY  gps.date.day()
    #define GPS_GET_HR  gps.time.hour()
    #define GPS_GET_MI  gps.time.minute()
    #define GPS_GET_SE  gps.time.second()
#else
    #define GPS_GET_SAT gpsData.data->navPVT.numSV
    #define GPS_GET_LON gpsData.data->navPVT.lon/10000000.0f
    #define GPS_GET_LAT gpsData.data->navPVT.lat/10000000.0f
    #define GPS_GET_ALT gpsData.data->navPVT.hMSL/1000.0f
    #define GPS_GET_DOP gpsData.data->navPVT.pDOP/100.0f
    #define GPS_GET_ACC (int)gpsData.data->navPVT.hAcc/1000.0f
    #define GPS_GET_YR  gpsData.data->navPVT.year
    #define GPS_GET_MO  gpsData.data->navPVT.month
    #define GPS_GET_DY  gpsData.data->navPVT.day
    #define GPS_GET_HR  gpsData.data->navPVT.hour
    #define GPS_GET_MI  gpsData.data->navPVT.minute
    #define GPS_GET_SE  gpsData.data->navPVT.second

  typedef enum _gpsDataType_t {
     GPS_DT_NONE,
     GPS_DT_NAV_PVT,
     GPS_DT_NAV_POSLLH,
     GPS_DT_NAV_STATUS
  }gpsDataType_t;

  #define GPS_UBX_CLASS 0
  typedef enum _gps_ubx_class_t{
    GPS_UBX_CLASS_NAV = 0x01,
    GPS_UBX_CLASS_RXM = 0x02,
    GPS_UBX_CLASS_INF = 0x04,
    GPS_UBX_CLASS_ACK = 0x05,
    GPS_UBX_CLASS_CFG = 0x06,
    GPS_UBX_CLASS_MON = 0x0A,
    GPS_UBX_CLASS_AID = 0x0B,
    GPS_UBX_CLASS_TIM = 0x0D,
    GPS_UBX_CLASS_LOG = 0x21
  }gps_ubx_class_t;

  #define GPS_UBX_ID 1
  typedef enum _gps_ubx_nav_id_t{
    GPS_UBX_NAV_POSECEF = 0x01,
    GPS_UBX_NAV_POSLLH  = 0x02,
    GPS_UBX_NAV_STATUS  = 0x03,
    GPS_UBX_NAV_PVT     = 0x07
  }gps_ubx_nav_id_t;
  typedef enum _gps_ubx_cfg_id_t{
    GPS_UBX_CFG_MSG  = 0x01,
    GPS_UBX_CFG_INF  = 0x02,
    GPS_UBX_CFG_RATE = 0x08
  }gps_ubx_cfg_id_t;

  #define NMEA_MSG_ON_OFF_GROUP 4
  typedef enum _gps_cfg_msg_group_t{
    GPS_CFG_MSG_NMEA = 0xF0,
    GPS_CFG_MSG_UBX  = 0x01
  }gps_cfg_msg_group_t;

  #define NMEA_MSG_ON_OFF_TYPE 5
  typedef enum _gps_cfg_msg_type_t{
    GPS_CFG_MSG_GGA, //0x00
    GPS_CFG_MSG_GLL, //0x01
    GPS_CFG_MSG_GSA, //0x02
    GPS_CFG_MSG_GSV, //0x03
    GPS_CFG_MSG_RMC, //0x04
    GPS_CFG_MSG_VGT, //0x05
    GPS_CFG_MSG_GRS, //0x06
    GPS_CFG_MSG_GST, //0x07
    GPS_CFG_MSG_ZDA, //0x08
    GPS_CFG_MSG_GBS, //0x09
    GPS_CFG_MSG_DTM, //0x0a
    GPS_CFG_MSG_TXT = 0x41
  }gps_cfg_msg_type_t;

  typedef enum _gps_serial_channel_t{
    GPS_CFG_SER_CH_I2C   = 0x06,
    GPS_CFG_SER_CH_UART1 = 0x07,
    GPS_CFG_SER_CH_UART2 = 0x08,
    GPS_CFG_SER_CH_USB   = 0x09,
    GPS_CFG_SER_CH_SPI   = 0x0a
  }gps_serial_channel_t;

  typedef enum _gps_on_off_t{
    GPS_CFG_OFF   = 0x00,
    GPS_CFG_ON    = 0x01
  }gps_on_off_t;
  const uint8_t UBX_HEADER[] = { 0xB5, 0x62 };

  typedef enum _gps_datetime_valid_t{
    GPS_DATE_TIME_NOT_VALID   = 0x00,
    GPS_DATE_TIME_DATE_VALID  = 0x01,
    GPS_DATE_TIME_TIME_VALID  = 0x02,
    GPS_DATE_TIME_RESOLVED    = 0x04,
    GPS_DATE_TIME_VALID       = 0x07
  }gps_datetime_valid_t;

  typedef enum _gps_fix_type_t{
    GPS_FIX_TYPE_NO_FIX   = 0x00,
    GPS_FIX_TYPE_2D_FIX   = 0x02,
    GPS_FIX_TYPE_3D_FIX   = 0x03
  }gps_fix_type_t;


  typedef __attribute__((__packed__)) struct NAV_PVT_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint32_t iTOW;          // GPS time of week of the navigation epoch (ms)

    uint16_t year;          // Year (UTC)
    uint8_t  month;         // Month, range 1..12 (UTC)
    uint8_t  day;           // Day of month, range 1..31 (UTC)
    uint8_t  hour;          // Hour of day, range 0..23 (UTC)
    uint8_t  minute;        // Minute of hour, range 0..59 (UTC)
    uint8_t  second;        // Seconds of minute, range 0..60 (UTC)
    uint8_t  valid;         // Validity Flags (see graphic below)
    uint32_t tAcc;          // Time accuracy estimate (UTC) (ns)
    int32_t  nano;          // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
    uint8_t  fixType;       // GNSSfix Type, range 0..5
    uint8_t  flags;         // Fix Status Flags
    uint8_t  reserved1;     // reserved
    uint8_t  numSV;         // Number of satellites used in Nav Solution

    int32_t  lon;           // Longitude (deg)
    int32_t  lat;           // Latitude (deg)
    int32_t  height;        // Height above Ellipsoid (mm)
    int32_t  hMSL;          // Height above mean sea level (mm)
    uint32_t hAcc;          // Horizontal Accuracy Estimate (mm)
    uint32_t vAcc;          // Vertical Accuracy Estimate (mm)

    int32_t  velN;          // NED north velocity (mm/s)
    int32_t  velE;          // NED east velocity (mm/s)
    int32_t  velD;          // NED down velocity (mm/s)
    int32_t  gSpeed;        // Ground Speed (2-D) (mm/s)
    int32_t  heading;       // Heading of motion 2-D (deg)
    uint32_t sAcc;          // Speed Accuracy Estimate
    uint32_t headingAcc;    // Heading Accuracy Estimate
    uint16_t pDOP;          // Position dilution of precision
    uint16_t reserved2;     // Reserved
    uint32_t reserved3;     // Reserved
  };

  typedef __attribute__((__packed__)) struct NAV_POSLLH_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint32_t iTOW;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
  };

  typedef __attribute__((__packed__)) struct NAV_STATUS_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint32_t iTOW;
    uint8_t  gpsFix;
    uint8_t  flags;
    uint8_t  fixStat;
    uint8_t  flags2;
    uint32_t ttff;
    uint32_t msss;
  };

  typedef __attribute__((__packed__)) struct MSG_HEAD_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
  };

  union UBXMessage_t {
    MSG_HEAD_t   head;
    NAV_POSLLH_t navPosllh;
    NAV_STATUS_t navStatus;
    NAV_PVT_t    navPVT;
  };

  struct gpsData_t {
    bool          valid;
    gpsDataType_t  type;
    UBXMessage_t*  data;
  };


#endif

int  GPSGetPosition(t_SendData* DataSet, int averages, int timeOut);
bool setBootTimeFromGPSTime(tm* bootTime, int timeOut);
void SetSysToGPS();
bool openGPS();
void closeGPS();
void GPSprocessData(void);
bool GPSwaitForData(int timeOut);
#ifndef  USE_TINY_GPS_LIB
void GPSsendMessage(uint8_t* message, int len);
void GPScalcChecksum(uint8_t* buffer, uint8_t* CK, int len);
void GPSdecodeData(uint8_t data);
#endif
#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void _PrintDataGPS();
void testGPS();
#endif

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
