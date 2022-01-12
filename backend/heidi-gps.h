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

#define GPS_RXD   GPIO_NUM_16
#define GPS_TXD   GPIO_NUM_17
#define GPS_UART_NO 1
#define GPS_BAUD 9600

#define WAIT_FOR_GPS_TIME 180000

#define GPS_GET_SAT gpsData.data->navPVT.numSV
#define GPS_GET_LON gpsData.data->navPVT.lon/10000000.0f
#define GPS_GET_LAT gpsData.data->navPVT.lat/10000000.0f
#define GPS_GET_ALT gpsData.data->navPVT.hMSL/1000.0f
#define GPS_GET_DOP gpsData.data->navPVT.pDOP/100.0f
#define GPS_GET_ACC gpsData.data->navPVT.hAcc/1000.0f
#define GPS_GET_YR  gpsData.data->navPVT.year
#define GPS_GET_MO  gpsData.data->navPVT.month
#define GPS_GET_DY  gpsData.data->navPVT.day
#define GPS_GET_HR  gpsData.data->navPVT.hour
#define GPS_GET_MI  gpsData.data->navPVT.minute
#define GPS_GET_SE  gpsData.data->navPVT.second
#define RESET_GPS_DATA gpsData.valid = false; gpsData.type = GPS_DT_NONE;

typedef struct _locationSet_t{
  double   lng;
  double   lat;
  double   alt;
  double   acc;
  double   hdop;
  int      gps_sat;
}locationSet_t;

typedef enum _gpsStatusType_t {
  GPS_NEVER_GOT_LOCK = 0,
  GPS_LESS_EHP_DATA,
  GPS_GOT_EHP_DATA,
  GPS_LESS_AOP_DATA,
  GPS_GOT_AOP_DATA,
  GPS_GOT_2D_LOCK,
  GPS_GOT_3D_LOCK
}gpsStatusType_t;

//ublox UBX declarations
  typedef enum _gpsDataType_t {
     GPS_DT_NONE,
     GPS_DT_NAV_PVT,
     GPS_DT_NAV_POSLLH,
     GPS_DT_NAV_STATUS,
     GPS_DT_CFG_PRT,
     GPS_DT_CFG_NAVX5,
     GPS_DT_AID_EPH,
     GPS_DT_AID_AOP_DATA,
     GPS_DT_AID_AOP_STATUS
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
    GPS_UBX_CFG_PRT   = 0x00,
    GPS_UBX_CFG_MSG   = 0x01,
    GPS_UBX_CFG_INF   = 0x02,
    GPS_UBX_CFG_RATE  = 0x08,
    GPS_UBX_CFG_NAVX5 = 0x23
  }gps_ubx_cfg_id_t;
  typedef enum _gps_ubx_aid_id_t{
    GPS_UBX_AID_EPH     = 0x31,
    GPS_UBX_AID_AOP_DT  = 0x33,
    GPS_UBX_AID_AOP_ST  = 0x60
  }gps_ubx_aid_id_t;

  #define MSG_ON_OFF_GROUP 4
  typedef enum _gps_cfg_msg_group_t{
    GPS_CFG_MSG_NMEA = 0xF0,
    GPS_CFG_MSG_UBX  = 0x01
  }gps_cfg_msg_group_t;

  #define MSG_ON_OFF_TYPE 5
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

  typedef __attribute__((__packed__)) struct UART_CONFIG_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint8_t  port;
    uint8_t  reserved01;
    uint16_t rxReady;
    uint32_t UARTmode;
    uint32_t baud;
    uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
    uint16_t reserved02;
  };

  typedef __attribute__((__packed__)) struct NAVX5_CONFIG_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint16_t version;
    uint8_t  noInterest01[25];
    uint8_t  aopCfg;
    uint8_t  noInterest02[12];
  };
  typedef __attribute__((__packed__)) struct AID_EPH_DATA_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint32_t svId; //satellite ID
    uint32_t handOver;
    uint32_t sf1d[8];
    uint32_t sf2d[8];
    uint32_t sf3d[8];
  };
  typedef __attribute__((__packed__)) struct AID_AOP_DATA_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint8_t  svId; //satellite ID
    uint8_t  data[59];
    uint8_t  optional0[48];
    uint8_t  optional1[48];
    uint8_t  optional2[48];
  };
  typedef __attribute__((__packed__)) struct AID_AOPSTATUS_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint32_t iTOW;
    uint8_t  aopCfg;
    uint8_t  status;
    uint8_t  noInterest01[14];
  };


  typedef __attribute__((__packed__)) struct MSG_HEAD_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
  };

  union UBXMessage_t {
    MSG_HEAD_t      head;
    NAV_POSLLH_t    navPosllh;
    NAV_STATUS_t    navStatus;
    NAV_PVT_t       navPVT;
    UART_CONFIG_t   UARTconf;
    NAVX5_CONFIG_t  NAVX5conf;
    AID_EPH_DATA_t  AIDephData;
    AID_AOP_DATA_t  AIDaopData;
    AID_AOPSTATUS_t AIDaopStatus;
  };

  struct gpsData_t {
    bool          valid;
    gpsDataType_t  type;
    UBXMessage_t*  data;
  };

bool GPSGetPosition(t_SendData* DataSet, int requiredAccuracy, int maxWorse, int timeOut);
bool setBootTimeFromGPSTime(tm* bootTime, int timeOut);
void SetSysToGPS();
bool openGPS();
void closeGPS();
bool GPSprocessData(int timeOut);
void GPSsendMessage(uint8_t* message, int len);
void GPScalcChecksum(uint8_t* buffer, uint8_t* CK, int len);
bool GPSdecodeData(uint8_t data);
bool GPSwaitForMessage(gpsDataType_t type, int timeout);
bool GPSsetupUART(int newBaudRate);
void setGPSserialBaudRate(int rate);
bool GPSenableAOP(void);
void GPSsaveAOPdata(void);
int  GPSuploadAOPdata(void);
uint8_t GPSEphermerisDataStatus(void);
uint8_t GPSAOPdataStatus(void);
#if (DEBUG_LEVEL > 0 )
void _PrintDataGPS(void);
void testGPS(void);
#endif
#ifdef HEIDI_CONFIG_TEST
void GPSprintBootData(void);
#endif

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
