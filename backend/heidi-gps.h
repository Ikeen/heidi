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

#define GPS_UART_NO      UART1
#define GPS_DEFAULT_BAUD 9600
#define GPS_HIGH_BAUD    57600   //19200,38400,57600,115200
#define GPS_MAX_SILENCE_MS 10000 //max time [ms] between 2 position fixes before break measurement (position got lost)
#define SUFFICIENT_DOP_VALUE 1.5
#define REQUIRED_DOP_VALUE   2.5

#ifdef USE_CASIC_GPS
#define GPS_GET_SAT gpsData.data->CASnavPV.numSV
#define GPS_GET_LON mkDouble(gpsData.data->CASnavPV.lon1, gpsData.data->CASnavPV.lon2)
#define GPS_GET_LAT mkDouble(gpsData.data->CASnavPV.lat1, gpsData.data->CASnavPV.lat2)
#define GPS_GET_ALT mkFloat(gpsData.data->CASnavPV.alt)
#define GPS_GET_PDOP mkFloat(gpsData.data->CASnavPV.pDOP)
//#define GPS_GET_ACC mkFloat(gpsData.data->CASnavPV.cAcc)
#define GPS_GET_YR  gpsData.data->CASnavTimeUTC.year
#define GPS_GET_MO  gpsData.data->CASnavTimeUTC.month
#define GPS_GET_DY  gpsData.data->CASnavTimeUTC.day
#define GPS_GET_HR  gpsData.data->CASnavTimeUTC.hour
#define GPS_GET_MI  gpsData.data->CASnavTimeUTC.minute
#define GPS_GET_SE  gpsData.data->CASnavTimeUTC.second
#define GPS_TIM_VALID gpsData.data->CASnavTimeUTC.valid >= GPS_DATE_TIME_VALID_CASIC
#define GPS_IS_NAV_DATA gpsData.type == GPS_DT_CASIC_NAV_PV
#define GPS_IS_DOP_DATA gpsData.type == GPS_DT_CASIC_NAV_DOP
#define GPS_GOT_2D_FIX gpsData.data->CASnavPV.posValid == GPS_FIX_TYPE_CASIC_2D
#define GPS_GOT_3D_FIX gpsData.data->CASnavPV.posValid >= GPS_FIX_TYPE_CASIC_3D
#define GPS_MSG_ACK_T CACK_ACK_NACK_t
#define GPS_MSG_ACK   CASack
#else
#define GPS_GET_SAT gpsData.data->navPVT.numSV
#define GPS_GET_LON gpsData.data->navPVT.lon/10000000.0f
#define GPS_GET_LAT gpsData.data->navPVT.lat/10000000.0f
#define GPS_GET_ALT gpsData.data->navPVT.hMSL/1000.0f
#define GPS_GET_PDOP gpsData.data->navPVT.pDOP/100.0f
//#define GPS_GET_ACC gpsData.data->navPVT.hAcc/1000.0f
#define GPS_GET_YR  gpsData.data->navPVT.year
#define GPS_GET_MO  gpsData.data->navPVT.month
#define GPS_GET_DY  gpsData.data->navPVT.day
#define GPS_GET_HR  gpsData.data->navPVT.hour
#define GPS_GET_MI  gpsData.data->navPVT.minute
#define GPS_GET_SE  gpsData.data->navPVT.second
#define GPS_TIM_VALID gpsData.data->navPVT.valid >= GPS_DATE_TIME_VALID
#define GPS_IS_NAV_DATA gpsData.type == GPS_DT_NAV_PVT
#define GPS_GOT_2D_FIX gpsData.data->navPVT.fixType == GPS_FIX_TYPE_UBX_2D
#define GPS_GOT_3D_FIX gpsData.data->navPVT.fixType == GPS_FIX_TYPE_UBX_3D
#define GPS_MSG_ACK_T ACK_ACK_NACK_t
#define GPS_MSG_ACK   ack
#endif

#define RESET_GPS_DATA gpsData.valid = false; gpsData.type = GPS_DT_NONE;


#define GPS_MSG_SYNC_LEN 2
#ifdef USE_CASIC_GPS
#define GPS_MSG_CK_LEN 4
#else
#define GPS_MSG_CK_LEN 2
#endif
#define GPS_MSG_HEAD_LEN 4
#define GPS_MSG_IGNORE_LEN 0xffff
#define GPS_MIN_EPH_DATA 5
#define GPS_MIN_AOP_DATA 5
#define GPS_LOCATION_AVG_CNT 10

typedef struct _locationSet_t{
  double   lng;
  double   lat;
  double   alt;
  double   pdop;
  int      gps_sat;
}locationSet_t;

typedef enum _gpsStatusType_t {
  GPS_NEVER_GOT_LOCK = 0,
  GPS_LESS_EHP_DATA,
  GPS_GOT_EHP_DATA,
  #ifdef SAVE_AOP_DATA
  GPS_GOT_AOP_DATA,
  #endif
  GPS_GOT_2D_LOCK,
  GPS_GOT_3D_LOCK
}gpsStatusType_t;

typedef enum _gpsWaitMsgRc_t {
  GPS_WAIT_MSG_OK = 0,
  GPS_WAIT_MSG_TIMEOUT,
  GPS_WAIT_MSG_NACK
}gpsWaitMsgRc_t;

//ublox UBX declarations
  typedef enum _gpsDataType_t {
    GPS_DT_NONE,
    GPS_DT_WRONG_LEN,
    GPS_DT_NAV_PVT,
    GPS_DT_NAV_POSLLH,
    GPS_DT_NAV_STATUS,
    GPS_DT_CFG_PRT,
    GPS_DT_CFG_NAVX5,
    GPS_DT_CFG_MSG,
    GPS_DT_CFG_POLLMSG,
    GPS_DT_CFG_RATE,
    GPS_DT_ACK,
    GPS_DT_NACK,
    GPS_DT_AID_EPH,
    GPS_DT_AID_AOP_DATA,
    GPS_DT_NAV_AOP_STATUS,
    GPS_DT_CASIC_NAV_STATUS,
    GPS_DT_CASIC_NAV_DOP,
    GPS_DT_CASIC_NAV_PV,
    GPS_DT_CASIC_NAV_TIMEUTC,
    GPS_DT_TEST_NEVER_GET_ONE
  }gpsDataType_t;

  #define GPS_UBX_PROTO_UBX  0x0001
  #define GPS_UBX_PROTO_NMEA 0x0002
  #define GPS_CASIC_PROTO_IN_BIN  0x0001
  #define GPS_CASIC_PROTO_IN_TXT  0x0002
  #define GPS_CASIC_PROTO_OUT_BIN  0x0010
  #define GPS_CASIC_PROTO_OUT_TXT  0x0020

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
  #define GPS_CASIC_CLASS 0
  typedef enum _gps_casic_class_t{
    GPS_CASIC_CLASS_NAV  = 0x01,
    GPS_CASIC_CLASS_TIM  = 0x02,
    GPS_CASIC_CLASS_RGPSsendMessageXM  = 0x03,
    GPS_CASIC_CLASS_ACK  = 0x05,
    GPS_CASIC_CLASS_CFG  = 0x06,
    GPS_CASIC_CLASS_MEAS = 0x07,
    GPS_CASIC_CLASS_MSG  = 0x08,
    GPS_CASIC_CLASS_MON  = 0x0A,
    GPS_CASIC_CLASS_AID  = 0x0B
  }gps_casic_class_t;
  typedef enum _gps_class_t{
    GPS_CLASS_NAV  = 0x01,
    GPS_CLASS_ACK  = 0x05,
    GPS_CLASS_CFG  = 0x06
  }_gps_class_t;

  #define GPS_UBX_ID 1
  typedef enum _gps_ubx_nav_id_t{
    GPS_UBX_NAV_POSECEF = 0x01,
    GPS_UBX_NAV_POSLLH  = 0x02,
    GPS_UBX_NAV_STATUS  = 0x03,
    GPS_UBX_NAV_PVT     = 0x07,
    GPS_UBX_NAV_AOP_ST  = 0x60
  }gps_ubx_nav_id_t;
  #define GPS_CASIC_ID 1
  typedef enum _gps_casic_nav_id_t{
    GPS_CASIC_NAV_STATUS  = 0x00,
    GPS_CASIC_NAV_DOP     = 0x01,
    GPS_CASIC_NAV_PV      = 0x03,
    GPS_CASIC_NAV_TIMEUTC = 0x10,
    GPS_CASIC_NAV_GPSINFO = 0x20
  }gps_casic_nav_id_t;
  typedef enum _gps_ack_id_t{
    GPS_ACK_NACK = 0x00,
    GPS_ACK_ACK  = 0x01
  }gps_ack_id_t;

  typedef enum _gps_ubx_cfg_id_t{
    GPS_UBX_CFG_PRT   = 0x00,
    GPS_UBX_CFG_MSG   = 0x01,
    GPS_UBX_CFG_INF   = 0x02,
    GPS_UBX_CFG_RATE  = 0x08,
    GPS_UBX_CFG_NAVX5 = 0x23
  }gps_ubx_cfg_id_t;
  typedef enum _gps_casic_cfg_id_t{
    GPS_CASIC_CFG_PRT     = 0x00,
    GPS_CASIC_CFG_MSG     = 0x01,
    GPS_CASIC_CFG_RATE    = 0x04,
    GPS_CASIC_CFG_POLLMSG = 0x10
  }gps_casic_cfg_id_t;
  typedef enum _gps_cfg_id_t{
    GPS_CFG_PRT = 0x00,
    GPS_CFG_MSG = 0x01,
  }gps_cfg_id_t;


  typedef enum _gps_ubx_aid_id_t{
    GPS_UBX_AID_EPH     = 0x31,
    GPS_UBX_AID_AOP_DT  = 0x33,
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
  #ifdef USE_CASIC_GPS
  const uint8_t GPS_MSG_SYNC[GPS_MSG_SYNC_LEN] = { 0xBA, 0xCE };
  #else
  const uint8_t GPS_MSG_SYNC[GPS_MSG_SYNC_LEN] = { 0xB5, 0x62 };
  #endif

  typedef enum _gps_datetime_valid_t{
    GPS_DATE_TIME_NOT_VAGPSsendMessageLID   = 0x00,
    GPS_DATE_TIME_DATE_VALID  = 0x01,
    GPS_DATE_TIME_TIME_VALID  = 0x02,
    GPS_DATE_TIME_VALID_CASIC = 0x03,
    GPS_DATE_TIME_RESOLVED    = 0x04,
    GPS_DATE_TIME_VALID       = 0x07
  }gps_datetime_valid_t;

  typedef enum _gps_fix_type_t{
    GPS_FIX_TYPE_NO_FIX   = 0x00,
    GPS_FIX_TYPE_UBX_2D   = 0x02,
    GPS_FIX_TYPE_UBX_3D   = 0x03,
    GPS_FIX_TYPE_CASIC_FAST = 0x05,
    GPS_FIX_TYPE_CASIC_2D   = 0x06,
    GPS_FIX_TYPE_CASIC_3D   = 0x07,

  }gps_fix_type_t;


  #ifdef USE_CASIC_GPS
  typedef __attribute__((__packed__)) struct MSG_HEAD_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
   };
  #else
  typedef __attribute__((__packed__)) struct MSG_HEAD_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
  };
  #endif
  typedef MSG_HEAD_t MSG_request_t;


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
  typedef __attribute__((__packed__)) struct CNAV_STATUS_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint32_t runTimeMs;
    uint16_t fixInterval;
    uint8_t  posValid;
    uint8_t  velValid;
    uint8_t  ephGPS[32];
    uint8_t  ephGLN[24];
    uint8_t  ephBDS[14];
    uint8_t  gpsUtcionFlag;
    uint8_t  bdsUtcionFlag;
  };
  typedef __attribute__((__packed__)) struct CNAV_DOP_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint32_t runTimeMs;
    uint32_t pDOP;
    uint32_t hDOP;
    uint32_t vDOP;
    uint32_t nDOP;
    uint32_t eDOP;
    uint32_t tDOP;
  };
  typedef __attribute__((__packed__)) struct CNAV_TIMEUTC_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint32_t runTimeMs;
    float    tAcc;          // Time accuracy estimate (UTC) (ns)
    float    msErr;
    uint16_t ms;
    uint16_t year;          // Year (UTC)
    uint8_t  month;         // Month, range 1..12 (UTC)
    uint8_t  day;           // Day of month, range 1..31 (UTC)
    uint8_t  hour;          // Hour of day, range 0..23 (UTC)
    uint8_t  minute;        // Minute of hour, range 0..59 (UTC)
    uint8_t  second;        // Seconds of minute, range 0..60 (UTC)
    uint8_t  valid;         // Validity Flags (see graphic below)
    uint8_t  timeSrc;       // Time source
    uint8_t  reserved;
  };
  typedef __attribute__((__packed__)) struct CNAV_PV_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint32_t runTimeMs;
    uint8_t  posValid;
    uint8_t  velValid;
    uint8_t  system;
    uint8_t  numSV;
    uint8_t  numSVGPS;
    uint8_t  numSVBDS;
    uint8_t  numSVGLONASS;
    uint8_t  res; //16
    uint32_t pDOP;
    uint32_t lon1;
    uint32_t lon2;
    uint32_t lat1;
    uint32_t lat2;
    uint32_t alt; //40
    uint32_t sepGEOid;
    uint32_t hAcc;
    uint32_t vAcc;
    uint32_t velN;
    uint32_t velE;
    uint32_t velU;
    uint32_t speed3D;
    uint32_t speed2D;
    uint32_t head;
    uint32_t sAcc;
    uint32_t cAcc; //80
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

  typedef __attribute__((__packed__)) struct CCFG_PRT_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint8_t  portID;
    uint8_t  protoMask;
    uint16_t Modus;
    uint32_t baudRate;
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
  typedef __attribute__((__packed__)) struct NAV_AOPSTATUS_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint32_t iTOW;
    uint8_t  aopCfg;
    uint8_t  status;
    uint8_t  noInterest01[14];
  };
  typedef __attribute__((__packed__)) struct ACK_ACK_NACK_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint8_t  clsID;
    uint8_t  msgID;
  };
  typedef __attribute__((__packed__)) struct CACK_ACK_NACK_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint8_t  clsID;
    uint8_t  msgID;
    uint16_t reserved;
  };
  typedef __attribute__((__packed__)) struct CCFG_POLLMSG_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint8_t  clsID;
    uint8_t  msgID;
    uint16_t reserved;
  };
  typedef __attribute__((__packed__)) struct CFG_MSG_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint8_t  clsID;
    uint8_t  msgID;
    uint8_t  i2cRate; //set to zero
    uint8_t  rate; //UART2
    uint32_t otherPorts; //set to zero
  };

  typedef __attribute__((__packed__)) struct CFG_RATE_t {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint16_t period_ms;
    uint16_t navRate;
    uint16_t timeRef;
  };

  typedef __attribute__((__packed__)) struct CCFG_MSG_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint8_t  clsID;
    uint8_t  msgID;
    uint16_t rate;
  };

  typedef __attribute__((__packed__)) struct CCFG_RATE_t {
    uint16_t len;
    uint8_t  cls;
    uint8_t  id;
    uint16_t period_ms;
    uint16_t reserved;
  };

  typedef struct MessageDebug_t { uint32_t* buf[20]; };

  union GPSMessage_t {
    MSG_HEAD_t      head; //each message has this head - this union member is just for generic access

    #ifndef USE_CASIC_GPS
    NAV_POSLLH_t    navPosllh;
    NAV_STATUS_t    navStatus;
    NAV_PVT_t       navPVT;
    NAV_AOPSTATUS_t navAOPstatus;
    UART_CONFIG_t   UARTconf;
    NAVX5_CONFIG_t  NAVX5conf;
    AID_EPH_DATA_t  AIDephData;
    AID_AOP_DATA_t  AIDaopData;
    CFG_MSG_t       cfgMSG;
    CFG_RATE_t      cfgRATE;
    #endif
    ACK_ACK_NACK_t  ack;
    #ifdef USE_CASIC_GPS
    CNAV_STATUS_t   CASnavStatus;
    CNAV_DOP_t      CASnavDOP;
    CNAV_PV_t       CASnavPV;
    CNAV_TIMEUTC_t  CASnavTimeUTC;
    CCFG_PRT_t      CAScfgPRT;
    CCFG_POLLMSG_t  CAScfgPOLLMSG;
    CCFG_MSG_t      CAScfgMSG;
    CCFG_RATE_t     CAScfgRATE;
    CACK_ACK_NACK_t CASack;
    #endif
    uint32_t        debug[20];
  };

  struct gpsData_t {
    bool           valid;
    gpsDataType_t  type;
    GPSMessage_t*  data;
  };

bool GPSGetPosition(t_SendData* DataSet, float required_pDOP, int maxWorse, int timeOut);
bool setBootTimeFromGPSTime(tm* bootTime, int timeOut);
void SetSysToGPS();
bool openGPS();
void closeGPS();
bool GPSprocessData(int timeOut);
void GPSsendMessage(uint8_t* message, int len);
void GPScalcChecksum(uint8_t* buffer, uint8_t* CK);
bool GPSdecodeData(uint8_t data);
gpsWaitMsgRc_t GPSwaitForMessage(gpsDataType_t type, int timeout, bool just_check_ack = false);
bool GPSsetupUART(int newBaudRate);
bool GPSsetupMessages(void);
void setGPSserialBaudRate(uint32_t rate, bool initialSetup = false);
#ifndef USE_CASIC_GPS
bool GPSenableAOP(void);
void GPSsaveAOPdata(void);
int  GPSuploadAOPdata(void);
#ifdef USE_AOP_STATUS
uint8_t GPSAOPdataStatus(void);
#endif
#endif
uint8_t GPSEphermerisDataStatus(void);
gpsDataType_t GPSgetPackageType(uint8_t cls, uint8_t id, uint16_t len);
bool GPSmsgNeedAck(gpsDataType_t type);
#if (DEBUG_LEVEL > 0 )
void _PrintDataGPS(void);
void testGPS(void);
#endif
#ifdef USE_CASIC_GPS
void GPSrawWriteString(String message);
#endif
#ifdef HEIDI_CONFIG_TEST
void GPSrawWriteMessage(uint8_t* message, int len);
#endif
#if (DEBUG_LEVEL > 0 )
#ifdef DEBUG_SERIAL_GPS
void GPSprintBootData(void);
#endif
#endif

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
