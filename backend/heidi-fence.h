/*
 * heidi-fence.h
 *
 *  Created on: 28.10.2020
 *      Author: frank
 */

#ifndef HEIDI_FENCE_H_
#define HEIDI_FENCE_H_
#include <math.h>
#include "heidi-defines.h"
#include "heidi-data.h"

#define EMPTY_POINT_VALUE 400000000 //irregular GPS value

typedef struct  _t_PointData{
  double  latitude;
  double  longitude;
}t_PointData;

typedef struct _t_intercept_data{
  t_PointData p1; //POI
  t_PointData p2; //point outside
  t_PointData p3; //segment of fence p1
  t_PointData p4; //segment of fence p2
  double a;
  double b;
  double c;
  double d;
  double e;
  double f;
  double p;
  double s;
  double t;
  //see ods sheet
}t_intercept_data;

typedef struct _t_distance_data{
  t_PointData p1; //POI
  t_PointData p3; //position vector of fence segment p1
  t_PointData p4; //segment of fence p2
  t_PointData v1; //directional vector of fence segment
  t_PointData v2; //vector
  t_PointData p5; //foot point
  double a;
  double b;
  double r;
  double d1;
  double d2;
  double d3;
  double d;
  //see ods sheet
}t_distance_data;

typedef enum {
  fcrc_ok_same,
  fcrc_ok_new,
  fcrc_failed
} fchkrc_t;

bool setFenceFromHTTPresponse(String response);
void clearFence();
int  addFencePoint(double laditude, double longitude);
void newFenceB64(String b64);
fchkrc_t checkFenceB64(String b64);
bool pointIn(t_PointData* point);
int  meterDistFromFence(t_PointData* point);
bool getPointOutside(t_PointData* point);
bool emptyPoint(t_FenceData* point);
void testGeoFencing();
void _PointToFence(t_PointData* from, t_FenceData* to);
void _FenceToPoint(t_FenceData* from, t_PointData* to);
void copyFencePole(t_FenceData* _from, t_FenceData* _to);
void copyPoint(t_PointData* _from, t_PointData* _to);


#endif /* HEIDI_FENCE_H_ */
