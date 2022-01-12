/*
 * heidi-fence.cpp
 *
 *  Created on: 28.10.2020
 *      Author: frank
 */

#include "heidi-debug.h"
#include "heidi-fence.h"
#include "heidi-sys.h"

bool setFenceFromHTTPresponse(String response)
{
  String fenceData = getCSVvalue(response, 2);
  String fenceCRC = getCSVvalue(response, 3);
  uint16_t c_crc = crc16F(fenceData);
  if ((fenceData.length() == 0) && (fenceData.length() == 0)) { return true; }
  if (fenceData.length() > 25){
    uint16_t t_crc = (uint16_t)hex2int(fenceCRC);
    if(t_crc == c_crc){
      fchkrc_t frc = checkFenceB64(fenceData);
      if (frc != fcrc_failed){
        if(frc == fcrc_ok_new){
          clrState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_1 | GPS_ALERT_PSD);
          clearFence();
          newFenceB64(fenceData);
          setState(NEW_FENCE);
          _D(DebugPrintln("new fence set", DEBUG_LEVEL_2));
        } _D(else DebugPrintln("new fence = old fence", DEBUG_LEVEL_2);)
        return true;
      } _D( else { DebugPrintln("check fence failed", DEBUG_LEVEL_1); } )
    } _D( else { DebugPrintln("fence CRC error", DEBUG_LEVEL_1); } )
  } _D( else { DebugPrintln("fence data error", DEBUG_LEVEL_1); } )
  return false;
}

void clearFence(){
  for(int i=0; i<FENCE_MAX_POS; i++){
    FenceDataSet[i]->latitude = EMPTY_POINT_VALUE;
    FenceDataSet[i]->longitude = EMPTY_POINT_VALUE;
  }
}

void newFenceB64(String b64){
  unsigned char buffer[256];
  int dataLen = b64Decode(b64, buffer);
  for(int i=1; i<dataLen; i+=8)
  {
    int lng = _copyBufferToInt32(buffer, i);
    int lat = _copyBufferToInt32(buffer, i+4);
    addFencePoint(double(lat) / 1000000, double(lng) / 1000000);
    _DD(DebugPrintln("set lat: " + String(double(lat) / 1000000,6) + ", lng: " + String(double(lng) / 1000000,6), DEBUG_LEVEL_3));
  }
  _D(DebugPrintln("Fence: "   + String(buffer[0]) + " poles set", DEBUG_LEVEL_2));
}

fchkrc_t checkFenceB64(String b64){
  /* we do not clear NEW_FENCE state here, if fence is not new. This is done when reaching its position */
  unsigned char buffer[256];
  int dataLen = b64Decode(b64, buffer);
  if ( buffer[0] < 3 ) { return fcrc_failed; }
  if ( buffer[0] > FENCE_MAX_POS ) { return fcrc_failed; }
  int p = 0;
  fchkrc_t rc = fcrc_ok_same;
  for(int i=1; i<dataLen; i+=8)
  {
    if(p >= FENCE_MAX_POS) { return fcrc_failed; }
    int lng = _copyBufferToInt32(buffer, i);
    int lat = _copyBufferToInt32(buffer, i+4);
    double dlat = double(lat) / 1000000;
    double dlng = double(lng) / 1000000;
    if (emptyPoint(FenceDataSet[p]) || (FenceDataSet[p]->latitude != GeoToInt(dlat)) || (FenceDataSet[p]->longitude != GeoToInt(dlng))){
      _D(
        if (emptyPoint(FenceDataSet[p])) { DebugPrintln("new fence by more than: "   + String(p-1) + " poles", DEBUG_LEVEL_2);} else {
          if (FenceDataSet[p]->latitude != GeoToInt(dlat)) { DebugPrintln("new fence by latitude: "   + String(FenceDataSet[p]->latitude) + " <> " + String(GeoToInt(dlat)), DEBUG_LEVEL_2);}
          if (FenceDataSet[p]->latitude != GeoToInt(dlat)) { DebugPrintln("new fence by longitude: "   + String(FenceDataSet[p]->longitude) + " <> " + String(GeoToInt(dlng)), DEBUG_LEVEL_2);}
        }
      )
      rc = fcrc_ok_new;
      break;
    }
    p++;
  }
  if(p < FENCE_MAX_POS){
    if (!emptyPoint(FenceDataSet[p])){ //1 point more...
      _D(DebugPrintln("new fence by less than: " + String(p) + " poles", DEBUG_LEVEL_2));
      rc = fcrc_ok_new;
    }
  }
  return rc;
}
int addFencePoint(double laditude, double longitude)
//returns new count of points, 0 if max was already reached
{
  int i=0;
  while(i<FENCE_MAX_POS){
    if(emptyPoint(FenceDataSet[i])) {break;}
    i++;
  }
  if(i == FENCE_MAX_POS){ return 0; }
  FenceDataSet[i]->latitude  = GeoToInt(laditude);
  FenceDataSet[i]->longitude = GeoToInt(longitude);
  _DD(DebugPrintln("Fence pole " + String(i) + ": " + String(FenceDataSet[i]->latitude,6) + ", " + String(FenceDataSet[i]->longitude,6), DEBUG_LEVEL_3));
  return i+1;
}

bool pointIn(t_PointData* point) //see ods sheet
{
  t_intercept_data inter;
  int intersections;
  bool noResult = true;

  if(invalidFence()){_DD(DebugPrintln("fence not set yet", DEBUG_LEVEL_3);) return false; }
  copyPoint(point, &inter.p1);
  if (!getPointOutside(&inter.p2)) { return false; }
  while (noResult){
    noResult = false;
    intersections = 0;

    _DD(DebugPrintln("P1: " + String(inter.p1.latitude, 5) + " ,    " + String(inter.p1.longitude, 5), DEBUG_LEVEL_3));
    _DD(DebugPrintln("P2: " + String(inter.p2.latitude, 5) + " ,    " + String(inter.p2.longitude, 5), DEBUG_LEVEL_3));

    inter.b = inter.p1.latitude  - inter.p2.latitude;
    inter.e = inter.p1.longitude - inter.p2.longitude;

    _DD(DebugPrintln("B : " + String(inter.b, 5) + " ,E : " + String(inter.e, 5), DEBUG_LEVEL_3));

    //check how often we cross the fence to get to the point outside
    for(int i=0; i<FENCE_MAX_POS; i++){
      if(emptyPoint(FenceDataSet[i])) {break;}
      _FenceToPoint(FenceDataSet[i], &inter.p3);
      if(i == (FENCE_MAX_POS-1))
      { _FenceToPoint(FenceDataSet[0], &inter.p4); }
      else if(emptyPoint(FenceDataSet[i+1]))
      { _FenceToPoint(FenceDataSet[0], &inter.p4); }
      else
      { _FenceToPoint(FenceDataSet[i+1], &inter.p4); }

      _DD(DebugPrintln("P3: " + String(inter.p3.latitude, 5) + " ,    " + String(inter.p3.longitude, 5), DEBUG_LEVEL_3));
      _DD(DebugPrintln("P4: " + String(inter.p4.latitude, 5) + " ,    " + String(inter.p4.longitude, 5), DEBUG_LEVEL_3));

      inter.a = inter.p4.latitude  - inter.p3.latitude;
      inter.d = inter.p4.longitude - inter.p3.longitude;
      inter.c = inter.p1.latitude  - inter.p3.latitude;
      inter.f = inter.p1.longitude - inter.p3.longitude;

      _DD(DebugPrintln("A : " + String(inter.a, 5) + " ,C : " + String(inter.c, 5), DEBUG_LEVEL_3));
      _DD(DebugPrintln("D : " + String(inter.d, 5) + " ,F : " + String(inter.f, 5), DEBUG_LEVEL_3));

      inter.p = (inter.a * inter.e) - (inter.d * inter.b);
      if(inter.p == 0.0){ continue; } //parallel or 1 segment has length 0 - no intersection
      inter.s = ((inter.c * inter.e)-(inter.f * inter.b)) / inter.p;
      inter.t = ((inter.a * inter.f)-(inter.d * inter.c)) / inter.p;

      _DD(DebugPrintln("P : " + String(inter.p, 5), DEBUG_LEVEL_3));
      _DD(DebugPrintln("S : " + String(inter.s, 5) + " ,T : " + String(inter.t, 5), DEBUG_LEVEL_3));

      if((inter.s > 0.0) && (inter.s < 1.0) &&  (inter.t > 0.0) && (inter.t < 1.0)) {intersections++; _DD(DebugPrintln("Intersection.", DEBUG_LEVEL_3)); }
      _DD(else { DebugPrintln("No Intersection.", DEBUG_LEVEL_3); })
      if((inter.s == 0.0) || (inter.s == 1.0) ||  (inter.t == 0.0) || (inter.t == 1.0))
      { //unfortunately we exactly met a pale on the way to the outside point - that means result would be indefinable
        //so we just shift the outside point a little bit to get another angle
        _DD(DebugPrintln("Pfosten!", DEBUG_LEVEL_3));
        if(inter.p1.latitude == inter.p2.latitude) //shifting longitude would make no sense
        { inter.p2.latitude -= 0.01; }
        else
        { inter.p2.longitude -= 0.01; }
        noResult = true;
        break;
      }
    }
  }
  //we are inside, if we cross the fence odd times
  return ((intersections % 2) == 1);
}

int meterDistFromFence(t_PointData* point) //see ods sheet
{
  t_distance_data dist;
  double distance = -1;
  double factorLat = 111300.0;
  double factorLon;
  _D(DebugPrintln("check distance: lat " + String(point->latitude, 6) + ", lon " + String(point->longitude, 6), DEBUG_LEVEL_2));
  copyPoint(point, &dist.p1);
  factorLon = cos(dist.p1.latitude * M_PI / 180) * factorLat;
  for(int i=0; i<FENCE_MAX_POS; i++){
    if(emptyPoint(FenceDataSet[i])) {break;}
    _FenceToPoint(FenceDataSet[i], &dist.p3);
    if(i == (FENCE_MAX_POS-1))
    { _FenceToPoint(FenceDataSet[0], &dist.p4); }
    else if(emptyPoint(FenceDataSet[i+1]))
    { _FenceToPoint(FenceDataSet[0], &dist.p4); }
    else
    { _FenceToPoint(FenceDataSet[i+1], &dist.p4); }
    dist.v1.latitude  = dist.p3.latitude  - dist.p4.latitude;
    dist.v1.longitude = dist.p3.longitude - dist.p4.longitude;
    dist.v2.latitude  = dist.p3.latitude  - dist.p1.latitude;
    dist.v2.longitude = dist.p3.longitude - dist.p1.longitude;
    dist.a = (dist.v1.latitude * dist.v2.latitude) + (dist.v1.longitude * dist.v2.longitude);
    dist.b = (dist.v1.latitude * dist.v1.latitude) + (dist.v1.longitude * dist.v1.longitude);
    _DD(DebugPrintln("A : " + String(dist.a, 10) + " ,B : " + String(dist.b, 10), DEBUG_LEVEL_3));
    if (dist.b != 0.0) //if == 0.0, segment has length 0 -> omit
    {
      dist.r = dist.a / dist.b;
      _DD(DebugPrintln("R : " + String(dist.r, 5), DEBUG_LEVEL_3));
      dist.p5.latitude  = dist.p3.latitude  - (dist.r * dist.v1.latitude);
      dist.p5.longitude = dist.p3.longitude - (dist.r * dist.v1.longitude);
      if((dist.r >= 0.0) && (dist.r <= 1.0)){ // foot point lies on segment
        double t1 = (dist.p5.latitude  - dist.p1.latitude)  * factorLat;
        double t2 = (dist.p5.longitude - dist.p1.longitude) * factorLon;
        t1 *= t1;
        t2 *= t2;
        dist.d = sqrt(t1+t2);
        _DD(DebugPrintln("Distance to foot point used.", DEBUG_LEVEL_3));
      } else {
        double t1 = (dist.p3.latitude  - dist.p1.latitude)  * factorLat;
        double t2 = (dist.p3.longitude - dist.p1.longitude) * factorLon;
        t1 *= t1;
        t2 *= t2;
        dist.d1 = sqrt((t1+t2));
        t1 = (dist.p4.latitude  - dist.p1.latitude)  * factorLat;
        t2 = (dist.p4.longitude - dist.p1.longitude) * factorLon;
        t1 *= t1;
        t2 *= t2;
        dist.d2 = sqrt((t1+t2));
        _DD(DebugPrintln("D1: " + String(dist.d1, 5) + " ,D2: " + String(dist.d2, 5), DEBUG_LEVEL_3));
        if( dist.d1 < dist.d2 ){ dist.d = dist.d1; }else{ dist.d = dist.d2; }
        _DD(DebugPrintln("Distance to fence segment end point used.", DEBUG_LEVEL_3));
      }
      _DD(DebugPrintln("distance " +String(i+1) + ": " + String(dist.d, 5), DEBUG_LEVEL_3));
      if(distance == -1) { distance = dist.d; }
      if(distance > dist.d) { distance = dist.d; }
    }
  }
  _D(DebugPrintln("final distance: " + String(distance, 5), DEBUG_LEVEL_2));
  return rint(distance);
}
bool getPointOutside(t_PointData* point)
{
  point->latitude = IntToGeo(FenceDataSet[0]->latitude);
  point->longitude = IntToGeo(FenceDataSet[0]->longitude);
  if((point->latitude == 0) && (point->longitude == 0)) {return false;}
  for (int i=1; i<FENCE_MAX_POS; i++){
    if(emptyPoint(FenceDataSet[i])){break;}
    if(IntToGeo(FenceDataSet[i]->latitude) < point->latitude)
      { point->latitude = IntToGeo(FenceDataSet[i]->latitude); }
    if(IntToGeo(FenceDataSet[i]->longitude) < point->longitude)
      { point->longitude = IntToGeo(FenceDataSet[i]->longitude); }
  }
  point->latitude  -= 0.05;
  point->longitude -= 0.05;
  return true;
}
bool emptyPoint(t_FenceData* point)
{
  if((point->latitude == EMPTY_POINT_VALUE) || (point->longitude == EMPTY_POINT_VALUE)){return true;}
  return false;
}

void _PointToFence(t_PointData* from, t_FenceData* to){
  to->latitude  = GeoToInt(from->latitude);
  to->longitude = GeoToInt(to->longitude);
}
void _FenceToPoint(t_FenceData* from, t_PointData* to){
  to->latitude  = IntToGeo(from->latitude);
  to->longitude = IntToGeo(from->longitude);
}

void copyFencePole(t_FenceData* _from, t_FenceData* _to)
{
  _to->latitude = _from->latitude;
  _to->longitude = _from->longitude;
}
void copyPoint(t_PointData* _from, t_PointData* _to)
{
  _to->latitude  = _from->latitude;
  _to->longitude = _from->longitude;
}

bool invalidFence(void){
  for (int i=0; i<FENCE_MAX_POS-1; i++){
    if(FenceDataSet[i]->latitude  != FenceDataSet[i+1]->latitude)  { return false; }
    if(FenceDataSet[i]->longitude != FenceDataSet[i+1]->longitude) { return false; }
  }
  //all points equal
  return true;
}

void testGeoFencing()
{
  _D(
  t_PointData poi;
  clearFence();

  addFencePoint(51.009199, 13.342415);
  addFencePoint(51.008855, 13.342501);
  addFencePoint(51.008831, 13.342844);
  addFencePoint(51.008909, 13.343155);

  //13.342487 51.009025

  poi.latitude = 51.009025;
  poi.longitude = 13.342487;

  if (pointIn(&poi)){
    DebugPrintln("Point IN.", DEBUG_LEVEL_1);
  } else {
    DebugPrintln("Point OUT.", DEBUG_LEVEL_1);
  }
  int32_t meter = meterDistFromFence(&poi);
  DebugPrintln(String(meter) + "m distance to fence", DEBUG_LEVEL_1);
  )
}

