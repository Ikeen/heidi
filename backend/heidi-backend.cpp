#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"
#include "TinyGPS++.h"
#include <HardwareSerial.h>


#define SCK     5    // GPIO5  -- SX1278's SCK
#define LED     2    // GPIO2  -- LED
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

/*
 Europe:
         433,05 MHz - 434,79 MHz (ISM-Band Region 1)
         863,00 MHz - 870,00 MHz (SRD-Band Europa)
 North America:
         902,00 MHz - 928,00 MHz (ISM-Band Region 2)
*/
#define BAND 434500000.00 //#define BAND    868E6
/* LoRa data rate = spreadingFactor * (SignalBandwidth / 2^spreadingFactor) * 4 / codingRateDenominator [bps]*/
#define spreadingFactor 9 //7..12 +1 ~ +2.5 dB SNR
#define SignalBandwidth 62.5E3 // BW/2 ~ +3-4 dB SNR 
#define preambleLength  8
#define codingRateDenominator 8 //5;8 -> 4/5; 4/8
/* 1098 bps*/
#define ANALOG_PIN_0 36

#define RECEIVER
//#define SENDER

int analog_value = 0;

SSD1306 display(0x3c, 4, 15);
String rssi = "RSSI --";
String packSize = "--";
String packet ;

String voltage = "0.0 V";
int counter;
int GpsCnt;
int loopCnt;

unsigned char buffer[256];
int bufcount = 0;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device 
// -> remember the pin modification in HardwareSerial.cpp
HardwareSerial GPSserial(1);

void displayLocationData(double latt, double lngg)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  String DisplayData="Position\nLat: " + String(latt, 6) + "\nLon: " + String(lngg, 6) + "\n";
  display.drawString(4 , 1 , DisplayData);
  display.display();
}

void setup()
{
  pinMode(LED,OUTPUT);
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、

  Serial.print("LoRa init ");
  Serial.begin(9600);
  while(!Serial) { Serial.print("."); }
  Serial.println(".. done");

  Serial.print("GPS init ");
  GPSserial.begin(9600);
  while(!GPSserial) { Serial.print("."); }
  Serial.println(".. done");

  Serial.print("Starting LoRa at ");
  Serial.print(String(BAND/1E6,DEC));
  Serial.print("MHz ");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println(" failed!");
    while (1);
  }
  Serial.println(".. done");

  Serial.println("Configuring LoRa");
  Serial.print(" - spreading factor: ");
  Serial.println(spreadingFactor);
  LoRa.setSpreadingFactor(spreadingFactor);
  Serial.print(" - signal bandwidth: ");
  Serial.println(SignalBandwidth);
  LoRa.setSignalBandwidth(SignalBandwidth);
  Serial.print(" - code rate: 4/");
  Serial.println(codingRateDenominator);
  LoRa.setCodingRate4(codingRateDenominator);
  Serial.print(" - preamble length: ");
  Serial.println(preambleLength);
  LoRa.setPreambleLength(preambleLength);
  Serial.println("done");

  //LoRa.receive();

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  #ifdef SENDER
  display.drawString(5,5,"LoRa Sender");
  #endif
  #ifdef RECEIVER
  display.drawString(5,5,"LoRa Receiver");
  #endif
  display.display();
  display.display();

  counter = 0;
  GpsCnt  = 0;
  loopCnt = 0;
  delay(4000);
}
//receiver
#ifdef RECEIVER
void loop()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packets
    Serial.print("Received packet. ");
    display.clear();
    
    // read packet and display
    String data = "";
	while (LoRa.available()) {
      data += (char) LoRa.read();
	}
    display.setFont(ArialMT_Plain_10);
    display.drawStringMaxWidth(0 , 5 , 128, data);
    display.display();
    Serial.print(data);
    
    // print RSSI of packet
    Serial.print(" with payload length ");
    Serial.println(packetSize);
    Serial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());
    Serial.print(" with SNR ");
    Serial.println(LoRa.packetSnr());
  }
  delay(100);
}
#endif

//sender
#ifdef SENDER
void loop()
{
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
      displayLocationData(gps.location.lat(), gps.location.lng());
      GpsCnt++;
      loopCnt = 0;
	  if (GpsCnt>=10) {
        digitalWrite(LED, HIGH); 
		counter++;
        LoRa.beginPacket();
        LoRa.print("Position\nLat: " + String(gps.location.lat(), 6) + "\nLon: " + String(gps.location.lng(), 6) + "\nPacket: " + String(counter));
        LoRa.endPacket(); //ret = 1:success | 0: fail
		GpsCnt = 0;
        digitalWrite(LED, LOW); 
		while (GPSserial.available() > 0){ GPSserial.read(); } // empty buffe
      }
    }
  }
  loopCnt++;
  if (loopCnt>=10) {
    displayLocationData(0.0, 0.0);
    digitalWrite(LED, HIGH);
    counter++;
    LoRa.beginPacket();
    LoRa.print("No GPS data.\nPacket: " + String(counter));
    LoRa.endPacket(); //ret = 1:success | 0: fail
    digitalWrite(LED, LOW);
	loopCnt = 0;
  }
  delay(250);
}
#endif

/*read analog pin

  #define ANALOG_PIN_0 36
  
  analog_value = analogRead(ANALOG_PIN_0);
  voltage = String(analog_value/4095*3.3, DEC) + " V";

*/
