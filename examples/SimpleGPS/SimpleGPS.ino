#include <MyGPS.h>

MyGPS GPS;  // GPS parsing object

void setup() 
{
	Serial.begin(115200);  // programming serial port
	Serial1.begin(57600);  // serial port 2 - GPS
	
	delay(3000);
	Serial.println("*** BEGIN PROGRAM ***");
  
	Serial1.println( _GPS_GGA_SENTENCE_ONLY ); // Only GGA data
	Serial1.println( _GPS_5HZ_REPORT_RATE ); 
	// Serial1.println( _GPS_57600_BAUD_RATE );  // demonstration line

}

void loop() 
{
  
  while(Serial1.available())  // This code feeds data into the GPS parser, call regularly to avoid missing data
  {
    GPS.input(Serial1.read());  // Adjust the serial port to refelect the one used in your project
  }
  
  if(GPS.newData())
  {
    Serial.print(GPS.positionX, 7);
    Serial.print(" ");
    Serial.print(GPS.positionY, 7);
    Serial.print(" ");
    Serial.println(GPS.positionZ, 7);
  }
  
  delay(1);
}


