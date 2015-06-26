/*
 *	Very Light GPS Parsing Library
 *	by Phillip Schmidt
 *	v0.1
 *
 *	
 */


#ifndef UAV_GPS_h
#define UAV_GPS_h

#include <arduino.h>


#define _GPS_BUFFER_SIZE 128
#define _GPS_RawToRadians (3.14159265359f / 108000000.0f)
#define _GPS_RawToDegrees (1.0f / 600000.0f)
#define _GPS_Earth_Radius 6371009.0f


// *** GPS CONFIGURATION SENTENCES ***
#define _GPS_GGA_SENTENCE_ONLY "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

#define _GPS_10HZ_REPORT_RATE  "$PMTK220,100*2F"
#define _GPS_5HZ_REPORT_RATE   "$PMTK220,200*2C"

#define _GPS_57600_BAUD_RATE   "$PMTK251,57600*2C"
#define _GPS_115200_BAUD_RATE  "$PMTK251,115200*1F"


class UAV_GPS
{
	public:
		UAV_GPS();

		void input(int);

		
		bool newData();

		unsigned long timeGPS, dTimeGPS;
		float positionX, positionY, positionZ;	// distance from home position
		float HDOP;
		float altitudeMSL, geoidSeperation;
		
		byte fixID, satCount;

		
	private:
	
		void parse();
		void parseGGA();
	
		void convertTime();
		void convertLatitude();		// Latitude -  DDMM.MMMM
		void convertNorthSouth();	// N/S indicator (Latitude)
		void convertLongitude();	// Longitude -  DDMM.MMMM
		void convertEastWest();		// E/W indicator (Longitude)
		void convertFixID();		// Position Fix Indicator
		void convertSatCount();		// Satellites Used
		void convertHDOP();			// HDOP
		void convertAltitude();		// MSL Altitude
		void convertGeoidSeparation();// Geoid Separation
	
		byte fieldNumber;
		byte sentenceType;
		char fieldData[16];
		byte dataEnd;
	

		bool checksumPass();
		byte checkSum, checksumFinal;
		
		bool newDataReady;
		
		
		unsigned long timeGPS_old;
		
		long latRaw, longRaw;			// integer lat/long in minutes x 10000
		long latRawHome, longRawHome;	// integer lat/long in minutes x 10000, home point offset
		float heightHome;
		bool homeSet;
	
};

#endif