/*
 *	Very Light GPS Parsing Library
 *	by Phillip Schmidt
 *	v0.1
 *
 *	
 */

#include "UAV_GPS.h"

UAV_GPS::UAV_GPS()
{
	
	fieldNumber = 0;
	sentenceType = 0;
	dataEnd = 0;
	homeSet = 0;
	
}


void UAV_GPS::input(int in) 
{
	
	switch (char(in))
	{
		case '$':			// Start of sentence, reset variables
			sentenceType = 0;
			dataEnd      = 0;	
			checkSum     = 0;
			fieldNumber  = 0;
			newDataReady = 0;	// unset flag to prevent using variables while parsing/converting
			break;
			
		case '*':			// End of data
			checksumFinal = checkSum;// store observed checksum until expected checksum arrives
			parse();			// convert last data field
			break;
			
		case ',':			// field delimiter	-- convert each field as it comes in
			//Serial.println("");
			//Serial.print(fieldNumber);
			checkSum = checkSum ^ ',';	// compute checksum
			parse();
			break;		
		
		case '\n':			// End of sentence
			//Serial.println("");
			if(checksumPass())
			{
				newDataReady = 1;
			}
			break;
			
		default:			// Data
			checkSum = checkSum ^ in;	// compute checksum
			fieldData[dataEnd] = in;	// place character in temporary array, increment counter
			dataEnd++;
			dataEnd = dataEnd % 16;	// prevent array overrun with bad data

	}
	
}


void UAV_GPS::parse()
{
	switch(sentenceType)
	{
		case 1:
			parseGGA();
			break;
			
		case 2:
			//parseGLL();
			break;

		case 3:
			//parseGSA();
			break;

		case 4:
			//parseGSV();
			break;

		case 5:
			//parseRMC();
			break;

		case 6:
			//parseVTG();
			break;
			
		// Add more cases in here for more sentences
			
		case 9:
			break; // do nothing
			
		case 0:
			if(//fieldData[2] == 'G' &&
				 fieldData[3] == 'G' &&
				 fieldData[4] == 'A' )
			{
				sentenceType = 1;
			}
			else if(//fieldData[2] == 'G' &&
					//fieldData[3] == 'L' &&
					  fieldData[4] == 'L' )
			{
				sentenceType = 2;
			}
			else if(//fieldData[2] == 'G' &&
					  fieldData[3] == 'S' &&
					  fieldData[4] == 'A' )
			{
				sentenceType = 3;
			}
			else if(//fieldData[2] == 'G' &&
					//fieldData[3] == 'S' &&
					  fieldData[4] == 'V' )
			{
				sentenceType = 4;
			}
			else if(//fieldData[2] == 'R' &&
					//fieldData[3] == 'M' &&
					  fieldData[4] == 'C' )
			{
				sentenceType = 5;
			}
			else if(//fieldData[2] == 'V' &&
					//fieldData[3] == 'T' &&
					  fieldData[4] == 'G' )
			{
				sentenceType = 6;
			}
			else
			{
				sentenceType = 9;		// unknown sentence
				//Serial.println("Unknown Sentence");
			}
			break;
			
		default:
			break;
			
	}
	//Serial.println(sentenceType);
	
	fieldNumber++;	// increment field number
	dataEnd   = 0;	// reset data pointer
}

void UAV_GPS::parseVTG()
{
	//printField();
	if(dataEnd)	// skip if there is no data in the field
	{
		switch(fieldNumber)
		{
			case 1:		// Time
				convertHeading();	// True Heading
				break;
			
			//case 2:
				//  T = Reference "True"
				//break;
			
			case 3:
				convertHeading();	// Magnetic Heading
				break;
			
			//case 4:
				// M = Reference "Magnetic"
				//break;
			
			case 5:
				convertSpeedKnots();
				break;
			
			//case 6:
				// Speed units = knots
				//break;
				
			case 7:
				convertSpeedKPH();
				break;
				
			//case 8:
				// Speed units = Kph
				//break;
				
			//case 9:
				//convertMode();
				//break;

			default:
				break;
				
		}

	}
	
}



void UAV_GPS::parseGGA()
{
	//printField();
	if(dataEnd)	// skip if there is no data in the field
	{
		switch(fieldNumber)
		{
			case 1:		// Time
				convertTime();
				break;
			
			case 2:
				convertLatitude();
				break;
			
			case 3:
				convertNorthSouth();
				break;
			
			case 4:
				convertLongitude();
				break;
			
			case 5:
				convertEastWest();
				break;
			
			case 6:
				convertFixID();
				break;
				
			case 7:
				convertSatCount();
				break;
				
			case 8:
				convertHDOP();
				break;
				
			case 9:
				convertAltitude();
				break;
				
			//case 10:
				// altitude units (m)
				//break;
				
			case 11:
				convertGeoidSeparation();
				break;
				
			// case 12:
				// Geoid separation units (m)
				// break;
				
			// case 13:
				// Age of Diff. Corr.
				// break;				

			// case 14:
				// Diff. Ref. Station ID
				// break;	

			default:
				break;
				
		}

	}
	
}


void UAV_GPS::convertTime() // Time -  HHMMSS.SSS ===> timeGPS (time in ms since midnight - GMT)
{
	//Serial.write(fieldData, 10);
	timeGPS_old = timeGPS;	

	timeGPS  = (fieldData[0] - 48) * 36000000UL;	//  H x 10
	timeGPS += (fieldData[1] - 48) * 3600000UL;		//  H x 1
	timeGPS += (fieldData[2] - 48) * 600000UL;		//  M x 10
	timeGPS += (fieldData[3] - 48) * 60000UL;		//  M x 1
	timeGPS += (fieldData[4] - 48) * 10000UL;		//  S x 10
	timeGPS += (fieldData[5] - 48) * 1000UL;		//  S x 1
	// skip period
	timeGPS += (fieldData[7] - 48) * 100UL;			// ms x 100
	timeGPS += (fieldData[8] - 48) * 10UL;			// ms x 10
	timeGPS += (fieldData[9] - 48) ;				// ms x 1

	if(timeGPS > timeGPS_old)
	{
		dTimeGPS = timeGPS - timeGPS_old;
	}
	else
	{
		dTimeGPS = (timeGPS + 86400000UL) - timeGPS_old;	// add 24hr if new time is smaller than old time
	}
	
	// Serial.print(timeGPS);
	// Serial.print(" ");
}


void UAV_GPS::convertLatitude()	// Latitude -  DDMM.MMMM
{
	latRaw  = (fieldData[0] - 48) * 6000000L;	// deg x 10
	latRaw += (fieldData[1] - 48) * 600000L;	// deg x 1
	latRaw += (fieldData[2] - 48) * 100000L;	// min x 10
	latRaw += (fieldData[3] - 48) * 10000L;		// min x 1
	// skip period
	latRaw += (fieldData[5] - 48) * 1000L;		// min / 10
	latRaw += (fieldData[6] - 48) * 100L;		// min / 100
	latRaw += (fieldData[7] - 48) * 10L;		// min / 1000
	latRaw += (fieldData[8] - 48) ;				// min / 10000
}

void UAV_GPS::convertNorthSouth()	// N/S indicator (Latitude)
{
	if(fieldData[0] == 'S')
	{
		latRaw *= -1L;
	}
	
	// Serial.print(latRaw*_GPS_RawToRadians, 7);
	// Serial.print(" ");
}


void UAV_GPS::convertLongitude()// Longitude -  DDMM.MMMM
{	
	longRaw  = (fieldData[0] - 48) * 60000000L;	// deg x 100
	longRaw += (fieldData[1] - 48) * 6000000L;	// deg x 10
	longRaw += (fieldData[2] - 48) * 600000L;	// deg x 1
	longRaw += (fieldData[3] - 48) * 100000L;	// min x 10
	longRaw += (fieldData[4] - 48) * 10000L;	// min x 1
	// skip period
	longRaw += (fieldData[6] - 48) * 1000L;		// min / 10
	longRaw += (fieldData[7] - 48) * 100L;		// min / 100
	longRaw += (fieldData[8] - 48) * 10L;		// min / 1000
	longRaw += (fieldData[9] - 48) ;			// min / 10000	
}

void UAV_GPS::convertEastWest()	// E/W indicator (Longitude)
{
	if(fieldData[0] == 'W')
	{
		longRaw *= -1L;
	}
	
	// Serial.print(longRaw*_GPS_RawToRadians, 7);
	// Serial.print(" ");
	
}


void UAV_GPS::convertFixID()	// Position Fix Indicator
{
	fixID = fieldData[0] - 48;
	
	// Serial.print(fixID);
	// Serial.print(" ");
}

	
void UAV_GPS::convertSatCount()	// Satellites Used
{
	int i = dataEnd - 1;
	
	satCount = fieldData[i] - 48;
	
	if(i)
	{
		satCount += (fieldData[0] - 48) * 10;
	}
	
	// Serial.print(satCount);
	// Serial.print(" ");
}


void UAV_GPS::convertHDOP()	// HDOP
{
	fieldData[dataEnd] = '\0';	// add termination char
	HDOP = float(atof(fieldData));

	// Serial.print(HDOP, 2);
	// Serial.print(" ");
}


void UAV_GPS::convertAltitude()	// MSL Altitude
{
	fieldData[dataEnd] = '\0';	// add termination char
	altitudeMSL = float(atof(fieldData));

	// Serial.print(altitudeMSL, 2);
	// Serial.print(" ");
}


void UAV_GPS::convertGeoidSeparation()	// Geoid Separation
{
	fieldData[dataEnd] = '\0';	// add termination char
	geoidSeperation = float(atof(fieldData));

	// Serial.print(geoidSeperation, 2);
	// Serial.print(" ");
}



bool UAV_GPS::newData()
{
	if(newDataReady)
	{
		newDataReady = 0;
		return 1;
	}
	
	return 0;
}

bool  UAV_GPS::checksumPass()
{
	
	static byte checkNum;	// expected checksum value
	
	if(fieldData[0] < 58)	// compute expected checksum -- I don't think this will ever get above 9... need to confirm
	{
		checkNum = (fieldData[0] - 48) << 4;
	}
	else
	{
		checkNum = (fieldData[0] - 55) << 4;
	}
	
	if(fieldData[1] < 58)
	{
		checkNum += fieldData[1] - 48;
	}
	else
	{
		checkNum += fieldData[1] - 55;
	}
	
	// Serial.println(checksumFinal, DEC);
	// Serial.println(checkNum, DEC);
	// Serial.println("");
	
	if(checkNum == checksumFinal)
	{
		//Serial.println("");
		if(fixID == 1 || fixID == 2)	// set new data flag if fix is valid
		{
			if(homeSet)
			{
				positionX = float(latRaw  - latRawHome)  * _GPS_RawToRadians * _GPS_Earth_Radius;
				positionY = float(longRaw - longRawHome) * _GPS_RawToRadians * _GPS_Earth_Radius;
				positionZ = altitudeMSL - heightHome;
				
				newDataReady = 1;			
			}
			else
			{
				static byte count = 0;
				latRawHome  = latRaw;
				longRawHome = longRaw;
				heightHome  = altitudeMSL;
				count++;
				
				if(count > 20)
				{
					homeSet = 1;
				}
			}
		}
		
		return 1;
	}
	
	Serial.println("CHECKSUM FAILURE");
	return 0;
}

