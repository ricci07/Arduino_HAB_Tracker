#include <SoftwareSerial.h>
#include <string.h>
#include <TinyGPS++.h>
#include <math.h>

SoftwareSerial mySerial(5, 4); // RX, TX
TinyGPSPlus gps;

#define BUFFER_SIZE 82


/*STIRK CODE
byte GPSBuffer[BUFFER_SIZE];
byte GPSindex = 0;
*/

unsigned int GPS_Satellites;
unsigned int GPS_Time;
unsigned int GPS_Date;
double GPS_Altitude;
double GPS_Speed;
double GPS_Course;
double GPS_Latitude;
double GPS_Longitude;

//Type in the launch location's GPS coords
const double LAUNCH_LOCATION_LAT = 52.682439; //Telford campus field
const double LAUNCH_LOCATION_LGN = -2.424624; //Telford campus field

//Type in your home location's GPS coords
const double HOME_LOCATION_LAT = 52.592296;
const double HOME_LOCATION_LGN = -2.192684;

int counter = 0;
 
void setup()
{
Serial.begin(9600); //hardware serial
mySerial.begin(9600); //software serial
mySerial.println("Ublox M8Q Data Parsing");
}
 
void loop(){

    if (Serial.available()){

      mySerial.write(Serial.read());
    
      //if (gps.encode(Serial.read())){

       // if (gps.location.isValid()){
        
          //mySerial.print(counter++);
          //mySerial.print(": ");
          //mySerial.println(gps.location.lat(), 6);

          
        //}
      //}
    }
    
  
  /*
  while (Serial.available()){
    if (gps.encode(Serial.read())){
 

/*anthony stike txtstring
    sprintf(txstring, "$$$$$AVA,%i,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%i",count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,rfm_temp);
    sprintf(txstring, "%s,%i",txstring,errorstatus);
    sprintf(txstring, "%s*%04X\n", txstring, gps_CRC16_checksum(txstring));
    
      if (gps.location.isValid()){
        //http://arduiniana.org/libraries/tinygpsplus/

       GPS_Satellites = gps.satellites.value();
       GPS_Altitude = gps.altitude.meters();
       GPS_Time = gps.time.value();
       GPS_Speed = gps.speed.mph();
       GPS_Course = gps.course.deg();
       GPS_Date = gps.date.value();
       GPS_Latitude = gps.location.lat();
       GPS_Longitude = gps.location.lng();

       double distanceMiles; //obtain current distance from launch location
        distanceMiles = TinyGPSPlus::distanceBetween(
        GPS_Latitude, 
        GPS_Longitude, 
        LAUNCH_LOCATION_LAT, 
        LAUNCH_LOCATION_LGN) / (1000.0 * 1.609344);

       double courseTo; //obtain the heading/direction
        courseTo = TinyGPSPlus::courseTo(
        GPS_Latitude,
        GPS_Longitude,
        LAUNCH_LOCATION_LAT,
        LAUNCH_LOCATION_LGN);

       // mySerial.print("Invalid GPS location");
       
       char* cardinal;
       cardinal = TinyGPSPlus::cardinal(courseTo);
       
      }
      }
    }
    */
  }








  
//checkGPS();
//if (Serial.available()){
//mySerial.write(Serial.read());
//}
//}

void checkGPS(){


  /*anthony stirk code
  int inByte;

  while (Serial.available() > 0){

    inByte = Serial.read();

    if ((inByte=='$') || (GPSindex>=80)){
      GPSindex=0;
    }

    if (inByte != '\r'){
      GPSBuffer[GPSindex++]=inByte;
    }

    if (inByte == '\n'){
      processGPSLine();
      GPSindex=0;
    } 
  }
  */
}

/*
void printGNGGALine(){ //remember it is checking ascii!!!!!!!!!!!!!!

  for (int i = 0; i < BUFFER_SIZE; i++){ //loop through buffer and print each character in sequence
    mySerial.print((char)GPSBuffer[i]); //convert dec to char to get the letter
  }
}

void printGNGGALine(int start, int out){ //remember it is checking ascii!!!!!!!!!!!!!!

  if (out > BUFFER_SIZE)
    out = BUFFER_SIZE;
    
  for (int i = start; i < out; i++){ //loop through buffer and print each character in sequence
    mySerial.print((char)GPSBuffer[i]); //convert dec to char to get the letter
  }
}
*/

/*
void processGPSLine(){

  if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'N') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A')){

    //mySerial.println("GNGGA Detected!");
    
    
    printGNGGALine(); //print the GPSLINE detected this is temp, remove later
    //processGNGGACommand();

    mySerial.println(""); //new line
    mySerial.print("Altitude: ");
    mySerial.print(GPS_Altitude);
    mySerial.print(" ");
    
    mySerial.print("Satellites: ");
    mySerial.print(GPS_Satellites);
    mySerial.print(" ");
    
    mySerial.print("GPS FixMode: ");
    mySerial.print(GPS_FixMode);
    mySerial.print(" ");

    mySerial.print("GPS Time: " ); //UTC time
    //printGNGGALine(7, 11);
    mySerial.print(GPS_Time);

    mySerial.println(""); //new line

    }
  }
*/

/*
void processGNGGACommand(){
  int i, j, k, integerPart;
  unsigned int Altitude;
  
  integerPart = 1;
  GPS_Satellites = 0; //reset global varible
  Altitude=0; //temp local variable

/*
  Sentence structure can be seen at:
  https://www.u-blox.com/en/docs/UBX-13003221 (p123)
  struct: $xxGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,altUnit,sep,sepUnit,diffAge,diffStation*cs<CR><LF>
  a 'quality' of 0 means there is no GPS lock, 1 or 2 means 2D, 3D, Combined GNSS/Dead Reckoning fix
*/

/*
//Anthony Stirk Firmware Coce: https://github.com/Upuaut/Ava/blob/master/fc_PavaR7.ino

  for (i=7, j=0, k=0; (i<GPSindex) && (j<9); i++) //start at 7 to ignore '$GNGGA' loop untill 9th comma
  { 
    if (GPSBuffer[i] == ','){ //the sentence structure is delimited by a comma, we need to extract the data between them
      
      j++; //comma counter
      k=0; //k doesn't appear to be used
      integerPart=1; //used in altiude retrival
      
    }else{

      if (j == 2){ //GPS UTC time
        
        if (GPSBuffer[i] >= '0'){

          GPS_Time *= 10;
          GPS_Time += GPSBuffer[i] - '0'; //https://forum.arduino.cc/index.php?topic=44055.0 (conv byte to int

          //https://circuits4you.com/2018/03/09/how-to-convert-int-to-string-on-arduino/ shows *10 thing
          //'0' must be subtracted to convert byte to int
          //GPS_Time += (unsigned int)(GPSBuffer[i] - '0');

        }

      }else if(j == 5){ //GPS fix mode

        if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
         
          GPS_FixMode = (unsigned int)(GPSBuffer[i] - '0'); //without - 0 the result is weird. Not sure what it means though
     
        }
        
      }else if (j == 6){ //6th comma brings us to the satellite count in the GNGGA string (first char after 6th comma)
      
        //Satellites Count
        if((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){ //check if the number of sats in between 0 and 9 (0-12 possible)
        
          GPS_Satellites = GPS_Satellites * 10; //if it is mult by 10
          GPS_Satellites += (unsigned int)(GPSBuffer[i] - '0'); 
        }
        
      }else if(j == 8){ //8th comma brings us to the satellite count in the GNGGA string
      
        //Altitude
       if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9') && integerPart){
     
          Altitude = Altitude * 10;
          Altitude += (unsigned int)(GPSBuffer[i] - '0'); 
        
       }else{
        integerPart = 0;
       }
      }
    }
    GPS_Altitude = Altitude;
  }
}
*/


  