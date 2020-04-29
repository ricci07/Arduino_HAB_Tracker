#include <SoftwareSerial.h> //for debugging only
#include <string.h>
#include <util/crc16.h>
#include <TinyGPS++.h>
#include <math.h>

SoftwareSerial mySerial(5, 4); // RX, TX for debugging only
TinyGPSPlus gps;

#define RADIOPIN 13
#define BUZZERPIN 12
#define OKLED 3
#define WARNLED 2
#define GPSLockPin 10; //needs to be confirmed

#define DEBUG; //comment out to prevent complimate of debug lines

unsigned int GPS_Satellites;
unsigned int GPS_Hour;
unsigned int GPS_Minute;
unsigned int GPS_Second;
unsigned int GPS_Date;
double GPS_Altitude;
double GPS_Speed;
double GPS_Course;
double GPS_Latitude;
double GPS_Longitude;

double GPS_Distance_To_Home_In_Miles;
double GPS_CourseTo;
char* GPS_Cardinial;

//Type in the launch location's GPS coords
const double LAUNCH_LOCATION_LAT = 52.682439; //Telford campus field
const double LAUNCH_LOCATION_LGN = -2.424624; //Telford campus field

//Type in your home location's GPS coords
const double HOME_LOCATION_LAT = 52.592296;
const double HOME_LOCATION_LGN = -2.192684;

//================================Settings===============================================

uint8_t bBuzzRtty = 0; //beep pcb buzzer in sequence with transmitted bits
uint8_t bBuzzBeacon = 0; //Bleep once at transmission start, bleep twice at transmission end
uint8_t bLEDBeacon = 0; // flash green led in the same way as BuzzBeacon
uint8_t bLEDRtty = 0;

//=======================================================================================

//char* myStrings[2] = {"The DATASTRING is passed to a procedure called rtty_txtstring which takes care of transmitting the data by breaking it down into characters, then it transmits the individual bits of those characters. The key to getting the baud rate correct is the timing. Theoretically 50 baud should be 1/50th of a second = 20000μ seconds. Now for some reason the Arduino (at least the one I have) doesn’t seem to be able to accurately count this time interval so the delay is achieved by doing one delay of 10000μS and one of 10150μS.", "Message 2"};

char datastring[80];

void setup() {
                  
  pinMode(RADIOPIN,OUTPUT);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(OKLED, OUTPUT);
  pinMode(WARNLED, OUTPUT);
  Serial.begin(9600); //hardware serial for GPS
  mySerial.begin(19200); //software serial for gps debugging
  setupGPS(); //turn off unneeded NMEA messages
  mySerial.println("Debugging Started...");
}
 
void loop() {


  if (Serial.available()){

    //mySerial.write(Serial.read()); //uncomment to see if GPS is outputting normal!!
    
    if (gps.encode(Serial.read())){
      if (gps.passedChecksum()){

       if (gps.location.isUpdated() && gps.altitude.isUpdated()){
       setGPSVariables(); //set all the global variables to the tinyURL parsed GNGGA info   
       buildTelemetryString(); // now start using them to build Tx string THIS MUST ONLY BE EXE when we have data avail i.e it must be inside this if statement or it will not work!!!!!!
      
        mySerial.print("Lat: ");
        mySerial.print(GPS_Latitude, 6);
        
        mySerial.print(" Long: ");
        mySerial.print(GPS_Longitude, 6);
        mySerial.print(" Alt: ");
        mySerial.print(GPS_Altitude);
        mySerial.println();

        //mySerial.print("\"");
        //mySerial.print(conv_doub_to_int(GPS_Latitude, 1));
        //mySerial.print("\"");
        //mySerial.println(); //newline

        }else{ //if not gps loc and gps alt
          mySerial.println("Waiting for updated GPS values!");
        }
      }else if (gps.failedChecksum()){
        mySerial.println("Checksum failed");
      }
    
    
      //if (gps.location.isValid()){

        //int lat_tmp = conv_doub_to_int(GPS_Latitude, 0);
        //sprintf(datastring, "Latitude:%i", lat_tmp);
        
       // mySerial.println(datastring);
       //mySerial.println(GPS_Latitude, 6);
       //}
    }
    
  }
}
   

  if (bBuzzBeacon){
    digitalWrite(BUZZERPIN, HIGH); //before start of tranmission of bits flash once
    delay(50);
    digitalWrite(BUZZERPIN, LOW);
  }

  if (bLEDBeacon){
    digitalWrite(OKLED, HIGH); //before start of tranmission of bits flash once
    delay(50);
    digitalWrite(OKLED, LOW);
  }
  
if (bBuzzBeacon){
  for (uint8_t i=0; i<2; i++){
    digitalWrite(BUZZERPIN, HIGH); //after end of transmission of bits buzz twice
    delay(50);
    digitalWrite(BUZZERPIN, LOW);
    delay(50);
  }
}

if (bLEDBeacon){
  for (uint8_t i=0; i<2; i++){
    digitalWrite(OKLED, HIGH); //after end of transmission of bits buzz twice
    delay(50);
    digitalWrite(OKLED, LOW);
    delay(50);
  }
}

 
void setupGPS(){ //turn off unuseful messages (non GCGGA)
  Serial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  delay(100);
  Serial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  delay(100);
  Serial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  delay(100);
  Serial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  delay(100);
  Serial.print("$PUBX,40,GSA,1,1,1,0*4E\r\n");
  delay(100);
  Serial.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
  delay(100);
}

void setGPSVariables(){ //set the global variables need to build tx string

  GPS_Satellites = gps.satellites.value();
  GPS_Altitude = gps.altitude.meters();
  GPS_Hour = gps.time.hour();
  GPS_Minute = gps.time.minute();
  GPS_Second = gps.time.second();
  GPS_Speed = gps.speed.mph();
  GPS_Course = gps.course.deg();
  GPS_Date = gps.date.value();
  GPS_Latitude = gps.location.lat();
  GPS_Longitude = gps.location.lng();

  //obtain the heading/direction from launch location
  GPS_CourseTo = TinyGPSPlus::courseTo(
  GPS_Latitude,
  GPS_Longitude,
  LAUNCH_LOCATION_LAT,
  LAUNCH_LOCATION_LGN);

  //obtain current distance from launch location
  GPS_Distance_To_Home_In_Miles = TinyGPSPlus::distanceBetween(
  GPS_Latitude, 
  GPS_Longitude, 
  LAUNCH_LOCATION_LAT, 
  LAUNCH_LOCATION_LGN) / (1000.0 * 1.609344); //conv km to miles

  //e.g NW SE SE NE etc.
  GPS_Cardinial = TinyGPSPlus::cardinal(GPS_CourseTo);
  
}

int32_t conv_doub_to_int(double input, char resPart){ // convert the decimal lat/long to integer parts that can be charred

    int integerPart = (int)input;
    double decimalPart = fmod(input,1);
    int32_t dPartInt = decimalPart*1000000;
    
   // double num_dec = input - (int)input; //get the decimal part e.g 0.564545
   // double  num_int = input-num_dec; //obtain the integer part
   // double num_dec_res = num_dec*1000000; //bring the decimal into integer (this was a pain to make work)

    return resPart == 0 ? integerPart : dPartInt; //return the int of dec part

}

void buildTelemetryString(){

   int lat_int = (int)GPS_Latitude;
   int32_t lat_dec;
   
   double decimalPart = fmod(GPS_Latitude,1);
   lat_dec = decimalPart*1000000;

   int lon_int = (int)abs(GPS_Longitude);
   int32_t lon_dec;

   double decPart = fmod(GPS_Longitude,1);
   lon_dec = abs(decPart)*1000000;

  //mySerial.println((int)GPS_Altitude);
  sprintf(datastring, "$$$RIC,%02d:%02d:%02d,%s%i.%ld,%s%i.%ld,%i", 
  GPS_Hour+1, GPS_Minute, GPS_Second, (GPS_Latitude < 0 ? "-" : ""), 
  lat_int, lat_dec, (GPS_Longitude < 0 ? "-" : ""), lon_int, lon_dec, 
  GPS_Satellites); // Puts the text in the datastring
  
//  mySerial.println(conv_doub_to_int(GPS_Latitude,1), 6);
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);

  rtty_txstring (datastring);
  
}
 
void rtty_txstring (char * string)
{
 
  /* Simple function to sent a char at a time to 
     ** rtty_txbyte function. 
    ** NB Each char is one byte (8 Bits)
    */
 
  char c;
 
  c = *string++;
 
  while ( c != '\0') //remeber all char arrays are terminated with "\0" 
  {
    rtty_txbyte (c);
    c = *string++;
  }
}
 
 
void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to 
    ** rtty_txbit function. 
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and 
    ** proceded with a 1. 0 = Start bit; 1 = Stop bit
    **
    */
 
  int i;
 
  rtty_txbit (0); // Start bit
 
  // Send bits for for char LSB first 
 
  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0); 

    c = c >> 1;
 
  }
 
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}
 
void rtty_txbit (int tbit)
{
  if (tbit)
  {
    // high
    digitalWrite(RADIOPIN, HIGH);
    if (bBuzzRtty) digitalWrite(BUZZERPIN, HIGH); //beep pcb buzzer in sequence with transmitted bits
    if (bLEDRtty) digitalWrite(OKLED, HIGH);
  }
  else
  {
    // low
    digitalWrite(RADIOPIN, LOW);
    if (bBuzzRtty) digitalWrite(BUZZERPIN, LOW); //beep pcb buzzer in sequence with transmitted bits
    if (bLEDRtty) digitalWrite(OKLED, LOW);
  }
      
                  delayMicroseconds(3370); // 300 baud
  //delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  //delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
                            // largest value that will produce an accurate delay is 16383
                            // See : http://arduino.cc/en/Reference/DelayMicroseconds
 
}
 
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}    

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}
