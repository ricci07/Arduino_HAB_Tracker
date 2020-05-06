#include <SoftwareSerial.h> //for debugging only
#include <string.h>
#include <util/crc16.h>
#include <TinyGPS++.h>
#include <util/delay.h>
#include <Wire.h> //needed for sensors
#include <Adafruit_Sensor.h> //adafruit unified sensor lib
#include <Adafruit_BMP280.h> //bmp280 lib
#include <BH1750.h> //light sensor lib
#include <EEPROM.h>

//RTTY @ 370Hz shift
const uint16_t RTTY_BAUD = 50; //change this to either 50 or 300 [[[rem dlfldigi too]]]
const uint8_t RTTY_ASCII = 7; //change this to either 7 or 8 [[[rem dlfldigi too]]]
const uint8_t START_BITS = 1; //[[[rem dlfldigi too]]]
const uint8_t STOP_BITS = 2; //[[[rem dlfldigi too]]]

//instances
SoftwareSerial  mySerial(5, 4); // RX/DB0, TX/DB1 for debugging only
TinyGPSPlus     gps;
Adafruit_BMP280 BMP280;
BH1750          lightmeter(0x23); //default pin->GND address

//PINS
#define RADIO_PIN 13
#define BUZZER_PIN 12
#define OK_LED 3
#define WARN_LED 2
#define GPS_LOCK_LED 10
#define BMP280_I2C_Address 0x76
#define BMP280_Altitude_Pressure_Ref 993.612 //home pressure
#define BATT_VOLTS_PIN A0
#define UVOUT_PIN A1 //uv sensor input
#define UVOUT_REF_PIN A2 //3V3 Reference pin

#define ARDUINO_RESET_PIN 11 //supply 0v/GND to reset arduino

const int16_t TIME_BETWEEN_TX = 10000; //encode and send RTTY sentence every 10secs
unsigned long RTTY_TX_Prev_Time = 0; //
const int16_t NO_GPS_Data_Reset_Thresh = 100; //RESET ARDUINO IF GPS DATA AFTER 100 ATTEMPTS

//#define DEBUG; //comment out to prevent complimate of debug lines

unsigned int GPS_Satellites;
unsigned int GPS_Hour;
unsigned int GPS_Minute;
unsigned int GPS_Second;
unsigned int GPS_Date;
double       GPS_Altitude;
double       GPS_Speed;
double       GPS_Course;
double       GPS_Latitude;
double       GPS_Longitude;
boolean      GPS_Data_Available;
uint8_t      No_GPS_Available_Count = 0;

double GPS_Distance_To_Home_In_Miles;
double GPS_Course_To_Launch_Location;
char*  GPS_Cardinial;

//Type in the launch location's GPS coords
const double LAUNCH_LOCATION_LAT = 52.682439; //Telford campus field
const double LAUNCH_LOCATION_LGN = -2.424624; //Telford campus field

//Type in your home location's GPS coords
const double HOME_LOCATION_LAT = 52.592296;
const double HOME_LOCATION_LGN = -2.192684;

//bmp280
float BMP280_Temp;
float BMP280_Pressure;
float BMP280_Altitude;

//BH1750
float BH1750_Lux;

//Voltage Sensor
double        Batt_Voltage; //query this var for voltage
int           Batt_Offset = 0; //correction offset if needed
const float   Batt_Fully_Charged_Voltage = 5.0;
uint16_t      Batt_Bleep_Start_Delay = 5000; //start delay because readings are only taken when gps data is updated
unsigned long Batt_Bleep_Prev = 0;
uint8_t       Batt_Low_Thresh = 50; //50%

//UV Sensor
float UV_Out_Voltage; //query this variable
float UV_Out_Intensity; //query this variable

//================================Aux Settings===============================================

uint8_t bBuzzRtty = 0; //beep pcb buzzer in sequence with transmitted bits
uint8_t bBuzzBeacon = 0; //Bleep once at transmission start, bleep twice at transmission end
uint8_t bLEDBeacon = 0; // flash green led in the same way as BuzzBeacon
uint8_t bLEDRtty = 0;
uint8_t bleepLowBattery = 1;

//=======================================================================================

void(* resetFunc) (void) = 0;
int EEPROM_ADDRESS = 0; //poweroff memory address used for
int EEPROM_VALUE;

char datastring[80]; //buffer for rtty sentence

void setup() {
                  
  pinMode(RADIO_PIN,OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(WARN_LED, OUTPUT);
  pinMode(GPS_LOCK_LED, OUTPUT);
  pinMode(BATT_VOLTS_PIN, INPUT); //added after, even though it worked without it :S
  pinMode(UVOUT_PIN, INPUT); //uv sensor input
  pinMode(UVOUT_REF_PIN, INPUT); //uv sensor 3v3 reference (using in calculation)


  //WHERE IS VOLTAGE SENSOR PINMODE STATMENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //why does it work without pinmode
  Serial.begin(9600); //hardware serial for GPS
  mySerial.begin(19200); //software serial for gps debugging

  Wire.begin(); //start I2C (light sensor specific)
  
  if (!BMP280.begin(BMP280_I2C_Address)){
    mySerial.println("No BMP280 found!!");
  }else{
    mySerial.println("BMP280 started sucessfully");
  }

  if (!lightmeter.begin(BH1750::CONTINUOUS_LOW_RES_MODE)){
    mySerial.println("No BH1750 found!!");
  }else{
    mySerial.println("BH1750 started succesfully");
  }

  setupGPS(); //turn off unneeded NMEA messages and change dynamic mode
  mySerial.println("Debugging Started...");
}

void setGPS_DynamicModel6(){ //change dynamic mode of gps to high altitude
  
  int gps_set_sucess=0;

  uint8_t setdm6[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
  0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E,
  0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x08 };
  
  while(!gps_set_sucess){
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t)); //command, sizeofArray
    gps_set_sucess = getUBX_ACK(setdm6);
  }
}

void sendUBX(uint8_t *MSG, uint8_t len) { //rem referencing arrays need pointer dereferencing asterisk
  Serial.flush(); //wait for any existing outgoing transmitted serial data to finish
  Serial.write(0xFF); //255 8bit
  _delay_ms(500);
  
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
    //------
    digitalWrite(WARN_LED, !digitalRead(WARN_LED)); //Pulse the warn LED to show command upload
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN)); //pulse the buzzer to indicate upload
    _delay_ms(25); //delay allows the led flashes to be visible
    //-----
 }
}
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

   // Construct the expected ACK packet
   ackPacket[0] = 0xB5; // header
   ackPacket[1] = 0x62; // header
   ackPacket[2] = 0x05; // class
   ackPacket[3] = 0x01; // id
   ackPacket[4] = 0x02; // length
   ackPacket[5] = 0x00;
   ackPacket[6] = MSG[2]; // ACK class
   ackPacket[7] = MSG[3]; // ACK id
   ackPacket[8] = 0; // CK_A
   ackPacket[9] = 0; // CK_B
 
  // Calculate the checksums
   for (uint8_t ubxi=2; ubxi<8; ubxi++) {
     ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
     ackPacket[9] = ackPacket[9] + ackPacket[8];
   }
 
  while (1) {
   
  // Test for success
   if (ackByteID > 9) {
     // All packets in order!
     //digitalWrite(OK_LED, HIGH);
     mySerial.println("GPS Dynamic mode 6 set successfully (ack recieved)");
     return true;
   }
 
// Timeout if no valid response in 3 seconds
 if (millis() - startTime > 3000) {
  return false;
 }

// Make sure data is available to read
 if (Serial.available()) {
  b = Serial.read();
 
// Check that bytes arrive in sequence as per expected ACK packet
 if (b == ackPacket[ackByteID]) {
  ackByteID++;
 }
 else {
  ackByteID = 0; // Reset and look again, invalid order
 }
 }
 }
}
 
void loop() {

  setBattVoltageVariables(); //having this here stops the buzzer from bleeps because it thinks the voltage level is 0v
  
  if (Serial.available()){

    //mySerial.write(Serial.read()); //uncomment to see if GPS is outputting normal!!
    
    if (gps.encode(Serial.read())){
      if (gps.passedChecksum()){

       if (gps.location.isUpdated() && gps.altitude.isUpdated()){ //only print out if we have non-zero data
         
         GPS_Data_Available = true; //used for gpslock led
         No_GPS_Available_Count =0; //used for gpslock led

            setBMP280Variables(); //set global variable of the BMP280 data
            setBH1750Variables();
            setML8511Variables();
            setGPSVariables(); //set all the global variables to the tinyURL parsed GNGGA info
         
            //resetFunc(); //use to reset the arduino in case of fatal error
       
         if (millis() - RTTY_TX_Prev_Time > TIME_BETWEEN_TX){ //transmit delay for sentences (if any)
          buildTelemetryString();
          RTTY_TX_Prev_Time = millis(); 
         }
         
         // now start using them to build Tx string
         //THIS MUST ONLY BE EXE when we have data avail 
         //i.e it must be inside this if statement or it will not work!!!!!!
  
        //======debugging tinyGPS++ and BMP280 stuff======
          mySerial.print("Lat: ");
          mySerial.print(GPS_Latitude, 6);
          mySerial.print(" Long: ");
          mySerial.print(GPS_Longitude, 6);
          mySerial.print(" GPS_Lock: " );
          mySerial.print(is_GPS_Locked());
          mySerial.print(" Alt: ");
          mySerial.print(GPS_Altitude);
          mySerial.print(" B_Temp: " );
          mySerial.print(BMP280_Temp);
          mySerial.print(" B_Press: ");
          mySerial.print(BMP280_Pressure);
          mySerial.print(" B_Alt: ");
          mySerial.print(BMP280_Altitude);
          mySerial.print(" L_Lux: ");
          mySerial.print(BH1750_Lux);
          mySerial.print(" Batt_Volts: ");
          mySerial.print(Batt_Voltage);
          mySerial.print(" Batt_Perc: ");
          mySerial.print(Get_Battery_Percentage());
          mySerial.print(" UV_Out_Volt: ");
          mySerial.print(UV_Out_Voltage);
          mySerial.print(" UV_Out_Intens: ");
          mySerial.print(UV_Out_Intensity);
          mySerial.println();
         //====================================

        }else{ //if not gps loc and gps alt
          mySerial.print("Waiting for updated GPS values! ("); //debug stuff
          mySerial.print(No_GPS_Available_Count);
          mySerial.print(")");
          mySerial.println();
          No_GPS_Available_Count++; //increment no data counter
          GPS_Data_Available = false;
        }
      }else if (gps.failedChecksum()){
        mySerial.println("Checksum failed"); //debug stuff
      }
    } //gps.encode end
  } //serial avail end

  if (No_GPS_Available_Count >= 5 && No_GPS_Available_Count < NO_GPS_Data_Reset_Thresh){ 
    //check because get no data 5 times row likely means we have lost gps lock
    //it is normal not get data sometimes inbetween serial checks, but not several times in a row.
   digitalWrite(GPS_LOCK_LED, LOW);
  }else if (No_GPS_Available_Count == NO_GPS_Data_Reset_Thresh){
    
    EEPROM_VALUE = EEPROM.read(EEPROM_ADDRESS);
    //READ/WRITE EEPROM sticky variables
    if (EEPROM_VALUE != 0){
      EEPROM.write(EEPROM_ADDRESS, 1);
      //mySerial.println(EEPROM_VALUE);
        if (EEPROM_VALUE == 255)
          EEPROM_VALUE=0; //prevent overflow
          
      //write byte to eeprom to signfy we resetted arduino
      //++addr &= EEPROM.length() - 1; //increment address without overflow
      mySerial.println("Resetting Arduino!!");
      resetFunc();
    }else{
      mySerial.println("Already resetted!");
      No_GPS_Available_Count = 0; //prevents TYPE overlow (255 max)
    }
    
  }else{
    digitalWrite(GPS_LOCK_LED, HIGH);
  }

 
   if (bleepLowBattery && (Get_Battery_Percentage() < Batt_Low_Thresh)){ //make buzzer "blip" when the battery is below 50%

        if (millis() - Batt_Bleep_Prev > Batt_Bleep_Start_Delay){
        digitalWrite(BUZZER_PIN, HIGH);
        delay(50);
        digitalWrite(BUZZER_PIN, LOW);
        Batt_Bleep_Prev = millis();
 
      }
}
  
} //loop end

boolean is_GPS_Locked(){
  if (GPS_Data_Available){ //if there is not data avail we will read outdated vals making it appear as if we do
    return GPS_Satellites >= 4; //4 is needed to get position
  }else{
    return false;
  }
}

float Get_Battery_Percentage(){
  return (Batt_Voltage/Batt_Fully_Charged_Voltage)*100; //output over input
}

void Reset_Arduino(){
  digitalWrite(ARDUINO_RESET_PIN, 0);
}

void setupGPS(){ //turn off unuseful messages (non GCGGA)
  Serial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  delay(100);
  Serial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  delay(100);
  Serial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  delay(100);
  Serial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  //delay(100);
  //Serial.print("$PUBX,40,GSA,1,1,1,0*4E\r\n");
  delay(100);
  Serial.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
  delay(100);
  setGPS_DynamicModel6(); //enable GPS flight mode
}

void setML8511Variables(){

  int UV_Level = averageAnalogRead(UVOUT_PIN);
  int UV_Ref_Level = averageAnalogRead(UVOUT_REF_PIN);

  UV_Out_Voltage = 3.3 / UV_Ref_Level * UV_Level;
  UV_Out_Intensity = mapfloat(UV_Out_Voltage, 0.99, 2.9, 0.0, 15.0);
  
  //ref: http://www.theorycircuit.com/arduino-uv-sensor-ml8511/
}

int averageAnalogRead(int pin)
{
  byte rCount = 8;
  uint32_t total = 0; 

  for(int i = 0; i < rCount; i++)
    total += analogRead(pin);
  total /= rCount;

  return(total);  
}


//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setBattVoltageVariables(){
  int Batt_Voltage_Raw = analogRead(BATT_VOLTS_PIN);
  Batt_Voltage = map(Batt_Voltage_Raw,0,1023,0,2500)+Batt_Offset;
  Batt_Voltage /= 100;
}

void setBH1750Variables(){
  BH1750_Lux = lightmeter.readLightLevel();
}

void setBMP280Variables(){
  //bmp280
  BMP280_Temp = BMP280.readTemperature();
  BMP280_Pressure = BMP280.readPressure();
  BMP280_Altitude = BMP280.readAltitude(BMP280_Altitude_Pressure_Ref); //1013.25 = sea level pressure/ 1 atmosphere
  //
}

void blinkLED(uint8_t pin, uint8_t num_of_blinks, int blink_delay){
  for (uint8_t i =0; i<= num_of_blinks;i++){
    digitalWrite(pin, HIGH);
    delay(blink_delay);
    digitalWrite(pin, LOW);
    delay(blink_delay);  
  }
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
  GPS_Course_To_Launch_Location = TinyGPSPlus::courseTo(
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
  GPS_Cardinial = TinyGPSPlus::cardinal(GPS_Course_To_Launch_Location);
  
}

/* probably will used inline or maybe not
int32_t conv_doub_to_int(double input, char resPart){ // convert the decimal lat/long to integer parts that can be charred

    int integerPart = (int)input;
    double decimalPart = fmod(input,1);
    int32_t dPartInt = decimalPart*1000000;
    
   // double num_dec = input - (int)input; //get the decimal part e.g 0.564545
   // double  num_int = input-num_dec; //obtain the integer part
   // double num_dec_res = num_dec*1000000; //bring the decimal into integer (this was a pain to make work)

    return resPart == 0 ? integerPart : dPartInt; //return the int of dec part

}

*/
void buildTelemetryString(){

  int lat_int = (int)abs(GPS_Latitude);
  int32_t lat_dec;
  
  double decimalPart = fmod(GPS_Latitude,1);
  lat_dec = abs(decimalPart)*1000000;              //split the latitude and longitude double into seperate integers
  
  int lon_int = (int)abs(GPS_Longitude);
  int32_t lon_dec;
  
  double decPart = fmod(GPS_Longitude,1);
  lon_dec = abs(decPart)*1000000;

  int batt_int = (int)Batt_Voltage;
  float battDecPart = fmod(Batt_Voltage,1);
  int32_t batt_dec = battDecPart*100;

  int uv_int = (int)UV_Out_Intensity;
  float uvDecPart = fmod(UV_Out_Intensity, 1);
  int32_t uv_dec = uvDecPart*100;

  sprintf(datastring, "$$$RIC,%02d:%02d:%02d,%s%i.%ld,%s%i.%ld,%i,%i,%i,%i,%s,%i,%i,%i,%i,%i.%ld,%i.%ld", 
  GPS_Hour+1, GPS_Minute, GPS_Second, (GPS_Latitude < 0 ? "-" : ""), 
  lat_int, lat_dec, (GPS_Longitude < 0 ? "-" : ""), lon_int, lon_dec, 
  GPS_Satellites,(int)GPS_Speed, (int)GPS_Course, (int)GPS_Course_To_Launch_Location, 
  GPS_Cardinial,(int)BMP280_Temp,(int)BMP280_Pressure, (int)BMP280_Altitude,
  (int)BH1750_Lux, batt_int, batt_dec, uv_int, uv_dec); // Puts the text in the datastring
  
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);
  
  rtty_txstring (datastring);
  
}
 
void rtty_txstring (char * string){
 
  /* Simple function to sent a char at a time to 
    ** rtty_txbyte function. 
    ** NB Each char is one byte (8 Bits)
    */
 
  char c;
 
  c = *string++;
 
  while ( c != '\0'){ //remember all char arrays are terminated with "\0" 
    rtty_txbyte (c);
    c = *string++;
  }
}
 
 
void rtty_txbyte (char c){
  /* Simple function to sent each bit of a char to 
    ** rtty_txbit function. 
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and 
    ** proceded with a 1. 0 = Start bit; 1 = Stop bit
    **
    */
 
 for (int x=0;x<START_BITS;x++){
  rtty_txbit (0); // Start bit
 }
  // Send bits for for char LSB first 
 
  for (int i=0;i<RTTY_ASCII;i++){ // Change this here 7 or 8 for ASCII-7 / ASCII-8
  
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0); 

    c = c >> 1;
 
  }

 for (int a=0; a<STOP_BITS;a++){
  //rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
 }
}
 
void rtty_txbit (int tbit){
  
  if (tbit){
    // high
    digitalWrite(RADIO_PIN, HIGH);
    if (bBuzzRtty) digitalWrite(BUZZER_PIN, HIGH); //beep pcb buzzer in sequence with transmitted bits
    if (bLEDRtty) digitalWrite(OK_LED, HIGH);
  }else{
    // low
    digitalWrite(RADIO_PIN, LOW);
    if (bBuzzRtty) digitalWrite(BUZZER_PIN, LOW); //beep pcb buzzer in sequence with transmitted bits
    if (bLEDRtty) digitalWrite(OK_LED, LOW);
  }  

  if (RTTY_BAUD == 50){
    delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
    delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
  }else if(RTTY_BAUD == 300){ 
    delayMicroseconds(3370); // 300 baud
  }else{
    mySerial.print ("INVALID BAUD RTTY BAUD RATE! USE 50 or 300");
    delayMicroseconds(3370); // 300 baud
  }

}
 
uint16_t gps_CRC16_checksum (char *string){
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++){
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}    

void wait(unsigned long delaytime){ // Arduino Delay doesn't get CPU Speeds below 8Mhz
  
  unsigned long now=millis();
  while((now+delaytime)>=millis()){
    //do nothing
  }
}
