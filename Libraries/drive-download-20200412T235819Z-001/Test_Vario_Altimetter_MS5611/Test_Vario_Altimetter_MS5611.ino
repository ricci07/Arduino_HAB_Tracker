#include <Wire.h>                      
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <MS5611.h>                   
/////////////////////////////////////////
// Software SPI (slower updates, more flexible pin options):
// pin 13 - Serial clock out (SCLK)
// pin 11 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(5, 4, 3);
///////////////////////////////////////// 
#define SpeakerPin 9                    //
#define UART_SPEED 57600                //
/////////////////////////////////////////
/////////////////////////////////////////
MS5611 barometer; // I2C
int      Temperature = 0;
long     Pressure = 101325;
float    Altitude;
float    Vario;
unsigned int      Battery_Vcc = 0;             
char     altitude_arr[6];            
char     vario_arr[5];               
short    filter=10; //115
short    samples=5; //30
float    alt[31];
long     tim[31];
long     outTmp=millis();
long     toneTime=millis();
bool     tonePlay = false;

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define sample_num_mdate  3000
  
float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float mx_sample[3];
float my_sample[3];
float mz_sample[3];

short maxsink;
short maxclimb;
short maxalt;

struct MyVario {
  char sn[5];
  bool calibrate;
  short mx;
  short my;
  short mz;
  short ax;
  short ay;
  short az;
  long p0 = 101325;
  short t=0;
  short sink =-150;
  short climb=40;
  short vtime;
  short base;
  short increment;
};

MyVario v;


void writeEEPROM()
{

  int eeAddress = 0;   
  EEPROM.put(eeAddress, v);
}


void readEEPROM()
{

  int eeAddress = 0;   
  EEPROM.get( eeAddress, v );
  
}
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}



unsigned int readVcc()                         
{
  unsigned int result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  long t=millis();
  while(t+2>millis()); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void setup()                // setup() function to setup all necessary parameters before we go to endless loop() function
{
  Serial.begin(UART_SPEED);       
  Wire.begin();                   
  barometer = MS5611();
  barometer.begin();
  
  pinMode(2,INPUT);
  digitalWrite(2,HIGH);
  pinMode(6,INPUT);
  digitalWrite(6,HIGH);
  pinMode(7,INPUT);
  digitalWrite(7,HIGH);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  
   // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  display.begin();
  display.setContrast(55);
  display.clearDisplay();   // clears the screen and buffer
  
}

void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}


void get_calibration_Data ()
{
    
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();
        
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);
        
        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

    }


    v.mx = (mx_sample[1] + mx_sample[0]) / 2*100;
    v.my = (my_sample[1] + my_sample[0]) / 2*100;
    v.mz = (mz_sample[1] + mz_sample[0]) / 2*100;


}


void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}


void getAccel_Data(void)
{
     // Read accelerometer and gyroscope
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
   
   
    // Create 16 bits values from 8 bits data
   
    // Accelerometer
    int16_t ax=Buf[0]<<8 | Buf[1];
    int16_t ay=Buf[2]<<8 | Buf[3];
    int16_t az=Buf[4]<<8 | Buf[5];
    
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
     // Read accelerometer and gyroscope
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
   
   
    // Create 16 bits values from 8 bits data
  
    // Gyroscope
    int16_t gx=Buf[8]<<8 | Buf[9];
    int16_t gy=Buf[10]<<8 | Buf[11];
    int16_t gz=Buf[12]<<8 | Buf[13];
    
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
   uint8_t ST1;
    do
    {
      I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    }
    while (!(ST1&0x01));
   
    // Read magnetometer data  
    uint8_t Mag[6];  
    I2Cread(MAG_ADDRESS,0x03,6,Mag);
   
    // Request next magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
   
   
   
    // Create 16 bits values from 8 bits data
   
    // Magnetometer
    int16_t mx=Mag[1]<<8 | Mag[0];
    int16_t my=Mag[3]<<8 | Mag[2];
    int16_t mz=Mag[5]<<8 | Mag[4];
  

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - v.mx/100;
    Mxyz[1] = Mxyz[1] - v.my/100;
    Mxyz[2] = Mxyz[2] - v.mz/100;
}

float readAltitude(float seaLevelhPa, float pressure) {
  float altitude;

  //pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
  return altitude;
}

void loop(void)
{
  long  tempo=millis();
  Pressure=0;
  Altitude=0;
  for(int i=0;i<filter;i++){
      Pressure+=barometer.getPressure();
      Altitude+=readAltitude(v.p0,barometer.getPressure());
  }

  Pressure/=filter;
  Altitude/=filter;
  Temperature =(int)barometer.getTemperature()/100+v.t;
  
  for(int cc=1;cc<=samples;cc++){                                   
    alt[(cc-1)]=alt[cc];
    tim[(cc-1)]=tim[cc];
  };
  
  alt[samples]=Altitude;
  tim[samples]=tempo;

   float sp=0;
    for (int cc=1;cc<samples;cc++)
    {
        sp+=(alt[cc+1]-alt[cc])/(tim[cc+1]-tim[cc]);
    }
    
   Vario=1000*sp/samples;

   if(Vario<0.1 && Vario>-0.1)
      Vario=0;
       
   if(Vario*100>v.climb){
       int freq=500+Vario*100;
       int freqTime;
       int silentTime;
       if(Vario<0.5)
           freqTime=300;
       else 
       if(Vario<1)
           freqTime=250;
       else
       if(Vario<1.5)
           freqTime=200;
       else
       if(Vario<2.5)
           freqTime=150;
       else
       if(Vario<4)
           freqTime=100;
       else
       if(Vario<5)
           freqTime=80;
       else
           freqTime=80;
       silentTime=freqTime;
       
       if(tonePlay==false && millis()-toneTime>silentTime){
           tone(8,freq);
           toneTime=millis();
           tonePlay=true;
       }
       if(tonePlay==true && millis()-toneTime<freqTime){
           tone(8,freq);
       }
       
      if(tonePlay==true && millis()-toneTime>freqTime){
               noTone(8);
               toneTime=millis();
               tonePlay=false;
          }
                 
    }
    
    if(Vario*100<v.sink){
       int freq=300+Vario*100;
           tone(8,freq);
           toneTime=millis();
           tonePlay=true;
       }
    if(tonePlay==true && Vario*100>v.sink && Vario*100<v.climb){
               noTone(8);
               toneTime=millis();
               tonePlay=false;
          }
           
    

    

if (millis()>outTmp+333)       //every 1/3 second send NMEA output over serial port
  {
    Battery_Vcc=readVcc();
    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); 
    getHeading();               
    getTiltHeading();
    
    String str_out =                                                                 
      String("LK8EX1"+String(",")+String(Pressure,DEC)+ String(",")+String(dtostrf(Altitude,0,0,altitude_arr))+String(",")+
      String(dtostrf((Vario*100),0,0,vario_arr))+String(",")+String(Temperature,DEC)+String(",")+String(Battery_Vcc,DEC)+String(","));
    unsigned int checksum_end,ai,bi;                                                 // Calculating checksum for data string
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
    //creating now NMEA serial output for LK8000. LK8EX1 protocol format:
    //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
    Serial.print("$");                     //print first sign of NMEA protocol
    Serial.print(str_out);                 // print data string
    Serial.print("*");                     //end of protocol string
    Serial.println(checksum_end,HEX);      //print calculated checksum on the end of the string in HEX

    outTmp=millis();
    mainDisplay();
  }

}
void mainDisplay(){

  //display main screen
  display.clearDisplay();   // clears the screen and buffer
  //Batery
  display.drawRect(0, 0, 15, 7, BLACK);
  display.drawLine(15, 2, 15,4, BLACK);
  for(int i=2;i<13-(3365-Battery_Vcc)/23;i++){
     display.drawLine(i, 2, i,4, BLACK);
     if(i%4==0) i++;
  }
  //Temperature
  display.setTextSize(1);
  display.setCursor(21,0);
  display.setTextColor(BLACK); // 'inverted' text
  display.println(Temperature);
  display.drawCircle(33,1,1,BLACK);
  //Time
  display.setTextSize(1);
  display.setCursor(42,0);
  display.setTextColor(BLACK); // 'inverted' text
  long flyTime=millis();
  int hours = flyTime/1000/60/60;
  int minutes = flyTime/1000/60%60;
  if(hours<10)
     display.print("0");
  display.print(hours);
   display.print(":");
  if(minutes<10)
     display.print("0");
  display.print(minutes);
  //vario bar
  display.drawRoundRect(74, 0, 10, 48, 2, BLACK);
  display.drawLine(74, 29, 84,29, BLACK);
  int barTmp; 
  if(Vario>=0)
      barTmp= abs(Vario*28/7);
  else
      barTmp= abs(Vario*18/5);    
  for(int i=0;i<barTmp;i++)
      if(Vario>0)
         display.drawLine(74, 28-i, 84,28-i, BLACK);
      else
         display.drawLine(74, 30+i, 84,30+i, BLACK);
      
  //vario text
  display.drawLine(25, 13, 72,13, BLACK);
  display.setCursor(25,14);
  display.setTextColor(WHITE, BLACK); // 'inverted' text
  display.setTextSize(2);
  int varioTmp=abs(Vario*10);
  if(Vario>0)
     display.print("+");
  else if(Vario<0)
    display.print("-");
  else
    display.print(" ");
  display.print(varioTmp/10);
  display.print(".");
  display.println(varioTmp%10);
  
  //Heading
  display.setTextSize(1);
  display.setCursor(0,13);
  display.setTextColor(BLACK); // 'inverted' text
  if(tiltheading>360-22 || tiltheading <=22)
     display.println(" N ");
  else
  if(tiltheading>22 && tiltheading <=90-22)
     display.println("ENE");
  else
  if(tiltheading>90-22 && tiltheading <=90+22)
     display.println(" E ");
  else
  if(tiltheading>90+22 && tiltheading <=180-22)
     display.println("ESE");
  else
  if(tiltheading>180-22 && tiltheading <=180+22)
     display.println(" S ");
  else
  if(tiltheading>180+22 && tiltheading <=270-22)
     display.println("VSV");
  else
  if(tiltheading>270-22 && tiltheading <=270+22)
     display.println(" V ");
  else
  if(tiltheading>270+22 && tiltheading <=360-22)
     display.println("VNV");         
  display.setCursor(0,22);
  if(tiltheading<100)
     display.print("0");
  display.println((int)tiltheading);
   //Altimeter
  display.setTextSize(2);
  display.setCursor(8,34);
  display.setTextColor(BLACK); // 'inverted' text
  int altitudeTmp=(int)Altitude;
  if(Altitude<1000)
     display.print(" ");
  if(Altitude<100)
     display.print(" ");
  if(Altitude<10)
     display.print(" ");  
  if(altitudeTmp<10000){
     display.print(altitudeTmp);
     display.println("m");
  }
  display.display();
}

