//run this code to see if the GPS is still output strings correctly

#include <SoftwareSerial.h>


SoftwareSerial mySerial(5, 4); // RX, TX
 
void setup()
{
Serial.begin(9600); //hardware serial
mySerial.begin(19200); //software serial
mySerial.println("Ublox M8Q Data Parsing");


//Serial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
//Serial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
//Serial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
//Serial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
//Serial.print("$PUBX,40,GSA,1,1,1,0*4E\r\n");
//Serial.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
// see https://www.u-blox.com/en/docs/UBX-13003221 (p140)
//May need to remove power for ublox to register the changes (particular when you toggle on the msgs)

}
 
void loop(){

    if (Serial.available()){
      //mySerial.println("avail");

      mySerial.write(Serial.read());
    
    }
  }
