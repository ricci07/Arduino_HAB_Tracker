 
#include <string.h>
#include <util/crc16.h>
 
#define RADIOPIN 13
#define BUZZERPIN 12
#define OKLED 2
#define WARNLED 3

//================================Settings===============================================

uint8_t bBuzzRtty = 0; //beep pcb buzzer in sequence with transmitted bits
uint8_t bBuzzBeacon = 0; //Bleep once at transmission start, bleep twice at transmission end
uint8_t bLEDBeacon = 0; // flash green led in the same way as BuzzBeacon
uint8_t bLEDRtty = 0;

//=======================================================================================

char *myStrings[3] = {"The DATASTRING is passed to a procedure called rtty_txtstring which takes care of transmitting the data by breaking it down into characters, then it transmits the individual bits of those characters. The key to getting the baud rate correct is the timing. Theoretically 50 baud should be 1/50th of a second = 20000μ seconds. Now for some reason the Arduino (at least the one I have) doesn’t seem to be able to accurately count this time interval so the delay is achieved by doing one delay of 10000μS and one of 10150μS.", "Lisa can suck my penis", "Third wheeling"};

char datastring[80];

void setup() {                
  pinMode(RADIOPIN,OUTPUT);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(OKLED, OUTPUT);
  pinMode(WARNLED, OUTPUT);
}
 
void loop() {
 
 for (int o=0; o < 3; o++){
  
 
  sprintf(datastring, myStrings[o]); // Puts the text in the datastring
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);

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
  
  rtty_txstring (datastring);

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

  delay(2000);
}
  
}
 
 
void rtty_txstring (char * string)
{
 
  /* Simple function to sent a char at a time to 
     ** rtty_txbyte function. 
    ** NB Each char is one byte (8 Bits)
    */
 
  char c;
 
  c = *string++;
 
  while ( c != '\0')
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
