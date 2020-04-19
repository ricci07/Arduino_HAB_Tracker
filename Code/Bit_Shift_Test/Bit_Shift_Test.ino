void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); //required to talk to console
  printOut1(97);
}

void loop() {
  // put your main code here, to run repeatedly:

}


 // Prints out Binary value (1 or 0) of byte
  void printOut1(int c) {

      Serial.print("Compare: ");
      Serial.println(c);
      
    for (int bits = 7; bits > -1; bits--) {
     // Compare bits 7-0 in byte

        Serial.print("Compare: ");
        Serial.println(c & (1<<1));
    
     
     if (c & (1 << bits)) {
     //  Serial.print ("1");
      }
      else {
       //Serial.print ("0");
      }
    }
  }
