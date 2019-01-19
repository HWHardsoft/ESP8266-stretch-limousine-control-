/*
 *  Mikrocontroller für Marwinski Controlboard
 *  Version 1.0
 *  Copyright (C) 2017  Hartmut Wendt  www.hwhardsoft.de
 *  
*/ 

#include <Wire.h>
#include "IRLibAll.h"


IRsend mySender; 

//#define _debug

//char inputString[101];           // a string to hold incoming data
String inputString ="";           // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup()   {   

  
  //PORTs
  DDRB = B00111111;       
  DDRC = B00111111;       
  DDRD = B11111100;

 
  
  Serial.begin(9600);
  // reserve 100 bytes for the inputString:
  //inputString.reserve(100);               
  delay(200);
  Wire.setClock(100000);    
  init_MCP23016();   
  delay(200);

}


void loop() {
  byte i1;
  byte i2;
  byte i3;
  long l1; 
  if (stringComplete) {
    #ifdef _debug
    Serial.println(inputString);
    #endif

    // Auf bestimmte Befehle reagieren
    // alle IOs setzen, inklusive Portexpander
    i1 = inputString.indexOf("IO="); 
    if (i1 < 255)
    {  
       #ifdef _debug
       Serial.println("IO=");  
       Serial.println(i1);      
       #endif
       
       // setze Ausgänge PortB:
       PORTB = StrToHex(inputString.substring(i1+3,i1+5),3);
       #ifdef _debug
       Serial.print("PortB:");              
       Serial.println(StrToHex(inputString.substring(i1+3,i1+5),3));
       #endif
       
       // setze Ausgänge PortC:
       PORTC = StrToHex(inputString.substring(i1+5,i1+7),3);
       #ifdef _debug
       Serial.print("PortC:");              
       Serial.println(StrToHex(inputString.substring(i1+5,i1+7),3));
       #endif
       
       // setze Ausgänge PORTD
       i2 = StrToHex(inputString.substring(i1+7,i1+9),3);
       i2 &= B11110100;
       #ifdef _debug
       Serial.print("PortD:");              
       Serial.println(i2);
       #endif
 
       PORTD &= B00001011;
       PORTD |= i2;     
       
       // setze Ausgänge Portexpander MCP23016:
       i2 = StrToHex(inputString.substring(i1+9,i1+11),3); 
       i3 = StrToHex(inputString.substring(i1+11,i1+13),3);
       #ifdef _debug
       Serial.print("GP0:");              
       Serial.print(i2);
       Serial.print("  GP1:");              
       Serial.println(i3);
       #endif
       write_MCP23016(i2,i3); 
    }
    
    
    // Auf bestimmte Befehle reagieren
    // Befehl Fernbedienung senden
    i1 = inputString.indexOf("IR=");  
    if (i1 < 255)
    {
       #ifdef _debug 
       Serial.println("IR");  
       Serial.println(i1);            
       Serial.println(inputString.substring(i1+3,i1+11)); 
       #endif
       l1 = StrToHex(inputString.substring(i1+3,i1+11),9);
       mySender.send(NEC,l1, 32);  
    }
    
       
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  }


}




/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

/*
 * Init the MCP23016 port expander via I2C
 * set all port as output
 */
void init_MCP23016() {
  Wire.begin();
  Wire.beginTransmission(0x20);  // set mcp23016 output
  Wire.write(0x06); 
  Wire.write(0x00);  // DDR Port0 all output
  Wire.write(0x00);  // DDR Port1 all output
  Wire.endTransmission(); 
  
}

/*
 * Set the GPIOs of port GP0 to the value of b1
 * Set the GPIOs of port GP1 to the value of b2
 */
void write_MCP23016(byte b1, byte b2) {
   Wire.beginTransmission(0x20);  // set mcp23016 for all output
   Wire.write(0x00); // begin here
   Wire.write(b1);
   Wire.write(b2); 
   Wire.endTransmission();
   
}

/*
byte StrToHex(String str)
{
  char ConvByte[12];
  str.toCharArray(ConvByte, 3);
  return (byte) strtol(ConvByte, NULL, 16);
}
*/

unsigned long StrToHex(String str, byte digits)
{
  char ConvByte[10];
  str.toCharArray(ConvByte, digits);
  return (unsigned long) strtol(ConvByte, NULL, 16);
}
