#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <SoftwareSerial.h>


// enable debug outputs via serial Port 1 (USB)
//#define _debug_snd
//#define _debug_rec


// Portkonfiguration
#define LEDBL 2     //interne blaue LED an GPIO2 des Wemos
#define LEDRT 14      //rote LED an GPIO5 (D1) des Wemos

SoftwareSerial ESPserial(12, 13); // RX (D6) | TX (D7)

// Name und Passwort des Access Points
#define SSID "xxxxx"
#define PASSWORD "xxxxx"

// Der Server nimmt Verbindungen auf diesem Port an
#define PORT 5444
#define IP_BROADCAST  "192.168.0.255"
WiFiUDP udpServer;


#define WIFICLIENT_MAX_PACKET_SIZE 200
// Puffer für hereinkommende UDP Nachrichten
char udp_buffer[WIFICLIENT_MAX_PACKET_SIZE+1];


// Variables
int i1;
char *test;
String s1;
String inputString;
int TMR_UDP_REC_BLOCK=0;


/** Wird beim Start einmal ausgeführt */
void setup()
{
    // LED blau aus
    pinMode(LEDBL, OUTPUT);
    digitalWrite(LEDBL, HIGH);

    // LED rot aus
    pinMode(LEDRT, OUTPUT);
    digitalWrite(LEDRT, LOW);

    // Seriellen Port 2 initialisieren
    ESPserial.begin(9600); // Start the software serial for communication with the ESP8266   
    
    // Gib dem seriellen Port Monitor von der Arduino
    // IDE ein bisschen Zeit.
    #ifdef _debug_snd
    Serial.begin(115200);   // Seriellen Port1 (USB) für Debugging initialisieren.  
    #endif
    delay(250);

    // Benutze einen externen AP
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);

    // UDP Server starten
    udpServer.begin(PORT);
   
}


/** Hauptschleife, wird wiederholt ausgeführt. */
void loop()
{
    process_incoming_udp();
    check_ap_connection();
    
    while (ESPserial.available()) {
      // get the new byte:
      char inChar = (char)ESPserial.read();
      #ifdef _debug
      Serial.print(inChar);
      #endif
      // yield();
      // add it to the inputString:
      inputString += inChar;
      //if (inChar == 0xFF) Nextion_auswerten();
      if (inChar == '|') Nextion_auswerten();
    }  
    if (TMR_UDP_REC_BLOCK) TMR_UDP_REC_BLOCK--;
    delay(1);

}


/** Empfange UDP Nachrichten und sende Echo zurück */
void process_incoming_udp()
{   
    if (udpServer.parsePacket()) 
    {
        // Nachricht abholen
        int len=udpServer.read(udp_buffer,sizeof(udp_buffer)-1);
        udp_buffer[len] = 0;
                
        // Nachricht anzeigen
        // Serial.print("Empfangen von ");
        // Serial.print(udpServer.remoteIP());
        // Serial.print(":");
        // Serial.print(udpServer.remotePort());
        // Serial.print(": ");
        // Serial.println(udp_buffer);
        
        // Nachricht als Echo zurück senden
        // udpServer.beginPacket(udpServer.remoteIP(), udpServer.remotePort());
        // udpServer.print("Echo: ");
        // udpServer.print(udp_buffer); 
        // udpServer.endPacket();

        // Nextion Befehle senden, wenn vorhanden
        // udpServer.beginPacket(udpServer.remoteIP(), udpServer.remotePort());
        // if (UDP_TMT_String.length > 0) {
        //  udpServer.println("TEST 123"); 
        //}        
        // udpServer.endPacket();
        
        
        // Auf bestimmte Befehle reagieren
        // Ist-Temperatur setzen
        if (strstr(udp_buffer, "IT="))
        {
            test = strstr(udp_buffer, "IT=");
            if (*(test + 3) == '-') {                     
               s1 = "-" + String(*(test + 4)) + String(*(test + 5));
            } else {
               s1 = String(*(test + 3)) + String(*(test + 4)); 
            }   
            #ifdef _debug_rec            
            Serial.print("Wert IT: ");
            Serial.println(s1.toInt());            
            #endif
            
            digitalWrite(LEDRT, HIGH);
            // udpServer.println("LED ist an");
            ESPserial.print("n0.val=");
            ESPserial.print(s1.toInt());            
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            
        }

        if (TMR_UDP_REC_BLOCK) return;
        // Soll-Temperatur setzen
        if (strstr(udp_buffer, "ST="))
        {
            test = strstr(udp_buffer, "ST=");
            s1 = String(*(test + 3)) + String(*(test + 4)); 
            #ifdef _debug_rec            
            Serial.print("Wert ST: ");
            Serial.println(s1.toInt());                     
            #endif
 
            //digitalWrite(LEDRT, LOW);                       
            ESPserial.print("n2.val=");
            ESPserial.print(s1.toInt());            
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);
          }

        // Luefter setzen
        if (strstr(udp_buffer, "FN="))
        {
            test = strstr(udp_buffer, "FN=");
            if (*(test + 3) == '1') {              
              i1=33;
            } else if (*(test + 3) == '2') {  
              i1=66;
            } else if (*(test + 3) == '3') {    
              i1=100;
            } else {
              i1=0;
            }  
              
            #ifdef _debug_rec            
            Serial.print("Wert FN: ");
            Serial.println(i1);            
            #endif
                                  
            ESPserial.print("j0.val=");
            ESPserial.print(i1);            
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);
          }
        
        // alle switches synchronisieren
        if (strstr(udp_buffer, "SW="))
        {            
            test = strstr(udp_buffer, "SW=");
            byte b0= StrToHex(String(*(test + 5)) + String(*(test + 6)),3);
            byte b1= StrToHex(String(*(test + 3)) + String(*(test + 4)),3);
            #ifdef _debug_rec            
            Serial.print("Wert SW Byte0: ");
            Serial.println(b0);            
            Serial.print("Wert SW Byte1: ");
            Serial.println(b1);            
            #endif                         

            // Status bt0
            ESPserial.print("bt0.val=");
            if (b0 & 0x01) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);

            // Status bt1
            ESPserial.print("bt1.val=");
            if (b0 & 0x02) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            
            // Status bt2
            ESPserial.print("bt2.val=");
            if (b0 & 0x04) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);
/*
            // Status bt3
            ESPserial.print("bt3.val=");
            if (b0 & 0x08) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);
*/
            // Status bt4
            ESPserial.print("bt4.val=");
            if (b0 & 0x10) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);

            // Status bt5
            ESPserial.print("bt5.val=");
            if (b0 & 0x20) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);

            // Status bt6
            ESPserial.print("bt6.val=");
            if (b0 & 0x40) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);

            // Status bt7
            ESPserial.print("bt7.val=");
            if (b0 & 0x80) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);

            // Status bt8
            ESPserial.print("bt8.val=");
            if (b1 & 0x01) {
              ESPserial.print(1);            
            } else {
              ESPserial.print(0);            
            }                  
            ESPserial.write(0xff);
            ESPserial.write(0xff);
            ESPserial.write(0xff);


        }        
     }    
}


/** Optional: Zeige an, wenn die Verbindung zum AP geändert wurde. */
void check_ap_connection()
{
    static wl_status_t preStatus = WL_DISCONNECTED;
    
    wl_status_t newStatus = WiFi.status();
    if (newStatus != preStatus)
    {
        if (newStatus == WL_CONNECTED)
        {
            digitalWrite(LEDRT, HIGH);
            #ifdef _debug_rec
            // Die eigene IP-Adresse anzeigen
            Serial.print("AP Verbindung aufgebaut, lausche auf ");
            Serial.print(WiFi.localIP());
            Serial.print(":");
            Serial.println(PORT);
            #endif
        }
        else
        {
            digitalWrite(LEDRT, LOW);
            #ifdef _debug_rec
            Serial.println("AP Verbindung verloren");
            #endif
            
        }
        preStatus = newStatus;
    }
}


unsigned long StrToHex(String str, byte digits)
{
  char ConvByte[10];
  str.toCharArray(ConvByte, digits);
  return (unsigned long) strtol(ConvByte, NULL, 16);
}


void Nextion_auswerten()
{
  
  int i2=inputString.indexOf("=");
  if (i2 > 0) {    
    #ifdef _debug_snd
    Serial.print("Nextion: ");
    Serial.print(inputString);
    Serial.println("    -   ");
    Serial.print(inputString.substring(i2-2,i2));
    Serial.print("=");
    Serial.println(inputString[i2+1]);                 
    #endif        
    udpServer.beginPacket(IP_BROADCAST, PORT);
    udpServer.print(inputString.substring(i2-2,i2));
    udpServer.print("=");
    udpServer.print(inputString[i2+1]);             
    udpServer.println(" ");   
    udpServer.endPacket();
    TMR_UDP_REC_BLOCK = 2000;  
    //delay(20);          
  }
    
  while(Serial.available()) {Serial.read();}
  inputString = "";

}



