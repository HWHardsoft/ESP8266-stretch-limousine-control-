#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <OneWire.h>
#include <SoftwareSerial.h>

// enable debug outputs via serial Port 1 (USB)
#define _debug



// Portkonfiguration
//#define LEDBL 2     //interne blaue LED an GPIO2 des Wemos
#define LEDRT 14      //rote LED an GPIO5 (D1) des Wemos

// Port B
#define REL_VI_ON_OFF 0x01
#define CEILING_MINUS 0x02
#define CEILING_F 0x04
#define CEILING_PLUS 0x08


// Port C
#define BAR_PLUS  0x01
#define FlOOR_MINUS  0x02
#define FlOOR_F  0x04
#define FlOOR_PLUS  0x08


// Port D
#define REL_I_ON_OFF  0x04
#define REL_II_ON_OFF 0x10
#define REL_III_ON_OFF 0x20
#define REL_IV_ON_OFF 0x40
#define REL_V_ON_OFF 0x80

// MCP_GP0
#define STARS_UP 0x01
#define STARS_DWN 0x02
#define STARS_SQUARE 0x04
#define STARS_LOCK 0x08
#define REL_WALL_UP 0x10
#define REL_WALL_DWN 0x20
#define IN_DOOR_SW 0x40
#define REL_G_LIGHT 0x80


// MCP_GP1
#define REL_FAN_I 0x01
#define REL_FAN_II 0x02
#define REL_FAN_III 0x04
#define REL_CLIMA 0x08
#define REL_HEATING 0x10
#define REL_INTERCOM 0x80

#define Timeout_wall_relay 50
#define Timeout_stars_relay 5
#define Timeout_buttons_blocked 7
#define Timeout_dimmer_blocked 14


SoftwareSerial ESPserial(12, 13); // RX (D6) | TX (D7)

// Name und Passwort des Access Points
#define SSID "xxxxxx"
#define PASSWORD "xxxxx"

// Der Server nimmt Verbindungen auf diesem Port an
#define PORT 5444
#define IP_BROADCAST  "192.168.0.255"
WiFiUDP udpServer;

#define WIFICLIENT_MAX_PACKET_SIZE 100
// Puffer für hereinkommende UDP Nachrichten
char udp_buffer[WIFICLIENT_MAX_PACKET_SIZE+1];

// OneWire DS18S20, DS18B20, DS1822 Temperature sensor
OneWire  ds(2);  // on pin D4 (a 4.7K resistor is necessary)
float celsius;        // Ist Temperatur


// Variables
byte PORTB = 0x00;
byte PORTC = 0x00;
byte PORTD = 0x00;
byte MCP_GP0 = 0x00;
byte MCP_GP1 = 0x00;
int  soll_temperatur=21;  //Soll Temperatur
byte bscheduler = 0;
byte sw_status0 = 0;  // Schalterzustände für Synchronisation
byte sw_status1 = 0;  // Schalterzustände für Synchronisation
byte fan_status = 0;  // Status Gebläse
char *test;
int TMO_WALL=0;          // Timeout Trennwand
int TMO_STARS=0;         // Timeout FB Sternenhimmel
int TMO_BUTTONS=0;       // Timeout für restliche Button
int FB_mode=0;             // Zaehler Mode Fernbedienung M1-6, P1 - P8
byte mood_mode=0;


/** Wird beim Start einmal ausgeführt */
void setup()
{
    // LED blau aus
    // pinMode(LEDBL, OUTPUT);
    // digitalWrite(LEDBL, HIGH);

    // LED rot aus
    pinMode(LEDRT, OUTPUT);
    digitalWrite(LEDRT, LOW);

    // Seriellen Port 2 initialisieren
    ESPserial.begin(9600); // Start the software serial for communication with the ESP8266   
    
    // Gib dem seriellen Port Monitor von der Arduino
    // IDE ein bisschen Zeit.
    #ifdef _debug
    Serial.begin(9600);   // Seriellen Port1 (USB) für Debugging initialisieren.  
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
    switch(bscheduler) 
    {
      // Temperatur abfragen
      case 0: 
        read_temperature();
        break;

      // Heizungssteuerung
      case 6: 
        clima_control();
        break;
      
        
      // Statusmeldung (Temperaturen & Lüfter senden)
      case 3:  
        udp_message_status1();
        break;

      // Statusmeldung (Schalterzustände senden)
      case 7:  
        udp_message_status2();
        break;

      // Schalter bearbeiten
      case 8:
        switches();
        break;
      
      // Alle Ports am uC und I2C Expander setzen
      case 9:  
        transmit_cmd_ports();
        break;      
      
    }
    bscheduler++;
    if (bscheduler > 9) bscheduler = 0;
    

    // Timeouts
    // Timeout Trennwand
    if (TMO_WALL) TMO_WALL--;
    if (TMO_WALL == 0) {
      // Motorrelais ausschalten
      MCP_GP0 &= ~(REL_WALL_UP + REL_WALL_DWN);
    }  

    // Timeout FB Sternenhimmel
    if (TMO_STARS) TMO_STARS--;
    if (TMO_STARS == 0) {
      // Relais für FB-Tasten ausschalten
      MCP_GP0 &= ~(STARS_UP + STARS_DWN + STARS_LOCK + STARS_SQUARE);
    }  

    // Timeout Tasten blockieren um Doppelbedienung zu vermeiden
    if (TMO_BUTTONS) TMO_BUTTONS--;
    if (TMO_BUTTONS == 0) { 
       PORTB &= ~CEILING_PLUS;
       PORTB &= ~CEILING_MINUS;
    }   
    delay(100);

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
        
        // UDP Buffer auswerten

        //-- Status Schalter ---------------------------------------------------------
        //-- bt0 All LIGHTS ----------------------------------------------------------
        if (strstr(udp_buffer, "s0=1")) {        
            sw_status0 |= 0x97;
            sw_status1 |= 0x01;          
            sw_status0 &= ~0x20; //STROBE  
            MCP_GP1 |= REL_INTERCOM;
            
        } 
        else if(strstr(udp_buffer, "s0=0")) {
            sw_status0 &= ~0x97;
            sw_status1 &= ~0x01;
            MCP_GP1 &= ~REL_INTERCOM;
        }        
        
        //-- bt1 SPOTS --------------------------------------------------------------
        if (strstr(udp_buffer, "s1=1")) {        
            sw_status0 |= 0x02;
        } 
        else if(strstr(udp_buffer, "s1=0")) {
            sw_status0 &= ~0x02;
        }        

        //-- bt2 FLOOR --------------------------------------------------------------
        if (strstr(udp_buffer, "s2=1")) {        
            sw_status0 |= 0x04;
        } 
        else if(strstr(udp_buffer, "s2=0")) {
            sw_status0 &= ~0x04;
        }        
/*
        //-- bt3 STARS LOCK-----------------------------------------------------------
        if (strstr(udp_buffer, "s3=1")) {        
            sw_status0 |= 0x08;
        } 
        else if(strstr(udp_buffer, "s3=0")) {
            sw_status0 &= ~0x08;
        }        
*/
        //-- bt4 STARS ---------------------------------------------------------------
        if (strstr(udp_buffer, "s4=1")) {        
            sw_status0 |= 0x10;
        } 
        else if(strstr(udp_buffer, "s4=0")) {
            sw_status0 &= ~0x10;
        }        

        //-- bt5 STROBE ---------------------------------------------------------------
        if (strstr(udp_buffer, "s5=1")) {        
            sw_status0 |= 0x20;
        } 
        else if(strstr(udp_buffer, "s5=0")) {
            sw_status0 &= ~0x20;
        }        

        //-- bt6 LASER --------------------------------------------------------------
        if (strstr(udp_buffer, "s6=1")) {        
            sw_status0 |= 0x40;
        } 
        else if(strstr(udp_buffer, "s6=0")) {
            sw_status0 &= ~0x40;
        }        

        //-- bt7 RIM LIGHTS ----------------------------------------------------------
        if (strstr(udp_buffer, "s7=1")) {        
            sw_status0 |= 0x80;
        } 
        else if(strstr(udp_buffer, "s7=0")) {
            sw_status0 &= ~0x80;
        }        

        //-- bt8 BACKLIGHT------------------------------------------------------------
        if (strstr(udp_buffer, "s8=1")) {        
            sw_status1 |= 0x01;
        } 
        else if(strstr(udp_buffer, "s8=0")) {
            sw_status1 &= ~0x01;
        }     

        
        //-- Status Tasten ------------------------------------------------------------
        //-- b0 Trennwand hoch/zu -----------------------------------------------------
        if (strstr(udp_buffer, "b0=1")) {        
             TMO_WALL = Timeout_wall_relay;
             MCP_GP0 |= REL_WALL_UP;
             MCP_GP0 &= ~REL_WALL_DWN;
             transmit_cmd_ports();
        } 
        else if(strstr(udp_buffer, "b0=0")) {
             MCP_GP0 &= ~REL_WALL_UP;
             MCP_GP0 &= ~REL_WALL_DWN;
             transmit_cmd_ports();
        }     
        //-- b1 Trennwand runter/auf --------------------------------------------------
        else if (strstr(udp_buffer, "b1=1")) {        
             TMO_WALL = Timeout_wall_relay;
             MCP_GP0 &= ~REL_WALL_UP;
             MCP_GP0 |= REL_WALL_DWN;
             transmit_cmd_ports();
        } 
        else if(strstr(udp_buffer, "b1=0")) {             
             MCP_GP0 &= ~REL_WALL_UP;
             MCP_GP0 &= ~REL_WALL_DWN;    
             transmit_cmd_ports();      
        }     

        //-- b7 MOOD -------------------------------------------------------------------
        else if (strstr(udp_buffer, "b7=1")) {      
          if (TMO_BUTTONS == 0) {   
            select_mood_mode();
            TMO_BUTTONS = Timeout_buttons_blocked;
          }   
        } 
        
        //-- b9 SET LEFT ---------------------------------------------------------------
        else if (strstr(udp_buffer, "b9=1")) {                          
          if (TMO_BUTTONS == 0) {        
             PORTB |= CEILING_PLUS;
             transmit_cmd_ports(); 
             TMO_BUTTONS = Timeout_buttons_blocked;
          }   
        } 
        //-- b10 SET RIGHT -------------------------------------------------------------
        else if (strstr(udp_buffer, "ba=1")) {        
          if (TMO_BUTTONS == 0) {        
             PORTB |= CEILING_MINUS;
             transmit_cmd_ports(); 
             TMO_BUTTONS = Timeout_buttons_blocked;
          }   
            
        } 
        //-- bt11 SET FLOOR ------------------------------------------------------------
        else if (strstr(udp_buffer, "b2=1")) {
          if (TMO_BUTTONS == 0) {        
             FB_protocol();
             TMO_BUTTONS = Timeout_buttons_blocked;
          }  
            
        } 

        //-- b11 STARS LOCK -----------------------------------------------------------
        else if (strstr(udp_buffer, "bb=1")) {     
            
             MCP_GP0 |= STARS_LOCK;
             TMO_STARS = Timeout_stars_relay;
             #ifdef _debug
             Serial.print("Stars Lock");
             #endif
             transmit_cmd_ports();
            
        } 

        //-- b8 DIMMER UP --------------------------------------------------------------
        else if (strstr(udp_buffer, "b8=1")) {  
           if (TMO_BUTTONS == 0) {        
             transmit_cmd_IR("61D6609F");
             TMO_BUTTONS = Timeout_dimmer_blocked;
          }       
            
        } 

        //-- b12 DIMMER DWN ------------------------------------------------------------
        else if (strstr(udp_buffer, "bc=1")) {     
          if (TMO_BUTTONS == 0) {        
             transmit_cmd_IR("61D67887");
             TMO_BUTTONS = Timeout_dimmer_blocked;
          }                 
            
        } 

        //-- Status Klima -------------------------------------------------------------
        //-- j0 FAN -------------------------------------------------------------------
        else if (strstr(udp_buffer, "j0=0")) {        
            fan_status = 0;
        } 
        else if(strstr(udp_buffer, "j0=1")) {
            fan_status = 1;
        }        
        else if(strstr(udp_buffer, "j0=2")) {
            fan_status = 2;
        }        
        else if(strstr(udp_buffer, "j0=3")) {
            fan_status = 3;
        }        

        //-- n2 SOLL Temperatur ------------------------------------------------------        
        else if (strstr(udp_buffer, "n2="))
        {            
            test = strstr(udp_buffer, "n2=");
            soll_temperatur = *(test + 3) - 0x30;            
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
            #ifdef _debug
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
            #ifdef _debug
            Serial.println("AP Verbindung verloren");
            #endif
            
        }
        preStatus = newStatus;
    }
}



void read_temperature() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) 
  {
    ds.reset_search();
    delay(250);
    return;
  }
 
  if (OneWire::crc8(addr, 7) != addr[7]) 
  {      
      return;
  }
 
 
  // the first ROM byte indicates which chip
  switch (addr[0]) 
  {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:      
      return;
  } 
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end  
  //delay(1000);
  delay(100);
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
 
  for ( i = 0; i < 9; i++) 
  {           
    data[i] = ds.read();
  }
 
  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) 
    {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } 
  else 
  {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
 
  }
  celsius = (float)raw / 16.0;
}


// UDP Status Temperaturen und Gebläse senden
void udp_message_status1() {
    udpServer.beginPacket(IP_BROADCAST, PORT);
    udpServer.print("ST=");
    udpServer.print(soll_temperatur);
    udpServer.print(" IT=");
    udpServer.print((int) celsius);
    udpServer.print(" FN=");
    udpServer.print(fan_status);
    /*
    udpServer.print(" SW=");
    String s1 = String(sw_status0, HEX);
    if (s1.length() < 2) s1 = "0" + s1;
    s1 = String(sw_status1, HEX) + s1;
    if (s1.length() < 4) s1 = "0" + s1;    
    udpServer.print(s1);
    */
    udpServer.println(" ");
    udpServer.endPacket();
  
}

// UDP Status Schalter senden
void udp_message_status2() {
    udpServer.beginPacket(IP_BROADCAST, PORT);
    udpServer.print("SW=");
    String s1 = String(sw_status0, HEX);
    if (s1.length() < 2) s1 = "0" + s1;
    s1 = String(sw_status1, HEX) + s1;
    if (s1.length() < 4) s1 = "0" + s1;    
    udpServer.print(s1);
    udpServer.println(" ");
    udpServer.endPacket();
  
}

// Funktionen Schalter
void switches() {
   // RIM Lights
   if (sw_status0 & 0x80) {        
      PORTD |= REL_III_ON_OFF;
   } else {
      PORTD &= ~REL_III_ON_OFF;
   }           

   // Backlight
   if (sw_status1 & 0x01) {        
      PORTB |= REL_VI_ON_OFF;
   } else {
      PORTB &= ~REL_VI_ON_OFF;
   }           

   // SPOTS
   if (sw_status0 & 0x02) {   
      MCP_GP0 |= IN_DOOR_SW;     
   } else {
      MCP_GP0 &= ~IN_DOOR_SW;     
   }           

   // STROBE
   if (sw_status0 & 0x20) {        
    
      PORTD &= ~REL_IV_ON_OFF;
      MCP_GP1 |= REL_INTERCOM;
      
   } else {
       PORTD |= REL_IV_ON_OFF;
       // MCP_GP1 &= ~REL_INTERCOM;
   }           

   // STARS
   if (sw_status0 & 0x10) {        
      PORTD |= REL_II_ON_OFF;
   } else {
      PORTD &= ~REL_II_ON_OFF;
   }           

   // LASER
   if (sw_status0 & 0x40) {        
      PORTD |= REL_I_ON_OFF;
   } else {
      PORTD &= ~REL_I_ON_OFF;
   }           

   //FLOOR
   if (sw_status0 & 0x04) {        
      PORTD |= REL_V_ON_OFF;
   } else {
      PORTD &= ~REL_V_ON_OFF;
   }           
  
}

// Klima und Heizungssteuerung - Zweipunktregler
void clima_control() {
    // luefter
    if (fan_status == 1) {
      // Stufe 1
      MCP_GP1 |= REL_FAN_I;
      MCP_GP1 &= ~REL_FAN_II;
      MCP_GP1 &= ~REL_FAN_III;
      
    } else if (fan_status == 2) {
      // Stufe 1
      MCP_GP1 |= REL_FAN_I;
      MCP_GP1 |= REL_FAN_II;
      MCP_GP1 &= ~REL_FAN_III;
    
      
    } else if (fan_status == 3) {
      // Stufe 3
      MCP_GP1 |= REL_FAN_I;
      MCP_GP1 |= REL_FAN_II;
      MCP_GP1 |= REL_FAN_III;

    } else {
      // AUS
      MCP_GP1 &= ~REL_FAN_I;
      MCP_GP1 &= ~REL_FAN_II;
      MCP_GP1 &= ~REL_FAN_III;

    }

    // Kuehlen oder Heizen?
    if (celsius < soll_temperatur) {
      // Heizen
      MCP_GP1 |= REL_HEATING;
      MCP_GP1 &= ~REL_CLIMA;
       
    } else if ((celsius > (soll_temperatur + 1)) && (celsius > 15)) {
      // Kuehlen
      MCP_GP1 &= ~REL_HEATING;
      MCP_GP1 |= REL_CLIMA;

    } else {
      // Temperatur stimmt!
      MCP_GP1 &= ~REL_HEATING;
      MCP_GP1 &= ~REL_CLIMA;   
    }
}


// MOOD modi wählen
void select_mood_mode() {
  switch(mood_mode)
  {
    case 0: // white
      sw_status0 = 0x02;  // Spots an, Strobo aus, Decke an
      sw_status1 = 0;
      switches();
      MCP_GP1 |= REL_INTERCOM;
      transmit_cmd_ports();   
      delay(200);
      transmit_cmd_IR("61D68877");  // W
      break;
      
    case 1: // Disco
      sw_status0 = 0xF4;  // Laser, Stars, Rim Light und Floor, Strobo an
      sw_status1 = 0;
      switches();
      MCP_GP1 |= REL_INTERCOM;
      transmit_cmd_ports();   
      delay(200);
      transmit_cmd_IR("61D600FF");  // P8
      break;

    case 2: // Nacht
      sw_status0 = 0x10;  // Stars, 
      sw_status1 = 0x00;
      switches();
      MCP_GP1 |= REL_INTERCOM; // Trennwandbeleuchtung & Deckenbalken an.
      transmit_cmd_ports();   
      delay(200);
      transmit_cmd_IR("61D648B7");  // M3
      break;
      
  }   
  mood_mode++;

  if (mood_mode > 2) mood_mode = 0;
}

//FB Protokoll auswählen
void FB_protocol() {
  switch(FB_mode)
  {
    case 0: // Button W1
      transmit_cmd_IR("61D68877");
      break;

    case 1: // Button m1
      transmit_cmd_IR("61D6C837");
      break;

    case 2: // Button m2
      transmit_cmd_IR("61D608F7");
      break;

    case 3: // Button m3
      transmit_cmd_IR("61D648B7");
      break;

    case 4: // Button m4
      transmit_cmd_IR("61D658A7");
      break;
      
    case 5: // Button m5
      transmit_cmd_IR("61D6C03F");
      break;
      
    case 6: // Button m6
      transmit_cmd_IR("61D640BF");
      break;
      
    case 7: // Button P1
      transmit_cmd_IR("61D610EF");
      break;
      
    case 8: // Button P2
      transmit_cmd_IR("61D6906F");
      break;
                 
    case 9: // Button P3
      transmit_cmd_IR("61D6D827");
      break;
      
    case 10: // Button P4
      transmit_cmd_IR("61D6F807");
      break;
      
    case 11: // Button P5
      transmit_cmd_IR("61D630CF");
      break;

    case 12: // Button P6
      transmit_cmd_IR("61D618E7");
      break;
    
    case 13: // Button P7
      transmit_cmd_IR("61D628D7");
      break;

    case 14: // Button P8
      transmit_cmd_IR("61D600FF");
      break;      
  }
  FB_mode++;

  if (FB_mode > 14) FB_mode = 0;
}


// -- Hilfsroutinen für Portexpander und Mikrocontroller -------------------------------------------------------------------------
// Übertragung der Statusdaten für alle IOs an den Mikrocontroller
void transmit_cmd_ports() {
  String S1 = "IO=";
  //convert PortB
  String S2 = String(PORTB, HEX);
  if (PORTB < 16) S2 = "0" + S2;
  S1 = S1 + S2;
  // convert PortC
  S2 = String(PORTC, HEX);
  if (PORTC < 16) S2 = "0" + S2;
  S1 = S1 + S2;
  // convert PortD  
  S2 = String(PORTD, HEX);
  if (PORTD < 16) S2 = "0" + S2;
  S1 = S1 + S2;
  // convert MCP_GP0  
  S2 = String(MCP_GP0, HEX);
  if (MCP_GP0 < 16) S2 = "0" + S2;
  S1 = S1 + S2;
  // convert MCP_GP1  
  S2 = String(MCP_GP1, HEX);
  if (MCP_GP1 < 16) S2 = "0" + S2;
  S1 = S1 + S2;
  
  #ifdef _debug  
  Serial.println(S1);
  #endif
  ESPserial.println(S1);  
} 

// Übertragung eines IR-Befehls an den Mikrocontroller
void transmit_cmd_IR(String S1) {  
  if (S1.length() != 8) return;
  S1 = "IR=" + S1;
  #ifdef _debug
  Serial.println(S1);
  #endif
  ESPserial.println(S1);   
} 

