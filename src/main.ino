/*------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Program:      SolarControl mit 2 Tempsensoren PT1000 zur Steuerung von 4-fach Relais.
                Außerdem enthalten ist eine Füllstandsmessung mit Pegelsonde für einen Heizöltank,
                sowie ein Bewegungsmelder zur Steuerung von Licht und Displayanzeige

  Description:  Dieses Programm misst die analolge Spannung an einem Port, welche von einem Drucksensor
                0-5m Meßhöhe und 4-20mA Stromschnittstelle erzeugt wird.
                Voraussetzung ist ein Meßwandler, welcher die 24V Versorgungsspannung an
                den Drucksensor liefert und 0-3,3V analoge Spannung ausgibt.
                Dienste:
                DHCP, wenn vorhanden, sonst wird eine feste IP mit 192.168.178.11 vergeben.

  Hardware:     Arduino MEGA 2560 PRO
                W5500 lite Netzwerk Shield
                OLED Display SSD1309 / SSD1306

        Pin-Ports:
        A10 = Analog IN (Analog-Sensor Heizöltank)
        D46 = Status 3-Wege-Ventil POOL

        I2C-Display:
        D20 = SDA
        D21 = SCL

        Relay-Pins:
        D39 = CH1 3-Wege-Ventil POOL
        D37 = CH2 SOLAR Umwälzpumpe
        D35 = CH3 Warmwasser Zirkulationspumpe
        D33 = CH4 Reserve

        Status-LED:
        D23 = LED_VALVE //Status-LED 3-Wege-Ventil
        D25 = LED_SOLAR //Status-LED Solarumwälzpumpe
        D27 = LED_ZIRC  //Status-LED Zirkulationspumpe
        D29 = LED_LIGHT //Status-LED Licht
        D31 = LED_AUTO  //Status-LED Automatik-Modus

        Button-Pin:
        D19 = Mode-Button

        PIR-Pin:
        D22 = Bewegungsmelder - Power
        D18 = Bewegungsmelder - Data

        T1 MAX31865-Pins:
        A14 = CS
        D32 = MOSI 
        D34 = MISO 
        D36 = SCK  

        T2 MAX31865-Pins:
        D3  = CS
        D4  = MOSI 
        D6  = MISO 
        D8  = SCK  

  Date:         25.02.2022
  Modified:     Release Version 1.0


  Author:       Jürgen Thaler

  Author:       Ethernet part: W.A. Smith, http://startingelectronics.com
                
  LICENSE:    MIT License

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

const String versions = "1.0";    // vor jeder änderung hier hochzählen

// Include some libraries
#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>    // Adafruit's Header file
#include <Adafruit_SPIDevice.h>  
#include <Adafruit_GFX.h>
#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#include <Wire.h>

#define USE_SERIAL  Serial

#define MODE_PIN 19         //Taster zum umschalten des MODE
#define MODE_POOL 0
#define MODE_PUFFERSPEICHER 1
#define MODE_AUTO 2
#define MODE_COUNT 10
bool setmode = false;
const char *modes[] = {"Pool", "Puffer", "Auto"};
int mode_count = MODE_COUNT;
int mode = MODE_AUTO;
int old_mode = MODE_AUTO;
int pin_stat;
int old_stat;



#define VALVE_PIN 39         //3-Wege-Ventil POOL / PUFFERSPEICHER
#define VALVE_POOL LOW
#define VALVE_PUFFERSPEICHER HIGH
int valve = VALVE_POOL;
int new_valve = VALVE_POOL;
int reason = 0;
const char            *valves[]                         = {"Pool", "Puffer"};
const char            *reasons[]                        = {"manuelle Steuerung", "Automatik", "ausgeschaltet"};

#define SOLAR_PIN 37         //Solar-Umwälzpumpe
#define ZIRC_PIN 35          //Zirkulationspumpe
#define LIGHT_PIN 33         //Licht

#define VALVE_STATUS_PIN 46  // STATUS des 3-Wege-Ventils (Abfrage des Ventils)

#define PIR_POWER_PIN 22     // Bewegungsmelder - Power
#define PIR_PIN 18           // Bewegungsmelder - Data
const uint32_t        PIR_TimerON                       = 60000;      //Zeit festlegen in ms, wie lange die LED leuchten soll
uint32_t              PIR_TimerComparison               = 0;          //Vergleichswert für den Zähler (Startwert muss 0 sein)

#define LED_VALVE_PIN 23     //Status-LED 3-Wege-Ventil
#define LED_SOLAR_PIN 25     //Status-LED Solarumwälzpumpe
#define LED_ZIRC_PIN 27      //Status-LED Zirkulationspumpe
#define LED_LIGHT_PIN 29     //Status-LED Licht
#define LED_AUTO_PIN 31      //Status-LED Automatik-Modus

// ##############################################################################################################################
// ---- HIER die Anpassungen vornehmen ----
// ##############################################################################################################################
// Hier die maximale Füllmenge des Behälters angeben. Dies gilt nur für symmetrische Behälter.
const float max_liter = 4500;
// Analoger Wert bei maximalem Füllstand (wird alle 30 Sekungen in der seriellen Konsole mit 9600 Baud gezeigt)
const int analog_max = 715;

// Dichte der Flüssigkeit - Bei Heizöl bitte "1.086" eintragen, aber nur wenn die Kalibrierung mit Wasser erfolgt ist!
// Bei Kalibrierung mit Wasser bitte "1.0" eintragen
const float dichte = 1.0;

// IP Adresse und Port des MQTT Servers
IPAddress mqttserver(192, 168, 178, 55);
const int mqttport = 1883;
// Wenn der MQTT Server eine Authentifizierung verlangt, bitte folgende Zeile aktivieren und Benutzer / Passwort eintragen
//#define mqttauth
const char mqttuser = "USER";
const char mqttpass = "PASSWORD";

// IP Adresse, falls kein DHCP vorhanden ist. Diese Adresse wird nur verwendet, wenn der DHCP-Server nicht erreichbar ist.
IPAddress ip(192, 168, 178, 11);

// MAC-Addresse bitte anpassen! Sollte auf dem Netzwerkmodul stehen. Ansonsten eine generieren.
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0A };

// TEMPERATURDIFFERENZ / HYSTERESE -> Hier kann die Temperaturdifferenz, wann die SOLAR_PUMP ein- und ausgeschaltet werden soll, eingestellt werden.
const byte hysterese_on = 7;
const byte hysterese_off = 2;

char buf[40];

long lastReconnectAttempt = 0;

// ##############################################################################################################################
// AB hier nichts mehr ändern!
// (Ausser ihr wisst, was ihr tut)
// ##############################################################################################################################

// MAX31865 P1000 Temperatur-Sensoren
// Use software SPI: CS, DI, DO, CLK
// MEGA 2560 PRO Pins:  max_1 =             A14, D32, D34, D36)
Adafruit_MAX31865 max_1 = Adafruit_MAX31865(68, 32, 34, 36);       // CS, MOSI, MISO, SCK
// MEGA 2560 PRO Pins:  max_2 =             D3, D4, D6, D8)
Adafruit_MAX31865 max_2 = Adafruit_MAX31865(3, 4, 6, 8);      // CS, MOSI, MISO, SCK

#define RREF      4300.0 // 4.3Kohm 
#define RNOMINAL  1000.0 // PT1000

float operatMax31865_1(void) {
  float solar_temp = max_1.temperature(RNOMINAL, RREF);
  return solar_temp;
}

float operatMax31865_2(void) {
  float puffer_temp = max_2.temperature(RNOMINAL, RREF);
  return puffer_temp;
}
//MQTT-Subscribe
float pool_temp;

#define POOL_AUTO 0
#define POOL_MAN 1
#define POOL_OFF 2
int pool_mode = POOL_OFF;
int new_pool_mode = POOL_OFF;
const char *pool_modes[] = {"auto", "manuell", "off"};

#define SOLAR_POOL 0
#define SOLAR_PUFFER 1
#define SOLAR_OFF 2
int solar_mode = SOLAR_PUFFER;
int new_solar_mode = SOLAR_PUFFER;
const char *solar_modes[] = {"Pool", "Puffer", "off"};

// No more delay
unsigned long startMillis;  // some global variables available anywhere in the program
unsigned long hstartMillis;  // some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long minute = 60000;  // one minute
const unsigned long zehnsekunden = 10000; // ten seconds
const unsigned long sekunde = 1000;  // one seconds
const unsigned long hsekunde = 500;  // Half second

unsigned long pinMillis;
unsigned long lastMillis = 0;

int secs = 0, mins = 0, hours = 0, days = 0;
char uptime[25];

float percent;
float liter;
boolean LCD_Page;

// Analog IN (Heizöltank)
int analogPin = A10;
const int messungen = 60;     // Anzahl Messungen
int myArray[messungen];       // Array für Messwerte
float analog = 0.0;           // Durchschnittswert
int pointer = 0;              // Pointer für Messung

// MQTT global vars
#include <PubSubClient.h>
unsigned int send_interval = 10; // the sending interval of indications to the server, by default 10 seconds
#define MQTT_KEEPALIVE 60;

boolean mqttconnected = false;

// MQTT definitions
void MqttCallback(char *topic, byte *payload, unsigned int length);
EthernetClient ethClient;
PubSubClient mqttclient;
// (mqttserver, mqttport, MqttCallback, ethClient);
#define MQTT_ID "SolarControl"

boolean reconnect() {
  if (mqttclient.connect(MQTT_ID, "SolarControl/LWT", 0, true, "Offline")) {
    // Once connected, publish an announcement...
    mqttclient.publish("SolarControl/LWT","Online", true);
        //USE_SERIAL.println("Connected to Mqtt-Server");
    // ... and resubscribe
    mqttclient.subscribe("SolarControl/LWT");
    mqttclient.loop();    
    mqttclient.subscribe("SolarControl/cmd/Mode");
    mqttclient.loop();
    mqttclient.subscribe("SolarControl/cmd/SOLAR_PUMP");
    mqttclient.loop();
    mqttclient.subscribe("SolarControl/cmd/ZIRC_PUMP");
    mqttclient.loop();
    mqttclient.subscribe("SolarControl/cmd/LIGHT");
    mqttclient.loop();
    mqttclient.subscribe("SolarControl/cmd/BWM");
    mqttclient.loop();
    mqttclient.subscribe("SolarControl/Mode");
    mqttclient.loop();
    mqttclient.subscribe("Heizoel/cmd/max_liter");
    mqttclient.loop();
    mqttclient.subscribe("POOL/Solar_Mode");
    mqttclient.loop();
    mqttclient.subscribe("POOL/temperature");
    mqttclient.loop();
    mqttclient.subscribe("POOL/Pool_Mode");
    mqttclient.loop();
        //USE_SERIAL.println("subscribing to SolarControl/cmd/Mode & SolarControl/LWT");
  }
  return mqttclient.connected();
}

char pl[30];

// Declare subs
void Mqttpublish();

void defaultEthernet(void) {
  Ethernet.begin(mac, ip);  // initialize Ethernet device
}

void Uptime() {
  secs++;
  secs = secs % 60;
  if (secs == 0) {
    mins++;
    mins = mins % 60;
    if (mins == 0) {
      hours++;
      hours = hours % 24;
      if (hours == 0) {
        days++;
        days = days % 10000;   // Nach 9999 Tagen zurück auf 0 (das sind 27 Jahre....)
      }
    }
  }
  sprintf(uptime, "%4dd %2dh %2dm", days, hours, mins);

  if (secs == 0) {        // Every Minute
    //USE_SERIAL.print(F("Uptime: "));
    //USE_SERIAL.println(uptime);
    // MQTT reconnect timeout
    //Mqttpublish();
  }
  if (secs % send_interval == 0) { // Alle 30 Sekunden
    LCD_Page = !LCD_Page;
    Mqttpublish();
    MAX31865();
  }
  if (mins == 0 && secs == 0) {      // Jede Stunde
    //USE_SERIAL.println(F("ONE HOUR"));
  }
}

void ReadAnalog() {
  // read the analog value and build floating middle
  myArray[pointer++] = analogRead(analogPin);      // read the input pin
  // myArray[pointer++] = 352;

  pointer = pointer % messungen;

  // Werte aufaddieren
  for (int i = 0; i < messungen; i++) {
    analog = analog + myArray[i];
  }
  // Summe durch Anzahl - geglättet
  analog = analog / messungen;

  percent = min(100 * analog / analog_max, 100);

  float calc = max_liter / analog_max;    // calculate percent
  calc = calc * dichte;                     // calculate dichte
  liter = min(analog * calc, max_liter);    // calculate liter

  //USE_SERIAL.print(F("Analog: "));
  //USE_SERIAL.println(analog);
  //USE_SERIAL.print(F("Prozent: "));
  //USE_SERIAL.println(percent);
  //USE_SERIAL.print(F("Liter: "));
  //USE_SERIAL.println(liter);
}


void MAX31865() {
  /*--------------------------------------------------------------
    MAX31865 Temperaturen Senden
    --------------------------------------------------------------*/
  uint16_t rtd1 = max_1.readRTD();
  uint16_t rtd2 = max_2.readRTD();

  max_1.enable50Hz(1);
  max_2.enable50Hz(1);

  //USE_SERIAL.print(F("RTD1 value: ")); //USE_SERIAL.println(rtd1);
  //USE_SERIAL.print(F("RTD2 value: ")); //USE_SERIAL.println(rtd2);
  float ratio1 = rtd1;
  float ratio2 = rtd2;
  ratio1 /= 32768;
  ratio2 /= 32768;
  //USE_SERIAL.print(F("Ratio PT1 = ")); //USE_SERIAL.println(ratio1,8);
  //USE_SERIAL.print(F("Ratio PT2 = ")); //USE_SERIAL.println(ratio2,8);
  //USE_SERIAL.print(F("Resistance PT1 = ")); //USE_SERIAL.println(RREF*ratio1,8);
  //USE_SERIAL.print(F("Resistance PT2 = ")); //USE_SERIAL.println(RREF*ratio2,8);
  //USE_SERIAL.print(F("Temperature PT1 = ")); //USE_SERIAL.println(max_1.temperature(RNOMINAL, RREF));
  //USE_SERIAL.print(F("Temperature PT2 = ")); //USE_SERIAL.println(max_2.temperature(RNOMINAL, RREF));

    // Check and print any faults
  uint8_t fault1 = max_1.readFault();
  if (fault1) {
    //USE_SERIAL.print(F("Fault 0x")); //USE_SERIAL.println(fault1, HEX);
    if (fault1 & MAX31865_FAULT_HIGHTHRESH) {
      //USE_SERIAL.println(F("PT1: RTD High Threshold"));
      mqttclient.publish("SolarControl/ERROR", "PT1: RTD High Threshold"); 
    }
    if (fault1 & MAX31865_FAULT_LOWTHRESH) {
      //USE_SERIAL.println(F("PT1: RTD Low Threshold"));
      mqttclient.publish("SolarControl/ERROR", "PT1: RTD Low Threshold"); 
    }
    if (fault1 & MAX31865_FAULT_REFINLOW) {
      //USE_SERIAL.println(F("PT1: REFIN- > 0.85 x Bias"));
      mqttclient.publish("SolarControl/ERROR", "PT1: REFIN- > 0.85 x Bias"); 
    }
    if (fault1 & MAX31865_FAULT_REFINHIGH) {
      //USE_SERIAL.println(F("PT1: REFIN- < 0.85 x Bias - FORCE- open"));
      mqttclient.publish("SolarControl/ERROR", "PT1: REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault1 & MAX31865_FAULT_RTDINLOW) {
      //USE_SERIAL.println(F("PT1: RTDIN- < 0.85 x Bias - FORCE- open"));
      mqttclient.publish("SolarControl/ERROR", "PT1: RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault1 & MAX31865_FAULT_OVUV) {
      //USE_SERIAL.println(F("PT1: Under/Over voltage"));
      mqttclient.publish("SolarControl/ERROR", "PT1: Under/Over voltage"); 
    }
    max_1.clearFault();
  }
  
  uint8_t fault2 = max_2.readFault();
  if (fault2) {
    //USE_SERIAL.print(F("Fault 0x")); //USE_SERIAL.println(fault2, HEX);
    if (fault2 & MAX31865_FAULT_HIGHTHRESH) {
      //USE_SERIAL.println(F("PT2: RTD High Threshold"));
      mqttclient.publish("SolarControl/ERROR", "PT2: RTD High Threshold"); 
    }
    if (fault2 & MAX31865_FAULT_LOWTHRESH) {
      //USE_SERIAL.println(F("PT2: RTD Low Threshold"));
      mqttclient.publish("SolarControl/ERROR", "PT2: RTD Low Threshold"); 
    }
    if (fault2 & MAX31865_FAULT_REFINLOW) {
      //USE_SERIAL.println(F("PT2: REFIN- > 0.85 x Bias"));
      mqttclient.publish("SolarControl/ERROR", "PT2: REFIN- > 0.85 x Bias"); 
    }
    if (fault2 & MAX31865_FAULT_REFINHIGH) {
      //USE_SERIAL.println(F("PT2: REFIN- < 0.85 x Bias - FORCE- open"));
      mqttclient.publish("SolarControl/ERROR", "PT2: REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault2 & MAX31865_FAULT_RTDINLOW) {
      //USE_SERIAL.println(F("PT2: RTDIN- < 0.85 x Bias - FORCE- open"));
      mqttclient.publish("SolarControl/ERROR", "PT1: RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault2 & MAX31865_FAULT_OVUV) {
      //USE_SERIAL.println(F("PT2: Under/Over voltage"));
      mqttclient.publish("SolarControl/ERROR", "PT1: Under/Over voltage"); 
    }
    max_2.clearFault();
  }
  //USE_SERIAL.println(F("Debug: End of MAX31865"));
  //USE_SERIAL.println();
}


void setup() {
  //USE_SERIAL.begin(9600);       // for //USE_SERIAL.printging

  /*--------------------------------------------------------------
     Ethernet init
    --------------------------------------------------------------*/
  Ethernet.begin(mac, ip);
  delay(1500);
  lastReconnectAttempt = 0;

    /*
  if (Ethernet.begin(mac) == 0) {
    //USE_SERIAL.println(F("Failed config using DHCP"));
    // DHCP not working, switch to static IP
    defaultEthernet();
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      //USE_SERIAL.println(F("Eth shield not found"));
    } else if (Ethernet.linkStatus() == LinkOFF) {
      //USE_SERIAL.println(F("Eth cable not conn"));
    }
  }*/

    // start MQTT client
  MqttConnect(mqttuser, mqttpass);

    /*--------------------------------------------------------------
     LCD init
    --------------------------------------------------------------*/
  
    // Display starten
  u8g2.begin();
  u8g2.setDisplayRotation(U8G2_R3);       // Display um 90Grad drehen
  u8g2.enableUTF8Print();

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR14_te);
    u8g2.drawStr(8,15,"Solar-");
    u8g2.drawStr(0,32,"Control");

    u8g2.setFont(u8g2_font_baby_tr);    
    u8g2.drawStr(0,50,"Juergen Thaler");
    u8g2.drawStr(43,58,"2022");

    u8g2.setFont(u8g2_font_baby_tr);
    u8g2.drawStr(0,125,"Version:");

    u8g2.setFont(u8g2_font_baby_tn);
    u8g2.setCursor(40, 125);
    u8g2.print(versions);
  } while ( u8g2.nextPage() );
  delay(2000);

  /*--------------------------------------------------------------
     Milliseconds start
    --------------------------------------------------------------*/
  startMillis = millis();  //initial start time

  /*-------------------------------------------------------------------
     Setup Pins für Valve and mode-setting
  */
  pinMode(MODE_PIN, INPUT_PULLUP);
  pin_stat = old_stat = digitalRead(MODE_PIN);

  pinMode(VALVE_STATUS_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  pinMode(PIR_POWER_PIN, OUTPUT);
    
  pinMode(VALVE_PIN, OUTPUT);
  pinMode(SOLAR_PIN, OUTPUT);
  pinMode(ZIRC_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);


  /*-------------------------------------------------------------------
     Setup LED-Pins für Valve und Relais
  */  
  pinMode(LED_VALVE_PIN, OUTPUT);
  pinMode(LED_SOLAR_PIN, OUTPUT);
  pinMode(LED_ZIRC_PIN, OUTPUT);
  pinMode(LED_LIGHT_PIN, OUTPUT);
  pinMode(LED_AUTO_PIN, OUTPUT);

  /*-------------------------------------------------------------------
     PINS beim Neustart schalten
  */  
  digitalWrite(VALVE_PIN, HIGH);
  digitalWrite(SOLAR_PIN, HIGH);
  digitalWrite(ZIRC_PIN, HIGH);
  digitalWrite(LIGHT_PIN, HIGH);
  digitalWrite(LED_VALVE_PIN, HIGH);
  digitalWrite(LED_SOLAR_PIN, HIGH);
  digitalWrite(LED_ZIRC_PIN, HIGH);
  digitalWrite(LED_LIGHT_PIN, HIGH);
  digitalWrite(LED_AUTO_PIN, HIGH);
  digitalWrite(PIR_POWER_PIN, HIGH);

  /*--------------------------------------------------------------
     MAX31865 init
    --------------------------------------------------------------*/
  max_1.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  max_2.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary

  // Zeige IP-Adresse im Debugfenster
  //USE_SERIAL.print(F("IP: "));
  //USE_SERIAL.println(Ethernet.localIP());

}

void MqttConnect(char *user, char* pass) {

  mqttclient.setClient(ethClient);
  mqttclient.setServer(mqttserver, mqttport);
  mqttclient.setCallback(MqttCallback);

  mqttconnected = mqttclient.connect(MQTT_ID, user, pass);
  if (mqttconnected) {
    //USE_SERIAL.println("Connected to Mqtt-Server");
    mqttclient.subscribe("SolarControl/cmd/Mode");
    mqttclient.subscribe("SolarControl/cmd/SOLAR_PUMP");
    mqttclient.subscribe("SolarControl/cmd/ZIRC_PUMP");
    mqttclient.subscribe("SolarControl/cmd/LIGHT");
    mqttclient.subscribe("SolarControl/cmd/BWM");
    mqttclient.subscribe("SolarControl/Mode");
    mqttclient.subscribe("Heizoel/cmd/max_liter");
    mqttclient.subscribe("SolarControl/LWT");
    mqttclient.subscribe("POOL/temperature");
    mqttclient.subscribe("POOL/Pool_Mode");
    mqttclient.subscribe("POOL/Solar_Mode");
    //USE_SERIAL.println("subscribing to SolarControl/cmd/Mode & SolarContol/LWT");
    //USE_SERIAL.println(F("Debug: End of MqttConnect"));
  }
}

void MqttCallback(char *topic, byte *payload, unsigned int length) {
  char *payloadvalue;
  char *payloadkey;

  payload[length] = '\0';
  payloadkey = (char *)&payload[0];

      //USE_SERIAL.print("Message arrived [");
      //USE_SERIAL.print(topic);
      //USE_SERIAL.print("] ");
      
      payload[length] = '\0';
      String message = (char*)payload;
      
      //USE_SERIAL.print("payload: [");
      //USE_SERIAL.print((char *)payload);
      //USE_SERIAL.println("]");
            
      //USE_SERIAL.println(message);
      
    //SOLAR_PUMP über MQTT schalten
  if (strcmp(topic, "SolarControl/cmd/SOLAR_PUMP") == 0) {
    if (strcmp(payloadkey, "on") == 0 || strcmp(payloadkey, "ON") == 0) {
      //USE_SERIAL.print("Schalte SOLAR_PUMP auf ON\n");
      digitalWrite(SOLAR_PIN, LOW);
      mqttclient.publish("SolarControl/SOLAR_PUMP", "on");
      mqttclient.publish("SolarControl/STATUS", "Solarpumpe wurde über MQTT eingeschaltet.");
    } else if (strcmp(payloadkey, "off") == 0 || strcmp(payloadkey, "OFF") == 0) {
      //USE_SERIAL.print("Schalte SOLAR_PUMP auf OFF\n");
      digitalWrite(SOLAR_PIN, HIGH);
      mqttclient.publish("SolarControl/SOLAR_PUMP", "off");
      mqttclient.publish("SolarControl/STATUS", "Solarpumpe wurde über MQTT ausgeschaltet.");
    }
  }

    //ZIRC_PUMP über MQTT schalten
  if (strcmp(topic, "SolarControl/cmd/ZIRC_PUMP") == 0) {
    if (strcmp(payloadkey, "on") == 0 || strcmp(payloadkey, "ON") == 0) {
      //USE_SERIAL.print("Schalte ZIRC_PUMP auf ON\n");
      digitalWrite(ZIRC_PIN, LOW);
      mqttclient.publish("SolarControl/ZIRC_PUMP", "on");
      mqttclient.publish("SolarControl/STATUS", "Zirkulationspumpe wurde über MQTT eingeschaltet.");
    } else if (strcmp(payloadkey, "off") == 0 || strcmp(payloadkey, "OFF") == 0) {
      //USE_SERIAL.print("Schalte ZIRC_PUMP auf OFF\n");
      digitalWrite(ZIRC_PIN, HIGH);
      mqttclient.publish("SolarControl/ZIRC_PUMP", "off");
      mqttclient.publish("SolarControl/STATUS", "Zirkulationspumpe wurde über MQTT ausgeschaltet.");
    }
  }

    //LIGHT über MQTT schalten
  if (strcmp(topic, "SolarControl/cmd/LIGHT") == 0) {
    if (strcmp(payloadkey, "on") == 0 || strcmp(payloadkey, "ON") == 0) {
      //USE_SERIAL.print("Schalte LIGHT auf ON\n");
      digitalWrite(LIGHT_PIN, LOW);
      mqttclient.publish("SolarControl/LIGHT", "on");
      mqttclient.publish("SolarControl/STATUS", "Licht wurde über MQTT eingeschaltet.");
    } else if (strcmp(payloadkey, "off") == 0 || strcmp(payloadkey, "OFF") == 0) {
      //USE_SERIAL.print("Schalte LIGHT auf OFF\n");
      digitalWrite(LIGHT_PIN, HIGH);
      mqttclient.publish("SolarControl/LIGHT", "off");
      mqttclient.publish("SolarControl/STATUS", "Licht wurde über MQTT ausgeschaltet.");
    }
  }

  //BWM über MQTT schalten
  if (strcmp(topic, "SolarControl/cmd/BWM") == 0) {
    if (strcmp(payloadkey, "on") == 0 || strcmp(payloadkey, "ON") == 0) {
      //USE_SERIAL.print("Schalte BWM auf ON\n");
      digitalWrite(PIR_POWER_PIN, HIGH);
      mqttclient.publish("SolarControl/BWM", "on");
      mqttclient.publish("SolarControl/STATUS", "Bewegungsmelder wurde über MQTT eingeschaltet.");
    } else if (strcmp(payloadkey, "off") == 0 || strcmp(payloadkey, "OFF") == 0) {
      //USE_SERIAL.print("Schalte BWM auf OFF\n");
      digitalWrite(PIR_POWER_PIN, LOW);
      mqttclient.publish("SolarControl/BWM", "off");
      mqttclient.publish("SolarControl/STATUS", "Bewegungsmelder wurde über MQTT ausgeschaltet.");
    }
  }
  
  //MQTT POOL-Temperatur
  String strTopic = String((char*)topic);
  
  if (strTopic  == "POOL/temperature") {
    char buffer[6];
    memcpy(buffer, payload, length);  //copy the payload to the buffer
    buffer[length] = '\0';  //terminate the string (NOT a String !
    pool_temp = String((char*)payload).toFloat();
    //USE_SERIAL.print("MQTT Pool-Temperatur:");
    //USE_SERIAL.println(pool_temp);
  }  

    //MQTT POOL-Modus
    if (strcmp(topic, "POOL/Pool_Mode") == 0) {
    if (strcmp(payloadkey, "0") == 0 || strcmp(payloadkey, "auto") == 0) {
      pool_mode = POOL_AUTO;
        //USE_SERIAL.print("MQTT Pool-Modus:");
        //USE_SERIAL.println(pool_mode);
    } else if (strcmp(payloadkey, "1") == 0 || strcmp(payloadkey, "on") == 0) {
      pool_mode = POOL_MAN;
        //USE_SERIAL.print("MQTT Pool-Modus:");
        //USE_SERIAL.println(pool_mode);
    } else if (strcmp(payloadkey, "2") == 0 || strcmp(payloadkey, "off") == 0) {
      pool_mode = POOL_OFF;
        //USE_SERIAL.print("MQTT Pool-Modus:");
        //USE_SERIAL.println(pool_mode);
    }
  }
  
    //MQTT POOL Solar-Modus
    if (strcmp(topic, "POOL/Solar_Mode") == 0) {
    if (strcmp(payloadkey, "0") == 0 || strcmp(payloadkey, "pool") == 0) {
      solar_mode = SOLAR_POOL;
        //USE_SERIAL.print("MQTT Solar-Modus:");
        //USE_SERIAL.println(solar_mode);
    } else if (strcmp(payloadkey, "1") == 0 || strcmp(payloadkey, "puffer") == 0) {
      solar_mode = SOLAR_PUFFER;
        //USE_SERIAL.print("MQTT Solar-Modus:");
        //USE_SERIAL.println(solar_mode);
    } else if (strcmp(payloadkey, "2") == 0 || strcmp(payloadkey, "off") == 0) {
      solar_mode = SOLAR_OFF;
        //USE_SERIAL.print("MQTT Solar-Modus:");
        //USE_SERIAL.println(solar_mode);
    }
  }

  if (strcmp(topic, "SolarControl/cmd/Mode") == 0) {
    if (strcmp(payloadkey, "0") == 0 || strcmp(payloadkey, "Pool") == 0) {
      mode = MODE_POOL;
        //USE_SERIAL.print("Modus:");
        //USE_SERIAL.println(mode);
    } else if (strcmp(payloadkey, "1") == 0 || strcmp(payloadkey, "Puffer") == 0) {
      mode = MODE_PUFFERSPEICHER;
        //USE_SERIAL.print("Modus:");
        //USE_SERIAL.println(mode);
    } else if (strcmp(payloadkey, "2") == 0 || strcmp(payloadkey, "Auto") == 0) {
      mode = MODE_AUTO;
        //USE_SERIAL.print("Modus:");
        //USE_SERIAL.println(mode);
    }
  }
  //USE_SERIAL.println(F("Debug: End of MqttCallback"));
}

void CheckEthernet() {
  if (Ethernet.hardwareStatus() != EthernetNoHardware) {
    /*--------------------------------------------------------------
       check ethernet services
      --------------------------------------------------------------*/
    switch (Ethernet.maintain()) {
      case 1:
        //renewed fail
        //USE_SERIAL.println(F("Error: renewed fail"));
        break;

      case 2:
        //renewed success
        //USE_SERIAL.println(F("Renewed success"));
        //print your local IP address:
        //USE_SERIAL.print(F("My IP address: "));
        //USE_SERIAL.println(Ethernet.localIP());
        break;

      case 3:
        //rebind fail
        //USE_SERIAL.println(F("Error: rebind fail"));
        break;

      case 4:
        //rebind success
        //USE_SERIAL.println(F("Rebind success"));
        //print your local IP address:
        //USE_SERIAL.print(F("My IP address: "));
        //USE_SERIAL.println(Ethernet.localIP());
        break;

      default:
        //nothing happened
        break;
    }
  }

  //USE_SERIAL.println(F("Debug: End of CheckEthernet"));
}

void loop() {

  ActionPIR();
  
  if (!mqttclient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    mqttclient.loop();
  }

  float solar_temp = operatMax31865_1();
  float puffer_temp = operatMax31865_2();

  /*--------------------------------------------------------------
    remove delay (half second)
    --------------------------------------------------------------*/
  
  pin_stat = digitalRead(MODE_PIN);
  pinMillis = millis();
  if (pin_stat == LOW) {
    if (pinMillis - lastMillis > 200) {
      //USE_SERIAL.println("Impuls Low");
      lastMillis = pinMillis;
      mode_count = MODE_COUNT;
      if (!setmode) {
        setmode = true;
        old_mode = mode;
        sprintf(buf, "Enter Settings-Mode %d (%s)", mode, modes[mode]);
        //USE_SERIAL.println(buf);
      } else {
        mode--;
        if (mode < 0) {
          mode = 2;
        }
        sprintf(buf, "Mode change to %d (%s)", mode, modes[mode]);
        //USE_SERIAL.println(buf);
      }
    }
  }
      

  /*--------------------------------------------------------------
    remove delay (one second)
    --------------------------------------------------------------*/
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= sekunde) { // Hier eine Sekunde warten
    startMillis = currentMillis;
    /******************************************************************************************
       1 Sekunden Takt
     *****************************************************************************************/
    // Hier die Funktionen im Sekundentakt
    // ###################################

    CheckEthernet();
    Uptime();
    ReadAnalog();
       

    if (setmode) {
      mode_count--;
      if (mode_count <= 0) {
        //sprintf(buf, "leaving Settings-Mode - change from %d to %d (%s)", old_mode, mode, modes[mode]);
        //USE_SERIAL.println(buf);
        old_mode = mode;
        setmode = false;
      }
    }
    if (!setmode) {
      if (mode == MODE_POOL) {
        new_valve = VALVE_POOL;
        reason = 0;
      } else if (mode == MODE_PUFFERSPEICHER) {
        new_valve = VALVE_PUFFERSPEICHER;
        reason = 2;
      } else if (mode == MODE_AUTO) {
        if (pool_mode == POOL_AUTO && solar_mode == SOLAR_POOL && mqttclient.connected()) {
          new_valve = VALVE_POOL;
          reason = 1;
        }
        if (solar_mode == SOLAR_PUFFER) {
          new_valve = VALVE_PUFFERSPEICHER;
          reason = 2;
        }
      }
    }
  

    /*-----------------------------------------------------------
       Wenn Ventil umzuschalten ist
    */
    if (new_valve != valve) {
      valve = new_valve;
      digitalWrite(VALVE_PIN, valve);
      sprintf(buf, "Set Valve to %s (%s)", valves[valve], reasons[reason]);
      //USE_SERIAL.println(buf);
    }
      write_lcd(); 

    /*-----------------------------------------------------------
       TEMPERATURDIFFERENZ / HYSTERESE Steuerung Solaranlage
    */
      
    if ((valve == VALVE_POOL) && mqttclient.connected()) {
      if ((solar_temp > pool_temp + hysterese_on) && digitalRead(SOLAR_PIN) == HIGH)  {
      digitalWrite (SOLAR_PIN,LOW);
      //USE_SERIAL.println(F("POOL: Temperaturdifferenz >8°C, Schalte Solarpumpe ein"));
      mqttclient.publish("SolarControl/STATUS", "POOL: Temperaturdifferenz >7°C, Schalte Solarpumpe ein"); 
    } else if ((solar_temp <= pool_temp + hysterese_off) && digitalRead(SOLAR_PIN) == LOW) {
      digitalWrite (SOLAR_PIN,HIGH);
      //USE_SERIAL.println(F("POOL: Temperaturdifferenz <=8°C, Schalte Solarpumpe aus"));
      mqttclient.publish("SolarControl/STATUS", "POOL: Temperaturdifferenz <=2°C, Schalte Solarpumpe aus");
      }
    } else {
      if ((solar_temp > puffer_temp + hysterese_on) && digitalRead(SOLAR_PIN) == HIGH)  {
      digitalWrite (SOLAR_PIN,LOW);
      //USE_SERIAL.println(F("PUFFER: Temperaturdifferenz >8°C, Schalte Solarpumpe ein"));
      mqttclient.publish("SolarControl/STATUS", "PUFFER: Temperaturdifferenz >7°C, Schalte Solarpumpe ein"); 
    } else if ((solar_temp <= puffer_temp + hysterese_off) && digitalRead(SOLAR_PIN) == LOW) {
      digitalWrite (SOLAR_PIN,HIGH);
      //USE_SERIAL.println(F("PUFFER: Temperaturdifferenz <=8°C, Schalte Solarpumpe aus"));
      mqttclient.publish("SolarControl/STATUS", "PUFFER: Temperaturdifferenz <=2°C, Schalte Solarpumpe aus");
      }
    } 
  }

  ////USE_SERIAL.println(F("Debug: End of LOOP"));
}

/******************************************************************************************
   Ende Loop
* ****************************************************************************************
   Beginn Unterprogramme
******************************************************************************************/

  
char* ip2CharArray(IPAddress IP) {
  static char a[16];
  sprintf(a, "%d.%d.%d.%d", IP[0], IP[1], IP[2], IP[3]);
  return a;
  }

void write_lcd(void) {

    if (!setmode) {

    if ( LCD_Page == false ) {

      //TEMPERATUR und FÜLLSTAND-SCREEN

      float solar_temp = operatMax31865_1();
      float puffer_temp = operatMax31865_2();
      int fuell;     

      //Balkenanzeige
      fuell = map(percent, 0, 100, 0, 61);       //Hoehe des Balkens ermitteln

      //Ausgabe Display
      u8g2.clearBuffer();
      u8g2.firstPage();
    do {
      // Banner
      u8g2.drawBox(0, 0, 64, 12);         //Hintergrund weiß
      u8g2.setFont(u8g2_font_beanstalk_mel_tr);  
      u8g2.setFontMode(0);                // Schrifthintergrund weiß
      u8g2.setDrawColor(0);               // Schriftfarbe Schwarz
      u8g2.drawStr(0, 11, "TEMPERATUREN");
      // Temperaturen
      u8g2.setFont(u8g2_font_squeezed_r6_tr);
      u8g2.setFontMode(1);                // Schrifthintergrund Schwarz
      u8g2.setDrawColor(1);               // Schriftfarbe Weiß
      u8g2.drawStr(0, 22, "Kollektor:");
      u8g2.setFont(u8g2_font_luBS08_tf);
      u8g2.setCursor(8, 35);
      u8g2.print(solar_temp, 1);
      //u8g2.drawGlyph(30, 35, 0xb9);	/* dec 185/hex b9 Grad-Symbol */
      u8g2.print("°C");
      u8g2.setFont(u8g2_font_squeezed_r6_tr);
      u8g2.drawStr(0, 47, "Puffer:");
      u8g2.setFont(u8g2_font_luBS08_tf);
      u8g2.setCursor(8, 60);
      u8g2.print(puffer_temp, 1);
      u8g2.print("°C");
        if (mqttclient.connected()) {
              // Pool-Temperatur, Online
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 72, "Pool:");
              u8g2.setFont(u8g2_font_luBS08_tf);
              u8g2.setCursor(8, 85);
              u8g2.print(pool_temp, 1);
              u8g2.print("°C");
          } else {
              // Pool-Temperatur, Offline
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 72, "Pool:");
              u8g2.setFont(u8g2_font_luBS08_tf);
              u8g2.setCursor(0, 85);
              u8g2.print("OFFLINE");
          }
      //Heizöltank
      u8g2.setFont(u8g2_font_squeezed_r6_tr);
      u8g2.drawStr(0, 100, "Heizoel:");
      u8g2.setDrawColor(2);
      u8g2.setFont(u8g2_font_squeezed_b7_tr);
      u8g2.setCursor(32, 101);
      u8g2.print(liter, 0);
      u8g2.print(" l");
      u8g2.setCursor(20, 111);
      u8g2.print(percent, 0);
      u8g2.print(" %");
      u8g2.drawFrame(0, 102, 63, 11);
      u8g2.drawBox(1, 103, fuell, 9);
      //Statuszeile
      u8g2.drawLine(0, 118, 63, 118);
      u8g2.setFont(u8g2_font_baby_tr);
                  if (mode == 2) {
                    u8g2.drawStr(0, 126, "Auto: ");
                    u8g2.setCursor(24, 126);
                    u8g2.print(valves[valve]);
                  } else {
                    u8g2.drawStr(0, 126, "Manuell: ");
                    u8g2.setCursor(35, 126);
                    u8g2.print(valves[valve]);
                  }
    } while ( u8g2.nextPage() );
    } else {
              //STATUS-SCREEN
              u8g2.clearBuffer();
              u8g2.firstPage();
          do {
              // Banner
            if (mqttclient.connected()) {
              u8g2.setDrawColor(2);
              u8g2.setFont(u8g2_font_waffle_t_all);
              u8g2.drawGlyph(50, 10, 0xe29b);	/* dec 58011/hex e29b Online */
            } else {
              u8g2.setDrawColor(2);
              u8g2.setFont(u8g2_font_siji_t_6x10);
              u8g2.drawGlyph(50, 10, 0xe21b);	/* dec 57883/hex e21b Offline */
            }
              u8g2.drawBox(0, 0, 63, 12);         //Hintergrund weiß
              u8g2.setFont(u8g2_font_beanstalk_mel_tr);  
              u8g2.setFontMode(0);                // Schrifthintergrund weiß
              u8g2.setDrawColor(0);               // Schriftfarbe Schwarz
              u8g2.drawStr(5, 11, "STATUS");
              u8g2.setFontMode(1);                // Schrifthintergrund Schwarz
              u8g2.setDrawColor(1);               // Schriftfarbe Weiß
              // STATUS Solarpumpe
            if (digitalRead(SOLAR_PIN) == LOW) {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 22, "Solarpumpe:");              
              u8g2.drawButtonUTF8(33, 33, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(56, 33, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            } else {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 22, "Solarpumpe:");              
              u8g2.drawButtonUTF8(38, 33, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(51, 33, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            }
            // STATUS 3-Wege-Ventil
            if (digitalRead(VALVE_PIN) == LOW) {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 46, "3-Wege-Ventil:");              
              u8g2.drawButtonUTF8(33, 57, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(56, 57, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            } else {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 46, "3-Wege-Ventil:");
              u8g2.drawButtonUTF8(38, 57, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(51, 57, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            }
            if (digitalRead(VALVE_STATUS_PIN) == LOW) {
              u8g2.drawButtonUTF8(3, 57, U8G2_BTN_INV|U8G2_BTN_BW2, 0,  1,  1, " POOL " );
            } else {
              u8g2.drawButtonUTF8(3, 57, U8G2_BTN_INV|U8G2_BTN_BW2, 0,  1,  1, "PUFFER" );
            }            
            // STATUS Zirkulationspumpe
            if (digitalRead(ZIRC_PIN) == LOW) {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 70, "Zirkulationspumpe:");
              u8g2.drawButtonUTF8(33, 81, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(56, 81, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            } else {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 70, "Zirkulationspumpe:");              
              u8g2.drawButtonUTF8(38, 81, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(51, 81, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            }
            // STATUS Licht
            if (digitalRead(LIGHT_PIN) == LOW) {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 94, "Beleuchtung:");              
              u8g2.drawButtonUTF8(33, 105, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(56, 105, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            } else {
              u8g2.setFont(u8g2_font_squeezed_r6_tr);
              u8g2.drawStr(0, 94, "Beleuchtung:");
              u8g2.drawButtonUTF8(38, 105, U8G2_BTN_HCENTER|U8G2_BTN_BW2, 10,  1,  1, "ON" );
              u8g2.drawButtonUTF8(51, 105, U8G2_BTN_INV|U8G2_BTN_BW2, 10,  1,  1, "OFF" );
            }
              //Statuszeile
      u8g2.drawLine(0, 118, 63, 118);
      u8g2.setFont(u8g2_font_baby_tr);
                  if (mode == 2) {
                    u8g2.drawStr(0, 126, "Auto: ");
                    u8g2.setCursor(24, 126);
                    u8g2.print(valves[valve]);
                  } else {
                    u8g2.drawStr(0, 126, "Manuell: ");
                    u8g2.setCursor(35, 126);
                    u8g2.print(valves[valve]);
                  }

  } while ( u8g2.nextPage() );
          }    
  } else {
    u8g2.clearBuffer();
    u8g2.firstPage();
  do {
    // Banner
    u8g2.drawBox(0, 0, 63, 12);
    u8g2.setFont(u8g2_font_beanstalk_mel_tr);
    u8g2.setFontMode(0);
    u8g2.setDrawColor(0);
    u8g2.drawStr(18, 11, "MODUS");
    u8g2.setFont(u8g2_font_helvR08_te);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(0, 30, "Bisher: ");
    u8g2.setFont(u8g2_font_luBS10_tr);
    u8g2.setCursor(3, 45);
    u8g2.print(modes[old_mode]);
    u8g2.setFont(u8g2_font_helvR08_te);
    u8g2.drawStr(0, 75, "Neu: ");
    u8g2.setFont(u8g2_font_luBS10_tr);
    u8g2.setCursor(3, 90);
    u8g2.print(modes[mode]);
    u8g2.setCursor(25, 120);
    u8g2.print(mode_count);
  } while ( u8g2.nextPage() );
  }
}

void Mqttpublish(void) {
  
    if (mqttclient.connected()) {
    dtostrf(analog, 5, 2, buf);
    mqttclient.publish("Heizoel/Analog", buf);
    dtostrf(liter, 5, 0, buf);
    mqttclient.publish("Heizoel/Liter", buf);
    dtostrf(max_liter, 1, 0, buf);
    mqttclient.publish("Heizoel/max_liter", buf);
    dtostrf(percent, 5, 0, buf);
    mqttclient.publish("Heizoel/Prozent", buf);

    if (mode == 2) {
      mqttclient.publish("SolarControl/Modus", "Auto");
    } else {
      mqttclient.publish("SolarControl/Modus", "Manuell");
    }
    dtostrf(mode, 1, 0, buf);
    mqttclient.publish("SolarControl/Mode", buf);
    dtostrf(solar_mode, 1, 0, buf);
    mqttclient.publish("SolarControl/Solar_Mode", buf);
    dtostrf(pool_mode, 1, 0, buf);
    mqttclient.publish("SolarControl/Pool_Mode", buf);
    mqttclient.publish("SolarControl/Ventil", valves[valve]);

    // STATUS VALVE_POOL
    if (digitalRead(VALVE_PIN) == HIGH) {
      mqttclient.publish("SolarControl/VALVE_POOL", "off");
    } else {
      mqttclient.publish("SolarControl/VALVE_POOL", "on");
    }

        // STATUS VALVE_STATUS
    if (digitalRead(VALVE_STATUS_PIN) == HIGH) {
      mqttclient.publish("SolarControl/VALVE_STATUS", "Puffer");
    } else {
      mqttclient.publish("SolarControl/VALVE_STATUS", "Pool");
    }
    
    // STATUS SOLAR_PUMPE
    if (digitalRead(SOLAR_PIN) == HIGH) {
      mqttclient.publish("SolarControl/SOLAR_PUMP", "off");
    } else {
      mqttclient.publish("SolarControl/SOLAR_PUMP", "on");
    }

    // STATUS ZIRKULATIONS_PUMPE
    if (digitalRead(ZIRC_PIN) == HIGH) {
      mqttclient.publish("SolarControl/ZIRC_PUMP", "off");
    } else {
      mqttclient.publish("SolarControl/ZIRC_PUMP", "on");
    }

    // STATUS BEWEGUNGSMELDER
    if (digitalRead(PIR_POWER_PIN) == LOW) {
      mqttclient.publish("SolarControl/BWM", "off");
    } else {
      mqttclient.publish("SolarControl/BWM", "on");
    }

    // PRESENCE-DETECTION
    if (digitalRead(PIR_POWER_PIN) == HIGH) {
          if (digitalRead(PIR_PIN) == HIGH) {
          mqttclient.publish("SolarControl/PRESENCE", "Motion");
        } else {
          mqttclient.publish("SolarControl/PRESENCE", "noMotion");
        }
      } else {
       mqttclient.publish("SolarControl/PRESENCE", "Offline");
      }

    // STATUS LICHT
    if (digitalRead(LIGHT_PIN) == HIGH) {
      mqttclient.publish("SolarControl/LIGHT", "off");
    } else {
      mqttclient.publish("SolarControl/LIGHT", "on");
    }
    
    // IP-Adresse übermitteln
    mqttclient.publish("IP", ip2CharArray(Ethernet.localIP()));
    
    // MAX31865 Temperaturen übermitteln
    dtostrf((max_1.temperature(RNOMINAL, RREF)), 1, 1, buf);
    mqttclient.publish("/SolarControl/pt1", buf);
    dtostrf((max_2.temperature(RNOMINAL, RREF)), 1, 1, buf);
    mqttclient.publish("/SolarControl/pt2", buf);
    
    // System
    mqttclient.publish("SolarControl/Uptime", uptime);
    mqttclient.publish("SolarControl/LWT","Online", true);
    
  } else {
    MqttConnect(mqttuser, mqttpass);
  }
  //USE_SERIAL.println(F("Debug: End of Mqttpublish"));
}

//PIR auswerten und Display und Licht anschalten
void ActionPIR()
{
  //wenn BWM eingeschaltet ist soll das Display bei erkannter Bewegung aktiviert werden
  if (digitalRead(PIR_POWER_PIN) == HIGH)
    {
      //wenn Bewegung erkannt Licht und Display einschalten und Leuchtdauer festlegen
      if (digitalRead(PIR_PIN) == HIGH)
      {
        digitalWrite(LIGHT_PIN, LOW);
        u8g2.setPowerSave(0);
        PIR_TimerComparison = millis() + PIR_TimerON;
      }

      //wenn die Zeit vorbei ist, Licht und Display ausschalten
      if (millis() >= PIR_TimerComparison)
      {
        digitalWrite(LIGHT_PIN, HIGH);
        u8g2.setPowerSave(1);
      }
    } else {
      //wenn BWM ausgeschaltet ist soll das Display immer an sein
      u8g2.setPowerSave(0);
    }
}