/* ////////////////////////////////////////////////////////////////////////////////////////////////////
 * ///////////////  Termostato per termocamino Edilkamin Acquatondo 29 Kit 1 vaso aperto  ///////////////
 * ////////////////////////////////////////////////////////////////////////////////////////////////////
 * 
 * -  Lettura della temperatura da sonda NTC e attivazione delle pompe di ricircolo, valvola deviatrice
 *    e attivazione dell'acqua sanitaria;
 * -  Hardware: Arduino Mega 2560 - ESP8266 ;
 *              TFT shield Elegoo 2.8" ILI9341;
 *              Relay Module Elegoo 4 relays;
 *              SD Card 32GB;
 *    Software: Software SPI (Abilitato in libreria SdFat);          
 *              Librerie MCUFRIEND;
 *              Librerie MCUFRIEND;
 *              Librerie Adafruit_gfx;
 *              Immagini BMP 24bit
 *              
 * Simulazione con pulsanti e LED:
 * Resistenza da 10K su ogni ingresso pulsante
 * Resistenza da 330 su ogni Led in uscita
 * Resistenza da 100 per buzzer
 *
 *               R1
 *(Vcc 5+)--+-->2.7k--+-->kty81-110-->(GND)
 *          |         |
 *          +->100nF--+-----> ADC2 (Analog Input)
 *              C1
 *
 *MCUFRIEND UNO shields have microSD on pins 10, 11, 12, 13
 *The official <SD.h> library only works on the hardware SPI pins
 *e.g. 50, 51, 52 on a Mega2560
 */

#include <SdFat.h>           // Use the SdFat library
SdFatSoftSpi<12, 11, 13> SD; //Bit-Bang on the Shield pins

#include <Adafruit_GFX.h>    // Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#include <FreeDefaultFonts.h>
#include <TouchScreen.h>
#include "WiFiEsp.h"
//#include <ESP8266WiFi.h>


#define MINPRESSURE 100
#define MAXPRESSURE 1000

#define SD_CS     10
#define PALETTEDEPTH   8     // support 256-colour Palette

#define BLACK      0x0000
#define BLUE       0x001F
#define RED        0xF800
#define GREEN      0x07E0
#define CYAN       0x07FF
#define MAGENTA    0xF81F
#define YELLOW     0xFFE0
#define WHITE      0xFFFF
#define BACKGROUND 0xE657


char namebuf[32] = "/";   //PERCORSO IMMAGINI BMP
char lastnamebuf[32] = "null";

File root;
int pathlen;

char* ssid = "FASTWEB-XL5NIY";    
char* password = "DYPY0MW849";        
IPAddress ip(192,168,1,101);  
IPAddress gateway(192,168,1,254);
IPAddress subnet(255,255,255,0);
IPAddress MAC = WL_MAC_ADDR_LENGTH;
char* mac; 


const int XP=8,XM=A2,YP=A3,YM=9; //240x320 ID=0x9341 
const int TS_LEFT=902,TS_RT=115,TS_TOP=72,TS_BOT=901;// CALIBRAZIONE TOUCH

const int OpompRicCC = 23;       // Pompa ricircolo circuito camino 
const int OpompRicRisc = 25;     // Pompa ricircolo riscaldamento
const int OvalDevRisc = 27;      // Valvola deviatrice riscaldamento
const int OvalDevSan = 29;       // Valvola deviatrice acqua sanitari
const int buzzer = 28;           // Buzzer


unsigned long int start_time;
unsigned long int end_time;
unsigned long int base_time = 5000;            // Tempo campionamento lettura sonda temperatura
unsigned long int start_snooze;
unsigned long int end_snooze;
unsigned long int SnoozeTime = 300000 ; // 5 x 60 000 [ms]
float R1 = 2685;
float Vin;

//int IincTemp = 4;              // Incremento temperatura attivazione ricircolo (Simulazione)
//int IdecTemp = 5;              // Diminuzione temperatura attivazione ricircolo (Simulazione)
//const int IselAttRis = 6;      // Modo selezione attivazione riscaldamento OFF/ON/AUTO (Simulazione)
//const int IselAcqSan = 7;      // Modo selezione Mattivazione acqua sanitari OFF/ON/AUTO (Simulazione)
int TempAttRicCC = 20;           // Temperatura attivazione ricircolo circuito camino (default)
int TempAttRisc = 50;            // Temperatura attivazione riscaldamento (default)
int LastTempAttRisc;             // Temperatura attivazione riscaldamento precedente
int TempAttAcqSan = 60;          // Temperatura attivazione Acqua sanitari 
int ModSetpoint = 0 ;            // 0 = Visualizza TempCamino, 1 = Modifica TempAttRisc
int TempPin = 14;                // Ingresso sonda temperatura  (15!)
int ImodFunRisc = 2;             // Modo funzionamento riscaldamento 0=OFF, 1=ON, 2=AUTO
int ImodFunAcqSan = 2;           // Modo funzionamento acqua sanitari 0=OFF, 1=ON, 2=AUTO
int TempCamino; // = 70;         // Temperatura camino
int Isteresi = 5;                // Isteresi disattivazione riscaldamento                   
int j = 10;                      // Numero di valori per array media ingresso sonda temperatura
float Tempc [10];                  // Array x temperatura sonda
float Media;                       // Media valori temperatura 
int i = 0;                       // Indice array
float Somma;                       // Appoggio 
int LastTempCamino = 50;         // Ultima temperatura letta
int StatoPulsIncr = 0;           // Stato pulsante incremento temperatura
int StatoPulsDecr = 0;           // Stato pulsante decremento temperatura
int StatoPulsSelRisc = 0;        // Stato pulsante selezione modo funzionamento riscaldamento 
int StatoPulsSelSan = 0;         // Stato pulsante selezione modo funzionamento sanitari
int StatoPulsModSet = 0;         // Stato pulsante modifica TempAttRisc
int StatoPulsAlarmOF = 0;
int Beep = 1;                    // Beep con Touch
int AlarmOF = 0;                 // Snooze alarm;
int Alarm = 0;
int diag = 0;                    // per diagnosi


TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;



void setup()
{
Serial.begin(9600);

  // initialize serial3 for ESP module
Serial3.begin(115200);

  // initialize ESP module
WiFi.init(&Serial3);

WiFi.begin(ssid, password);
WiFi.config(ip); 

while ( WiFi.status() != WL_CONNECTED) {

  Serial.print("Connecting to: ");
  Serial.println(ssid);
  Serial.print(".");
  delay(500);
  }  
  /*
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
 
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  */
  
    pinMode(OpompRicCC,OUTPUT);
    pinMode(OpompRicRisc,OUTPUT);
    pinMode(OvalDevRisc, OUTPUT);
    pinMode(OvalDevSan, OUTPUT);
    pinMode(buzzer, OUTPUT);
//    pinMode(IincTemp, INPUT);// (Simulazione)
//    pinMode(IdecTemp, INPUT);// (Simulazione)
//    pinMode(ImodFunRisc, INPUT);// (Simulazione) 
//    pinMode(IselAcqSan, INPUT);// (Simulazione) 
    pinMode(buzzer, OUTPUT);
    
    
    uint16_t ID = tft.readID();
    Serial.print("TFT with ID:0x");
    Serial.println(ID, HEX);
    if (ID == 0x0D3D3) ID = 0x9481;
    tft.begin(ID);
    tft.fillScreen(0x0000);
    tft.setTextColor(0xFFFF, 0x0000);    
    tft.setRotation(1);  //ORIENTAMENTO TFT
    bool good = SD.begin(SD_CS);
    if (!good) {
        Serial.print(F("cannot start SD"));
        while (1);
    }
    root = SD.open(namebuf);
    pathlen = strlen(namebuf);

///////////////////  IMPOSTO BACKGROUND  /////////////////////////

    char *nm = namebuf + pathlen;
    File f = root.openNextFile();
    uint8_t ret;
    uint32_t start;
    tft.fillScreen(BLACK);
//    delay(1000);
    delay(200);
    f.getName(nm, 32 - pathlen);
    f.close();
    strlwr(nm);
    tft.fillScreen(0);
    start = millis();
    showBMP("background.bmp", 0, 0);  //aggiungere "ret = " x diagnostica
    showBMP("button_TEMP.bmp", 60, 50);
    delay(100);
    //drawRect(50, 50, 100, 100, BLACK);

    upd_risc_mod();
    upd_wat_mod();

    LastTempAttRisc = TempAttRisc;


//////////////////////////////////////////////////////////////////
    
}

void loop()
{
 /* 
//////  Leggo  la temperatura dell'impianto ogni " base_time"    ////////
//////  in un array di 10 valori e faccio la media  ////////
*/


  end_time=millis();
  if((end_time-start_time)>=base_time){
    start_time = end_time;
    
    i=0;
    Somma = 0;

    for (i=0; i<j; i++){


    float ukty = 5 * Vin / 1023.0 ;
    Vin = analogRead(TempPin); 
    float a = 0.00001874*1000;
    float b = 0.007884*1000;
    float c = 1000 - R1*ukty/(5-ukty);
    float delta = b * b - 4 * a * c;
    float delta1 = sqrt (delta);
    float x2 =(-b + delta1)/(2 * a);
    Tempc[i] = x2 + 25 ;

    
    Somma = Somma + Tempc[i]; //- Tempc[(i+1)%j];    
    TempCamino = Somma/(i+1); //(j-1);
/* 
    Serial.print("Vin = "); 
    Serial.println(Vin);
    Serial.print("R1 = "); 
    Serial.println(R1);
    
    Serial.print("Aggiungo all'array i= "); 
    Serial.print(i);
    Serial.print(" di ");
    Serial.print(j);
    Serial.print(", ");   
    Serial.println(Tempc[i]);
    Serial.print("Somma:  "); 
    Serial.println(Somma);
    Serial.print("TempCamino:  ");    
    Serial.println(TempCamino); 
*/     
    }
  }


//if (TempCamino >= 90) {
  alarm_check();
//}


if (ModSetpoint == 0){
  if (TempCamino == LastTempCamino) {}    
     else {    
        upd_lcd();
        upd_bars();
        upd_risc_mod();
        upd_wat_mod();
     }
  }

 ///////////////  Leggo pressione touch

 
    uint16_t xpos, ypos;  //screen coordinates
    tp = ts.getPoint();   //tp.x, tp.y are ADC values

    xpos = map(tp.y, TS_TOP, TS_BOT, 0, tft.width());
    ypos = map(tp.x, TS_RT, TS_LEFT, 0, tft.height());
    
//        Serial.println(" click! :");
//        Serial.println(tp.z);

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE) { //CONTROLLO PRESSIONE TOUCH

if (diag == 1) {
  
      Serial.print("pressure = ");
      Serial.println(tp.z);
      
      Serial.print("xpos = ");
      Serial.println(xpos);
      
      Serial.print("ypos = ");
      Serial.println(ypos);      
      
      Serial.println(" in range ");
    }
             
///////////////            Fine lettura pressione touch             ///////////////////////

///////////////  CONTROLLO QUALE PULSANTE E' STATO PREMUTO PREMUTO  ///////////////

      if (xpos > 250 && xpos < 300  && ypos > 90 && ypos < 130){  // PULSANTE SELEZIONE MODO RISCALDAMENTO
        StatoPulsSelRisc = 1;
//        Serial.println(" click riscaldamento! ");
        }
      if (xpos > 250 && xpos < 300  && ypos > 150 && ypos < 190){ // PULSANTE SELEZIONE MODO ACQUA SANITARI
        StatoPulsSelSan = 1;
//        Serial.println(" click acqua sanitari! ");
        }
      if (xpos > 60 && xpos < 180  && ypos > 50 && ypos < 90){    // PULSANTE SELEZIONE MODIFICA SETPOINT
        StatoPulsModSet = 1;
//        Serial.println(" click SET/TEMP! "); 
        }        
      
      if (ModSetpoint == 1) {
        if (xpos > 60 && xpos < 118  && ypos > 200 && ypos < 230){    // PULSANTE UP
          StatoPulsIncr = 1;
//          Serial.println(" click incremento Setpoint "); 
        }  
      
        if (xpos > 60 && xpos < 180  && ypos > 90 && ypos < 190){    // PULSANTE DOWN
          StatoPulsDecr = 1;
//          Serial.println(" click decremento Setpoint ");
        }          
      }

      if (Alarm == 1) {
//        Serial.println(" aspetto click Snooze "); 
        
        if (xpos > 60 && xpos < 180  && ypos > 90 && ypos < 190){    // PULSANTE SNOOZE
          StatoPulsAlarmOF = AlarmOF = 1;
          start_snooze = millis();
//          Serial.println(" click Snooze "); 
        }

      }    
  
    }



/////////////// BEEP ON TOUCH /////////////// 

      if ((Beep == 1)&( (StatoPulsSelRisc || StatoPulsSelSan || StatoPulsModSet || StatoPulsIncr || StatoPulsDecr || StatoPulsAlarmOF) ==1)){
        tone(buzzer, 1200);
        delay(10);
        noTone(buzzer);
      }
      
 ///////////////  SELEZIONE PULSANTE RISCALDAMENTO  ///////////////

//StatoPulsSelRisc = digitalRead(IselAttRis); // Simulazione 

if (StatoPulsSelRisc == 1){
  ImodFunRisc++ ;
  if (ImodFunRisc > 2) ImodFunRisc = 0 ;
  upd_risc_mod();
}


////////  LETTURA PULSANTE SELEZIONE ACQUA SANITARI ////////

//StatoPulsSelSan = digitalRead(IselAcqSan); <<-- Simulazione
if (StatoPulsSelSan == 1) {
  ImodFunAcqSan++ ;
  if (ImodFunAcqSan > 2) ImodFunAcqSan = 0 ;
  upd_wat_mod();
}

if (diag == 1) {
Serial.print("Modo acqua sanitari:  ");
Serial.println(ImodFunAcqSan);
Serial.print("Pulsante Modo acqua sanitari: ");
Serial.println(StatoPulsSelSan);
}

 ///////////////  SELEZIONE SETPOINT  ///////////////

if (StatoPulsModSet == 1) {
  ModSetpoint++ ;
  
  if (ModSetpoint > 1) {
    ModSetpoint = 0 ;
    showBMP("button_TEMP.bmp", 60, 50);
    upd_lcd();
  }
  else{
  chg_setpoint();
  }
  /*
  Serial.println(" click SET/TEMP! ");
  Serial.print("Modo modifica Setpoint: ");
  Serial.println(ModSetpoint);  
  */
}

///////////////  INCREMENTO/DECREMENTO SETPOINT SETPOINT  ///////////////

if (StatoPulsIncr == 1) {
  TempAttRisc++;
  if (TempAttRisc >75) {
    TempAttRisc = 75;   
  }
  chg_setpoint();
}

if (StatoPulsDecr == 1) {
  TempAttRisc--;
  if (TempAttRisc < TempAttRicCC) {
    TempAttRisc = TempAttRicCC;        
  }
  chg_setpoint();
}

if (TempAttRisc != LastTempAttRisc) {
  upd_risc_mod();
  LastTempAttRisc = TempAttRisc;
  }
 
////////////  RESET STATO PULSANTI  ///////////

StatoPulsSelRisc = 0;
StatoPulsSelSan = 0;
StatoPulsModSet = 0;
StatoPulsIncr = 0;
StatoPulsDecr = 0;
StatoPulsAlarmOF = 0;
 
if (diag == 1) {
    Serial.print("TempCamino:  ");    
    Serial.println(TempCamino); 
}
  
   LastTempCamino = TempCamino;
//}

}   

/////////////////////////////////////////////////////// END LOOP
/*
////////////  Icona WiFi  ///////////////


while ( WiFi.status() == WL_CONNECTED) {
 showBMP("WiFi_ON.bmp", 280, 50);
}
else {
  showBMP("WiFi_OF.bmp", 280, 50);  
}
break;
////////////  Icona MQTT  ///////////////

*/

////////////  Aggiorno temperatura nel 7 segmenti  ///////////////

void upd_lcd(void){

    drawRect(50, 50, 100, 100, BLACK);
    if (TempCamino >= 99) TempCamino = 99;
    if (TempCamino < 0) TempCamino = 0;     
    tft.fillRect(60, 90, 120, 100, BACKGROUND);
    tft.setFont(&FreeSevenSegNumFont);
    tft.setCursor(58, 190);
    tft.setTextColor(BLACK);
    tft.setTextSize(2);
    tft.print(TempCamino, DEC);


    if (abs(TempCamino - LastTempCamino)>2){
    LastTempCamino = TempCamino; //TODO
    }
    }

  
////////////  Attivo il modo di modifica Setpoint  ////////////

void chg_setpoint(void) {


    drawRect(50, 50, 100, 100, BLACK);
    if (TempCamino >= 99) TempCamino = 99;
    if (TempCamino < 0) TempCamino = 0;     
    showBMP("button_SET.bmp", 60, 50);
    tft.fillRect(60, 90, 120, 100, BACKGROUND);
    tft.setFont(&FreeSevenSegNumFont);
    tft.setCursor(58, 190);
    tft.setTextColor(RED);
    tft.setTextSize(2);
    tft.print(TempAttRisc, DEC);
}  
  
//////////// Aggiorno la barra della temperatura  ////////////////

void upd_bars(void){


if (TempCamino >= 20) {
  showBMP("20.bmp", 20, 170); 
}
else {
  showBMP("20_OF.bmp", 20, 170);
}
if (TempCamino >= 30) {
  showBMP("30.bmp", 20, 140);
}
else {
  showBMP("30_OF.bmp", 20, 140);
}
if (TempCamino >= 40){
  showBMP("40.bmp", 20, 110);
} 
else {
  showBMP("40_OF.bmp", 20, 110);   
}
if (TempCamino >= 50) {
  showBMP("50.bmp", 20, 80); 
}
else {
  showBMP("50_OF.bmp", 20, 80);
}
if (TempCamino >= 60) {
  showBMP("60.bmp", 20, 50); 
}
else {
  showBMP("60_OF.bmp", 20, 50);
}
if (TempCamino >= 70) {
  showBMP("70.bmp", 20, 10); 
}
else {
  showBMP("70_OF.bmp", 20, 10);
}
if (TempCamino >= 80) {
  showBMP("80.bmp", 60, 10);
}
else {
  showBMP("80_OF.bmp", 60, 10);
}
if (TempCamino >= 90) {
  showBMP("90.bmp", 90, 10);
}
else {
  showBMP("90_OF.bmp", 90, 10);
  showBMP("alarm_OF.bmp", 120, 10);
}
} //////////// Fine aggiornamento barre ///////////////

////////////  Attivo l'allarme se supera i 90 gradi  /////////////

void alarm_check(void){

if (AlarmOF == 1) {end_snooze = millis();
if (diag == 1) {
Serial.print("end_snooze: ");
Serial.println(end_snooze);
Serial.print("start_snooze: ");
Serial.println(start_snooze);
}
}

//////////////   Tasto Snooze (clicca su temperatura per 1 sec )  ////////////////


if (Alarm == 1 ) {   if((end_snooze-start_snooze)>=SnoozeTime){
      start_snooze = end_snooze;
      AlarmOF = 0;
    }
}
if (diag == 1) {    
Serial.print("Alarm: ");
Serial.println(Alarm);
Serial.print("AlarmOF: ");
Serial.println(AlarmOF);
}

if (TempCamino >= 90) {
  if (AlarmOF == 0) {

    tone(buzzer, 2000); // Send 1,2KHz sound signal...
    showBMP("alarm.bmp", 120, 10);
    Alarm=1;
  }
}
else { if (TempCamino < 88){
  showBMP("alarm_OF.bmp", 120, 10);
  Alarm=0;
  AlarmOF=0;
  noTone(buzzer);}
  }
  
} 

////////////  attivo il modo di funzionamento riscaldamento   ////////////

void upd_risc_mod(void){
  
/*    Serial.print("Modo riscaldamento: ");
    Serial.println(ImodFunRisc);
*/  
 switch (ImodFunRisc){

   case 0: {
    //Modo OFF
    digitalWrite(OpompRicCC, LOW);
    showBMP("OF.bmp", 200, 50);
    digitalWrite(OpompRicRisc, LOW);
    showBMP("OF.bmp", 240, 50);
    digitalWrite(OvalDevRisc, LOW);
    showBMP("button_OF.bmp", 250, 90);    
    }        
    break;

   case 1: {
    //Modo ON
    digitalWrite(OpompRicCC, HIGH);
    showBMP("ON.bmp", 200, 50);
    digitalWrite(OpompRicRisc, HIGH);
    showBMP("ON.bmp", 240, 50);
    digitalWrite(OvalDevRisc, HIGH);
    showBMP("button_ON.bmp", 250, 90);   
    }        
    break;
    
   case 2: {
    //Modo Auto
    if (TempCamino >= TempAttRicCC) {
      digitalWrite(OpompRicCC, HIGH);      
      showBMP("ON.bmp", 200, 50); 
//      Serial.println("solo ricircolo acceso");     
    }
    else {if (TempCamino < (TempAttRicCC-Isteresi)){
            digitalWrite(OpompRicCC, LOW); 
            showBMP("OF.bmp", 200, 50);
    } 
//        Serial.println("tutto spento");

    }
    if (TempCamino >= TempAttRisc) {

      digitalWrite(OpompRicRisc, HIGH);
      showBMP("ON.bmp", 200, 50); 
      digitalWrite(OvalDevRisc, HIGH);
      showBMP("ON.bmp", 240, 50); 
//      Serial.println("tutto acceso");
      }
    else {
          if (TempCamino < (TempAttRisc-Isteresi)){
//            Serial.println("solo ricircolo acceso");
             digitalWrite(OpompRicRisc, LOW); 
             digitalWrite(OvalDevRisc, LOW);
             showBMP("OF.bmp", 240, 50); 
          }
      }
    showBMP("button_AUTO.bmp", 250, 90);
    StatoPulsSelRisc == 0;
    }
    break;
 }
  
  }

////////////  attivo il modo di funzionamento acqua sanitari  ////////////

void upd_wat_mod(void) {

/*   
    Serial.print("Modo sanitari: ");
    Serial.println(ImodFunAcqSan);
*/
 switch (ImodFunAcqSan){

   case 0: {
    //Modo OFF
    digitalWrite(OvalDevSan, LOW);
    showBMP("OF.bmp", 280, 50);
    showBMP("button_OF.bmp", 250, 150);   

    }        
    break;

   case 1: {
    //Modo ON
    digitalWrite(OvalDevSan, HIGH);
    showBMP("ON.bmp", 280, 50);
    showBMP("button_ON.bmp", 250, 150);   

    }        
    break;
    
   case 2: {
    //Modo Auto
    if (TempCamino >= TempAttAcqSan) {
      digitalWrite(OvalDevSan, HIGH);
      showBMP("ON.bmp", 280, 50);
    }
    else {
      if (TempCamino < (TempAttAcqSan - Isteresi)){
      digitalWrite(OvalDevSan, LOW);
      showBMP("OF.bmp", 280, 50);
      }  
    }
    StatoPulsSelSan == 0;
    showBMP("button_AUTO.bmp", 250, 150);
    }
    break;
      
  }
}

/*

////////////////////  DIAGNOSI APERTURA BMP  /////////////////////

    switch (ret) {
                case 0:
                    Serial.print(millis() - start);
                    Serial.println(F("ms"));
                    delay(5000);
                    break;
                case 1:
                    Serial.println(F("bad position"));
                    break;
                case 2:
                    Serial.println(F("bad BMP ID"));
                    break;
                case 3:
                    Serial.println(F("wrong number of planes"));
                    break;
                case 4:
                    Serial.println(F("unsupported BMP format"));
                    break;
                case 5:
                    Serial.println(F("unsupported palette"));
                    break;
                default:
                    Serial.println(F("unknown"));
                    break;
            }

*/
 


#define BMPIMAGEOFFSET 54

#define BUFFPIXEL      20

uint16_t read16(File& f) {
    uint16_t result;         // read little-endian
    f.read(&result, sizeof(result));
    return result;
}

uint32_t read32(File& f) {
    uint32_t result;
    f.read(&result, sizeof(result));
    return result;
}

uint8_t showBMP(char *nm, int x, int y)
{
    File bmpFile;
    int bmpWidth, bmpHeight;    // W+H in pixels
    uint8_t bmpDepth;           // Bit depth (currently must be 24, 16, 8, 4, 1)
    uint32_t bmpImageoffset;    // Start of image data in file
    uint32_t rowSize;           // Not always = bmpWidth; may have padding
    uint8_t sdbuffer[3 * BUFFPIXEL];    // pixel in buffer (R+G+B per pixel)
    uint16_t lcdbuffer[(1 << PALETTEDEPTH) + BUFFPIXEL], *palette = NULL;
    uint8_t bitmask, bitshift;
    boolean flip = true;        // BMP is stored bottom-to-top
    int w, h, row, col, lcdbufsiz = (1 << PALETTEDEPTH) + BUFFPIXEL, buffidx;
    uint32_t pos;               // seek position
    boolean is565 = false;      //

    uint16_t bmpID;
    uint16_t n;                 // blocks read
    uint8_t ret;

    if ((x >= tft.width()) || (y >= tft.height()))
        return 1;               // off screen

    bmpFile = SD.open(nm);      // Parse BMP header
    bmpID = read16(bmpFile);    // BMP signature
    (void) read32(bmpFile);     // Read & ignore file size
    (void) read32(bmpFile);     // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile);       // Start of image data
    (void) read32(bmpFile);     // Read & ignore DIB header size
    bmpWidth = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    n = read16(bmpFile);        // # planes -- must be '1'
    bmpDepth = read16(bmpFile); // bits per pixel
    pos = read32(bmpFile);      // format
    if (bmpID != 0x4D42) ret = 2; // bad ID
    else if (n != 1) ret = 3;   // too many planes
    else if (pos != 0 && pos != 3) ret = 4; // format: 0 = uncompressed, 3 = 565
    else if (bmpDepth < 16 && bmpDepth > PALETTEDEPTH) ret = 5; // palette 
    else {
        bool first = true;
        is565 = (pos == 3);               // ?already in 16-bit format
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * bmpDepth / 8 + 3) & ~3;
        if (bmpHeight < 0) {              // If negative, image is in top-down order.
            bmpHeight = -bmpHeight;
            flip = false;
        }

        w = bmpWidth;
        h = bmpHeight;
        if ((x + w) >= tft.width())       // Crop area to be loaded
            w = tft.width() - x;
        if ((y + h) >= tft.height())      //
            h = tft.height() - y;

        if (bmpDepth <= PALETTEDEPTH) {   // these modes have separate palette
            bmpFile.seek(BMPIMAGEOFFSET); //palette is always @ 54
            bitmask = 0xFF;
            if (bmpDepth < 8)
                bitmask >>= bmpDepth;
            bitshift = 8 - bmpDepth;
            n = 1 << bmpDepth;
            lcdbufsiz -= n;
            palette = lcdbuffer + lcdbufsiz;
            for (col = 0; col < n; col++) {
                pos = read32(bmpFile);    //map palette to 5-6-5
                palette[col] = ((pos & 0x0000F8) >> 3) | ((pos & 0x00FC00) >> 5) | ((pos & 0xF80000) >> 8);
            }
        }

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x + w - 1, y + h - 1);
        for (row = 0; row < h; row++) { // For each scanline...
            // Seek to start of scan line.  It might seem labor-
            // intensive to be doing this on every line, but this
            // method covers a lot of gritty details like cropping
            // and scanline padding.  Also, the seek only takes
            // place if the file position actually needs to change
            // (avoids a lot of cluster math in SD library).
            uint8_t r, g, b, *sdptr;
            int lcdidx, lcdleft;
            if (flip)   // Bitmap is stored bottom-to-top order (normal BMP)
                pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
            else        // Bitmap is stored top-to-bottom
                pos = bmpImageoffset + row * rowSize;
            if (bmpFile.position() != pos) { // Need seek?
                bmpFile.seek(pos);
                buffidx = sizeof(sdbuffer); // Force buffer reload
            }

            for (col = 0; col < w; ) {  //pixels in row
                lcdleft = w - col;
                if (lcdleft > lcdbufsiz) lcdleft = lcdbufsiz;
                for (lcdidx = 0; lcdidx < lcdleft; lcdidx++) { // buffer at a time
                    uint16_t color;
                    // Time to read more pixel data?
                    if (buffidx >= sizeof(sdbuffer)) { // Indeed
                        bmpFile.read(sdbuffer, sizeof(sdbuffer));
                        buffidx = 0; // Set index to beginning
                        r = 0;
                    }
                    switch (bmpDepth) {          // Convert pixel from BMP to TFT format
                        case 24:
                            b = sdbuffer[buffidx++];
                            g = sdbuffer[buffidx++];
                            r = sdbuffer[buffidx++];
                            color = tft.color565(r, g, b);
                            break;
                        case 16:
                            b = sdbuffer[buffidx++];
                            r = sdbuffer[buffidx++];
                            if (is565)
                                color = (r << 8) | (b);
                            else
                                color = (r << 9) | ((b & 0xE0) << 1) | (b & 0x1F);
                            break;
                        case 1:
                        case 4:
                        case 8:
                            if (r == 0)
                                b = sdbuffer[buffidx++], r = 8;
                            color = palette[(b >> bitshift) & bitmask];
                            r -= bmpDepth;
                            b <<= bmpDepth;
                            break;
                    }
                    lcdbuffer[lcdidx] = color;

                }
                tft.pushColors(lcdbuffer, lcdidx, first);
                first = false;
                col += lcdidx;
            }           // end cols
        }               // end rows
        tft.setAddrWindow(0, 0, tft.width() - 1, tft.height() - 1); //restore full screen
        ret = 0;        // good render
    }
    bmpFile.close();
    return (ret);
}

/*    
void showmsgXY(int x, int y, int sz, const GFXfont *f, char *msg)
{   
    int16_t x1, y1;
    uint16_t wid, ht;
    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(GREEN);
    tft.setTextSize(sz);
    tft.print(msg);
    delay(1000);
}
*/
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){}
