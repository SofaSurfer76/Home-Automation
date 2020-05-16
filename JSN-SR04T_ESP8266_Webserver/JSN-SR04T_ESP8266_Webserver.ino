

// Load Wi-Fi library
#include <ESP8266WiFi.h>

#define triggerPIN 1 //2 //3
#define echoPIN 3 //0 //2



#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     1      // Reset pin #RX
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* ssid     = "FASTWEB-XL5NIY";
const char* password = "DYPY0MW849";

// Set web server port number to 80
WiFiServer server(80);
// Variable to store the HTTP request
String header;


IPAddress ip(192,168,1,100);  
IPAddress gateway(192,168,1,254);
IPAddress subnet(255,255,255,0);

unsigned int distanceCM;
String litri; // = "9999"; //String
int litri_vis;
int last_litri_vis = 0;
const int meas_type = 1;  // 0 = continuos; 1 = on event  

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

int returnCM;                           // Variable containg processed sounding

//****************** SETUP **********************
void setup() {
  
  Wire.begin(2, 0);           // set I2C pins (SDA = GPIO2, SCL = GPIO0)

// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

//  display.clearDisplay();

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.setRotation(2);
  display.display();
  delay(2000);            // Splashscreen delay

  // Clear the buffer
  display.clearDisplay();
  
  pinMode(triggerPIN, OUTPUT);          // Set the trigPin as an Output (sr04t or v2.0)
  //pinMode(echoPIN, INPUT);            // Set the echoPin as an Input (sr04t)
  pinMode(echoPIN,INPUT_PULLUP);        // Set the echoPin as an Input with pullup (sr04t or v2.0)
//  Serial.begin(115200);

    // Connect to Wi-Fi network with SSID and password
//  Serial.print("Connecting to ");
//  Serial.println(ssid);

  display.setRotation(2);
//  display.drawRect(1, 1, display.width(), display.height(), WHITE); 
  display.setTextSize(1);
  display.setTextColor(WHITE);        
  display.setCursor(1,1);
  display.println(F("Connecting to "));
  display.display();
  display.println(ssid);
  display.display();

  
  WiFi.begin(ssid, password);
  WiFi.config(ip, gateway, subnet); 
//  WiFi.setAutoReconnect(true);
  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print(".");
  display.print(".");
  display.display();
  delay(500);
  }
    display.println(".");
  
//  Print local IP address
//  Serial.printf("Connection status: %d\n", WiFi.status());
//  Serial.println("");
//  Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
//  Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());
//  Serial.print("IP: ");
//  Serial.println(WiFi.localIP());

  display.print("IP: ");
  display.println(WiFi.localIP());
  display.display();
  display.print("MAC: ");
  display.println(WiFi.macAddress());
  display.display();
  delay(5000);
  display.clearDisplay();

  server.begin();
  // End of setup
}

//******************* LOOP **********************
void loop() {
    if (meas_type == 0) measure();

WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
  if (meas_type == 1) measure();
//    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
//        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();            
            client.println("<DOCTYPE html> <HTML>");
            client.println("<head><style>");
            client.println("body {background-color: powderblue;}");
            client.println("h1 { color: red; text-align: center ; font-size: 80px;}");
            client.println("h2 { color: blue; text-align: center ; font-size: 160px;}");
            client.println("p { color: red; text-align: center;  font-size: 80px;}");
            client.println("</style></head>");
            client.println("<body><h1> Livello acqua pozzo</h1>");
            client.println("<h2>" + litri + "</h2>");
            client.println("<p> LITRI </p>");
            client.println(" </body>");
            client.println("</HTML>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;            
        } else { // if you got a newline, then clear currentLine
            currentLine = "";
        }
      } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
//    client.stop();
//    Serial.println("Client disconnected.");
//    Serial.println("");
  }

  
  // Pause between readings at least60 ms
  delay(1000);
  // End of Main Loop
}

void measure(void){
  
  int distanceCM = 0;                     
  unsigned long durationMS = 0;           
  // Do sounding here
  distanceCM = 0;
  durationMS = 0;
  // Clear the trigger pin
  digitalWrite(triggerPIN, LOW);
  delayMicroseconds(2);
  // Sets the trigger on HIGH state for 10 micro seconds
  digitalWrite(triggerPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPIN, LOW);
  // wait for the echo
  durationMS = pulseIn(echoPIN, HIGH);
  // Calculating the distance
  distanceCM = (((int) durationMS * 0.034) / 2);
  // Prints the distance on the Serial Monitor
  // Serial.print("Sample: ");
  // Serial.println(distanceCM);
  litri = 5500-(10*distanceCM);  // distance = -(litri -5500)/10
  litri_vis = litri.toInt();
  if (litri.toInt()<0) litri_vis=0;
  if (litri.toInt()>5200) (litri_vis=9999);
  if (litri_vis != last_litri_vis) {
  update_bar();
  update_value(); 
}
}
void update_value(void){

//  display.clearDisplay();
  display.fillRect(display.width()/2 +2, 1, display.width(), display.height(), BLACK);  
  display.setTextSize(2);
  display.setTextColor(WHITE);        
  display.setCursor(64,6);
  display.println(F("LITRI"));
  display.setCursor(70,34);
  display.println(litri_vis);
  display.display();
  last_litri_vis = litri_vis;

}

void update_bar(void){

  display.drawRect(1, 1, (display.width())/2 -12, display.height()-1, WHITE);

  if (litri_vis > 1250) display.fillRect(4, ((display.height()/4)*3)+1, (display.width())/2 - 18, (display.height()/4 -4), WHITE); 
  else                  display.fillRect(4, ((display.height()/4)*3)+1, (display.width())/2 - 18, (display.height()/4 -4), BLACK); 
  display.display();

  if (litri_vis > 2500) display.fillRect(4, ((display.height()/4)*2)+2, (display.width())/2 - 18, (display.height()/4 -4), WHITE);
  else                  display.fillRect(4, ((display.height()/4)*2)+2, (display.width())/2 - 18, (display.height()/4 -4), BLACK);
  display.display();
  
  if (litri_vis > 3750) display.fillRect(4, ((display.height()/4))+3, (display.width())/2 - 18, (display.height()/4 -4), WHITE); 
  else                  display.fillRect(4, ((display.height()/4))+3, (display.width())/2 - 18, (display.height()/4 -4), BLACK);  
  display.display();

  if (litri_vis > 5000) display.fillRect(4, 4, (display.width())/2 - 18, (display.height()/4 -4), WHITE);
  else                  display.fillRect(4, 4, (display.width())/2 - 18, (display.height()/4 -4), BLACK);
  display.display();

  
  last_litri_vis = litri_vis;


}
