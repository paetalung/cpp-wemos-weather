#include "time.h"
#include "ESP8266WiFi.h"
#include "DHT.h"
#include "TridentTD_LineNotify.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// Sensor = Temp
// #define DHTPIN 4 // GPIO PIN 4    //temp sensor
// #define DHTTYPE DHT11 // DHT 11   // temp sensor

 // Sensor = BMP280
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp; // I2C

// Sensor = Volatage Sensor
int offset = 30;// set the correction offset value
bool check_blackout = false;  
// Call function of Sensor Lib
//DHT dht(DHTPIN, DHTTYPE);

// Hotspot@OFFICE
bool wifiCheck_status = true;
int wifiCount = 0;
const char* ssid     = "XXXXXXX";                    // your network SSID (name)
const char* password = "XXXXXXX";                      // your network password
// Hotspot@HOME
// const char* ssid     = "XXXXXXX;                            // your network SSID (name)
// const char* password = "XXXXXXX";                    // your network password
#define LINE_TOKEN "XXXXXXX" // แก้ Line Token

// Pressure param @ VTSE
const double  Elevation = 1365.7;  // centimeter  13.657 m.
double  Elevation_feet = Elevation/30;  // feet
//const double  baro_error = -0.07;  // mb.
double  Elevation_QFE = Elevation_feet/30;
double hight_floor = 1025;  // cm.
double heat_circuit = 0.35; // Heat from Circuit

// Time param
const char* ntpServer = "ntp.ku.ac.th";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 7*3600;
const long min_to_post = 50;           // Setting time for send data in min
const long sec_to_post = 05;            // Setting time for send data in sec
bool time_trigger = false;              // Trigger for send data

char buffer[80];

// Function for Print time
void printLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
 
  strftime (buffer,80,"%d %B %Y \nUTC+7 @ %H:%M \n",timeinfo); //:%S
  
  if(timeinfo->tm_min == min_to_post && timeinfo->tm_sec == sec_to_post){   //  timeinfo->tm_min == min_to_post &&
    time_trigger = true;
    Serial.println(time_trigger);
    Serial.println(buffer);
    //break;
  }
  
}

void wifiReconnect()
{  
      wifiCheck_status = false;
      delay(50);
      
      // We reconnecting to a WiFi network
      Serial.print("\n\n  Reconnecting to ");
      Serial.println(ssid);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      
      while (WiFi.status() != WL_CONNECTED)
          {
            delay(500);
            Serial.print(".");
          }
      Serial.println();
      Serial.println("WiFi reconnected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      
      // Connect to Line API
      delay(100);
      LINE.setToken(LINE_TOKEN);
      LINE.notify("WiFi reconnected!");
      wifiCount = 0;
   
}

void setup() 
{
  Serial.begin(115200);
  
  //dht.begin();
  //Serial.println();
  //Serial.println(F("BMP280 test"));
  //Serial.println(LINE.getVersion());
  delay(10);

  // Start BMP280
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  // We start by connecting to a WiFi network
  Serial.print("\n\nConnecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  wifiCheck_status = true;
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Connect to Line API
  LINE.setToken(LINE_TOKEN);
  LINE.notify("Start!");

  // Connect to Time Server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("\nWaiting for time");
  unsigned timeout = 5000;
  unsigned start = millis();
  while (!time(nullptr)) 
  {
    Serial.print(".");
    delay(1000);
  }
  delay(1000);
  //Serial.println("Time...");
}

void loop() 
{
  int time_to_read = 1000;
  
  // Sensor = Volatage Sensor 
  int volt = analogRead(A0);// read the input
  double V = map(volt,0,1023, 0, 2500) - offset;// map 0-1023 to 0-2500 and add correction offset
  V /=100;// divide by 100 to get the decimal values
  
  //Serial.print("Voltage: ");
  //Serial.print(voltage);//print the voltge
  //Serial.println("V");
  
  
  String VI="";

  if(V >= 5.0 && check_blackout == true){
       VI="...Ready...";
       check_blackout = false;
       time_trigger = true;
       time_to_read = 100;
    }
    
    if(V<=2.0){
       VI="...BLACKOUT...!!";
       time_trigger = true;
       time_to_read = 5000;
       check_blackout = true;
    }else if(V<5.0){
       VI="...BROWNOUT...!!";
       time_trigger = true;
       time_to_read = 50;
    }else{
      VI="...Ready...";
    }    
  
  //time_t now = time(nullptr);
  //Serial.print(ctime(&now));
  
  printLocalTime(); // Check time to print
  
  if(time_trigger == true){
      
      String val = "Obs Room\n";
      //int h = dht.readHumidity(); // ความชื้น DHT
      //int t = dht.readTemperature(); // อุณหภูมิ DHT

      double t = bmp.readTemperature(); // อุณหภูมิ BMP280
             t -= heat_circuit ; // decreast by temp in circuit
      double QNH = bmp.readPressure() / 100.0F ; // ความกด BMP280
      double a = bmp.readAltitude(1013.25); // ความสูง BMP280
      double QFE_VTSE = QNH - Elevation_QFE;
             QFE_VTSE = QFE_VTSE+(hight_floor/900)-0.25;
      double QFE_VTSE_inhg = QFE_VTSE/33.864;

            
        // Check status of Sensor DHT
       //if (isnan(h) || isnan(t)) {
        //Serial.println("Failed to read from DHT sensor!");
       // return;
        //}
      val = val + buffer +"\n";
      
      val = val +"QNH [hPa] : " + QNH +"\n";
      val = val +"QFE [hPa] : " + QFE_VTSE +"\n";
      val = val +"QFE '[inHg] : " + QFE_VTSE_inhg +"\n";
      val = val +"Alti [m]: " + a +"\n\n";
      
      val = val +"Temp [C] : "+ t+"\n";
      val = val +"Volt [V] : " + V*34.5 + "\n";
      val = val +"Power : " + VI;
      
      
      Serial.println(val);
      LINE.notify(val);
      time_trigger = false;
  }  

  // Check for reconnect WiFi
  if(wifiCount == 10*1000){
          if(WiFi.status() != WL_CONNECTED){          
               wifiReconnect();             
          }
          if(wifiCheck_status == false){
              LINE.notify("WiFi Reconnected");
              wifiCheck_status = true;
          }
  }
  
  wifiCount += 1000;
  delay(time_to_read);
  
}
