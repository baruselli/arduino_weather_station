#include "credentials.h"
//includes myWriteAPIKey for thingspeak,ssid and pass for wifi
//#include <SPI.h>
//SD
 #include <SD.h> 
 const int chipSelect = 53;

//DHT
 #include "DHT.h"
 #define DHTPIN 4     // what pin we're connected to DHT 22
 #define DHTTYPE DHT22   // DHT 22  (AM2302)
 #define DHT11PIN 7     // what pin we're connected to DHT 11
 #define DHT11TYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);
DHT dht11(DHT11PIN, DHT11TYPE);

//ds18b20 temp 
#include <OneWire.h> 
#define ONE_WIRE_BUS 5  //DS18B20
OneWire oneWire(ONE_WIRE_BUS);
#include <DallasTemperature.h>
DallasTemperature sensors(&oneWire);

//pressure
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C   

//lux rgb sensor
#include "Adafruit_TCS34725.h"
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp, lux;

//uv digital
#include "Adafruit_SI1145.h"
Adafruit_SI1145 uv = Adafruit_SI1145();  

//////////////////////////////
//rtc
#include <Wire.h>
#include "RTClib.h"
// #include "DS3231.h"
//RTClib RTC;
//DS3231  rtc(SDA, SCL);
RTC_DS3231 RTC;
/////////////////////////////

//wind
const int ReedPin = 2;     // the number of the Reed Switch pin
volatile int turns=0;       //number of turns, volatile so it is always in memory
int turns_write;
unsigned long  int debounceDelay = 15;    // the debounce time; increase if the output flickers
unsigned long  int time0=0;
unsigned long int time1=0;
unsigned long last_interrupt_time = 0;
unsigned long interrupt_time;
long int time_wind0=0;
long int time_wind1=0;
const int n_times=1500;
unsigned int all_times[n_times];

//wifi & Thingspeak
#include "ThingSpeak.h"
#include"WiFiEspClient.h"
#include "WiFiEsp.h"
unsigned long myChannelNumber = 329109;


int status = WL_IDLE_STATUS; // the Wifi radio's status
WiFiEspClient client;
int resetpin=13;

 
//strings and files
String dataString;
String dataString2;
String dataString_tot;
File dataFile3;
File dataFile;
File dataFile2;
String data_file="data15.txt";
String wind_file="wind_4.txt";   //for 
String wind_file_c="wind2g.txt"; //for all the times of switching
//int wait_seconds=60;
long wait_millisec=300000;

/////////
String read_sensors(){
  Serial.print("Reading sensors...");
  dataString_tot = "";
  dataString_tot +=read_time(TRUE); //2
  dataString_tot +=read_dht22(TRUE);//2
  dataString_tot +=read_dht11(TRUE);//2
  dataString_tot +=read_bmp(TRUE);  //
  dataString_tot +=read_ds18b20(TRUE); //1
  dataString_tot +=read_analog(TRUE);
//  dataString_tot +=read_tcs(FALSE);
dataString_tot += return_none(6);
//  dataString_tot +=read_uv(FALSE);
dataString_tot += return_none(3);
  Serial.println("...OK reading sensors");
  return dataString_tot;
}
/////////////////

 void setup() {
    Serial.begin(115200);

  pinMode(chipSelect, OUTPUT); 
  digitalWrite(chipSelect, HIGH);
   Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    }
    else{
      Serial.println("card initialized.");
    }
  
 //reads data
 dataString=read_sensors();
 Serial.println(dataString);
 write_to_sd(data_file,dataString);



//wifi
Serial2.begin(115200);
pinMode(resetpin, OUTPUT);      // sets the digital pin as output
connectwifi();
//thingspeak
ThingSpeak.begin(client);

//wind
 time0=millis();
 time_wind0=time0;
 Serial.println(time_wind0);
 pinMode(ReedPin, INPUT);
 attachInterrupt(digitalPinToInterrupt(ReedPin),my_interrupt_handler,RISING);

 Serial.println("OK setup");
 
    }

///////////////////////////////////////
     
void loop() {
while (Serial2.available()) {
Serial.write(Serial2.read());
}
while (Serial.available()) {
Serial2.write(Serial.read());
}
time1=millis();
/*
if(time1%10000==0){
 Serial.print(time1);
 Serial.print(" ");
 Serial.print(time0);
 Serial.print(" ");
 Serial.print(time1-time0);
 Serial.print(" ");
 Serial.println(wait_millisec);
}*/

if( time1-time0>wait_millisec){
Serial.print(time0);
Serial.print(" ");
Serial.print(time1);
Serial.print(" ");
Serial.println(time1-time0);

time0=time1;



//wind
time_wind1=millis();
detachInterrupt(ReedPin);
turns_write=turns;
float velocity=float(turns_write)/float(time_wind1-time_wind0)*float(1000./4.*10.);
dataString2 = read_time(TRUE);
dataString2 += String(turns_write);                   
dataString2 += "\t";                   
dataString2 += String(time_wind1-time_wind0);                   
dataString2 += "\t";                   
dataString2 += String(velocity);    
Serial.print(time_wind0);
Serial.print(" ");               
Serial.println(time_wind1);               
Serial.print(dataString2);
write_to_sd(wind_file,dataString2);
if (turns>1) write_to_sd_wind(); //this writes all interrupt times
ThingSpeak.setField(8,velocity); //4 turns per second are 10 km/h
turns=0;
attachInterrupt(digitalPinToInterrupt(ReedPin),my_interrupt_handler,RISING);
time_wind0=millis();


//all the sensors
dataString=read_sensors();
Serial.println(dataString);
write_to_sd(data_file,dataString);

//thingspeak
//long int timethings=millis();
if (status==WL_CONNECTED){
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
}
else
{
Serial.println("No wifi, no thingspeak");
}

if (status!=WL_CONNECTED){
connectwifi();
}

}
    }
    
////////////Support functions


/////////////////////////////reads time sensor
String read_time(bool use){
   Wire.begin();
  if(use){
   /* Serial.print("RTC ");
    dataString  =rtc.getDateStr();                      //date
    dataString += "\t";
    dataString +=rtc.getTimeStr();                      //time
    dataString += "\t";
    return dataString ;*/
    Serial.print("RTC ");
    DateTime now = RTC.now();
    dataString=String(now.day())+"."+String(now.month())+"."+String(now.year())+"\t"+String(now.hour())+":"+String(now.minute())+":"+String(now.second())+"\t";
    return dataString;
}
else return "00.00.0000\t00:00:00\t";
}


/////////////////////////////reads dht22 temp sensor
String read_dht22(bool use){
    if(use){
    Serial.print("DHT22 ");
    dht.begin();
    dataString  = String(dht.readTemperature());        //T from DHT22
    dataString += "\t";
    dataString += String(dht.readHumidity());           //H from DHT22
    dataString += "\t";
    return dataString ;
    }
    else return return_none(2);
}

/////////////////////////////reads dht11 temp sensor
String read_dht11(bool use){
    if(use){
    Serial.print("DHT11 ");
    dht11.begin();
    dataString  = String(dht11.readTemperature());      //T from DHT11
    dataString += "\t";
    int hum=dht11.readHumidity();
    dataString += String(hum);         //H from DHT11
    dataString += "\t";
    ThingSpeak.setField(2,hum);
    return dataString ;
}
else return return_none(2);
}


/////////////////////////////reads bmp pressure sensor
String read_bmp(bool use){
  if(use && bmp.begin()){
    Serial.print("BMP ");
    dataString  = String(bmp.readTemperature());        //T from BMP280
    dataString += "\t";
    float pressure=bmp.readPressure();
    dataString += String(pressure);           //P from BMP280
    dataString += "\t";
    ThingSpeak.setField(3,pressure);
    return dataString;
    }
    else return return_none(2);
}


/////////////////////////////reads analog outputs
String read_analog(bool use){
  if(use){
      Serial.print("Analog0-6 ");
    int rain=analogRead(A0);
    dataString  = rain;                       //rain
    dataString += "\t";
    ThingSpeak.setField(4,1023-rain);
    int light = -200; //analogRead(A4);                       //light,  more light less signal
    dataString += light;
    dataString += "\t";
    int rain2=analogRead(A2);                       //rain2  
    dataString += rain2;
    dataString += "\t";
    int light2=analogRead(A3);                       //light2, more light less signal
    dataString += light2;
    dataString += "\t";
    int ir=analogRead(A1);
    dataString += ir;                       //IR, more light less signal
    ThingSpeak.setField(6,1023-ir);
    dataString += "\t";
    int light3=analogRead(A5);
    dataString += light3;                       //light 3, more light more signal
    ThingSpeak.setField(5,(1023-light2+light3)/2);
    dataString += "\t";
    int uv=analogRead(A6);
    ThingSpeak.setField(7,uv);
    dataString += uv;                       //UV
    dataString += "\t";
return dataString;
  }
else return return_none(7);
}


/////////////////////////////reads ds18b20 temp sensor
String read_ds18b20(bool use){
  if(use){
    Serial.print("DS18B20 ");
    sensors.begin();
    sensors.requestTemperatures(); // Send the command to get temperature readings DS18B20
    float temp=sensors.getTempCByIndex(0);
    dataString = String(temp);   //T fr om DS18B20
    dataString += "\t";
    ThingSpeak.setField(1,temp);
    return dataString;
  }
  else return return_none(1);
}


/////////////////////////////reads lux tcs sensor
String read_tcs(bool use){
  if(use && tcs.begin()){
    Serial.print("TCS ");
    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);
    dataString = String(r);        //
    dataString += "\t";
    dataString += String(g);        //
    dataString += "\t";
    dataString += String(b);        //
    dataString += "\t";
    dataString += String(c);        //
    dataString += "\t";
    dataString += String(colorTemp);        //
    dataString += "\t";
    dataString += String(lux);        //
    dataString += "\t";
    return dataString;
     }
     else return_none(6);
}


/////////////////////////////reads uv sensor
String read_uv(bool use){
  if ( use && uv.begin()){
    Serial.print("UV ");
    dataString =String(uv.readVisible());
    dataString += "\t";
    dataString+=String(uv.readIR());     
    dataString += "\t";
    dataString +=String(uv.readUV()/100.0);
    return dataString;     
    }
      else return return_none(3);
}     

///////////////////returns -200 if sensor does not work
String return_none(int n){
  dataString="";
for (int i=0; i<n;i++){
  dataString+="-200\t";
}
return dataString;
}

/////////////////writes file to sd
void write_to_sd(String data_file, String dataString){
  Serial.print(" Writing to "+ data_file + "...");
  //bool sd_ok=SD.begin(chipSelect);
 // Serial.print(sd_ok);
 //  if(SD.begin(chipSelect)){
//   dataFile = SD.open(string_to_char(data_file), FILE_WRITE);
   dataFile = SD.open(data_file, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.flush();
    dataFile.close();
    Serial.println("...OK writing to "+ data_file);
  }
  else {
    Serial.println("...error opening" + data_file);
  }
  //SD.end();
//}
//else
//{Serial.println("...no sd card!");
//}
}


////////////////////
void write_to_sd_wind(){
  wind_file_c;
  Serial.print(" Writing to "+ wind_file_c + "...");
   dataFile = SD.open(wind_file_c, FILE_WRITE);
  if (dataFile) {
    dataFile.println(read_time(TRUE));
    for (int i=1;i<min(turns,n_times);i++){
    dataFile.println(all_times[i]-all_times[i-1]);  
    }
    dataFile.flush();
    dataFile.close();
    Serial.println("...OK writing to "+ wind_file_c);
  }
  else {
    Serial.println("...error opening" + wind_file_c);
  }
}

/////////// wind interrupt
void my_interrupt_handler()
{
 interrupt_time = millis();
 // If interrupts come faster than debounceDelay, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > debounceDelay)
 {
//Serial.println(interrupt_time);  
if(turns<n_times){
  all_times[turns]=interrupt_time;
}
turns++;
// if(SD.begin(chipSelect)){
//dataFile3 = SD.open(string_to_char(wind_file_c), FILE_WRITE); 
/*dataFile3 = SD.open(wind_file_c, FILE_WRITE); 
dataFile3.println(interrupt_time);
dataFile3.flush();
dataFile3.close();*/
//SD.end();
// }
//if (turns % 20 ==0){
//    Serial.println(turns);  
//   }
last_interrupt_time = interrupt_time;
 }
 
}


char string_to_char(String string){
 char filename[string.length()+1];
   string.toCharArray(filename, sizeof(filename));
   return filename;
   }


void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectwifi(){
long int timewifi0=millis();
   digitalWrite(resetpin, LOW);   // sets the esp8266 off
 //  delay(5000);
   digitalWrite(resetpin, HIGH);  // sets the esp8266 on   
while ( status != WL_CONNECTED && millis()- timewifi0 <20000) {
   WiFi.init(&Serial2);
   //digitalWrite(resetpin, HIGH);   // sets the LED on
   
   Serial2.println("AT+RST");
   WiFi.disconnect();
   WiFi.reset();
   Serial.print("Attempting to connect to WPA SSID: ");
   Serial.println(ssid);
   status = WiFi.begin(ssid, pass);   // Connect to WPA/WPA2 network
}
if (status==WL_CONNECTED){
printWifiStatus();
}
else{
  Serial.println("No wifi");
}
}



