
/*****************************************************

*******************************
*** Ethernet Sensor station ***
*******************************

Components:
- Arduino UNO (or DUE)
- Arduino ETHERNET shield (normal or PoE)
- Adafruit DHT11: temperature & humidity sensor
- MQ135: gas sensor
- HC-SR505: motion sensor

******************************************************

  Settings for running the code on Arduino IDE (Tools):
  Board: "Arduino Genuino/Uno" or "Arduino Due (Programming Port)"
  Upload Speed: "9600"
  Programmer: "AVRISP mkll"
  
  For Linux, if needed and to allow communication
  on serial port, please enter in a terminal:
  sudo chmod a+rw /dev/ttyACM0

******************************************************

  The only data that must be checked or changed by users in this code
  are the ThingsBoard login details:
  
  const char* server = "[ThingsBoard account url]";
  const char* username = "[Authentification token]";

*****************************************************/


/* Libraries */

// Adafruit Unified Sensor Lib: https://www.arduinolibraries.info/libraries/adafruit-unified-sensor
#include <Adafruit_Sensor.h>    // For using an unified sensor abstraction layer
// DHT Sensor Library: https://www.arduinolibraries.info/libraries/dht-sensor-library
#include <DHT.h>                // For monitoring the DHT11 Temperature & Humidity sensor
#include <DHT_U.h>
// Ethernet: https://www.arduinolibraries.info/libraries/ethernet
#include <Ethernet.h>           // For establishing Internet connection via Ethernet
// PubSubClient: https://www.arduinolibraries.info/libraries/pub-sub-client
#include <PubSubClient.h>       // For establishing MQTT connection


/* Pinout */

#define DHTPIN    7   // Digital pin connected to the DHT11 sensor
#define HCPIN     6   // Digital pin connected to the HC-SR505 sensor
#define MQPIN_AN  0   // Analog pin (A0) connected to the MQ135 sensor
//#define MQPIN_DIG 2   // Digital pin connected to the MQ135 sensor
#define RESETPIN  12  // Pin connected to the Arduino Reset pin


/* Initialization & Variables declaration */

/* DHT11 */
#define DHTTYPE DHT11    // DHT 11
//#define DHTTYPE DHT22  // DHT 22 (AM2302)
//#define DHTTYPE DHT21  // DHT 21 (AM2301)
DHT_Unified dht(DHTPIN, DHTTYPE);

/* Data variables */
float temperature1,       // Used to store temperature
      humidity1,          // Used to store humidity
      air_quality,        // Used to store air quality
      motion_detector;    // Used to store movement

/* Ethernet */
EthernetClient ethernetClient;
PubSubClient mqttClient(ethernetClient);
byte mac[] = { 0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED }; // Random MAC address to connect as
int status; // Ethernet status

/* Time variables */
unsigned long previousMillis = 0;   // Last time updated
long interval = 1000;               // Interval at which to publish data (milliseconds)


/****************************** ThingsBoard login details ******************************/

/* For thingsboard.tec-gateway.com */
const char* server = "thingsboard.tec-gateway.com";     // MQTT Broker (i.e. server): must be checked (or changed if needed)
const char* username = "nimbus_ethernet";               // Authentification token: must be checked (or changed if needed)

const char* client_id = "Arduino";                                    // Can be anything
const int port = 1883;                                                // Default MQTT port
const char* topicToPublish_DATA = "v1/devices/me/telemetry";          // Topic address to publish to for sending data
const char* topicToPublish_ATTRIBUTES = "v1/devices/me/attributes";   // Topic address to publish to for sending attributes

/****************************************************************************************/


/*****************************************************
*****************************************************/


void setup() {
  
    /* Sets the serial port to 9600 */
    Serial.begin(9600);  // Opens serial communication
    while (!Serial) {
      ; // Waits for serial port to connect
    }

  /* Initializes devices */
  dht.begin();

  /* Prints information screen */
  Serial.println();
  Serial.println(F("*******************************"));
  Serial.println(F("*** Ethernet Sensor station ***"));
  Serial.println(F("*******************************"));
  Serial.println();
  Serial.println(F("Components:"));
  Serial.println(F("- Arduino UNO (or DUE)"));
  Serial.println(F("- Arduino ETHERNET shield (normal or PoE)"));
  Serial.println(F("- Adafruit DHT11: temperature & humidity sensor"));
  Serial.println(F("- MQ135: gas sensor"));
  Serial.println(F("- HC-SR505: motion sensor"));
  Serial.println();

  /* Prints sensors details */
  
  // Print DHT11 temperature & humidity sensor details
  Serial.println(F("************************************"));
  Serial.println(F("Temperature & Humidity Sensor"));
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:  ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Temperature range:   ["));
  Serial.print(sensor.min_value); Serial.print(F("°C ; "));
  Serial.print(sensor.max_value); Serial.println(F("°C]"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  dht.humidity().getSensor(&sensor);
  Serial.print  (F("Humidity range:      ["));
  Serial.print(sensor.min_value); Serial.print(F("% ; "));
  Serial.print(sensor.max_value); Serial.println(F("%]"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("************************************"));
  Serial.println();

  /* HC-SR505 */
  pinMode(HCPIN,INPUT);
  digitalWrite(HCPIN,LOW);

  status = Ethernet.begin(mac);       // Start the internet connection
  CheckEthernet();
  delay(1500);                        // Allows the hardware to sort itself out
  mqttClient.setServer(server, port); // Configures the server adress and port
  mqttClient.connect(client_id, username, username);

}


/*****************************************************
*****************************************************/


/* The order of methods to handle data */
void workflow(void) {
  
  measure();           // First measure and save the data
  send_data();         // Publish data to ThingsBoard
  
}


/* Sends data with MQTT to ThingsBoard Cloud IoT platform */
void send_data(void) {
  
  char dht11[200];
  char mq135[200];
  char hcsr505[200];

  sprintf(dht11,"{\"Temperature\":%f,\"Humidity\":%f}",temperature1,humidity1);
  sprintf(mq135,"{\"Air_quality\":%f}",air_quality);
  sprintf(hcsr505,"{\"Motion_detector\":%f}",motion_detector);
  
  int result_dht11 = mqttClient.publish(topicToPublish_DATA, dht11);        // Publishes JSON data to ThingsBoard
  int result_mq135 = mqttClient.publish(topicToPublish_DATA, mq135);        // Publishes JSON data to ThingsBoard
  int result_hcsr505 = mqttClient.publish(topicToPublish_DATA, hcsr505);    // Publishes JSON data to ThingsBoard

  // Checks if data have been successfully sent
  if (result_dht11==0) {
    Serial.print(F("Sending from DHT11 failed"));
  }
  if (result_mq135==0) {
    Serial.print(F("Sending from MQ135 failed"));
  }
  if (result_hcsr505==0) {
    Serial.print(F("Sending from HC-SR505 failed"));
  }  
}


/* Connects and reconnects MQTT */
void ConnectMQTT() {
  
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.println(F("Attempting MQTT connection...")); // Attempts to connect
    if (mqttClient.connect(client_id, username, username)) { // PASSWORD null, because authentication token is put into username
      Serial.println(F("Connected!"));
      workflow(); // publish data, once connected
    }
    else {
      Serial.println(F("Not connected!"));
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
  
}


/* Measures all the sensor values available */
void measure(void) {

  Serial.println();
  Serial.println(F("************************************"));
  Serial.println(); 

  /* DHT11 */
  
  Serial.println(F("Adafruit DHT11: temperature & humidity sensor"));
  
  // Gets temperature event and print its value
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    temperature1=event.temperature; // Saves the temperature value on global variable
    Serial.print(F("Temperature = "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }

  // Gets humidity event and print its value
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    humidity1=event.relative_humidity; // Saves the humidity value on global variable
    Serial.print(F("Humidity = "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  Serial.println();

  /* MQ135 */
  
  Serial.println(F("MQ135: gas sensor"));
  air_quality = analogRead(MQPIN_AN);     // Reads analog input pin 0 and saves the air quality value on global variable
  Serial.print("Air Quality = ");
  Serial.print(air_quality, DEC);  // Prints the value read
  Serial.println(" PPM");
  Serial.println();

  /* HC-SR505 */
  Serial.println(F("HC-SR505: motion sensor"));
  if(digitalRead(HCPIN)==HIGH) {
    motion_detector=1; // Saves the movement value on global variable
    Serial.println("Somebody is here");
  }
  else {
    motion_detector=0; // Saves the movement value on global variable
    Serial.println("Nobody is here");
  }
  Serial.println();
  Serial.println(F("************************************"));
  Serial.println();

}


void ConnectEthernet() {
  while (status == 0) {
    status = Ethernet.begin(mac);
    Serial.println("Trying to connect to Ethernet");
    delay(3000);
  } 
  Serial.println("Connected to Ethernet");
}


void CheckEthernet() {
  Serial.println("Checking the Ethernet");
  if (status == 0) {
    Serial.println("DHCP connection failed");
    ConnectEthernet();
    }
  else {
    Serial.println("DHCP connection succesfull"); 
  }
  
}


// Reboots the Arduino if needed
void Reset() 
{
    
  // Activates the output pin
  pinMode(RESETPIN, OUTPUT); 
  // Resets the Arduino by forcing the low level
  digitalWrite(RESETPIN, LOW);
}


/*****************************************************
*****************************************************/


void loop() {
  
  unsigned long currentMillis = millis(); // Stores the time since the board started

  // If MQTT disconnected, reboots the ESP32
  if (!mqttClient.connected()) {
    Reset();
  }
  
  // When the time set to interval has passed, it will execute a command (publishing data to ThingsBoard)
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis; // Resets the previous millis, so that it will continue to publish data
    workflow(); // Workflow to use data
  }
  
  mqttClient.loop(); // Called to maintain connection to server 
  
}
