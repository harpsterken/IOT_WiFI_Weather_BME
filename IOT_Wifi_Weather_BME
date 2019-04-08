// IOT WiFi Weather Project
//  Read BME sensor and write data to the 20x4 LCD and send the data to my Firebase cloud database
//
// Notes:
//  A certificate for firebase must be added to the WiFi module in order to connect to Firebase!
//  www.firebaseio.com
//  For instructions, currently refer to https://www.arduino.cc/en/Tutorial/FirmwareUpdater
//
// Sunfounder LCD I2C Board to UNO WiFi Rev 2 connections
//  GND <---> GND
//  VCC <---> 5V
//  SDA <---> SDA
//  SCL <---> SCL
//
// HiLetgo BME280 I2C Board to UNO WiFi Rev 2 connections
//  VCC <---> 3.3V
//  GND <---> GND
//  SCL <---> SCL
//  SDA <---> SDA
//  CSB <---> NC
//  SDO <---> NC

// Include the library code
#include <WiFiNINA.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "arduino_secrets.h"

// Please enter your sensitive data in the added file/tab arduino_secrets.h
char ssid[] = SECRET_SSID;                        // your network SSID (name)
char pass[] = SECRET_PASS;                        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                                 // your network key Index number (needed only for WEP)

const char fireBaseServer[] = FIREBASE_SERVER;    // Path to the Firebase application
const String fireBaseSecret = FIREBASE_SECRET;    // Secret for the database, it's deprecated but what I want to use for authentication

// Initialize the Wifi client library
int status = WL_IDLE_STATUS;
WiFiSSLClient client;

// Define/initialize the HiLetgo BME280 Sensor using I2C
Adafruit_BME280 bme;

// Define/initialize the LCD (20 chars by 4 lines) using I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);

const String putString = "PUT /weather.json?auth=" + fireBaseSecret + " HTTP/1.1";            // Initialize string used for PUT command
String jsonValue = "";                                                                        // Initialize string used for JSON data values
String humidity = "0";                                                                        // Initialize display humidity variable
String temperature = "0";                                                                     // Initialize display temperature variable
String pressure = "0";                                                                        // Initialize display pressure variable

float tempCalibration = -3;                                                                   // temperature calibration value
float humidCalibration = 7;                                                                   // humidity calibration value
float pressCalibration = .11;                                                                 // pressure calibration value

float prevTempF = 0;                                                                          // define last temperature value sent to the database
float prevHumid = 0;                                                                          // define last humidity value sent to the database
float prevPress = 0;                                                                          // define last pressure value sent to the database

// ------------------------------------
// Method to create the wifi connection
// ------------------------------------
void createConnection() {
  
  // disconnect the Wifi network 
  status=WiFi.disconnect();
  
  // attempt to connect to Wifi network
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the status:
  //printWifiStatus();
 }
 
// -------------------------------------------
// Method to print the wifi status information
// -------------------------------------------
void printWifiStatus() {

  // print the SSID of the network you're attached to:
  Serial.print("SSID: "); 
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// -----------------------------------------------------
// Method makes a HTTP connection to the Firebase server
// -----------------------------------------------------
void httpRequest() {
  
  // Close any connection before send a new request.
  // This will free the socket on the Nina module
  client.stop();

  //Serial.print("Trying to Connect to Firebase server: ");
  //Serial.println(fireBaseServer);

  if (client.connectSSL(fireBaseServer, 443)) {
    //Serial.println("Connected to Firebase server");

    // Populate the json value string to use for PUT
    jsonValue = "{\"humidity\": " + humidity + ", \"temperature\": " + temperature + ", \"pressure\": " + pressure + "}";

    // Make the HTTP request to update temperature to the Firebase server
    client.println(putString);
    client.print("Host: ");
    client.println(fireBaseServer);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(jsonValue.length());
    client.println("Connection: close");
    client.println();
    client.print(jsonValue);
  } else {
    // if you couldn't make a connection:
    //Serial.println("Connection to Firebase server failed"); 
    createConnection();    
  }
}

// ------------
// Setup method
// ------------
void setup() {

  // Define a degree symbol for the LCD
  byte degreeSymbol[8] = {
    0b00110,
    0b01001,
    0b01001,
    0b00110,
    0b00000,
    0b00000,
    0b00000,
    0b00000
  };

  // Initialize the LCD display
  lcd.init();                           // Initialize the LCD
  lcd.backlight();                      // Turn the LCD backlight on
  lcd.clear();                          // Clear the LCD screen, position cursor to upper-left corner
  lcd.setCursor(0, 0);                  // Set the cursor to column 0, line 0
  lcd.print("- IOT_WiFi_Weather -");    // Print heading verbiage to the LCD
  lcd.setCursor(0, 1);                  // Set the cursor to column 0, line 1
  lcd.print("Temperature:");            // Print heading verbiage to the LCD
  lcd.setCursor(0, 2);                  // Set the cursor to column 0, line 2
  lcd.print("Humidity:");               // Print heading verbiage to the LCD
  lcd.setCursor(0, 3);                  // Set the cursor to column 0, line 3
  lcd.print("Pressure:");               // Print heading verbiage to the LCD

  // Create degree custom character
  lcd.createChar(0, degreeSymbol);

  // Initialize serial and wait for port to open (Needed for native USB port only)
  Serial.begin(9600);
  while (!Serial) {
    // waiting for serial port to connect
  }

  // Check for the BME280 sensor/set the device's I2C address
  if (! bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    // don't continue
    while (true);
  }

  // Set the BME280 parameters for weather monitoring
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,     // temperature
                  Adafruit_BME280::SAMPLING_X1,     // pressure
                  Adafruit_BME280::SAMPLING_X1,     // humidity
                  Adafruit_BME280::FILTER_OFF   );

  // Check for the WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.2.1") {
    Serial.println("Please upgrade the firmware");
  }

  // Create wifi connection 
  createConnection();
}

// ------------
// Loop method
// ------------
void loop() {

  // Only needed in forced mode, has no effect in normal mode
  bme.takeForcedMeasurement();

  // Initialize reading work variables
  float avgTemp = 0;
  float avgHumid = 0;
  float avgPress = 0;
  int readLimit = 10;
  
  // Read the BME280 readLimit times and sum the temperature, humidity, and pressure to smooth out variances
  for (int i=1; i<=readLimit; i+=1) {
    avgTemp = avgTemp + bme.readTemperature();
    avgHumid = avgHumid + bme.readHumidity();
    avgPress = avgPress + bme.readPressure();
    delay(5000);                          // Wait 5 seconds
  }

  // Calculate the averages
  avgTemp /= readLimit;
  avgHumid /= readLimit;
  avgPress /= readLimit;
  
  // Convert the average temperature from Centigrade to Fahrenheit and adjust by calibration value
  float readTemp = ((avgTemp * 1.8) + 32.0) + tempCalibration;
  readTemp = float(int(readTemp * 10 + 0.5)) / 10;

  // Adjust the average humidity for display
  float readHumid = avgHumid + humidCalibration;
  readHumid = float(int(readHumid * 10 + 0.5)) / 10;

  // Take the average pressure in Pascals, divide by 3389.39 to get inches-Hg and adjust by calibration value
  float readPress = (avgPress / 3389.29) + pressCalibration;

  // Only process the temperature, humidity, pressure data if it has changed since the last read
  if ((readTemp != prevTempF) || (readHumid != prevHumid) || (readPress != prevPress)) {

    // Convert the temperature, humidity, and pressure values to desired string format
    temperature = String(readTemp);
    temperature = temperature.substring(0, (temperature.length() - 1));
    humidity = String(readHumid);
    humidity = humidity.substring(0, (humidity.length() - 1));
    pressure = String(readPress);

    // Update the LCD display with the current temperature, humidity, and pressure
    lcd.setCursor(13, 1);                 // Set the cursor to column 13, line 1
    lcd.print(temperature);               // Print the temperature value to the LCD
    lcd.print((char)0);                   // Print the degree character to the LCD
    lcd.print("F");
    lcd.setCursor(13, 2);                 // Set the cursor to column 13, line 2
    lcd.print(humidity);                  // Print the humidity value to the LCD
    lcd.print("%");
    lcd.setCursor(13, 3);                 // Set the cursor to column 13, line 3
    lcd.print(pressure);                  // Print the pressure value to the LCD

    // Connect and send the data to Firebase
    httpRequest();

    // Update the previous readings variables
    prevTempF = readTemp;
    prevHumid = readHumid;
    prevPress = readPress;

    delay(120000);                         // Wait 2 minutes
  }
}
