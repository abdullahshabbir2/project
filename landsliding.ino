#include <Adafruit_FONA.h>
#include <Adafruit_MPU6050.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <BH1750.h>
#include <OneWire.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <DallasTemperature.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Abdullah"
#define WIFI_PASSWORD "abdullah321"

// Create a WiFiClientSecure object
WiFiClientSecure client;

//sim800L configuration
#define SIM800L_RX 27
#define SIM800L_TX 26
#define SIM800L_PWRKEY 4
#define SIM800L_RST 5
#define SIM800L_POWER 23

HardwareSerial *sim800lSerial = &Serial1;
Adafruit_FONA sim800l = Adafruit_FONA(SIM800L_PWRKEY);

#define maxVal 20
#define minVal -20

//mpu6050 variables
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
float Ax, Ay, Az, Gx, Gy, Gz;

//bmp180 variable
Adafruit_BMP085 bmp;
float altitude, pressure, pressure_seaLevel;

//raindrop sensor variables
const int rainSensorMin = 0;     // sensor minimum
const int rainSensorMax = 4095;  // sensor maximum
const int rainSensorPin = 34;    // Pin 34 (GPIO34) for the raindrop sensor
int rainSensorValue;
int range;

//dht22 (temperature and humidity) sensor variables
const int dht22Pin = 32;
DHT dht(dht22Pin, DHT22);
float temperature;
float humidity;

//soil moisture sensor
const int soilMoisturePin = 33;
int dryValue = 0;     // Sensor value in dry conditions
int wetValue = 4095;  // Sensor value in wet conditions
int moistureSensorValue;
int soilMoistureLevel;

//light intensity (BH1750) sensor variables
BH1750 lightMeter;
float lux;

//vibration sensor variables
const int vibrationSensor = 14;
long vibration;

//soil temperature sensor
const int lm35Pin = 2;
float soilTemperature;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//Week Days
String weekDays[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

//Month names
String months[12] = { "January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December" };

void setup(void) {
  Serial.begin(9600);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();

  Serial.println("");
  delay(500);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  }

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(18000);

  Wire.begin();
  lightMeter.begin();

  Serial.println(F("BH1750 Test "));

  pinMode(vibrationSensor, INPUT);
  dht.begin();

  sim800lSerial->begin(4800, SERIAL_8N1, SIM800L_TX, SIM800L_RX);

  pinMode(SIM800L_PWRKEY, OUTPUT);
  pinMode(SIM800L_POWER, OUTPUT);

  digitalWrite(SIM800L_POWER, HIGH);
  delay(1000);
  digitalWrite(SIM800L_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(SIM800L_PWRKEY, LOW);
  delay(5000);

  if (!sim800l.begin(*sim800lSerial)) {
    Serial.println(F("Couldn't find GSM SIM800L"));
    while (1)
      ;
  }

  Serial.println(F("GSM SIM800L is OK"));
}

void sendData(String Ax, String Ay, String Az, String Gx, String Gy, String Gz, String altitude, String pressure, String sea_level, String lux, String rainStatus, String soilTemp, String vibration, float temperature, float humidity, String alert) {
  // wait for WiFi connection
  if ((WiFi.status() == WL_CONNECTED)) {

    timeClient.update();

    time_t epochTime = timeClient.getEpochTime();

    String time = timeClient.getFormattedTime();
    Serial.print("Formatted Time: ");
    Serial.println(time);

    //Get a time structure
    struct tm *ptm = gmtime((time_t *)&epochTime);

    int monthDay = ptm->tm_mday;

    int currentMonth = ptm->tm_mon + 1;

    int currentYear = ptm->tm_year + 1900;

    //Print complete date:
    String date = String(monthDay) + "-" + String(currentMonth) + "-" + String(currentYear);
    Serial.print("Current date: ");
    Serial.println(date);

    Serial.println("");

    delay(500);

    // Disable certificate verification
    client.setInsecure();

    HTTPClient https;

    Serial.print("[HTTPS] begin...\n");
    if (https.begin(client, "https://ap-southeast-1.aws.data.mongodb-api.com/app/data-qhtjy/endpoint/data/v1/action/insertOne")) {  // HTTPS

      https.addHeader("Content-Type", "application/json");
      https.addHeader("api-key", "kYL7CBHeWQMTjz3nWelYoZGox3eIgVxkS3ArFmigUaV3vHRkni9CvNhag1apIXsW");
      String payload = "{\r\n\"dataSource\":\"Cluster0\",\r\n\"database\":\"landsliding\",\r\n\"collection\":\"node\",\r\n\"document\": {\"Accel_x\": \"" + Ax + "\",\"Accel_y\": \"" + Ay + "\",\"Accel_z\": \"" + Az + "\",\"Gyro_x\": \"" + Gx + "\",\"Gyro_y\": \"" + Gy + "\",\"Gyro_z\": \"" + Gz + "\",\"altitude\": \"" + altitude + "\",\"pressure\": \"" + pressure + "\",\"Pressure at Sea Level\": \"" + sea_level + "\",\"lux\": \"" + lux + "\",\"rainStatus\": \"" + rainStatus + "\",\"soilTemperature\": \"" + soilTemp + "\",\"vibration\": \"" + vibration + "\",\"temperature\": \"" + String(temperature) + "\",\"humidity\": \"" + String(humidity) + "\",\"date\": \"" + date + "\",\"time\": \"" + time + "\",\"alert\": \"" + alert + "\"}\r\n}";
      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      int httpCode = https.POST(payload);

      // httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        Serial.printf("[HTTPS] GET... code: %d\n", httpCode);

        // file found at server
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = https.getString();
          Serial.println(payload);
        }
      } else {
        Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
      }

      https.end();
    } else {
      Serial.printf("[HTTPS] Unable to connect\n");
    }
  }
}

void loop() {

  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(101325);
  pressure_seaLevel = bmp.readSealevelPressure();

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Altitude = ");
  Serial.print(altitude);
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(pressure_seaLevel);
  Serial.println(" Pa");

  Serial.println();
  delay(500);

  // Read the analog value from LM35 sensor
  int sensorValue = analogRead(lm35Pin);
  Serial.println(sensorValue);

  // Convert analog reading to voltage
  float voltage = sensorValue * (5.00 / 4095.0);

  // Convert voltage to temperature in Celsius
  float temperatureC = voltage * 100.0;

  // Print temperature to serial monitor
  Serial.print("Soil Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  delay(500);

  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  Ax = accel.acceleration.x;
  Ay = accel.acceleration.y;
  Az = accel.acceleration.z;
  Gx = gyro.gyro.x;
  Gy = gyro.gyro.y;
  Gz = gyro.gyro.z;

  Serial.print("Accel X: ");
  Serial.print(Ax);
  Serial.print(" \tY: ");
  Serial.print(Ay);
  Serial.print(" \tZ: ");
  Serial.print(Az);
  Serial.print(" m/s^2 ");

  Serial.print("Gyro X: ");
  Serial.print(Gx);
  Serial.print(" \tY: ");
  Serial.print(Gy);
  Serial.print(" \tZ: ");
  Serial.print(Gz);
  Serial.println(" radians/s ");
  Serial.println();

  bool check3 = check_acc(Ax, Ay, Az);
  delay(500);

  lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  Serial.println();

  delay(500);

  // Read temperature and humidity from the DHT sensor
   temperature = dht.readTemperature();
   humidity = dht.readHumidity();

  Serial.print("Temperature:");
  Serial.println(temperature);

  Serial.print("Humidity:");
  Serial.println(humidity);
  Serial.println();

  delay(500);

  rainSensorValue = analogRead(rainSensorPin);
  Serial.print("rain Sensor Value:");
  Serial.println(rainSensorValue);
  Serial.println();

  range = map(rainSensorValue, rainSensorMin, rainSensorMax, 0, 3);
  String rainStatus = "";

  switch (range) {
    case 0:
      rainStatus = "Rain Warning";
      Serial.println("Raining");
      break;
    case 1:
      rainStatus = "Rain Warning";
      Serial.println("Raining");
      break;
    case 2:
      rainStatus = "Rain Warning";
      Serial.println("Raining");
      break;
    case 3:
      rainStatus = "No Rain";
      Serial.println("Not Raining");
      break;
  }

  delay(500);

  moistureSensorValue = analogRead(soilMoisturePin);
  Serial.println("Soil Moisture:");
  Serial.println(moistureSensorValue);
  Serial.println();

  soilMoistureLevel = map(moistureSensorValue, dryValue, wetValue, 100, 0);

  Serial.print("Moisture Level: ");
  Serial.print(soilMoistureLevel);
  Serial.println("%");
  Serial.println();

  bool check1 = check_moisture(soilMoistureLevel);
  delay(500);

  vibration = pulseIn(vibrationSensor, HIGH);
  delay(50);

  float vibrationFloat = static_cast<float>(vibration);

  Serial.println("vibration:");
  Serial.println(vibrationFloat);
  Serial.println();

  bool check2 = check_vibration(vibrationFloat);
  delay(500);

  String alert = "";

  if (check1 == false && check2 == false && check3 == false) {
    alert = "0";
  } else if (check3 == true) {
    alert = "3";
  } else if (check2 == true) {
    alert = "2";
  } else if (check1 == true) {
    alert = "1";
  }

  Serial.println(alert);
  
  delay(200);

  sendData(String(Ax), String(Ay), String(Az), String(Gx), String(Gy), String(Gz), String(altitude), String(pressure), String(pressure_seaLevel), String(lux), rainStatus, String(temperatureC), String(vibrationFloat), temperature, humidity, alert);
  delay(200);
}

//check accelerometer values
bool check_acc(float xValue, float yValue, float zValue) {
  if (xValue < minVal || xValue > maxVal || yValue < minVal || yValue > maxVal || zValue < minVal || zValue > maxVal) {
    // Specify the recipient's phone number and SMS content
    String phoneNumber = "+923204352526";  // Replace with the recipient's phone number
    String message = "EMERGENCY ALERT!!";

    // Convert String objects to character arrays
    char phoneNumberArray[phoneNumber.length() + 1];
    char messageArray[message.length() + 1];
    phoneNumber.toCharArray(phoneNumberArray, phoneNumber.length() + 1);
    message.toCharArray(messageArray, message.length() + 1);

    // Send SMS
    if (sim800l.sendSMS(phoneNumberArray, messageArray)) {
      Serial.println(F("SMS sent successfully!"));
    }
    return true;
  }
  return false;
}

bool check_vibration(float vibration) {
  if (vibration >= 1000) {
    // Specify the recipient's phone number and SMS content
    String phoneNumber = "+923204352526";  // Replace with the recipient's phone number
    String message = "EMERGENCY ALERT!! Vibration: " + String(vibration);

    // Convert String objects to character arrays
    char phoneNumberArray[phoneNumber.length() + 1];
    char messageArray[message.length() + 1];
    phoneNumber.toCharArray(phoneNumberArray, phoneNumber.length() + 1);
    message.toCharArray(messageArray, message.length() + 1);

    // Send SMS
    if (sim800l.sendSMS(phoneNumberArray, messageArray)) {
      Serial.println(F("SMS sent successfully!"));
    }
    return true;
  }
  return false;
}

bool check_moisture(int level) {
  if (level >= 65) {
    // Specify the recipient's phone number and SMS content
    String phoneNumber = "+923204352526";  // Replace with the recipient's phone number
    String message = "EMERGENCY ALERT!! level: " + String(level);

    // Convert String objects to character arrays
    char phoneNumberArray[phoneNumber.length() + 1];
    char messageArray[message.length() + 1];
    phoneNumber.toCharArray(phoneNumberArray, phoneNumber.length() + 1);
    message.toCharArray(messageArray, message.length() + 1);

    // Send SMS
    if (sim800l.sendSMS(phoneNumberArray, messageArray)) {
      Serial.println(F("SMS sent successfully!"));
    }
    return true;
  }
  return false;
}