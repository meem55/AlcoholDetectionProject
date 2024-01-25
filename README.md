#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

const int motorPin1 = D1; // Connect this pin to input 1 on the L293D motor driver
const int motorPin2 = D2; // Connect this pin to input 2 on the L293D motor driver
const int alcoholSensorPin = A0; // Connect the analog pin of the alcohol sensor to A0
const int ledPin = D8;
const int buz = D7;
const char* ssid = "Meem";
const char* password = "xxxxxxx";
WiFiClient client;

long myChannelNumber = 2289941; // Replace with your ThingSpeak Channel Number
const char myWriteAPIKey[] = "MBABP3M1I3U82ZSF"; // Replace with your ThingSpeak Write API Key

#define GPS_RX D5
#define GPS_TX D6
#define GPS_BAUD 9600
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(ledPin, OUTPUT); // Set LED pin as OUTPUT
  pinMode(buz, OUTPUT);    // Set Buzzer pin as OUTPUT

  Serial.begin(9600);
  gpsSerial.begin(GPS_BAUD);

  connectToWiFi();
  ThingSpeak.begin(client);
}

void loop() {
  int alcoholLevel = analogRead(alcoholSensorPin);

  if (alcoholLevel > 700) {
    stopMotor();
    Serial.println("Alcohol detected! Motor stopped.");
    Serial.println("Data sent to ThingSpeak.");
    Serial.println("Notification message sent to recipients number");
  } else {
    runMotor();
    Serial.println("No alcohol detected. Motor running.");
  }

  delay(1000);
}

void connectToWiFi() {
  Serial.println("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }

  Serial.println("Connected to WiFi");
}

void runMotor() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(buz, LOW);
}

void stopMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(ledPin, HIGH);
  digitalWrite(buz, HIGH);

  int alcoholValue = analogRead(alcoholSensorPin);
  float voltage = (alcoholValue / 1023.0) * 3.3;
  float alcoholConcentration = map(voltage, 0, 3.3, 0, 100);

  Serial.print("Alcohol Concentration: ");
  Serial.println(alcoholConcentration);

  ThingSpeak.writeField(myChannelNumber, 1, alcoholConcentration, myWriteAPIKey);
  delay(2000);

  if (gps.location.isValid()) {
    float latitude = static_cast<float>(gps.location.lat());
    float longitude = static_cast<float>(gps.location.lng());

    ThingSpeak.writeField(myChannelNumber, 2, latitude, myWriteAPIKey);
    ThingSpeak.writeField(myChannelNumber, 3, longitude, myWriteAPIKey);
  } else {
    Serial.println("GPS is not connected yet.");
  }

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      Serial.println(gps.location.lat());
    }
  }

  delay(1000);
}
