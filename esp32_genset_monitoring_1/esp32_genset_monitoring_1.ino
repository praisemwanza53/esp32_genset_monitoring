#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// === OLED Settings ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === Pin Definitions ===
const int ptcPin = 19;         // A19 - PTC Temp Sensor
const int relayPin = 21;       // Relay Output
const int trigPin = 23;        // Ultrasonic TRIG
const int echoPin = 22;        // Ultrasonic ECHO

// === Tank Parameters ===
const float TANK_HEIGHT_CM = 30.0;  // Adjust to your actual tank height

// === GPS Serial (UART2) ===
HardwareSerial GPSserial(2);   // UART2 for GPS
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  GPSserial.begin(9600, SERIAL_8N1, 16, 17); // GPS: RX=16, TX=17

  pinMode(ptcPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Fuel Monitor Booting...");
  display.display();
  delay(1500);
}

// === Ultrasonic Read Function ===
float readUltrasonicCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  float distance = duration * 0.034 / 2;
  return distance;
}

void loop() {
  // === Read Temperature ===
  int ptcRaw = analogRead(ptcPin);
  float voltage = ptcRaw * (3.3 / 4095.0); 
  float temperature = (voltage - 0.5) * 100.0;

  // === Fuel Level from Ultrasonic ===
  float distance = readUltrasonicCM();
  float fuelLevel = TANK_HEIGHT_CM - distance;
  fuelLevel = constrain(fuelLevel, 0, TANK_HEIGHT_CM);

  // === Fuel Percentage ===
  float fuelPercent = (fuelLevel / TANK_HEIGHT_CM) * 100.0;
  String fuelStatus = (fuelLevel < 5.0) ? "LOW" : "OK";

  // === GPS Read ===
  while (GPSserial.available()) {
    gps.encode(GPSserial.read());
  }

  // === Relay Logic ===
  if (temperature > 40.0 || fuelLevel < 5.0) {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
  }

  // === Serial Monitor Output ===
  Serial.print("Temp: ");
  Serial.print(temperature, 1);
  Serial.print("C | Fuel: ");
  Serial.print(fuelLevel, 1);
  Serial.print("cm (");
  Serial.print(fuelPercent, 0);
  Serial.print("%) | Status: ");
  Serial.print(fuelStatus);
  if (gps.location.isValid()) {
    Serial.print(" | GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
  }
  Serial.print(" | Relay: ");
  Serial.println(digitalRead(relayPin) ? "ON" : "OFF");

  // === OLED Display Output ===
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temperature, 1);
  display.println(" C");

  display.print("Fuel: ");
  display.print(fuelLevel, 1);
  display.print("cm ");
  display.print(fuelPercent, 0);
  display.println("%");

  display.print("Relay: ");
  display.println(digitalRead(relayPin) ? "ON" : "OFF");

  if (gps.location.isValid()) {
    display.print("Lat:");
    display.println(gps.location.lat(), 2);
    display.print("Lng:");
    display.println(gps.location.lng(), 2);
  } else {
    display.println("GPS: No Fix");
  }

  display.display();
  delay(1000);
}
