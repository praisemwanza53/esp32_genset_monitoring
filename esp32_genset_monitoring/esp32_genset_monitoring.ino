#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// PTC sensor on Analog pin
const int ptcPin = 19;  // A19

// Proximity sensor E3F-DS30C4
const int proximityPin = 18;  // D18

// Relay control
const int relayPin = 21;  // D21

// GPS Setup on UART2
HardwareSerial GPSserial(2);  // UART2 (use GPIO16 RX, GPIO17 TX)
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  GPSserial.begin(9600, SERIAL_8N1, 16, 17);  // GPS: baudrate, config, RX, TX

  pinMode(ptcPin, INPUT);
  pinMode(proximityPin, INPUT);
  pinMode(relayPin, OUTPUT);

  digitalWrite(relayPin, LOW);  // Initially off

  Serial.println("System Initialized");
}

void loop() {
  // -------- Read PTC Sensor --------
  int ptcRaw = analogRead(ptcPin);
  float voltage = ptcRaw * (3.3 / 4095.0); // ESP32 ADC = 12-bit (0–4095)
  float temperature = (voltage - 0.5) * 100.0; // Adjust based on PTC characteristics

  // -------- Read Proximity Sensor --------
  bool objectDetected = digitalRead(proximityPin) == LOW; // Assuming active LOW

  // -------- Read GPS Data --------
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  // -------- Display Readings --------
  Serial.print("PTC Temp: ");
  Serial.print(temperature);
  Serial.print("°C | Proximity: ");
  Serial.print(objectDetected ? "Object Detected" : "No Object");

  if (gps.location.isUpdated()) {
    Serial.print(" | GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
  }

  Serial.println();

  // -------- Actuate Relay --------
  if (temperature > 40.0 || objectDetected) {
    digitalWrite(relayPin, HIGH);  // Turn ON relay
  } else {
    digitalWrite(relayPin, LOW);   // Turn OFF relay
  }

  delay(1000); // Update every second
}
