#include <Arduino.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// --- PIN DEFINITIONS ---
const int motorKiriMaju = 25;
const int motorKiriMundur = 26;
const int motorKananMaju = 27;
const int motorKananMundur = 14;
const int enableMotorKiri = 12;
const int enableMotorKanan = 13;

// PWM
const int ledcChannelKiri = 0;
const int ledcChannelKanan = 1;
const int freqPWM = 5000;
const int resolutionPWM = 8;

// Sensor
const int sensorIRKanan = 32;
const int sensorIRKiri = 33;
const int trigPin = 18;
const int echoPin = 19;

// Konstanta
const int GARIS_TERDETEKSI = LOW;
const int JARAK_AMAN_MUSUH = 20;
const int JARAK_MAKS_DETEKSI_MUSUH = 100;

long durasi;
int jarak = 0;

bool sudahCounter = false;
bool cariKeKanan = true;  // Arah pencarian musuh awal

// --- MOTOR FUNCTIONS ---
void maju(int kecepatan) {
  analogWrite(enableMotorKanan, kecepatan);
  analogWrite(enableMotorKiri, kecepatan);
  digitalWrite(motorKiriMaju, HIGH);
  digitalWrite(motorKiriMundur, LOW);
  digitalWrite(motorKananMaju, HIGH);
  digitalWrite(motorKananMundur, LOW);
}

void mundur(int kecepatan) {
  analogWrite(enableMotorKanan, kecepatan);
  analogWrite(enableMotorKiri, kecepatan);
  digitalWrite(motorKiriMaju, LOW);
  digitalWrite(motorKiriMundur, HIGH);
  digitalWrite(motorKananMaju, LOW);
  digitalWrite(motorKananMundur, HIGH);
}

void belokKiri(int kecepatan) {
  analogWrite(enableMotorKanan, kecepatan);
  analogWrite(enableMotorKiri, kecepatan);
  digitalWrite(motorKiriMaju, HIGH);
  digitalWrite(motorKiriMundur, LOW);
  digitalWrite(motorKananMaju, LOW);
  digitalWrite(motorKananMundur, HIGH);
}

void belokKanan(int kecepatan) {
  analogWrite(enableMotorKanan, kecepatan);
  analogWrite(enableMotorKiri, kecepatan);
  digitalWrite(motorKiriMaju, LOW);
  digitalWrite(motorKiriMundur, HIGH);
  digitalWrite(motorKananMaju, HIGH);
  digitalWrite(motorKananMundur, LOW);
}

void berhenti() {
  analogWrite(enableMotorKanan, 0);
  analogWrite(enableMotorKiri, 0);
  digitalWrite(motorKiriMaju, LOW);
  digitalWrite(motorKiriMundur, LOW);
  digitalWrite(motorKananMaju, LOW);
  digitalWrite(motorKananMundur, LOW);
}

void geser(int kecepatan1, int kecepatan2) {
  analogWrite(enableMotorKanan, kecepatan1);
  analogWrite(enableMotorKiri, kecepatan2);
  digitalWrite(motorKiriMaju, HIGH);
  digitalWrite(motorKiriMundur, LOW);
  digitalWrite(motorKananMaju, HIGH);
  digitalWrite(motorKananMundur, LOW);
}

// --- ULTRASONIC FUNCTION ---
int bacaJarakUltrasonik() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  durasi = pulseIn(echoPin, HIGH, 10000);
  if (durasi == 0) return -1;

  jarak = durasi * 0.034 / 2;
  Serial.println(jarak);
  SerialBT.println(jarak);
  return jarak;
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  SerialBT.begin("SUMO_BOT");
  Serial.println("Robot Sumo Siap!");
  SerialBT.println("Bluetooth aktif. Pasangkan dengan 'SUMO_BOT'");

  pinMode(motorKiriMaju, OUTPUT);
  pinMode(motorKiriMundur, OUTPUT);
  pinMode(motorKananMaju, OUTPUT);
  pinMode(motorKananMundur, OUTPUT);
  pinMode(enableMotorKiri, OUTPUT);
  pinMode(enableMotorKanan, OUTPUT);

  pinMode(sensorIRKanan, INPUT);
  pinMode(sensorIRKiri, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  delay(500);
}

// --- MAIN LOOP ---
void loop() {
  int nilaiIRKanan = digitalRead(sensorIRKanan);
  int nilaiIRKiri = digitalRead(sensorIRKiri);
  int jarakMusuh = bacaJarakUltrasonik();

  Serial.println(nilaiIRKanan);
  Serial.println(nilaiIRKiri);
  Serial.println(jarakMusuh);
  SerialBT.println(nilaiIRKanan);
  SerialBT.println(nilaiIRKiri);
  SerialBT.println(jarakMusuh);

  // Perintah Bluetooth manual
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    switch (cmd) {
      case 'f': maju(200); SerialBT.println("Maju!"); break;
      case 'b': mundur(200); SerialBT.println("Mundur!"); break;
      case 'l': belokKiri(200); SerialBT.println("Belok Kiri!"); break;
      case 'r': belokKanan(200); SerialBT.println("Belok Kanan!"); break;
      case 's': berhenti(); SerialBT.println("Stop!"); break;
    }
  }

  // Deteksi garis
  if (nilaiIRKanan == GARIS_TERDETEKSI || nilaiIRKiri == GARIS_TERDETEKSI) {
    Serial.println("Garis terdeteksi! Mundur dan belok.");
    SerialBT.println("Garis terdeteksi! Mundur dan belok.");
    
    // Mundur
    mundur(250);
    delay(200);

    // Belok berdasarkan sensor
    if (nilaiIRKanan == GARIS_TERDETEKSI && nilaiIRKiri == GARIS_TERDETEKSI) {
      belokKanan(180);
      delay(300);
    } else if (nilaiIRKanan == GARIS_TERDETEKSI) {
      belokKiri(170);
      delay(200);
    } else if (nilaiIRKiri == GARIS_TERDETEKSI) {
      belokKanan(170);
      delay(200);
    }

    berhenti();
    delay(50);

    // Ganti arah pencarian musuh
    cariKeKanan = !cariKeKanan;
    Serial.print("Arah pencarian musuh selanjutnya: ");
    Serial.println(cariKeKanan ? "Kanan" : "Kiri");
    SerialBT.print("Arah pencarian musuh selanjutnya: ");
    SerialBT.println(cariKeKanan ? "Kanan" : "Kiri");
  }

  // Musuh sangat dekat
  else if (jarakMusuh != -1 && jarakMusuh < JARAK_AMAN_MUSUH && jarakMusuh > 0) {
    Serial.println("Musuh sangat dekat! Serang!");
    SerialBT.println("Musuh sangat dekat! Serang!");
    belokKanan(200);
    delay(150);
    maju(200);
    delay(400);
  }

  // Musuh terdeteksi dalam jarak
  else if (jarakMusuh != -1 && jarakMusuh < JARAK_MAKS_DETEKSI_MUSUH) {
    if (!sudahCounter) {
      Serial.println("Musuh terdeteksi! Counter kiri sebentar.");
      SerialBT.println("Musuh terdeteksi! Counter kiri sebentar.");
      belokKiri(80);
      delay(50);
      sudahCounter = true;
    }

    Serial.println("Musuh terdeteksi! Maju!");
    SerialBT.println("Musuh terdeteksi! Maju!");
    maju(255);
  }

  // Tidak ada musuh atau garis
  else {
    Serial.println("Mencari musuh...");
    SerialBT.println("Mencari musuh...");

    if (cariKeKanan) {
      belokKanan(130);
      Serial.println("Putar ke kanan.");
      SerialBT.println("Putar ke kanan.");
    } else {
      belokKiri(130);
      Serial.println("Putar ke kiri.");
      SerialBT.println("Putar ke kiri.");
    }

    delay(80);
    sudahCounter = false;
  }

  delay(5);
}
