#include <WiFi.h>
#include "time.h"
#include <AccelStepper.h>

// --- CONFIGURAZIONE WIFI ---
const char* ssid     = "IL_TUO_SSID";
const char* password = "LA_TUA_PASSWORD";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // Italia: UTC+1
const int   daylightOffset_sec = 3600; // Ora legale (+1h)

// --- PINOUT SCHEMA KICAD ---
#define MOTOR_H_IN1 1 
#define MOTOR_H_IN2 2 
#define MOTOR_H_IN3 3 
#define MOTOR_H_IN4 4 
#define MOTOR_M_IN1 5  
#define MOTOR_M_IN2 6  
#define MOTOR_M_IN3 7  
#define MOTOR_M_IN4 8  
#define HALL_H 9  
#define HALL_M 10 

// --- PARAMETRI MECCANICI ---
// Calcolati su: 2048 passi motore * Ratio
const float STEPS_PER_REV_H = 9011.2;  // Lancetta Ore
const float STEPS_PER_REV_M = 18432.0; // Lancetta Minuti

// Inizializzazione motori (Sequenza corretta per AccelStepper con ULN2003: 1, 3, 2, 4)
AccelStepper stepperH(AccelStepper::FULL4WIRE, MOTOR_H_IN1, MOTOR_H_IN3, MOTOR_H_IN2, MOTOR_H_IN4);
AccelStepper stepperM(AccelStepper::FULL4WIRE, MOTOR_M_IN1, MOTOR_M_IN3, MOTOR_M_IN2, MOTOR_M_IN4);

void setup() {
  Serial.begin(115200);
  
  pinMode(HALL_H, INPUT_PULLUP);
  pinMode(HALL_M, INPUT_PULLUP);

  stepperH.setMaxSpeed(800);
  stepperH.setAcceleration(400);
  stepperM.setMaxSpeed(800);
  stepperM.setAcceleration(400);

  // Connessione WiFi
  Serial.print("Connessione a "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  
  // Configura l'ora via NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Esegue l'Homing (allineamento magneti)
  Serial.println("\nHoming in corso...");
  doHoming();
}

void loop() {
  static unsigned long lastUpdate = 0;
  
  // Aggiorna la posizione target ogni secondo
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    updateClockPosition();
  }

  // Muove fisicamente i motori
  stepperH.run();
  stepperM.run();
}

void doHoming() {
  // Reset Minuti
  while (digitalRead(HALL_M) == HIGH) {
    stepperM.setSpeed(-400); 
    stepperM.runSpeed();
  }
  stepperM.setCurrentPosition(0);
  stepperM.moveTo(200); // Spostamento di sicurezza fuori dal raggio del sensore
  
  // Reset Ore
  while (digitalRead(HALL_H) == HIGH) {
    stepperH.setSpeed(-400);
    stepperH.runSpeed();
  }
  stepperH.setCurrentPosition(0);
  stepperH.moveTo(200);
  
  while(stepperH.distanceToGo() != 0 || stepperM.distanceToGo() != 0) {
    stepperH.run();
    stepperM.run();
  }
}

void updateClockPosition() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Errore recupero tempo");
    return;
  }

  // Calcolo posizione target assoluta
  // Ore: (ore % 12 + minuti/60) * (passi_totali / 12)
  float hourFraction = (timeinfo.tm_hour % 12) + (timeinfo.tm_min / 60.0);
  long targetH = (long)(hourFraction * (STEPS_PER_REV_H / 12.0));

  // Minuti: (minuti + secondi/60) * (passi_totali / 60)
  float minFraction = timeinfo.tm_min + (timeinfo.tm_sec / 60.0);
  long targetM = (long)(minFraction * (STEPS_PER_REV_M / 60.0));

  stepperH.moveTo(targetH);
  stepperM.moveTo(targetM);
  
  Serial.printf("Ora: %02d:%02d:%02d -> TargetH: %ld, TargetM: %ld\n", 
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, targetH, targetM);
}