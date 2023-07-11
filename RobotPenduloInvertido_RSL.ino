/*
Realizado por Ricardo Sandí
Se necesita:
Una baterìa LIpo 3s de 11 v
Un driver puente H 
Un arduino nano 33 iot
Dos motores de corriente continua

*/


#include <Arduino_LSM6DS3.h>

// Definición de pines del driver L298N
const int enableAPin = 3;   // Pin ENABLE A del driver L298N
const int enableBPin = 5;   // Pin ENABLE B del driver L298N
const int motorAPin1 = 2;   // Pin IN1 del driver L298N
const int motorAPin2 = 4;   // Pin IN2 del driver L298N
const int motorBPin1 = 7;   // Pin IN3 del driver L298N
const int motorBPin2 = 8;   // Pin IN4 del driver L298N

// Constantes de control
const float Kp = 0.8;
const float Ki = 0.1;
const float Kd = 1;


// Variables de control
float setpoint = -5.5;     // Punto de ajuste deseado (ángulo vertical)
float prevError = 0.0;    // Error anterior
float integral = 0.0;     // Término integral

void setup() {
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Error al iniciar el giroscopio y acelerómetro.");
    while (1);
  }

  // Configuración de los pines del driver L298N
  pinMode(enableAPin, OUTPUT);
  pinMode(enableBPin, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
}

void loop() {
  // Lectura del giroscopio
  float gyroX, gyroY, gyroZ;
  float accX, accY, accZ;
  IMU.readGyroscope(gyroX, gyroY, gyroZ);

  IMU.readAcceleration(accX, accY, accZ);
  
  // Cálculo del ángulo a partir del giroscopio
  float angle = gyroY;

  // Cálculo del error
  float error = setpoint - angle;

  // Término proporcional
  float P = Kp * error;

  // Término integral
  integral += Ki * error;

  // Término derivativo
  float derivative = Kd * (error - prevError);
  prevError = error;

  // Cálculo de la señal de control
  float controlSignal = P + integral + derivative;



  // Limitación de la señal de control
  if (controlSignal > 255) {
    controlSignal = 255;
  } else if (controlSignal < -255) {
    controlSignal = -255;
  }



  Serial.print("Angulo: (Lectura de GyroY) ");
  Serial.print(angle);
  Serial.print(", error (setpoint-angulo): ");
  Serial.print(error);
  Serial.print(", valor de P: (P * error)");
  Serial.print(P);
  Serial.print(", controlsignal: "); 
  Serial.print(controlSignal);
  Serial.print(", angulo: ");
  Serial.println(angle);
  //delay(1000);

  // Control de los motores
  if (controlSignal > 0) {
    // Motor A en sentido horario
    digitalWrite(motorAPin1, HIGH);
    digitalWrite(motorAPin2, LOW);
    analogWrite(enableAPin, controlSignal);
    // Motor B en sentido horario
    digitalWrite(motorBPin1, HIGH);
    digitalWrite(motorBPin2, LOW);
    analogWrite(enableBPin, controlSignal);
  } else {
    // Motor A en sentido antihorario
    digitalWrite(motorAPin1, LOW);
    digitalWrite(motorAPin2, HIGH);
    analogWrite(enableAPin, -controlSignal);
    // Motor B en sentido antihorario
    digitalWrite(motorBPin1, LOW);
    digitalWrite(motorBPin2, HIGH);
    analogWrite(enableBPin, -controlSignal);
  }
}
