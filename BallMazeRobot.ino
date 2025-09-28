#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>  // Librería para el control PID

// Definir pines de los LEDs
const int ledVerde = 6;
const int ledRojo = 7;
const int ledArduino = 13;  // LED integrado en el pin 13 del Arduino
const int trigPin = 23;  // Pin TRIG del ultrasonido
const int echoPin = 22;  // Pin ECHO del ultrasonido

// Inicialización del sensor ultrasónico
long duration;
int distance;

// Inicialización del sensor MPU6050 (giroscopio)
MPU6050 mpu;

// Variables para el giro
double setpoint = 90;  // Queremos que el robot gire 90 grados
double input, output;
double Kp = 1, Ki = 0, Kd = 0;  // Ajustar ganancias del PID para mayor estabilidad

// Crear objeto PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Variables para el filtrado de las lecturas del giroscopio
double gz_offset = 0;   // Offset de gz (calibración inicial)
long sum_gz = 0;  // Suma para el promedio de gz
int sampleCount = 100;  // Número de muestras para el promedio

// Estado del giro
bool isTurning = false;  // Indica si el robot está girando

void setup() {
  Serial.begin(115200);
  
  // Configuración de los pines de los LEDs
  pinMode(ledVerde, OUTPUT);
  pinMode(ledRojo, OUTPUT);
  pinMode(ledArduino, OUTPUT);  // Configurar el LED integrado como salida
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Iniciar comunicación I2C y MPU6050
  Wire.begin();
  mpu.initialize();
  
  // Verificar conexión con el MPU6050
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente.");
  } else {
    Serial.println("Error al conectar el MPU6050.");
    while(1); // Detener el código si no se conecta
  }
  
  // Calibración inicial del giroscopio para ajustar el offset
  calibrarGiroscopio();
  
  // Configuración del PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);  // Límite de salida del PID

  // Apagar los LEDs al inicio
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledRojo, LOW);
  digitalWrite(ledArduino, LOW);  // Apagar LED integrado
}

void loop() {
  // Leer la distancia del sensor ultrasónico
  distanciaUltrasonico();

  // Si la distancia es menor que 15 cm, decidir hacia dónde girar
  if (distance < 15 && !isTurning) {
    // Activar LED integrado para indicar que se detectó un obstáculo
    digitalWrite(ledArduino, HIGH);  // Enciende LED integrado
    
    // Decidir la dirección de giro (si es a la izquierda o derecha)
    digitalWrite(ledRojo, LOW);  // Apagar LED rojo al iniciar giro
    digitalWrite(ledVerde, HIGH);  // Enciende LED verde para girar a la derecha
    Serial.println("Obstáculo detectado. Girando a la derecha.");
    girarDerecha(90);  // Girar 90 grados a la derecha
    isTurning = true;  // Marcar que estamos girando
    digitalWrite(ledVerde, LOW);  // Apagar LED verde después de girar
    digitalWrite(ledArduino, LOW);  // Apagar LED integrado después de completar el giro
  } else {
    // Si no hay obstáculos, apaga los LEDs
    digitalWrite(ledRojo, LOW);  // Apagar LED rojo
    digitalWrite(ledVerde, LOW); // Apagar LED verde
    digitalWrite(ledArduino, LOW);  // Apagar LED integrado
    Serial.println("En línea recta");
  }

  delay(100);  // Espera 100 ms antes de hacer la siguiente lectura
}

void distanciaUltrasonico() {
  // Generar el pulso TRIG para el sensor ultrasónico
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Leer la duración del pulso ECHO
  duration = pulseIn(echoPin, HIGH);
  
  // Calcular la distancia en centímetros
  distance = duration * 0.0344 / 2;  // Fórmula para convertir el tiempo a distancia
}

void girarDerecha(int grados) {
  int16_t gx, gy, gz;
  double desired_angle = grados;  // Queremos girar los grados especificados

  // Mientras no hayamos girado la cantidad deseada
  while (abs(input) < desired_angle) {
    // Obtener las lecturas del giroscopio
    mpu.getRotation(&gx, &gy, &gz);
    
    // Restar el offset del giroscopio (calibración)
    gz -= gz_offset;  // Compensar el offset del giroscopio
    
    // Asignar el valor de gz a input
    input = gz;
    
    // Ejecutar el PID para obtener la salida
    myPID.Compute();
    
    // Si estamos por encima del valor, enciende el LED rojo (corrección a la izquierda)
    if (input > setpoint) {
      digitalWrite(ledRojo, HIGH);  // Enciende LED rojo
      digitalWrite(ledVerde, LOW);  // Apaga LED verde
    }
    // Si estamos por debajo del valor, enciende el LED verde (corrección a la derecha)
    else if (input < setpoint) {
      digitalWrite(ledVerde, HIGH);  // Enciende LED verde
      digitalWrite(ledRojo, LOW);    // Apaga LED rojo
    }

    delay(100);  // Espera 100 ms antes de hacer la siguiente lectura
  }

  // Detener el giro después de 90 grados
  digitalWrite(ledVerde, LOW);  // Apagar LED verde cuando termine de girar
}

void calibrarGiroscopio() {
  sum_gz = 0;  // Reiniciar la suma
  for (int i = 0; i < sampleCount; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum_gz += gz;
    delay(10);  // Espera breve entre lecturas
  }
  
  // Calcular el offset (promedio de gz)
  gz_offset = sum_gz / sampleCount;
  Serial.print("Calibración completada. Offset de gz: ");
  Serial.println(gz_offset);
}
