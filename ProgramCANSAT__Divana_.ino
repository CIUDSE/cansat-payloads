// ==== INCLUDES ====
#include <Wire.h>
#include <Adafruit_BNO08x.h>     // IMU
#include <MS5611.h>              // Sensor de presión y temperatura
#include <TinyGPS++.h>           // GPS
#include <SD.h>                  // Tarjeta SD

// ==== OBJETOS DE SENSORES ====
Adafruit_BNO08x bno08x;
MS5611 ms5611;
TinyGPSPlus gps;

// ==== DEFINICIÓN DE PINES ====
#define MQ135_PIN A0        // Pin analógico para sensor MQ135
#define XBEE_SERIAL Serial1 // Puerto serial para XBee
#define GPS_SERIAL Serial2  // Puerto serial para GPS
#define SD_CS 10            // Pin CS para la tarjeta SD (puede variar)

// ==== VARIABLES GLOBALES ====
File logfile; // Archivo para guardar datos en la SD

// ==== SETUP ====
void setup() {
  // Serial para depuración por USB (a tu PC)
  Serial.begin(115200);

  // Inicializar XBee (velocidad: 57600 baudios, debe coincidir con el otro XBee)
  XBEE_SERIAL.begin(57600);

  // Inicializar GPS (la mayoría funciona a 9600 baudios)
  GPS_SERIAL.begin(9600);

  // Inicializar I2C (usualmente Wire.begin() ya lo hace)
  Wire.begin();

  // Inicializar BNO08x (IMU)
  if (!bno08x.begin_I2C()) {
    Serial.println("ERROR: No se detectó el BNO08x!");
    while (1);
  } else {
    Serial.println("BNO08x inicializado correctamente.");
  }

  // Inicializar MS5611 (presión/temperatura)
  if (!ms5611.begin()) {
    Serial.println("ERROR: No se detectó el MS5611!");
    while (1);
  } else {
    Serial.println("MS5611 inicializado correctamente.");
  }

  // Inicializar tarjeta SD
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR: No se pudo inicializar la SD!");
  } else {
    logfile = SD.open("data.csv", FILE_WRITE);
    if (logfile) {
      // Escribir encabezados en el archivo CSV (opcional)
      logfile.println("temperature,pressure,gyroX,gyroY,gyroZ,accelX,accelY,accelZ,latitude,longitude,co2");
      logfile.flush();
    }
  }
}

// ==== LOOP PRINCIPAL ====
void loop() {
  // Leer MS5611 (presión y temperatura)
  ms5611.read();
  float temperature = ms5611.getTemperature();  // °C
  float pressure = ms5611.getPressure();        // mbar (hPa)

  // Leer BNO08x (acelerómetro y giroscopio)
  sensors_event_t accel, gyro, tempEvent;
  bno08x.getEvent(&accel, &gyro, &tempEvent);

  // Leer GPS
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }
  // Si la posición es válida, la usamos; si no, usamos 0.0
  float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  float lon = gps.location.isValid() ? gps.location.lng() : 0.0;

  // Leer sensor de CO2 (MQ135) como valor analógico (0–1023)
  int co2Raw = analogRead(MQ135_PIN);
  // Opcional: convertirlo a ppm si tienes la calibración

  // 5Formar cadena CSV para enviar y guardar
  String dataStr = String(temperature, 2) + "," + String(pressure, 2) + "," +
                   String(gyro.gyro.x, 2) + "," + String(gyro.gyro.y, 2) + "," + String(gyro.gyro.z, 2) + "," +
                   String(accel.acceleration.x, 2) + "," + String(accel.acceleration.y, 2) + "," + String(accel.acceleration.z, 2) + "," +
                   String(lat, 6) + "," + String(lon, 6) + "," +
                   String(co2Raw);

  // 6Enviar por XBee
  XBEE_SERIAL.println(dataStr);

  // También imprimir por USB para depuración
  Serial.println(dataStr);

  //  Guardar en la tarjeta SD
  if (logfile) {
    logfile.println(dataStr);
    logfile.flush(); // Asegura que se escriba de inmediato
  }

  // Esperar un tiempo antes de la siguiente lectura
  delay(200); // 200 ms = 5 veces por segundo
}

