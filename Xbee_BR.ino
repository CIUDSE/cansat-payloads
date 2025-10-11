/* Receptor XBee PRO S3B (modo AT)
   RP2040
   Manuel Francisco Ibarra Ojeda
*/

#define XBEE_UART   Serial1
#define XBEE_BAUD   115200


void setup() {
  Serial.begin(115200);          // USB CDC para monitor serie
  while (!Serial) { /* esperar al USB en placas con auto-reset */ }

  // Configurar UART1 con los pines deseados
  XBEE_UART.begin(XBEE_BAUD);

  Serial.println("XBee PRO S3B (AT) - Receptor RP2040 listo.");
}

void loop() {
  // Datos desde el XBee hacia el monitor serie
  while (XBEE_UART.available()) {
    Serial.write(XBEE_UART.read());
  }

  }
}
