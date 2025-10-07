/* Receptor XBee PRO S3B (modo AT)
   RP2040
   Manuel Francisco Ibarra Ojeda
*/

#define XBEE_UART   Serial1
#define XBEE_BAUD   9600

// Pines del RP2040 que usaremos para el UART1
#define XBEE_RX_PIN 5   // GP5  -> RX del RP2040 (conectado al TX del XBee)
#define XBEE_TX_PIN 4   // GP4  -> TX del RP2040 (conectado al RX del XBee)

void setup() {
  Serial.begin(115200);          // USB CDC para monitor serie
  while (!Serial) { /* esperar al USB en placas con auto-reset */ }

  // Configurar UART1 con los pines deseados
  XBEE_UART.setRX(XBEE_RX_PIN);
  XBEE_UART.setTX(XBEE_TX_PIN);
  XBEE_UART.begin(XBEE_BAUD);

  Serial.println("XBee PRO S3B (AT) - Receptor RP2040 listo.");
}

void loop() {
  // Datos desde el XBee hacia el monitor serie
  while (XBEE_UART.available()) {
    Serial.write(XBEE_UART.read());
  }

  // (Opcional) Eco inverso: escribir desde el monitor serie al XBee
  while (Serial.available()) {
    XBEE_UART.write(Serial.read());
  }
}
