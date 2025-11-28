#include <WiFi.h>
#include <WiFiUdp.h>

// ===== CONFIGURACIÓN DE RED =====
// Estas credenciales deben coincidir con tu red WiFi
const char* ssid = "MiRedWifi";         // Nombre de la red WiFi
const char* password = "123456789";     // Contraseña de la red

// ===== CONFIGURACIÓN DE COMUNICACIÓN UDP =====
WiFiUDP Udp;                            // Objeto para manejar comunicación UDP
const unsigned int localUdpPort = 12345; // Puerto donde el ESP32 escuchará mensajes
char incomingPacket[255];               // Buffer (almacenamiento temporal) para datos recibidos
String data = "";                       // Variable para procesar los datos como texto

// ===== PINES DE CONTROL DE MOTORES =====
// Configuración típica para driver L298N
// IN1, IN2 controlan dirección del motor izquierdo
// IN3, IN4 controlan dirección del motor derecho
// ENA, ENB controlan la velocidad (PWM) de cada motor
const int IN1 = 18;  // Control dirección motor izquierdo (bit 1)
const int IN2 = 19;  // Control dirección motor izquierdo (bit 2)
const int IN3 = 32;  // Control dirección motor derecho (bit 1)
const int IN4 = 33;  // Control dirección motor derecho (bit 2)
const int ENA = 5;   // PWM motor izquierdo (velocidad)
const int ENB = 25;  // PWM motor derecho (velocidad)

// ===== PARÁMETROS DE VELOCIDAD =====
const int PWM_MAX = 255;        // Valor máximo de PWM (8 bits: 0-255)
const float maxSpeed = 100.0;   // Velocidad máxima esperada en los mensajes UDP

void setup() {
  // ===== INICIALIZACIÓN DEL MONITOR SERIAL =====
  Serial.begin(115200);  // Velocidad de comunicación serial para debug

  // ===== CONFIGURACIÓN DE PINES COMO SALIDAS =====
  pinMode(IN1, OUTPUT);  // Los pines de control deben ser salidas
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // ===== CONEXIÓN A WiFi =====
  WiFi.begin(ssid, password);           // Inicia conexión WiFi
  Serial.print("Conectando a WiFi...");
  
  // Bucle de espera hasta estar conectado
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);                         // Espera medio segundo
    Serial.print(".");                  // Imprime punto de progreso
  }
  
  Serial.println("\n✅ Conectado a WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());       // Muestra la IP asignada al ESP32

  // ===== INICIO DEL SERVIDOR UDP =====
  Udp.begin(localUdpPort);              // ESP32 empieza a escuchar en el puerto UDP
  Serial.print("Esperando mensajes en el puerto UDP: ");
  Serial.println(localUdpPort);
}

void loop() {
  // ===== VERIFICAR SI HAY DATOS UDP ENTRANTES =====
  int packetSize = Udp.parsePacket();   // Regresa el tamaño del paquete si hay datos
  
  if (packetSize) {  // Si se recibió un paquete
    
    // ===== LEER EL PAQUETE UDP =====
    int len = Udp.read(incomingPacket, 255);  // Lee hasta 255 bytes
    if (len > 0) {
      incomingPacket[len] = 0;  // Agrega terminador de cadena (null terminator)
    }

    // ===== MOSTRAR DATOS RECIBIDOS =====
    Serial.print("Mensaje UDP recibido: ");
    Serial.println(incomingPacket);

    // ===== PARSEO DE DATOS =====
    // Se espera formato: "velocidad1,velocidad2"
    int sepIndex = String(incomingPacket).indexOf(',');  // Busca la coma separadora
    
    if (sepIndex > 0) {  // Si encontró la coma
      // Extrae las dos velocidades del string
      float vel1 = String(incomingPacket).substring(0, sepIndex).toFloat();      // Antes de la coma
      float vel2 = String(incomingPacket).substring(sepIndex + 1).toFloat();     // Después de la coma
      
      // ===== CONTROL DE MOTORES =====
      moverMotor(vel1, IN1, IN2, ENA);    // Motor izquierdo
      moverMotor(-vel2, IN3, IN4, ENB);   // Motor derecho (en dirección opuesta al primero)
    }
  }
}

// ===== FUNCIÓN PARA CONTROLAR UN MOTOR =====
// vel: velocidad deseada (puede ser positiva o negativa)
// in1, in2: pines de control de dirección
// pwmPin: pin para controlar velocidad (PWM)
void moverMotor(float vel, int in1, int in2, int pwmPin) {
  
  // ===== CALCULAR VALOR PWM =====
  // Convierte la velocidad recibida a un valor PWM de 0-255
  // abs(vel) toma el valor absoluto
  // Se divide entre maxSpeed para normalizar
  // Se multiplica por PWM_MAX para escalar a 0-255
  // min() asegura que nunca exceda PWM_MAX
  int pwm = min((int)(abs(vel) / maxSpeed * PWM_MAX), PWM_MAX);

  // ===== CONTROL DE DIRECCIÓN =====
  if (vel > 0) {  // Velocidad positiva = avanzar
    digitalWrite(in1, HIGH);  // IN1 activo
    digitalWrite(in2, LOW);   // IN2 inactivo
    
  } else if (vel < 0) {  // Velocidad negativa = retroceder
    digitalWrite(in1, LOW);   // IN1 inactivo
    digitalWrite(in2, HIGH);  // IN2 activo
    
  } else {  // Velocidad cero = detener
    digitalWrite(in1, LOW);   // Ambos inactivos
    digitalWrite(in2, LOW);
  }

  // ===== APLICAR VELOCIDAD =====
  analogWrite(pwmPin, pwm);  // Envía señal PWM al pin de velocidad
}
