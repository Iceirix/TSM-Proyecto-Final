// ========== DEFINICIÓN DE PINES ==========
// Se usan #define en lugar de variables para ahorrar memoria
// Estos son los mismos pines que en el código principal

#define ENA 5    // Pin PWM para controlar velocidad del Motor 1 (izquierdo)
#define IN1 18   // Pin de dirección del Motor 1 - Bit 1
#define IN2 19   // Pin de dirección del Motor 1 - Bit 2

#define ENB 25   // Pin PWM para controlar velocidad del Motor 2 (derecho)
#define IN3 33   // Pin de dirección del Motor 2 - Bit 1
#define IN4 32   // Pin de dirección del Motor 2 - Bit 2

/*
NOTA SOBRE EL CONTROL DE DIRECCIÓN:
Para cada motor, los pines IN funcionan así:
- IN1=HIGH, IN2=LOW  → Motor gira en un sentido (adelante)
- IN1=LOW,  IN2=HIGH → Motor gira en sentido contrario (atrás)
- IN1=LOW,  IN2=LOW  → Motor detenido (freno)
- IN1=HIGH, IN2=HIGH → NO recomendado (cortocircuito en el driver)
*/

void setup() {
    // ========== CONFIGURACIÓN DE PINES COMO SALIDAS ==========
    // Es fundamental configurar los pines antes de usarlos
    pinMode(ENA, OUTPUT);  // Habilita salida PWM para velocidad Motor 1
    pinMode(IN1, OUTPUT);  // Control de dirección Motor 1
    pinMode(IN2, OUTPUT);  // Control de dirección Motor 1
    
    pinMode(ENB, OUTPUT);  // Habilita salida PWM para velocidad Motor 2
    pinMode(IN3, OUTPUT);  // Control de dirección Motor 2
    pinMode(IN4, OUTPUT);  // Control de dirección Motor 2

    // ========== INICIALIZACIÓN DEL MONITOR SERIAL ==========
    Serial.begin(115200);  // Abre comunicación serial a 115200 baudios
                           // Permite ver mensajes de debug en el monitor serial
    Serial.println("Inicializando motores...");
}

void loop() {
    // Este bucle se repite infinitamente, probando cada motor en secuencia
    
    // ========================================
    // ===== PRUEBA MOTOR 1 (IZQUIERDO) =====
    // ========================================
    
    // ===== MOTOR 1: ADELANTE =====
    Serial.println("Motor 1: Adelante");
    digitalWrite(IN1, HIGH);    // IN1 activado
    digitalWrite(IN2, LOW);     // IN2 desactivado
                               // Esta combinación hace girar el motor adelante
    analogWrite(ENA, 255);     // Aplica velocidad máxima (255 = 100% PWM)
                               // PWM (Pulse Width Modulation) controla la velocidad
    delay(2000);               // Mantiene el motor girando 2 segundos (2000 ms)

    // ===== MOTOR 1: ATRÁS =====
    Serial.println("Motor 1: Atras");
    digitalWrite(IN1, LOW);     // IN1 desactivado
    digitalWrite(IN2, HIGH);    // IN2 activado
                               // Esta combinación invierte el sentido de giro
    // NOTA: No se vuelve a escribir ENA porque ya está en 255
    // El motor seguirá a la misma velocidad pero en reversa
    delay(2000);               // Mantiene el motor girando en reversa 2 segundos

    // ===== MOTOR 1: DETENIDO =====
    Serial.println("Motor 1: Detenido");
    digitalWrite(IN1, LOW);     // Ambos pines en LOW
    digitalWrite(IN2, LOW);     // Esto detiene el motor (freno)
    // NOTA: ENA sigue en 255, pero con ambos IN en LOW el motor no gira
    // Opcionalmente se podría poner analogWrite(ENA, 0) para desactivar completamente
    delay(1000);               // Pausa de 1 segundo antes de probar el otro motor

    // ========================================
    // ===== PRUEBA MOTOR 2 (DERECHO) =====
    // ========================================
    
    // ===== MOTOR 2: ADELANTE =====
    Serial.println("Motor 2: Adelante");
    digitalWrite(IN3, HIGH);    // IN3 activado
    digitalWrite(IN4, LOW);     // IN4 desactivado
                               // Motor 2 gira adelante
    analogWrite(ENB, 255);     // Velocidad máxima al Motor 2
    delay(2000);               // Mantiene girando 2 segundos

    // ===== MOTOR 2: ATRÁS =====
    Serial.println("Motor 2: Atras");
    digitalWrite(IN3, LOW);     // IN3 desactivado
    digitalWrite(IN4, HIGH);    // IN4 activado
                               // Motor 2 gira en reversa
    delay(2000);               // Mantiene girando en reversa 2 segundos

    // ===== MOTOR 2: DETENIDO =====
    Serial.println("Motor 2: Detenido");
    digitalWrite(IN3, LOW);     // Ambos pines en LOW
    digitalWrite(IN4, LOW);     // Detiene el Motor 2
    delay(1000);               // Pausa de 1 segundo

    // ========== FIN DEL CICLO ==========
    // El bucle loop() vuelve a empezar automáticamente
    // Se repite la secuencia completa: Motor1→Motor2→Motor1→Motor2...
}
