import socket
import struct
import pygame

# ========== CONFIGURACIÓN UDP ==========
UDP_IP = "192.168.137.110"   # IP del ESP32
UDP_PORT = 12345             # Puerto UDP del ESP32

# ========== PARÁMETROS DEL ROBOT DIFERENCIAL ==========
R = 0.05   # Radio de rueda en metros (5 cm)
L = 0.20   # Distancia entre ruedas en metros (20 cm)

# ========== INICIALIZACIÓN DEL JOYSTICK ==========
pygame.init()              # Inicializa todos los módulos de pygame
pygame.joystick.init()     # Inicializa el sistema de joysticks
joystick = pygame.joystick.Joystick(0)  # Conecta al primer joystick (índice 0)
joystick.init()            # Inicializa el joystick

# ========== CREAR SOCKET UDP ==========
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Socket para enviar datos

def enviar_velocidades(v, w):
    """
    Convierte velocidades del robot a velocidades de ruedas y las envía por UDP.
    
    v: velocidad lineal del robot (m/s)
    w: velocidad angular del robot (rad/s)
    """
    
    # ===== CINEMÁTICA INVERSA DIFERENCIAL =====
    # Convierte v (lineal) y w (angular) a velocidades de cada rueda
    v_r = (2 * v + w * L) / (2 * R)   # Velocidad rueda derecha
    v_l = (2 * v - w * L) / (2 * R)   # Velocidad rueda izquierda

    # ===== ESCALADO A VALORES PWM =====
    # Multiplica por 100 para amplificar (ajustar según necesidad)
    # Limita al rango [-255, 255] que espera el ESP32
    pwm_r = max(-255, min(255, int(v_r * 100)))
    pwm_l = max(-255, min(255, int(v_l * 100)))

    # ===== FORMATEO DE DATOS =====
    # Crea string en formato "pwm_der,pwm_izq"
    data = f"{pwm_r},{pwm_l}".encode()  # Convierte a bytes

    # ===== ENVÍO POR UDP =====
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(data, (UDP_IP, UDP_PORT))  # Envía el paquete
        print(f"Velocidades enviadas: Derecha = {pwm_r} m/s, Izquierda = {pwm_l} m/s")

# ========== BUCLE PRINCIPAL ==========
print("Control Xbox listo. Joystick izquierdo = avanzar/retroceder, derecho = girar")

running = True
while running:
    
    # ===== PROCESAR EVENTOS DE PYGAME =====
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # Si se cierra la ventana
            running = False
    
    # ===== LEER EJES DEL CONTROL XBOX =====
    # Los ejes van de -1.0 a 1.0
    # Eje 1: Stick izquierdo vertical (adelante/atrás)
    # Eje 3: Stick derecho horizontal (girar izq/der)
    vx = -joystick.get_axis(1)   # Negativo porque el eje está invertido
    w  =  joystick.get_axis(3)   # Velocidad angular (giro)

    # ===== ESCALADO DE VELOCIDADES =====
    # Multiplica por factores para limitar velocidades máximas
    v = vx * 0.2    # Velocidad lineal máxima: 0.2 m/s (ajustable)
    w = w * 1.0     # Velocidad angular máxima: 1 rad/s (ajustable)

    # ===== ENVIAR COMANDOS AL ROBOT =====
    enviar_velocidades(v, w)

    # ===== DELAY ENTRE LECTURAS =====
    pygame.time.wait(100)  # Espera 100ms (10 Hz de actualización)
