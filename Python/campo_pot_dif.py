import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import socket

# ========== CONFIGURACIÓN DE COMUNICACIÓN UDP ==========
UDP_IP = "192.168.137.110"  # IP del ESP32 (debe coincidir con la IP asignada)
UDP_PORT = 12345             # Puerto UDP donde escucha el ESP32
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Crea socket UDP

# ========== CONFIGURACIÓN DE MARCADORES ArUco ==========
MARKER_SIZE_METERS = 0.1     # Tamaño real del marcador en metros (10 cm)
MARKER_IDS = [0, 1, 2]       # IDs de los marcadores:
                             # 0 = marcador de referencia (origen del sistema)
                             # 1 = marcador en el robot
                             # 2 = marcador en el objetivo

# Configuración del detector ArUco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # Diccionario 4x4 con 50 marcadores
parameters = aruco.DetectorParameters()                         # Parámetros por defecto
detector = aruco.ArucoDetector(aruco_dict, parameters)          # Crea el detector

# ========== CONFIGURACIÓN DE CÁMARA ==========
cap = cv.VideoCapture(1, cv.CAP_DSHOW)  # Captura video de cámara índice 1

# Parámetros de calibración de la cámara (obtenidos previamente)
# [fx, fy, cx, cy] donde:
# fx, fy = distancia focal en píxeles (eje x, y)
# cx, cy = centro óptico en píxeles
camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]

# Matriz intrínseca de la cámara (3x3)
camera_matrix = np.array([[camera_params[0], 0, camera_params[2]],
                          [0, camera_params[1], camera_params[3]],
                          [0, 0, 1]], dtype=np.float32)

# Coeficientes de distorsión de la lente [k1, k2, p1, p2, k3]
dist_coeffs = np.array([0.0906, 0.3189, -0.0028, 0.00018, -2.998])

# ========== PARÁMETROS DEL ROBOT Y CONTROL ==========
kv = 8     # Ganancia de velocidad lineal (campos potenciales)
kw = 5     # Ganancia de velocidad angular (campos potenciales)
rp = 0.05  # Radio de proximidad en metros (5 cm) - distancia de parada
ap = 5     # Ángulo de proximidad en grados - tolerancia angular

# Parámetros físicos del robot diferencial
r_rueda = 0.03  # Radio de las ruedas en metros (3 cm)
l_eje = 0.12    # Distancia entre ruedas en metros (12 cm)

# ========== FUNCIONES AUXILIARES ==========

def obtener_matriz_homogenea(rvec, tvec):
    """
    Convierte vector de rotación y traslación a matriz homogénea 4x4.
    Matriz homogénea permite representar rotación y traslación juntas.
    """
    R, _ = cv.Rodrigues(rvec)  # Convierte vector de rotación a matriz 3x3
    T = np.eye(4)               # Crea matriz identidad 4x4
    T[:3, :3] = R               # Inserta matriz de rotación
    T[:3, 3] = tvec             # Inserta vector de traslación
    return T

def transformar_a_referencia_0(T_obj, T_ref):
    """
    Transforma la pose de un objeto al sistema de coordenadas del marcador 0.
    T_obj: transformación del objeto respecto a la cámara
    T_ref: transformación del marcador de referencia (0) respecto a la cámara
    Retorna: transformación del objeto respecto al marcador 0
    """
    T_0_cam = np.linalg.inv(T_ref)  # Invierte la transformación para obtener cámara→marcador 0
    return T_0_cam @ T_obj           # Multiplica matrices para cambiar de referencia

def mapear_velocidad(val):
    """
    Limita las velocidades al rango permitido por el motor [-255, 255].
    """
    return max(-255, min(255, int(val)))

def enviar_velocidades_udp(vl, vr):
    """
    Envía las velocidades de las ruedas por UDP al ESP32.
    vl: velocidad rueda izquierda
    vr: velocidad rueda derecha
    Formato: "vl,vr"
    """
    try:
        data = f"{vl:.2f},{vr:.2f}".encode()  # Formatea string y convierte a bytes
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))  # Envía por UDP
        print(f"Enviado -> vL: {vl:.2f}, vR: {vr:.2f}")
    except Exception as e:
        print(f"Error UDP: {e}") # Mensaje en caso de error con UDP

def calcular_campos_potenciales(xr, yr, a_r, xo, yo):
    """
    Calcula velocidades usando el método de campos potenciales artificiales.
    
    Parámetros:
    xr, yr: posición del robot
    a_r: orientación del robot (radianes)
    xo, yo: posición del objetivo
    
    Retorna:
    v: velocidad lineal
    w: velocidad angular
    """
    # ===== VELOCIDAD LINEAL =====
    d = np.sqrt((xo - xr)**2 + (yo - yr)**2)  # Distancia euclidiana al objetivo
    v = kv * d if d > rp else 0               # Velocidad proporcional a distancia
                                              # Si está cerca (d < rp), detener

    # ===== VELOCIDAD ANGULAR =====
    a_o = np.degrees(np.arctan2(yo - yr, xo - xr)) % 360  # Ángulo hacia el objetivo
    a_r_deg = np.degrees(a_r)                             # Ángulo actual del robot
    
    # Calcula diferencia angular normalizada a [-180, 180]
    da = a_o - a_r_deg
    if da > 180: da -= 360   # Ajusta si está fuera de rango
    if da < -180: da += 360
    
    # Velocidad angular proporcional al error angular
    w = kw * np.sin(np.radians(da)) if abs(da) >= ap else 0
    
    return v, w

def diferencial_inverse_kinematics(v, w, r, l):
    """
    Cinemática inversa del robot diferencial.
    Convierte velocidad lineal (v) y angular (w) del robot
    a velocidades individuales de cada rueda.
    
    v: velocidad lineal del centro del robot (m/s)
    w: velocidad angular del robot (rad/s)
    r: radio de las ruedas (m)
    l: distancia entre ruedas (m)
    
    Fórmulas derivadas de la cinemática diferencial:
    vr = (2v + wl) / (2r)  # velocidad rueda derecha
    vl = (2v - wl) / (2r)  # velocidad rueda izquierda
    """
    vr = (2*v + w*l) / (2*r)
    vl = (2*v - w*l) / (2*r)
    return vl, vr

# ========== BUCLE PRINCIPAL ==========
while True:
    # ===== CAPTURA DE FRAME =====
    ret, frame = cap.read()  # Lee frame de la cámara
    if not ret:
        break  # Sale si no puede leer

    # ===== DETECCIÓN DE MARCADORES ArUco =====
    corners, ids, _ = detector.detectMarkers(frame)  # Detecta todos los marcadores en el frame

    if ids is not None:  # Si se detectó al menos un marcador
        
        # ===== DIBUJAR MARCADORES DETECTADOS =====
        aruco.drawDetectedMarkers(frame, corners, ids)  # Dibuja contornos y IDs
        
        # ===== ESTIMACIÓN DE POSE DE CADA MARCADOR =====
        # Calcula posición 3D y orientación de cada marcador detectado
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,              # Esquinas detectadas
            MARKER_SIZE_METERS,   # Tamaño real del marcador
            camera_matrix,        # Parámetros intrínsecos
            dist_coeffs           # Coeficientes de distorsión
        )

        # ===== ALMACENAR POSES DE LOS MARCADORES =====
        poses = {}  # Diccionario para guardar poses por ID
        for i, id in enumerate(ids.flatten()):
            poses[id] = (rvecs[i][0], tvecs[i][0])  # Guarda rvec y tvec
            # Dibuja ejes 3D del marcador (X=rojo, Y=verde, Z=azul)
            cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i][0], tvecs[i][0], 0.05)

        # ===== PROCESAMIENTO SI ESTÁN LOS 3 MARCADORES =====
        if 0 in poses and 1 in poses and 2 in poses:
            
            # ===== OBTENER MATRICES HOMOGÉNEAS =====
            T_cam_0 = obtener_matriz_homogenea(*poses[0])  # Referencia → Cámara
            T_cam_1 = obtener_matriz_homogenea(*poses[1])  # Robot → Cámara
            T_cam_2 = obtener_matriz_homogenea(*poses[2])  # Objetivo → Cámara

            # ===== TRANSFORMAR A SISTEMA DE REFERENCIA 0 =====
            T_0_1 = transformar_a_referencia_0(T_cam_1, T_cam_0)  # Robot en ref 0
            T_0_2 = transformar_a_referencia_0(T_cam_2, T_cam_0)  # Objetivo en ref 0

            # ===== EXTRAER POSICIONES 2D =====
            xr, yr = T_0_1[0, 3], T_0_1[1, 3]  # Posición X,Y del robot
            xo, yo = T_0_2[0, 3], T_0_2[1, 3]  # Posición X,Y del objetivo
            
            # ===== EXTRAER ORIENTACIÓN DEL ROBOT =====
            # Calcula ángulo de orientación desde la matriz de rotación
            theta_r = np.arctan2(T_0_1[1, 0], T_0_1[0, 0])

            # ===== CALCULAR VELOCIDADES CON CAMPOS POTENCIALES =====
            v, w = calcular_campos_potenciales(xr, yr, theta_r, xo, yo)
            
            # ===== CINEMÁTICA INVERSA =====
            vl, vr = diferencial_inverse_kinematics(v, w, r_rueda, l_eje)
            
            # ===== ENVIAR COMANDOS AL ROBOT =====
            enviar_velocidades_udp(mapear_velocidad(vl), mapear_velocidad(vr))
            
        else:
            # Si faltan marcadores, detener el robot
            enviar_velocidades_udp(0, 0)
            print("⚠️ Faltan marcadores")
    else:
        # Si no hay detección, detener el robot
        enviar_velocidades_udp(0, 0)
        print("⚠️ Sin detección")

    # ===== VISUALIZACIÓN =====
    cv.imshow("Campos Potenciales - Ref ArUco 0", frame)
    
    # ===== SALIR CON TECLA 'q' =====
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# ===== LIMPIEZA =====
cap.release()           # Libera la cámara
cv.destroyAllWindows()  # Cierra ventanas
