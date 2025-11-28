# 🤖 Navegación Autónoma de Robot Móvil Diferencial

## Proyecto Final – Robótica Móvil | Primavera 2025
**Universidad Nacional Autónoma de México (UNAM)**  
Facultad de Ingeniería – División de Ingeniería Mecánica e Industrial (DIMEI)  
Carrera: Ingeniería Mecatrónica

---

## 📋 Descripción del Proyecto

Este repositorio contiene la implementación completa de un sistema de **navegación autónoma para un robot móvil diferencial** que utiliza:

- **Marcadores ArUco** para localización y seguimiento
- **Visión global** mediante cámara cenital fija
- **Campos potenciales artificiales (método Villela)** para la generación de trayectorias reactivas
- **Comunicación WiFi/UDP** para control remoto en tiempo real

### 🎯 Objetivo Principal

Desarrollar un sistema donde un robot diferencial de dos ruedas se desplace de forma autónoma hacia una meta (estática o móvil) identificada mediante marcadores ArUco, todo procesado por una computadora central que envía comandos individuales a cada motor del robot.

---

## 🗂️ Estructura del Repositorio

```
📦 TSM-Proyecto-Final/
├── 📁 Arduino IDE/
│   ├── ESP32-DIF-RECIBE.ino      # Control principal con WiFi/UDP
│   └── ESP32-DIFER-PRUEBA.ino    # Código de prueba de motores
├── 📁 Python/
│   ├── campo_pot_dif.py          # Navegación autónoma con visión
│   └── dife_controlx.py          # Control manual con gamepad Xbox
└── 📄 README.md                   # Este archivo
```

---

## 📁 Contenido de las Carpetas

### 🔌 Arduino IDE/

Contiene los archivos `.ino` para programar el **ESP32** desde el IDE de Arduino:

#### **1. ESP32-DIF-RECIBE.ino**
- ✅ Programa principal para el robot
- ✅ Conexión WiFi y recepción de comandos UDP
- ✅ Control de dos motores DC mediante driver L298N
- ✅ Recibe velocidades en formato `"vel_izq,vel_der"`
- ✅ Convierte velocidades a señales PWM para los motores

#### **2. ESP32-DIFER-PRUEBA.ino**
- ✅ Código de diagnóstico y prueba
- ✅ Verifica funcionamiento individual de cada motor
- ✅ Secuencia automática: adelante → atrás → detener
- ✅ Útil para validar conexiones antes del ensamblaje completo

---

### 🐍 Python/

Scripts en Python para el procesamiento de visión, cálculo de trayectorias y comunicación con el robot:

#### **1. campo_pot_dif.py**
**Navegación autónoma con visión por computadora**

- ✅ Captura de video desde cámara cenital
- ✅ Detección en tiempo real de marcadores ArUco (IDs: 0, 1, 2)
  - **ID 0**: Marcador de referencia (origen del sistema de coordenadas)
  - **ID 1**: Marcador en el robot (permite estimar posición y orientación)
  - **ID 2**: Marcador en la meta (objetivo a alcanzar)
- ✅ Estimación de poses 3D usando parámetros de calibración de cámara
- ✅ Transformación de coordenadas al sistema de referencia del marcador 0
- ✅ Implementación del **método Villela de campos potenciales artificiales**:
  - Campo atractivo hacia la meta
  - Cálculo de velocidad lineal (`v`) proporcional a la distancia
  - Cálculo de velocidad angular (`w`) proporcional al error de orientación
- ✅ Cinemática inversa diferencial para convertir `(v, w)` → `(vel_izq, vel_der)`
- ✅ Envío continuo de comandos por UDP al ESP32

#### **2. dife_controlx.py**
**Control manual con gamepad Xbox**

- ✅ Interfaz con control Xbox usando PyGame
- ✅ Control intuitivo:
  - **Joystick izquierdo (vertical)**: Avanzar/retroceder
  - **Joystick derecho (horizontal)**: Girar izquierda/derecha
- ✅ Conversión de entradas del joystick a velocidades del robot
- ✅ Cinemática inversa diferencial
- ✅ Envío de comandos por UDP a 10 Hz

---

## 🔧 Configuración del Hardware

### Componentes Necesarios

| Componente | Descripción |
|------------|-------------|
| **ESP32** | Microcontrolador con WiFi integrado |
| **Driver L298N** | Controlador de motores DC de doble puente H |
| **2× Motores DC** | Motores con reductora para las ruedas |
| **Fuente de alimentación** | 7-12V para motores (según especificaciones) |
| **Cámara USB** | Para visión global (montada en el techo) |
| **Marcadores ArUco** | 3 marcadores impresos del diccionario DICT_4X4_50 |

### Conexiones ESP32 ↔ L298N

```
ESP32          L298N
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Pin 18   →     IN1  (Motor Izquierdo)
Pin 19   →     IN2  (Motor Izquierdo)
Pin 5    →     ENA  (PWM Izquierdo)

Pin 33   →     IN3  (Motor Derecho)
Pin 32   →     IN4  (Motor Derecho)
Pin 25   →     ENB  (PWM Derecho)

GND      →     GND
```

---

## ⚙️ Configuración del Software

### 🔹 Para ESP32 (Arduino IDE)

#### Requisitos Previos
1. **IDE de Arduino** instalado (versión 1.8.x o 2.x)
2. **Core ESP32 de Espressif Systems** instalado

#### ⚠️ Versión Crítica
```
ESP32 by Espressif Systems – Versión 2.0.11
```
**IMPORTANTE**: Usar otra versión puede causar errores de compilación o comportamiento inesperado.

#### Instalación del Core ESP32
1. Abre Arduino IDE
2. Ve a `Archivo` → `Preferencias`
3. En "Gestor de URLs Adicionales de Tarjetas", agrega:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Ve a `Herramientas` → `Placa` → `Gestor de tarjetas`
5. Busca "ESP32" e instala la **versión 2.0.11**

#### Librerías Necesarias
Las siguientes librerías vienen incluidas con el core ESP32:
```cpp
#include <WiFi.h>      // Para conectividad WiFi
#include <WiFiUdp.h>   // Para comunicación UDP
```

#### Configuración WiFi
Antes de cargar el código, modifica estas líneas en `ESP32-DIF-RECIBE.ino`:
```cpp
const char* ssid = "MiRedWifi";      // ← Nombre de tu red WiFi
const char* password = "123456789";  // ← Contraseña de tu red
```

#### Carga del Código
1. Conecta el ESP32 por USB
2. Selecciona la placa: `Herramientas` → `Placa` → `ESP32 Dev Module`
3. Selecciona el puerto COM correcto
4. Haz clic en "Subir"
5. Abre el Monitor Serial (115200 baudios) para ver la IP asignada

---

### 🔹 Para Python

#### Requisitos del Sistema
- **Python 3.8 o superior**
- **Sistema operativo**: Windows, Linux o macOS
- **Editor recomendado**: Visual Studio Code (opcional)

#### 🌟 Entorno Virtual (Recomendado)
Usar un entorno virtual ayuda a mantener las dependencias organizadas:

```bash
# Crear entorno virtual
python -m venv venv

# Activar entorno virtual
# En Windows:
venv\Scripts\activate
# En Linux/macOS:
source venv/bin/activate
```

#### 📦 Instalación de Dependencias

**⚠️ IMPORTANTE**: Instalar estas versiones específicas para evitar problemas de compatibilidad:

```bash
pip install numpy==2.2.1
pip install opencv-contrib-python==4.10.0.84
pip install pygame==2.6.1
```

#### Verificación de Instalación
```bash
python -c "import cv2, numpy, pygame; print('✅ Librerías instaladas correctamente')"
```

#### Configuración de IP del ESP32
En ambos archivos Python (`campo_pot_dif.py` y `dife_controlx.py`), actualiza la IP del ESP32:

```python
UDP_IP = "192.168.137.110"  # ← Cambia esto a la IP que mostró el ESP32
UDP_PORT = 12345
```

---

## 🚀 Instrucciones de Uso

### 📝 Prueba Inicial de Hardware

**Antes de todo**, verifica que los motores funcionen correctamente:

1. Carga `ESP32-DIFER-PRUEBA.ino` en el ESP32
2. Abre el Monitor Serial (115200 baudios)
3. Observa la secuencia de prueba:
   - Motor 1 adelante (2 seg)
   - Motor 1 atrás (2 seg)
   - Motor 1 detenido (1 seg)
   - Motor 2 adelante (2 seg)
   - Motor 2 atrás (2 seg)
   - Motor 2 detenido (1 seg)
4. ✅ Si algún motor gira al revés, intercambia los cables IN1/IN2 o IN3/IN4

---

### 🎮 Modo 1: Control Manual con Xbox

1. Carga `ESP32-DIF-RECIBE.ino` en el ESP32
2. Conecta el gamepad Xbox a la computadora
3. Ejecuta el script de Python:
   ```bash
   python Python/dife_controlx.py
   ```
4. Controla el robot:
   - **Stick izquierdo (arriba/abajo)**: Avanzar/retroceder
   - **Stick derecho (izq/der)**: Girar

---

### 🤖 Modo 2: Navegación Autónoma con Visión

#### Preparación del Entorno

1. **Instala la cámara** en posición cenital (mirando hacia abajo)
2. **Prepara los marcadores ArUco**:
   - Genera 3 marcadores del diccionario `DICT_4X4_50` con IDs 0, 1, 2
   - Imprime en tamaño de **10 cm × 10 cm**
   - Monta sobre superficies planas y rígidas

3. **Coloca los marcadores**:
   - **ID 0**: Fijo en una esquina (origen de coordenadas)
   - **ID 1**: En el robot (orientado hacia el frente del robot)
   - **ID 2**: En la meta (paleta o posición fija)

#### Calibración de Cámara

Si es necesario, calibra tu cámara y actualiza estos parámetros en `campo_pot_dif.py`:

```python
camera_params = [fx, fy, cx, cy]  # Parámetros intrínsecos
dist_coeffs = np.array([k1, k2, p1, p2, k3])  # Coeficientes de distorsión
```

#### Ejecución

1. Carga `ESP32-DIF-RECIBE.ino` en el ESP32
2. Verifica que los 3 marcadores sean visibles en la cámara
3. Ejecuta el script:
   ```bash
   python Python/campo_pot_dif.py
   ```
4. Observa:
   - Ventana con visualización en tiempo real
   - Ejes 3D dibujados sobre cada marcador detectado
   - El robot se desplaza automáticamente hacia la meta

5. **Para detener**: Presiona la tecla `Q`

---

## 🧮 Fundamentos Técnicos

### Cinemática del Robot Diferencial

El robot utiliza un modelo diferencial de dos ruedas. Las ecuaciones de cinemática inversa son:

```
v_derecha = (2v + wL) / (2r)
v_izquierda = (2v - wL) / (2r)
```

Donde:
- `v`: Velocidad lineal del centro del robot (m/s)
- `w`: Velocidad angular del robot (rad/s)
- `L`: Distancia entre ruedas (m)
- `r`: Radio de las ruedas (m)

### Método Villela de Campos Potenciales

El algoritmo genera un campo atractivo hacia la meta:

1. **Velocidad lineal**:
   ```
   v = kv × d    si d > rp
   v = 0         si d ≤ rp
   ```
   - `d`: Distancia euclidiana al objetivo
   - `kv`: Ganancia de velocidad lineal
   - `rp`: Radio de proximidad (zona de parada)

2. **Velocidad angular**:
   ```
   w = kw × sin(Δθ)    si |Δθ| ≥ ap
   w = 0               si |Δθ| < ap
   ```
   - `Δθ`: Error angular (diferencia entre orientación actual y deseada)
   - `kw`: Ganancia de velocidad angular
   - `ap`: Ángulo de proximidad (tolerancia angular)

### Transformaciones de Coordenadas

El sistema utiliza matrices homogéneas 4×4 para transformar las posiciones del robot y la meta desde el sistema de coordenadas de la cámara al sistema de referencia del marcador ID 0:

```
T_0_robot = T_0_cam × T_cam_robot
T_0_meta = T_0_cam × T_cam_meta
```

---

## 🎯 Escenarios de Prueba

Según los requisitos del proyecto, se deben validar tres escenarios:

### 1️⃣ Meta Fija
- Coloca el marcador ID 2 en una posición estática
- El robot debe navegar directamente hacia la meta y detenerse al llegar

### 2️⃣ Meta Móvil Lenta
- Mueve manualmente la paleta con el marcador ID 2 a velocidad baja
- El robot debe seguir la meta en movimiento

### 3️⃣ Meta Móvil Rápida
- Mueve la meta a mayor velocidad o con trayectorias impredecibles
- Observa la capacidad de respuesta del sistema

---

## 📊 Parámetros Ajustables

### En `campo_pot_dif.py`:

```python
# Control de velocidades
kv = 8          # Ganancia velocidad lineal (↑ = más agresivo)
kw = 5          # Ganancia velocidad angular (↑ = giros más rápidos)
rp = 0.05       # Radio de parada en metros (5 cm)
ap = 5          # Tolerancia angular en grados

# Parámetros físicos del robot
r_rueda = 0.03  # Radio de rueda (3 cm)
l_eje = 0.12    # Distancia entre ruedas (12 cm)

# Tamaño de marcadores
MARKER_SIZE_METERS = 0.1  # 10 cm
```

### En `dife_controlx.py`:

```python
# Escalado de velocidades del gamepad
v = vx * 0.2    # Velocidad lineal máxima: 0.2 m/s
w = w * 1.0     # Velocidad angular máxima: 1 rad/s
```

---

## 🐛 Solución de Problemas Comunes

### ❌ El ESP32 no se conecta a WiFi

**Solución**:
- Verifica que el SSID y contraseña sean correctos
- Asegúrate de estar en una red de 2.4 GHz (ESP32 no soporta 5 GHz)
- Verifica que el router no tenga filtrado MAC activo

### ❌ Los motores no responden

**Solución**:
- Ejecuta `ESP32-DIFER-PRUEBA.ino` para verificar conexiones
- Verifica que la fuente de alimentación tenga suficiente corriente
- Comprueba que los cables estén conectados a los pines correctos
- Verifica que el L298N esté recibiendo alimentación externa

### ❌ No se detectan los marcadores ArUco

**Solución**:
- Asegúrate de que la iluminación sea uniforme y suficiente
- Verifica que los marcadores no estén doblados o dañados
- Aumenta el contraste imprimiendo en papel de buena calidad
- Ajusta la altura de la cámara para capturar toda el área
- Verifica que estés usando el diccionario correcto: `DICT_4X4_50`

### ❌ El robot se mueve muy rápido o muy lento

**Solución**:
- Ajusta las ganancias `kv` y `kw` en `campo_pot_dif.py`
- Modifica el factor de escalado en la cinemática inversa
- Verifica que `maxSpeed` en el ESP32 coincida con los valores enviados

### ❌ Error de versión de librerías Python

**Solución**:
```bash
pip uninstall opencv-contrib-python numpy pygame
pip install numpy==2.2.1
pip install opencv-contrib-python==4.10.0.84
pip install pygame==2.6.1
```

---

## 📚 Referencias y Recursos

### Documentación Oficial
- [Documentación ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [PyGame Documentation](https://www.pygame.org/docs/)

### Material del Curso
- Apuntes de clase de Robótica Móvil
- Presentaciones del Dr. Víctor Javier González Villela

---

## 📋 Entregables del Proyecto

Según los requisitos del proyecto, debes entregar:

### 📹 Video Demostrativo (Máx. 3 minutos)
- ✅ Escenario 1: Meta fija
- ✅ Escenario 2: Meta móvil lenta
- ✅ Escenario 3: Meta móvil rápida
- ✅ Explicaciones claras de cada escenario

### 💻 Código Fuente
- ✅ Código comentado (este repositorio)
- ✅ README completo (este archivo)
- ✅ Instrucciones de instalación y uso

### 📄 Reporte Técnico (PDF)
- ✅ Título y objetivo
- ✅ Descripción del sistema
- ✅ Metodología
- ✅ Resultados experimentales
- ✅ Análisis de desempeño
- ✅ Conclusiones y reflexiones

### 🎤 Presentación Oral (10-15 minutos)
- ✅ Explicación del sistema
- ✅ Retos enfrentados
- ✅ Resultados obtenidos
- ✅ Reflexiones finales

---

## 👥 Autor

**Proyecto desarrollado para el curso de Robótica Móvil**  
Profesor: Dr. Víctor Javier González Villela  
Facultad de Ingeniería, UNAM  
Semestre: Primavera 2025

---

## 📝 Licencia

© Derechos Reservados  
Este proyecto ha sido desarrollado con fines académicos para la asignatura de Robótica Móvil.  
División de Ingeniería Mecánica e Industrial (DIMEI)  
Universidad Nacional Autónoma de México

---

## 🤝 Contribuciones

Si encuentras algún error o deseas sugerir mejoras:

1. Abre un **Issue** describiendo el problema
2. Proporciona detalles específicos (mensajes de error, comportamiento esperado vs. observado)
3. Si es posible, adjunta capturas de pantalla o logs

---

## 📞 Soporte

Para dudas específicas del proyecto, consulta:
- Material del curso en la plataforma oficial
- Horarios de asesoría del profesor
- Foros de discusión de la clase

---

**¡Buena suerte con tu proyecto! 🚀🤖**
