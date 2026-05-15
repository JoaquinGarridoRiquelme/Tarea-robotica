# Laboratorio 2 – Navegación reactiva con filtrado y fusión de sensores en Webots

**Asignatura:** ICI 4150 – Robótica y Sistemas Autónomos 2026-01  
**Laboratorio:** Nº 2  
**Integrantes / RUT:**
*   Benjamín Gómez Beltrán - [21.039.315-3]
*   Joaquín Garrido Riquelme - [20.882.540-2]
*   Pablo Aguilera Tapia - [21.712.853-6]
*   Cristian Mejias Pino - [18.654.182-0]
*   Cristobal Rubilar Vicencio - [21.711.065-3]
---

## Tabla de contenidos

1. [Objetivo](#objetivo)
2. [Robot y sensores utilizados](#robot-y-sensores-utilizados)
3. [Frecuencia de muestreo](#frecuencia-de-muestreo)
4. [Arquitectura del controlador](#arquitectura-del-controlador)
5. [Análisis de señales registradas](#análisis-de-señales-registradas)
6. [Estimación de avance con encoders](#estimación-de-avance-con-encoders)
7. [Filtro de media móvil](#filtro-de-media-móvil)
8. [Filtro de Kalman](#filtro-de-kalman)
9. [Lógica de navegación reactiva](#lógica-de-navegación-reactiva)
10. [Escenarios de prueba](#escenarios-de-prueba)
11. [Resultados y análisis](#resultados-y-análisis)
12. [Conclusiones](#conclusiones)
13. [Instrucciones de ejecución](#instrucciones-de-ejecución)

---

## Objetivo

Implementar un sistema de navegación reactiva en Webots para un robot móvil diferencial (e-puck), utilizando sensores de distancia y encoders de rueda. Se aplica un filtro de media móvil sobre las lecturas crudas y un filtro de Kalman para fusionar la información del movimiento (encoders) con la percepción del entorno (sensores frontales), mejorando la toma de decisiones ante obstáculos.

---

## Robot y sensores utilizados

### Robot: e-puck (Webots)

El robot **e-puck** es un robot diferencial con dos ruedas motrices independientes. Sus parámetros físicos relevantes son:

| Parámetro | Valor |
|-----------|-------|
| Radio de rueda (`r`) | 0.0205 m |
| Distancia entre ruedas | 0.052 m |
| Velocidad máxima de rueda | 6.28 rad/s |

### Sensores utilizados

El e-puck dispone de 8 sensores de proximidad infrarrojo (ps0–ps7) distribuidos alrededor de su cuerpo. En este laboratorio se utilizan los siguientes:

| Sensor Webots | Posición | Rol |
|--------------|----------|-----|
| `ps0` | Frontal derecho | Detección de obstáculos frontales |
| `ps7` | Frontal izquierdo | Detección de obstáculos frontales |
| `ps2` | Lateral izquierdo | Elección de dirección de giro |
| `ps5` | Lateral derecho | Elección de dirección de giro |
| `left wheel sensor` | Encoder izquierdo | Estimación de movimiento |
| `right wheel sensor` | Encoder derecho | Estimación de movimiento |

> Los sensores de distancia entregan valores en unidades arbitrarias (u.a.), donde valores altos indican mayor proximidad al obstáculo.

---

## Frecuencia de muestreo

El controlador se ejecuta en sincronía con el paso de simulación de Webots:

| Parámetro | Valor |
|-----------|-------|
| Paso de simulación (`basicTimeStep`) | 64 ms |
| Tiempo de muestreo `Ts` | **0.064 s** |
| Frecuencia de muestreo `fs = 1/Ts` | **≈ 15.625 Hz** |

Todas las señales (crudas, filtradas y estimadas) se registran con esta misma frecuencia y se exportan a `sensor_log.csv` al finalizar la simulación.

---

## Arquitectura del controlador

El controlador (`lab2_controller.py`) sigue el siguiente flujo en cada paso de simulación:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        PASO DE SIMULACIÓN (64 ms)                   │
│                                                                     │
│  1. Leer ps0, ps7, ps2, ps5 (distancia)                             │
│  2. Leer encoders izq / der (ángulo acumulado)                      │
│                                                                     │
│  3. front_raw = promedio(ps0, ps7)                                  │
│                                                                     │
│  4. front_filtered = MediaMóvil(front_raw, ventana=5)               │
│                                                                     │
│  5. Δθ = encoder_actual − encoder_anterior → Δs = r·Δθ (avance)    │
│     Δd_sensor = −Δs · SENSOR_SCALE   (modelo: avanzar reduce dist.) │
│                                                                     │
│  6. Kalman.predict(Δd_sensor) → d̂⁻_k                               │
│     Kalman.update(front_raw) → d̂_k   (distancia estimada)          │
│                                                                     │
│  7. Decisión:  d̂_k < umbral → girar   |   d̂_k ≥ umbral → avanzar  │
│                                                                     │
│  8. Enviar velocidades a motores                                     │
│  9. Registrar datos en CSV                                          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Análisis de señales registradas

Durante la simulación se almacenan las siguientes señales por cada muestra:

| Señal | Descripción |
|-------|-------------|
| `front_raw` | Promedio ps0 y ps7 sin procesar |
| `front_filtered` | Señal frontal suavizada (media móvil) |
| `dist_kalman` | Distancia frontal estimada por Kalman |
| `kalman_gain` | Ganancia de Kalman en cada iteración |
| `ps2_raw` | Sensor lateral izquierdo crudo |
| `ps5_raw` | Sensor lateral derecho crudo |
| `left_enc` / `right_enc` | Ángulo acumulado de encoders [rad] |
| `linear_adv_m` | Avance lineal por paso [m] |

### Características observadas en las señales

- **Señal cruda:** presenta variaciones abruptas por ruido del sensor infrarrojo, especialmente cuando el robot está en movimiento o el obstáculo tiene superficies irregulares.
- **Señal filtrada (MA):** muestra tendencia más suave pero con un retardo inherente proporcional al tamaño de la ventana (aquí 5 muestras ≈ 0.32 s de retardo).
- **Estimación Kalman:** combina ambas fuentes; converge más rápidamente que la MA ante cambios reales de distancia y es más robusta al ruido.

Los gráficos se generan con `plot_signals.py` y se guardan en `signal_analysis.png`.

---

## Estimación de avance con encoders

Los encoders del e-puck entregan el **ángulo acumulado** en radianes para cada rueda.

### Conversión a desplazamiento lineal

Entre dos instantes consecutivos $k-1$ y $k$:

$$\Delta\theta_L = \theta_L^{(k)} - \theta_L^{(k-1)}, \quad \Delta\theta_R = \theta_R^{(k)} - \theta_R^{(k-1)}$$

$$s_L = r \cdot \Delta\theta_L, \quad s_R = r \cdot \Delta\theta_R$$

$$\Delta s = \frac{s_L + s_R}{2}$$

donde $r = 0.0205$ m es el radio de la rueda y $\Delta s$ es el avance lineal del robot en ese paso.

### Conversión a unidades del sensor

Para integrar el avance en el modelo de Kalman, el desplazamiento $\Delta s$ [m] se convierte a las mismas unidades que el sensor de distancia mediante una escala empírica:

$$\Delta d_{sensor} = -\Delta s \times K_{escala}$$

El signo negativo refleja que avanzar hacia un obstáculo **reduce** la distancia. El valor de $K_{escala}$ se calibró experimentalmente.

---

## Filtro de media móvil

Se aplica un filtro de media móvil simple de ventana deslizante sobre las lecturas frontales:

$$y_k = \frac{1}{N} \sum_{i=0}^{N-1} x_{k-i}$$

| Parámetro | Valor |
|-----------|-------|
| Tamaño de ventana `N` | 5 muestras |
| Retardo introducido | ~2.5 muestras ≈ 0.16 s |

**Ventaja:** sencillo de implementar, elimina picos de ruido de alta frecuencia.  
**Limitación:** introduce retardo y no modela la incertidumbre de forma explícita.

---

## Filtro de Kalman

El filtro de Kalman 1D estima la distancia frontal $\hat{d}_k$ combinando la predicción por movimiento y la medición del sensor.

### Variables

| Símbolo | Descripción |
|---------|-------------|
| $\hat{d}_k$ | Estimación de distancia frontal en el instante $k$ |
| $\hat{d}_k^-$ | Predicción (antes de corrección) |
| $P_k^-$ | Covarianza de la predicción |
| $P_k$ | Covarianza actualizada |
| $z_k$ | Medición del sensor frontal |
| $Q$ | Varianza del proceso (modelo) |
| $R$ | Varianza del sensor (medición) |
| $K_k$ | Ganancia de Kalman |

### Etapa 1: Predicción

$$\hat{d}_k^- = \hat{d}_{k-1} + \Delta d_k$$

$$P_k^- = P_{k-1} + Q$$

Donde $\Delta d_k$ es la variación de distancia estimada a partir de los encoders. Si el robot avanza, la distancia al obstáculo frontal disminuye.

### Etapa 2: Corrección

$$K_k = \frac{P_k^-}{P_k^- + R}$$

$$\hat{d}_k = \hat{d}_k^- + K_k (z_k - \hat{d}_k^-)$$

$$P_k = (1 - K_k) P_k^-$$

### Parámetros utilizados

| Parámetro | Valor | Interpretación |
|-----------|-------|----------------|
| $Q$ | 0.1 | El modelo de movimiento es bastante preciso |
| $R$ | 15.0 | Los sensores IR presentan ruido notable |
| $P_0$ | 1.0 | Incertidumbre inicial moderada |

### Comportamiento de la ganancia $K_k$

- **$R$ grande (sensor ruidoso):** $K_k$ disminuye → el filtro confía más en el modelo de movimiento.
- **$P_k^-$ grande (modelo incierto):** $K_k$ aumenta → el filtro confía más en el sensor.
- La ganancia converge a un valor estacionario tras los primeros ciclos.

---

## Lógica de navegación reactiva

La toma de decisiones se basa exclusivamente en la **distancia estimada por Kalman** y en los **sensores laterales**:

```
IF dist_kalman < OBSTACLE_THRESHOLD (150 u.a.):
    IF ps2 (izq) > ps5 (der):
        → Girar a la derecha  (obstáculo más cercano por izquierda)
    ELSE:
        → Girar a la izquierda (obstáculo más cercano por derecha)
ELSE:
    → Avanzar
```

| Parámetro | Valor |
|-----------|-------|
| `OBSTACLE_THRESHOLD` | 150 u.a. |
| `FWD_SPEED` | 4.0 rad/s |
| `TURN_SPEED` | 3.0 rad/s |

### Comparación de estrategias de decisión

| Estrategia | Estabilidad | Giros innecesarios | Robustez |
|-----------|-------------|-------------------|---------|
| Señal cruda | Baja | Alta (reacciona al ruido) | Baja |
| Media móvil | Media | Media | Media |
| **Kalman (usado)** | **Alta** | **Baja** | **Alta** |

---

## Escenarios de prueba

### Escenario 1: Simple (`scenario1_simple.wbt`)

- Arena cuadrada de 2×2 m
- 3 obstáculos cúbicos de 15 cm colocados a distintas distancias
- Robot parte desde el centro mirando al frente
- **Objetivo:** verificar que el robot detecta obstáculos frontales y elige correctamente la dirección de giro

### Escenario 2: Complejo (`scenario2_complex.wbt`)

- Arena cuadrada de 3×3 m
- 2 pasillos estrechos formados por paredes paralelas
- 4 obstáculos adicionales dispersos
- Robot parte desde una esquina con orientación diagonal
- **Objetivo:** verificar la capacidad de navegar en espacios reducidos, evitar colisiones sucesivas y mantener dirección estable

---

## Resultados y análisis

### Señales registradas

Los gráficos generados por `plot_signals.py` muestran:

1. **Sensor frontal crudo vs media móvil:** La MA elimina picos espurios pero sigue el contorno general de la distancia real.
2. **Kalman vs crudo:** El estimador de Kalman produce una señal significativamente más estable, con menor amplitud de oscilación y sin retardo artificial.
3. **Ganancia de Kalman:** Converge rápidamente hacia un valor estacionario, indicando equilibrio entre confianza en el modelo y confianza en el sensor.
4. **Encoders:** Muestran incremento monótono durante avance y plateau durante giros.
5. **Sensores laterales:** Permiten identificar claramente de qué lado está el obstáculo al momento del giro.

### Comportamiento observado

| Métrica | Señal cruda | MA | Kalman |
|---------|------------|-----|--------|
| Giros innecesarios | Frecuentes | Moderados | Mínimos |
| Detección anticipada | No | No | Sí (modelo) |
| Estabilidad durante avance | Baja | Media | Alta |
| Colisiones | Ocasionales | Reducidas | Mínimas |

---

## Conclusiones

1. **El ruido del sensor es significativo.** Las lecturas crudas del infrarrojo del e-puck presentan variaciones de hasta ±20–30 u.a. en condiciones estáticas, lo que justifica el uso de técnicas de filtrado.

2. **La media móvil mejora la estabilidad pero introduce retardo.** Con una ventana de 5 muestras (~0.32 s), el filtro reduce el ruido a costa de responder más lentamente a cambios reales.

3. **El filtro de Kalman ofrece el mejor compromiso.** Al incorporar el modelo de movimiento (encoders), anticipa la variación de distancia y corrige con el sensor solo en la medida necesaria (ponderada por $K_k$). Esto reduce los giros innecesarios sin sacrificar capacidad de reacción.

4. **La fusión sensorial es clave para navegación robusta.** La combinación encoder + sensor infrarrojo, mediante el esquema de predicción-corrección de Kalman, produce una estimación coherente incluso cuando alguna de las fuentes es momentáneamente ruidosa.

5. **La lógica reactiva funciona correctamente en ambos escenarios.** En el escenario simple el robot evita los 3 obstáculos sin colisionar. En el escenario complejo navega por los pasillos ajustando la dirección según los sensores laterales.

---

## Instrucciones de ejecución

### Requisitos

- [Webots R2023b](https://cyberbotics.com/) o superior instalado
- Python 3.8+ (incluido en Webots)
- `matplotlib` para generación de gráficos (opcional)

```bash
pip install matplotlib
```

### Pasos

1. **Clonar el repositorio:**
   ```bash
   git clone <url-repositorio>
   cd lab2
   ```

2. **Abrir Webots y cargar el mundo:**
   - Ir a `File → Open World`
   - Seleccionar `worlds/scenario1_simple.wbt` o `worlds/scenario2_complex.wbt`

3. **Verificar que el controlador está asignado:**
   - En el árbol de nodos, seleccionar el nodo `E-puck`
   - Confirmar que `controller` apunta a `lab2_controller`

4. **Ejecutar la simulación:**
   - Presionar el botón ▶ en Webots
   - El robot comenzará a navegar reactivamente
   - Los logs aparecen en la consola de Webots

5. **Detener la simulación:**
   - Presionar ■; el archivo `sensor_log.csv` se genera automáticamente en `controllers/lab2_controller/`

6. **Generar gráficos de señales:**
   ```bash
   cd controllers/lab2_controller
   python plot_signals.py
   # → genera signal_analysis.png
   ```

### Estructura del repositorio

```
lab2/
├── controllers/
│   └── lab2_controller/
│       ├── lab2_controller.py   ← Controlador principal
│       ├── plot_signals.py      ← Generador de gráficos
│       └── sensor_log.csv       ← Generado al ejecutar la simulación
├── worlds/
│   ├── scenario1_simple.wbt    ← Escenario simple (3 obstáculos)
│   └── scenario2_complex.wbt   ← Escenario complejo (pasillos + obstáculos)
└── README.md                    ← Este archivo (informe del laboratorio)
```

### Parámetros configurables

Todos los parámetros clave están agrupados al inicio de `lab2_controller.py`:

```python
# Kalman
Q = 0.1    # varianza del proceso
R = 15.0   # varianza del sensor

# Navegación
OBSTACLE_THRESHOLD = 150   # umbral de distancia para girar
FWD_SPEED  = 4.0           # velocidad de avance [rad/s]
TURN_SPEED = 3.0           # velocidad de giro [rad/s]

# Filtro de media móvil
MA_WINDOW = 5              # tamaño de ventana
```
