# Sistema de Control de Acceso Inteligente

## Descripción General

Este proyecto desarrolla un sistema de control de acceso inteligente que permite desbloquear una puerta (simulada por un LED externo) a través de diversas interfaces:

Teclado hexadecimal
Comandos desde internet con ESP01 (UART3)
Comandos desde PC host (UART2)
Botón físico (simulando apertura desde el interior)
Comandos especiales para depuración (#A# y #C#)

## Componentes de Hardware

- Placa Nucleo_L476RG (STM32L476RG)
- Pantalla OLED SSD1306
- Teclado hexadecimal
- LED externo (simula cerradura)
- LED LD2 integrado en la placa (heartbeat)
- Módulo ESP01 (WiFi)
- Botón integrado en la placa (para abrir desde el interior)
- Botón externo opcional (timbre)

## Diagrama de Conexiones

```
+-------------------+         +------------+
|                   |         |            |
|  NUCLEO_L476RG    |-------->| SSD1306    |
|                   |   I2C   | (OLED)     |
+-------------------+         +------------+
    |    |    |    |
    |    |    |    |          +------------+
    |    |    |    +--------->|            |
    |    |    |         UART2 | PC Host    |
    |    |    |               | (ST-Link)  |
    |    |    |               +------------+
    |    |    |
    |    |    |               +------------+
    |    |    +-----------+-->|            |
    |    |            GPIO|   | LED LD2    |
    |    |                |   | (Heartbeat)|
    |    |                |   +------------+
    |    |                |
    |    |                |   +------------+
    |    |                |   |            |
    |    |                +-->| LED Externo|
    |    |                |   | (Cerradura)|
    |    |                |   +------------+
    |    |                |
    |    |                |   +------------+
    |    |                |   |            |
    |    |                +-->| Botón B1   |
    |    |                    | (Interior) |
    |    |                    +------------+
    |    |
    |    |                    +------------+
    |    |                    |            |
    |    +------------------->| Teclado    |
    |                 GPIO    | Hexadecimal|
    |                         +------------+
    |
    |                         +------------+
    +------------------------>|            |
                       UART3  | ESP01      |
                              | (WIFI)     |
                              +------------+
```

## Estructura del Software

El proyecto se organiza siguiendo una arquitectura modular y utiliza las siguientes tecnologías:

- HAL (Hardware Abstraction Layer) de STM32
- Drivers para periféricos
- Controladores para componentes externos

### Módulos Implementados

1. **Gestión de Periféricos**
   - Inicialización y configuración de GPIO, UART, I2C
   - Manejo de interrupciones

2. **Drivers de Componentes**
   - Driver SSD1306 (pantalla OLED)
   - Driver de teclado hexadecimal
   - Control de LED (heartbeat y cerradura)

3. **Comunicación**
   - Implementación UART para comunicación con PC host
   - Integración ESP01 para conectividad WiFi
   - Ring buffer para manejo eficiente de datos

4. **Lógica de Aplicación**
   - Sistema de autenticación
   - Manejo de comandos
   - Rutinas de eventos

## Funcionalidades Implementadas

### 1. Autenticación
- Validación de contraseña ingresada por teclado
- Autenticación por comandos remotos

### 2. Indicadores
- LD2 como heartbeat (indicador de sistema en funcionamiento)
- LED externo como indicador de estado de la cerradura
- Pantalla OLED para mensajes de estado y configuración

### 3. Comunicación
- Protocolo UART para depuración y control desde PC
- Comandos a través de WiFi
- Comandos especiales de diagnóstico

### 4. Máquina de Estados para Control de Puerta
- **CERRADO**: Estado por defecto, puerta cerrada
- **ABIERTO_TEMPORAL**: Puerta abierta temporalmente (después de presionar botón una vez)
- **ABIERTO_PERMANENTE**: Puerta abierta permanentemente (después de presionar botón dos veces consecutivas o introducir clave correcta)
- **TRANSICIÓN**: La puerta regresa a estado CERRADO desde ABIERTO_PERMANENTE cuando se presiona el botón nuevamente

```
     +------------+
     |            |
     |   CERRADO  <-----------------+
     |            |                 |
     +-----+------+                 |
           |                        |
   Botón   |    Clave               |
   simple  |    Correcta            |
           v                        |
     +------------+                 |
     |  ABIERTO   |   Botón         |
     |PERMANENTE  |--------------->+|
     |            |                 |
     +-----+------+                 |
           |                        |
           | Doble                  |
           | presión                |
           v                        |
     +------------+                 |
     |  ABIERTO   |                 |
     | TEMPORAL   +------+          |
     |            |      |          |
     +------------+      |          |
                         | Timeout  |
                         +----------+
```

## Comandos del Sistema

| Comando | Descripción |
|---------|-------------|
| #*A*#   | Abrir cerradura (depuración) |
| #*C*#   | Cerrar cerradura (depuración) |
| Botón (1 presión) | Abrir permanentemente |
| Botón (2 presiones rápidas) | Abrir temporalmente |
| Botón (en estado abierto permanente) | Cerrar |

## Herramientas de Desarrollo

- VS Code IDE
- STM32CubeMX para configuración inicial
- YAT o Teraterm para COM o TCP/IP

## Recursos Adicionales

- Diseño de procesadores
- Programación en ASM (para entender el funcionamiento del procesador)
- Programación en C con acceso directo a registros (para entender el funcionamiento de los perifericos)
- Uso de HAL para abstraer hardware
- Implementación de drivers propios
- Manejo de periféricos (GPIO, UART, I2C)
- Integración de componentes externos
- Técnicas de bajo consumo energético

---
