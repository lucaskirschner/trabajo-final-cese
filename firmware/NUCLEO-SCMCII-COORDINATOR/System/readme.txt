===============================================================================
SYSTEM
===============================================================================

Esta carpeta contiene la arquitectura principal del firmware de la plataforma
embebida. Su organización se basa en una estructura modular por capas, con el
objetivo de facilitar la reutilización de código, el mantenimiento, la
escalabilidad y la adaptación del sistema a distintos escenarios de
automatización industrial.


===============================================================================
APP/
===============================================================================

Contiene la lógica de aplicación del sistema.

En esta capa se implementa el comportamiento funcional de cada tipo de estación
(central o remota), así como también la lógica personalizada desarrollada por
el usuario final. Aquí se integran los distintos servicios y drivers del sistema
para construir la funcionalidad completa de la aplicación.

Esta carpeta puede incluir:

- Aplicación de estación central.
- Aplicación de estación remota.
- Lógica de usuario.
- Máquinas de estados.
- Algoritmos de control.
- Procesamiento de datos.

La capa APP no debería acceder directamente al hardware, sino utilizar las APIs
provistas por las capas inferiores.


===============================================================================
BSP/  (Board Support Package)
===============================================================================

Contiene la capa de abstracción del hardware de la placa.

Su función es desacoplar el firmware de los detalles específicos del hardware
utilizado, encapsulando el acceso a periféricos, GPIOs y recursos físicos de la
plataforma.

Esta carpeta puede incluir:

- Configuración de GPIO.
- Manejo de LEDs.
- Lectura de botones.
- Control de señales de habilitación.
- Mapeo de pines.
- Inicialización básica de periféricos.

El objetivo es permitir la migración del firmware entre distintas revisiones de
hardware o placas minimizando modificaciones en las capas superiores.


===============================================================================
CONFIG/
===============================================================================

Contiene los archivos globales de configuración del sistema.

Aquí se centralizan parámetros de compilación, habilitación de módulos,
selección de funcionalidades y definiciones generales utilizadas por múltiples
capas del firmware.

Esta carpeta puede incluir:

- Configuración de estación central/remota.
- Selección de módulos habilitados.
- Parámetros del sistema.
- Definiciones globales.
- Configuración de hardware.
- Configuración de red y comunicación.

El objetivo es facilitar la parametrización del sistema desde un único punto.


===============================================================================
DRIVERS/
===============================================================================

Contiene los controladores de dispositivos y periféricos externos.

Esta capa encapsula la lógica de comunicación y control de los distintos
componentes conectados al microcontrolador, proporcionando APIs reutilizables e
independientes de la aplicación.

Esta carpeta puede incluir:

- Drivers SPI/I2C/UART.
- Drivers de módulos inalámbricos.
- Drivers Ethernet.
- Drivers de entradas y salidas industriales.
- Drivers de sensores.
- Drivers de RTC.
- Drivers de expansores externos.

La capa DRIVERS debe enfocarse únicamente en el manejo funcional del
dispositivo, evitando incluir lógica de aplicación.


===============================================================================
RTOS/
===============================================================================

Contiene la estructura relacionada con el sistema operativo en tiempo real.

Aquí se organiza la creación y administración de tareas, colas, semáforos,
mutexes, timers y demás recursos asociados al RTOS utilizado por el sistema.

Esta carpeta puede incluir:

- Definición de tareas.
- Inicialización del scheduler.
- Gestión de colas y eventos.
- Configuración de prioridades.
- Temporizadores de software.
- Hooks del RTOS.

El objetivo es mantener centralizada toda la infraestructura de concurrencia y
sincronización del sistema.


===============================================================================
SERVICES/
===============================================================================

Contiene módulos de servicios reutilizables de alto nivel.

Estos módulos implementan funcionalidades generales utilizadas por la aplicación
y los distintos subsistemas del firmware, actuando como intermediarios entre los
drivers y la lógica de aplicación.

Esta carpeta puede incluir:

- Gestión de comunicación.
- Gestión de eventos.
- Servicios de temporización.
- Watchdog lógico.
- Diagnóstico y monitoreo.
- Manejo de errores.
- Registro de eventos y logs.
- Gestión de estados del sistema.

La capa SERVICES permite desacoplar la lógica de aplicación de detalles
específicos de implementación y promueve una arquitectura más modular y
escalable.

===============================================================================