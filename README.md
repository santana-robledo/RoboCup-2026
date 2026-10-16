# RoboCup 2026 | Robot de Fútbol con Visión Omnidireccional

## Descripción

Este repositorio documenta el desarrollo de un robot móvil omnidireccional diseñado para participar en competencias de fútbol robótico, como parte del proyecto RoboCup 2026.

El proyecto integra diseño electrónico, sistemas embebidos, control de movimiento y visión artificial mediante una cámara omnidireccional, con el propósito de desarrollar una plataforma robótica capaz de percibir su entorno y desplazarse en una cancha de fútbol.

El sistema combina hardware y software para integrar la percepción visual, el procesamiento de información y el control del robot.

## Objetivo del proyecto

Diseñar e implementar un robot omnidireccional para fútbol robótico, integrando los componentes electrónicos, computacionales y mecánicos necesarios para el movimiento y la percepción del entorno.

Los objetivos principales son:

- Implementar el desplazamiento omnidireccional del robot.
- Integrar un sistema de visión omnidireccional para observar el entorno de juego.
- Desarrollar la comunicación entre los sistemas de procesamiento y control.
- Integrar los componentes electrónicos y el hardware del robot.
- Preparar la plataforma para su participación en competencias de fútbol robótico.

## Arquitectura del sistema

El proyecto se organiza en diferentes componentes que trabajan de manera conjunta.

### 1. Sistema de control embebido

**[ESP32](./ESP32/)**

Contiene los archivos relacionados con el sistema embebido del robot, incluyendo el control de hardware y la implementación de funciones de bajo nivel.

El ESP32 permite gestionar tareas de control e interacción con los componentes electrónicos, de acuerdo con la implementación del sistema.

### 2. Procesamiento y visión artificial

**[Raspberry Pi](./RaspBerry%20Pi/)**

Contiene los archivos relacionados con la plataforma Raspberry Pi, utilizada como parte del sistema de procesamiento del robot.

Esta plataforma permite integrar tareas computacionales relacionadas con el procesamiento de información y la visión artificial, según la arquitectura implementada.

### 3. Diseño electrónico

**[Esquemáticos](./Esquematicos/)**

Contiene los diagramas y documentos relacionados con el diseño electrónico del sistema.

Esta sección permite documentar las conexiones y la organización de los componentes electrónicos que conforman la plataforma robótica.

## Visión omnidireccional

El robot incorpora un sistema de visión omnidireccional diseñado para obtener información visual del entorno de juego.

Este tipo de visión permite observar una región amplia alrededor del robot mediante una configuración óptica apropiada, lo que puede facilitar la percepción de la cancha y de los elementos presentes durante el partido.

Entre las posibles tareas asociadas con este sistema se encuentran:

- Captura de imágenes del entorno.
- Procesamiento de imágenes.
- Detección y localización de objetos relevantes para el juego.
- Obtención de información visual para apoyar la toma de decisiones.
- Integración de la información visual con el sistema de control.

Las funciones específicas dependen de los algoritmos implementados en el software del proyecto.

## Movimiento omnidireccional

La plataforma utiliza un sistema de locomoción omnidireccional que permite desplazarse en distintas direcciones sin necesidad de orientar previamente el frente del robot hacia la dirección de movimiento.

Este tipo de locomoción resulta útil en fútbol robótico, donde se requieren desplazamientos laterales, diagonales y ajustes rápidos de posición.

El desarrollo del sistema contempla aspectos relacionados con:

- Cinemática del robot.
- Control de movimiento.
- Integración de motores y componentes electrónicos.
- Comunicación entre los módulos de control.
- Pruebas de funcionamiento de la plataforma.

## Documentación de la competencia

### Póster del proyecto

[Consultar el póster de TLALTRON](./POSTER_TLALTRON_.pdf)

Documento de presentación del proyecto y de sus características principales.

### Reglamento de competencia

[Consultar las reglas de la Copa FutBotMX 2026](./Reglas_Copa_FutBotMX_2026-01-19.pdf)

Documento de referencia para conocer los lineamientos de la competencia.

## Herramientas y tecnologías

Las tecnologías y plataformas relacionadas con el proyecto incluyen:

- **ESP32:** control embebido e interacción con componentes electrónicos.
- **Raspberry Pi:** plataforma de procesamiento computacional.
- **Python y C/C++:** lenguajes aplicables al desarrollo de software y sistemas embebidos, según el módulo.
- **Visión artificial:** procesamiento e interpretación de información visual.
- **Electrónica:** integración de componentes, conexiones y circuitos.
- **Git y GitHub:** control de versiones y documentación del proyecto.

## Aplicaciones y áreas de conocimiento

Este proyecto integra conocimientos de diferentes áreas de ingeniería:

- Robótica móvil.
- Sistemas embebidos.
- Visión artificial.
- Sistemas electrónicos.
- Cinemática y control de robots.
- Integración de hardware y software.
- Procesamiento de información para sistemas autónomos.

## Estructura del repositorio

La estructura principal del repositorio es la siguiente:

```text
RoboCup-2026/
├── ESP32/
├── Esquematicos/
├── RaspBerry Pi/
├── POSTER_TLALTRON_.pdf
├── Reglas_Copa_FutBotMX_2026-01-19.pdf
└── README.md
```

## Autor y proyecto

**Santana Robledo**

Repositorio del proyecto: [RoboCup-2026](https://github.com/santana-robledo/RoboCup-2026)

Perfil de GitHub: [santana-robledo](https://github.com/santana-robledo)

---

Este repositorio reúne la documentación y los archivos de desarrollo de una plataforma robótica omnidireccional para fútbol, integrando visión omnidireccional, electrónica, sistemas embebidos y control de movimiento.

<img width="899" height="1599" alt="Prototipo" src="https://github.com/user-attachments/assets/dfd14dea-16af-4e9d-ba6d-6235623663da" />
