# Practica4: Sistemas Operativos en Tiempo Real
## Objetivo: El objetivo de la práctica es comprender el funcionamiento de un sistema operativo en tiempo real. 
## Apartado 1:
En este apartado intentaremos cemostrar como FREERTOS maneja la ejecución de múltiples tareas en un ESP32. 
Los materiales usados han sido:
1. Placa ESP32
### Código usado:
```
#include <Arduino.h>
#include<FreeRTOS.h>
#include <task.h>



void anotherTask(void *parameter);  // Declaración de la función antes de su uso

void setup() {
    Serial.begin(115200); // Ajustado a una velocidad estándar

    /* Creación de una nueva tarea */
    xTaskCreate(
        anotherTask,    // Función de la tarea
        "Another Task", // Nombre de la tarea
        10000,         // Tamaño del stack
        NULL,          // Parámetro de la tarea
        1,             // Prioridad
        NULL           // Manejador de la tarea
    );
}

/* loop() es ejecutado por el ESP32 en la tarea predeterminada */
void loop() {
    Serial.println("Esto es la tarea principal del ESP32");
    delay(1000);
}

/* Función que se ejecuta como una tarea adicional */
void anotherTask(void *parameter) {
    for (;;) {  // Bucle infinito
        Serial.println("Esto es otra tarea");
        delay(1000);
    }
    vTaskDelete(NULL);  // Nunca se ejecutará debido al bucle infinito
}
```
### Funcionamiento del código:
1. Tenemos un serial.begin (115200) que configura la comunicación serial a 112500 baudios.
2. Creamos una nueva tarea llamada anotherTask a partir de xTaskCreate.
3. Cada segundo que se entiende con la línea delay (1000) se muestra por pantalla ("Esto es la tarea principal del ESP32").
4. Se ejecuta en un bucle infinito (for(;;)), imprimiendo "Esto es otra tarea" cada segundo.

###Salidas:
En el puerto serie se muestra:
```
this is ESP32 Task

```

## Apartado 2:
En este apartado se quiere implementar dos tareas que trabajen de manera sincronizada para encender y apagar un LED usando semáforos en FreeRTOS.
Los materiales usados han sido: 
1. Placa ESP32
2. Cable
3. LED
### Código usado
```
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


const int ledPin = ;  // Pin del LED
SemaphoreHandle_t xSemaphore;  // Semáforo para sincronizar las tareas


// Tarea para encender el LED
void TaskTurnOn(void *parameter) {
  for (;;) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) { // Toma el semáforo
      digitalWrite(ledPin, HIGH);
      Serial.println("LED ENCENDIDO");
      delay(1000);  // Espera un segundo antes de liberar el semáforo
      xSemaphoreGive(xSemaphore); // Libera el semáforo
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Pequeña espera antes de intentar tomar el semáforo nuevamente
  }
}


// Tarea para apagar el LED
void TaskTurnOff(void *parameter) {
  for (;;) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) { // Toma el semáforo
      digitalWrite(ledPin, LOW);
      Serial.println("LED APAGADO");
      delay(1000);  // Espera un segundo antes de liberar el semáforo
      xSemaphoreGive(xSemaphore); // Libera el semáforo
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);


  xSemaphore = xSemaphoreCreateBinary();  // Crear el semáforo


  xTaskCreate(TaskTurnOn, "Encender LED", 1000, NULL, 1, NULL);
  xTaskCreate(TaskTurnOff, "Apagar LED", 1000, NULL, 1, NULL);


  xSemaphoreGive(xSemaphore); // Inicializa el semáforo en "libre"
}


void loop() {
  // No se necesita código en loop(), ya que todo ocurre en las tareas de FreeRTOS
}

```
### Funcionamiento del código:
1. Se crean las tareas: Tarea 1: Enciende el LED. Tarea 2: Apaga el LED.
2. Uso del semáforo: Sincronizamos las tareas, utlizando un semáforo binario. Este semáforo tiene dos estados posibles: xSemaphoreTake (espera hasta que el semáforo esté disponible) y xSemaphoreGive (libera el semáforo para que otra tarea lo pueda usar).
3. Funcionamiento de las tareas:
Tarea 1 (Encender LED): Esta tarea se ejecuta y espera hasta que el semáforo esté disponible. Luego enciende el LED y libera el semáforo, permitiendo que la otra tarea lo apague.
Tarea 2 (Apagar LED): Después de recibir el semáforo, apaga el LED y libera el semáforo para que la tarea 1 pueda volver a encender el LED.
4. Sincronización:
Las tareas están sincronizadas gracias al semáforo, garantizando que solo una de ellas acceda al LED a la vez.
###Salidas:
![image](https://github.com/user-attachments/assets/44078ea4-19de-437e-8035-5145da60d705)
![image](https://github.com/user-attachments/assets/5ec7379d-4761-4915-a444-5662119a36d9)



