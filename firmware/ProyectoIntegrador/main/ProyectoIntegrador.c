/*! @mainpage Proyecto Integrador : Corrector de Postura
 *
 * @section genDesc General Description
 *
 * PostureCare es un sistema de monitoreo de postura corporal que utiliza un
 * acelerómetro analógico ADXL335 para medir la inclinación del usuario y detectar malas posturas.
 * Funcionamiento:
 * Calibración inicial de 3 segundos al encender el dispositivo.
 * Monitoreo continuo de la inclinación corporal.
 * Si la postura incorrecta se mantiene durante 3 segundos, se activa una advertencia(LED amarillo).
 * Si se mantiene durante más de 5 segundos, se activa una alerta (LED rojo).
 * Además, el sistema puede enviar los datos al celular vía Bluetooth en tiempo real.
 *
 * @section hardConn Hardware Connections
 *
 * | Peripheral         | ESP32 EDU-CIAA | Descripción                            |
 * |:------------------:|:---------------|:---------------------------------------|
 * | Acelerómetro eje X | CH1 (ADC)      | Salida analógica X                     |
 * | Acelerómetro eje Y | CH2 (ADC)      | Salida analógica Y                     |
 * | Acelerómetro eje Z | CH3 (ADC)      | Salida analógica Z                     |
 * | LED verde          | LED_1         | Indica buena postura                   |
 * | LED amarillo       | LED_2         | Indica advertencia (3s)                |
 * | LED rojo           |          | Indica mala postura (5s)               |
 * | Buzzer             | GPIO_x         | Alerta sonora                          |
 * | Bluetooth          | BLE int.       | Comunicación con celular               |
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 23/10/2025 | Integración con driver ADXL335                 |
 * | 22/10/2025 | Document creation		
 * | 12/11/2025 | Implementación completa con Bluetooth          |                         |
 *
 * @author
 * Anahí Chaves (natalia.chaves@ingenieria.uner.edu.ar)
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "buzzer.h"
#include "ble_mcu.h"
#include "ADXL335.h"
/*==================[macros and definitions]=================================*/
/**
 * @def PERIODO_MUESTREO_AC
 * @brief Periodo de muestreo del acelerómetro en milisegundos
 */
#define PERIODO_MUESTREO_AC 1000 
/**
 * @def UMBRAL_INCLINACION
 * @brief Umbral de inclinación en grados para considerar mala postura
 * @details Cualquier desviación mayor a este valor activa las alertas
 */
#define UMBRAL_INCLINACION 12.0f 
/**
 * @def TIEMPO_ADVERTENCIA
 * @brief Tiempo en ms para activar advertencia (LED amarillo)
 */
#define TIEMPO_ADVERTENCIA 3000  
/**
 * @def TIEMPO_ALERTA
 * @brief Tiempo en ms para activar alerta (LED rojo + buzzer)
 */
#define TIEMPO_ALERTA 5000      
/**
 * @def TIEMPO_CALIBRACION
 * @brief Tiempo en ms para la calibración inicial del acelerómetro
 */
#define TIEMPO_CALIBRACION 3000  

/**==================[internal data definition]===============================*/

/**
 * @struct acelerometro_data_t
 * @brief Estructura para almacenar datos del acelerómetro
 * 
 * Contiene las aceleraciones en los tres ejes y el ángulo calculado
 */
typedef struct
{
    float ax;
    float ay;
    float az;
    float angulo;
} acelerometro_data_t;

/** @brief Variable global con los últimos datos del acelerómetro */
volatile acelerometro_data_t datos_acelerometro = {0};

/**@var posture_state 
 * @brief Estado actual de la postura
 * @details 0 = correcta, 1 = advertencia (3s), 2 = alerta (5s)
 */
volatile uint8_t posture_state = 0;

/** @brief Tiempo acumulado en postura incorrecta (ms) */
volatile uint32_t bad_posture_time = 0;

/* Variables de calibración */
static float base_x = 0, base_y = 0, base_z = 0;

/**
 * @var calibrado
 * @brief Indica si el sistema completó la calibración inicial
 */
static bool calibrado = false;
TaskHandle_t ble_task_handle = NULL;


/*==================[internal functions declaration]=========================*/

/**
 * @brief Calcula el ángulo de inclinación a partir de los valores del acelerómetro.
 * Utiliza el producto punto entre el vector de aceleración actual y el vector
 * de referencia (calibrado) para determinar el ángulo de desviación.
 * @param ax Aceleración en eje X
 * @param ay Aceleración en eje Y
 * @param az Aceleración en eje Z
 * @return Ángulo de inclinación en grados
 */
static float CalcularAnguloDesviacion(float ax, float ay, float az)
{
    //Producto punto entre vector actual y vector de referencia
    float dot = (ax * base_x) + (ay * base_y) + (az * base_z);
    //Magnitud de ambos vectores
    float mag_base = sqrtf(base_x * base_x + base_y * base_y + base_z * base_z);
    float mag_actual = sqrtf(ax * ax + ay * ay + az * az);
    //Coseno del ángulo entre ambos vectores
    float cos_theta = dot / (mag_base * mag_actual);

    // Evitar valores fuera de rango [-1,1] por errores de redondeo
    if (cos_theta > 1.0f)
        cos_theta = 1.0f;
    if (cos_theta < -1.0f)
        cos_theta = -1.0f;
    //Convertir de rad a grados
    return acosf(cos_theta) * (180.0f / 3.14159f);
}

/**
 * @brief Aplica un filtro de promedio para suavizar las lecturas.
 * Reduce el ruido en las mediciones del acelerómetro.
 * @param nuevo Valor nuevo leído del acelerómetro
 * @param previo Valor previo filtrado
 * @return valor filtrado
 */
static float FiltroSuavizado(float nuevo, float previo)
{
    return (0.8f * previo) + (0.2f * nuevo); // promedio móvil simple
}

/**
 * @brief Tarea que lee el acelerómetro periódicamente.
 *
 * Esta tarea se ejecuta cada PERIODO_MUESTREO_AC milisegundos.
 * Realiza las siguientes acciones: 
  1. Lee los valores ax, ay, az del acelerómetro.
  2. Aplica un filtro de suavizado.
  3. Durante los primeros TIEMPO_CALIBRACION ms, acumula las lecturas para
     calcular la calibración.
  4. Después de la calibración, calcula el ángulo de inclinación continuamente.  la posición base.
 */
void LeerAcelerometro(void *pvParameter)
{
    float suma_x = 0, suma_y = 0, suma_z = 0;
    uint16_t muestras = 0;

    TickType_t start_time = xTaskGetTickCount();

    while (true)
    {   //Leer valores del acelerometro
        float ax = ReadXValue();
        float ay = ReadYValue();
        float az = ReadZValue();
        // Aplicar filtro de suavizado
        datos_acelerometro.ax = FiltroSuavizado(ax, datos_acelerometro.ax);
        datos_acelerometro.ay = FiltroSuavizado(ay, datos_acelerometro.ay);
        datos_acelerometro.az = FiltroSuavizado(az, datos_acelerometro.az);

        // Calibración inicial
        if (!calibrado)
        {
            suma_x += datos_acelerometro.ax;
            suma_y += datos_acelerometro.ay;
            suma_z += datos_acelerometro.az;
            muestras++;
            //Verifica si terminó el tiempo de calibración
            if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(TIEMPO_CALIBRACION))
            {   //Calcula los promerios como valores de referencia
                base_x = suma_x / muestras;
                base_y = suma_y / muestras;
                base_z = suma_z / muestras;
                calibrado = true;
                printf("✅ Calibracion completa: X=%.2f Y=%.2f Z=%.2f\r\n", base_x, base_y, base_z);
            }
        }
        else
        {   // Calcular ángulo de desviación respecto a la posición de referencia.
            datos_acelerometro.angulo = CalcularAnguloDesviacion(
                datos_acelerometro.ax,
                datos_acelerometro.ay,
                datos_acelerometro.az);
        }

        // Espera hasta el siguiente periodo de muestreo.
        vTaskDelay(1000 / PERIODO_MUESTREO_AC);
    }
}

/**
 * @brief Tarea que evalúa la postura del usuario en base al ángulo de inclinación.
 *
 * Si el ángulo supera el umbral definido (UMBRAL_INCLINACION), se considera postura incorrecta.
 * Si se mantiene más de 3 s, cambia a estado de advertencia (LED amarillo).
 * Si supera 5 s, pasa a estado de alerta (LED rojo + buzzer).
 * Si vuelve a postura correcta, se reinicia el temporizador y el estado.
 * Se ejecuta cada PERIODO_MUESTREO_AC milisegundos.
 */
void ProcesarPostura(void *pvParameter)
{
    
    while (true)
    {
        if (calibrado)
        {   
            // Verificar si el ángulo supera el umbral de inclinación
            if (fabs(datos_acelerometro.angulo) > UMBRAL_INCLINACION)
            {   //Acumula tiempo en mala postura
                bad_posture_time += PERIODO_MUESTREO_AC;
                
                //Actualiza estado según el tiempo acumulado
                if (bad_posture_time >= TIEMPO_ALERTA)
                    posture_state = 2;
                else if (bad_posture_time >= TIEMPO_ADVERTENCIA)
                    posture_state = 1;
            }
            else
            {  
                //Postura correcta: reiniciar contador y estado
                bad_posture_time = 0;
                posture_state = 0;
            }
        }
        // Esperar al siguiente periodo
        vTaskDelay(pdMS_TO_TICKS(PERIODO_MUESTREO_AC));
    }
}
/**
 * @brief Tarea que actualiza los LEDs y buzzer según el estado de postura.
 *
 * Estado 0 → LED verde encendido (postura correcta)
 * Estado 1 → LED amarillo encendido (advertencia)
 * Estado 2 → LED rojo encendido + buzzer (alerta)
 * Se actualiza cada 1 segundo.
 */
void ActualizarIndicadores(void *pvParameter)
{
    while (true)
    {
        switch (posture_state)
        {
        case 0: // Postura correcta
            LedOn(LED_1);
            LedOff(LED_2);
            LedOff(LED_3);
            BuzzerOff();
            break;
        case 1: // Advertencia
            LedOff(LED_1);
            LedOn(LED_2);
            LedOff(LED_3);
            BuzzerOff();
            break;
        case 2: // Alerta
            LedOff(LED_1);
            LedOff(LED_2);
            LedOn(LED_3);
            BuzzerOn();
            break;
        default:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Tarea que envía datos de postura al celular vía Bluetooth BLE.
 *
 * Envía en tiempo real a la aplicación:
 * -Aceleraciones X,Y,Z en g.
 * -Ángulo de inclinación en grados.
 * -Estado de postura (correcta, advertencia, alerta).
 * Envía datos cada 100 ms.
 * El prefijo * indica inicio de dato en protocolo Bluetooth Electronics.
 */
void Bluetooth(void *pvParameter)
{
    char buffer[124];
    while (true)
    {
        //Convertir estado numérico a texto
        const char *estado_texto;
        switch(posture_state){
            case 0: estado_texto = "Correcta"; break;
            case 1: estado_texto = "Incorrecta-Advertencia"; break;
            case 2: estado_texto = "Incorrecta-Alerta"; break;
            default: estado_texto = "Desconocido"; break;
        }

        // Enviar datos individuales (para que los reciba cada widget)
        snprintf(buffer, sizeof(buffer),
                 "*X%.2fg\n*Y%.2fg\n*Z%.2fg\n*A%.2f\n*E%s\n",
                 datos_acelerometro.ax,
                 datos_acelerometro.ay,
                 datos_acelerometro.az,
                 datos_acelerometro.angulo,
                 estado_texto);
            BleSendString(buffer);
        // Esperar al siguiente envío
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

/*==================[external functions definition]==========================*/
void app_main(void)
{

    // Inicialización de periféricos
    ADXL335Init();
    LedsInit();
    BuzzerInit(GPIO_4); // Pin  al buzzer
    
    //Configuración de Bluetooth
    ble_config_t ble_device = {
        .device_name = "PostureCare",
        .func_p = BLE_NO_INT, // No se espera recepción de datos
    };
    BleInit(&ble_device); // Inicializar Bluetooth

    // Creación de tareas
    xTaskCreate(LeerAcelerometro, "LeerAcelerometro", 2048, NULL, 5, NULL);
    xTaskCreate(ProcesarPostura, "ProcesarPostura", 2048, NULL, 5, NULL);
    xTaskCreate(ActualizarIndicadores, "ActualizarIndicadores", 2048, NULL, 5, NULL);
    xTaskCreate(Bluetooth, "Bluetooth", 2048, NULL, 5, NULL);
}
/*==================[end of file]============================================*/