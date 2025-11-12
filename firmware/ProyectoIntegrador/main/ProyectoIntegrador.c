/*! @mainpage Proyecto Integrador : Corrector de Postura
 *
 * @section genDesc General Description
 *
 * PostureCare es un sistema de monitoreo de postura corporal que utiliza un
 * acelerómetro analógico ADXL335 para medir la inclinación del usuario y detectar malas posturas.
 * Si la postura incorrecta se mantiene durante 3 segundos, se activa una advertencia
 * (LED amarillo). Si se mantiene durante más de 5 segundos, se activa una alerta
 * (LED rojo y buzzer). Además, el sistema puede enviar los datos al celular vía Bluetooth.
 *
 * @section hardConn Hardware Connections
 *
 * | Peripheral         | ESP32 EDU-CIAA | Descripción                            |
 * |:------------------:|:---------------|:---------------------------------------|
 * | Acelerómetro eje X | CH1 (ADC)      | Salida analógica X                     |
 * | Acelerómetro eje Y | CH2 (ADC)      | Salida analógica Y                     |
 * | Acelerómetro eje Z | CH3 (ADC)      | Salida analógica Z                     |
 * | LED verde          | GPIO_x         | Indica buena postura                   |
 * | LED amarillo       | GPIO_x         | Indica advertencia (3s)                |
 * | LED rojo           | GPIO_x         | Indica mala postura (5s)               |
 * | Buzzer             | GPIO_x         | Alerta sonora                          |
 * | Bluetooth          | BLE int.       | Comunicación con celular               |
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 23/10/2025 | Integración con driver ADXL335                 |
 * | 22/10/2025 | Document creation		                         |
 *
 * @author
 * Anahí Chaves
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

#define PERIODO_MUESTREO_AC 1000 // Periodo de muestreo del acelerómetro en ms
#define UMBRAL_INCLINACION 12.0f // Umbral de inclinación en grados
#define TIEMPO_ADVERTENCIA 3000  // Tiempo para activar advertencia en ms
#define TIEMPO_ALERTA 5000       // Tiempo para activar alerta en ms
#define TIEMPO_CALIBRACION 3000  // ms de calibración inicial (~3 s)

/**==================[internal data definition]===============================*/

/** @brief Estructura para almacenar datos del acelerómetro */
typedef struct
{
    float ax;
    float ay;
    float az;
    float angulo;
} acelerometro_data_t;

/** @brief Variable global con los últimos datos del acelerómetro */
volatile acelerometro_data_t datos_acelerometro = {0};

/** @brief Estado actual de la postura
 *  0 = correcta, 1 = advertencia (3s), 2 = alerta (5s)
 */
volatile uint8_t posture_state = 0;

/** @brief Tiempo acumulado en postura incorrecta (ms) */
volatile uint32_t bad_posture_time = 0;
/* Variables de calibración */
static float base_x = 0, base_y = 0, base_z = 0;
static bool calibrado = false;
TaskHandle_t ble_task_handle = NULL;


/*==================[internal functions declaration]=========================*/

/**
 * @brief Calcula el ángulo de inclinación a partir de los valores del acelerómetro.
 * @param ax Aceleración en eje X
 * @param ay Aceleración en eje Y
 * @param az Aceleración en eje Z
 * @return Ángulo de inclinación en grados
 */
static float CalcularAnguloDesviacion(float ax, float ay, float az)
{
    float dot = (ax * base_x) + (ay * base_y) + (az * base_z);
    float mag_base = sqrtf(base_x * base_x + base_y * base_y + base_z * base_z);
    float mag_actual = sqrtf(ax * ax + ay * ay + az * az);
    float cos_theta = dot / (mag_base * mag_actual);

    // Evitar valores fuera de rango por redondeo
    if (cos_theta > 1.0f)
        cos_theta = 1.0f;
    if (cos_theta < -1.0f)
        cos_theta = -1.0f;

    return acosf(cos_theta) * (180.0f / 3.14159f);
}

/**
 * @brief Aplica un pequeño filtro promedio para suavizar las lecturas.
 */
static float FiltroSuavizado(float nuevo, float previo)
{
    return (0.8f * previo) + (0.2f * nuevo); // promedio móvil simple
}

/**
 * @brief Tarea que lee el acelerómetro periódicamente.
 *
 * Esta tarea se ejecuta cada PERIODO_MUESTREO_AC milisegundos.
 * Obtiene los valores ax, ay, az del acelerómetro
 * y calcula el ángulo de inclinación.
 */
void LeerAcelerometro(void *pvParameter)
{
    float suma_x = 0, suma_y = 0, suma_z = 0;
    uint16_t muestras = 0;

    TickType_t start_time = xTaskGetTickCount();

    while (true)
    {
        float ax = ReadXValue();
        float ay = ReadYValue();
        float az = ReadZValue();

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

            if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(TIEMPO_CALIBRACION))
            {
                base_x = suma_x / muestras;
                base_y = suma_y / muestras;
                base_z = suma_z / muestras;
                calibrado = true;
                printf("✅ Calibracion completa: X=%.2f Y=%.2f Z=%.2f\r\n", base_x, base_y, base_z);
            }
        }
        else
        {
            datos_acelerometro.angulo = CalcularAnguloDesviacion(
                datos_acelerometro.ax,
                datos_acelerometro.ay,
                datos_acelerometro.az);
        }

        // arreglar
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
 */
void ProcesarPostura(void *pvParameter)
{
    // TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        if (calibrado)
        {
            if (fabs(datos_acelerometro.angulo) > UMBRAL_INCLINACION)
            {
                bad_posture_time += PERIODO_MUESTREO_AC;

                if (bad_posture_time >= TIEMPO_ALERTA)
                    posture_state = 2;
                else if (bad_posture_time >= TIEMPO_ADVERTENCIA)
                    posture_state = 1;
            }
            else
            {
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
 * Envía:
 * - >angle:<valor>     → Ángulo en grados.
 * - >state:<valor>     → Estado (0=correcta, 1=advertencia, 2=alerta).
 */
void Bluetooth(void *pvParameter)
{
    char buffer[124];
    while (true)
    {
        // Texto legible del estado
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