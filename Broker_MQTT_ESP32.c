#include <stdio.h> //Libreria Nativa de C
#include "esp_system.h"
#include "freertos/FreeRTOS.h" //Libreria de FreeRTOS para gestion y ejecucion multitareas
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h" //Libreria para manejo de GPIO (pines)
#include "esp_log.h" //Libreria para manejo de logs (Mensajes de debug, alertas de eventos y advertencias criticas)
#include "sdkconfig.h"
#include "mqtt_client.h" //Libreria para manejo de MQTT desde el Cliente (el mismo ESP32)
#include "esp_event.h" //Libreria para manejo de eventos
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h" //Libreria para manejo y configuracion personalizada WiFi
#include "nvs.h"
#include "nvs_flash.h" //Libreria para manejo de Memoria no Volatil, necesaria y utilizada para el almacenamiento de datos WiFi
#include "esp_adc_cal.h"
#include "driver/adc.h" //Libreria para manejo de ADC (Convertidor Analogico Digital(Lector de Sensores))

/** 
* Los Tags son etiquetas o prefijos que indican la procedencia de una alerta por evento gracias a los logs
* Estos se envian por UART hacia el monitor serie, similar a el monitor serial de ArduinoIDE
* @example [MQTT_Client]: MQTT Client Inicializado Con Exito
* @example [Wifi]: El modo Estacion (STA) a sido inicializado con exito
* @example [ESP_System]: Reinicio por falla de alimentacion...
*/

static const char *MQTT_TAG = "MQTT_Client";
static const char *WIFI_TAG = "WiFi";
static const char *ESP_TAG = "ESP_System";

uint8_t sensor_state;
uint16_t potenciometer_state;

/** @brief el puntero al string ""v1/devices/me/telemetry" es la forma de seleccionar 
 * el topico de Thingsboard al que se desean enviar los datos"
 * 
 * Los Topicos son temas que se utilizan para publicar mensajes, esto se ve mas en Brokers locales donde las tematicas son creadas 
 * por uno mismo y la modularidad es mas alta, por ejemplo se puede crear un topico llamado sensores, donde se dirigan todos los
 * datos recopilados de sensores, en este caso "v1/devices/me/telemetry" es un topico general que sirve para alimentar los graficos
 * deseados por envio de strings en formato JSON.
 * 
 * */
 char *topic = "v1/devices/me/telemetry";

 /** @brief Arreglo usado para contener la parte util del mensaje 
  * (significado literal de payload, parte util del mensaje sin encabezado de direccion IP, MAC y demas datos sensibles)*/

 char payload[50]; 

/*Los manejadores (Handle) permiten conectar distintas tareas y funciones, favoreciendo 
el traspaso de datos o manejo de eventos en el sistema, ademas permiten ingresar configuraciones a los perifericos y protocolos del esp32*/

QueueHandle_t sensor_queue;
QueueHandle_t potenciometer_queue;
esp_mqtt_client_handle_t mqtt_client;

#define WIFI_SSID "LIB-3321567" //No hackear
#define WIFI_PASS "UpbkFd6qfr9e"

#define THINGSBOARD_URL "mqtt://thingsboard.cloud:1883"
#define THINGSBOARD_TOKEN "jpkwvr6W6XAzwpCnyOQ1" 
/*El Token (THINGSBOARD_TOKEN) anade la primera capa de seguridad, ya que solo el propietario 
puede obtenerlo y es obligatorio para conectarte con el broker*/

#define HIGH 1 // Definiciones de estados lógicos para mayor claridad en el código
#define LOW 0


/** 
 * @note Se declara la existencia de la funcion mqtt_client_launch(void) sin definir su contenido para evitar conflictos con el compilador
 * 
 * El manejador de eventos de la funcion se crea antes que la funcion misma de lanzamiento del cliente MQTT
 * y dentro de este manejador se hace llamado a la funcion para lanzar el cliente MQTT, lo cual causa el error de compilacion 
 * 
*/

esp_err_t mqtt_client_launch(void); 



/**
 * @brief Asigna un valor dado de un rango a otro.
 *
 * Funcion de Arduino definida en ESP-IDF al carecer de esta funcion propiamente.
 * 
 * Esta función toma un valor `x` que está dentro del rango de `in_min` a `in_max` 
 * y lo asigna a un valor correspondiente dentro del rango de `out_min` a `out_max`.
 *
 * @param x El valor que se asignará.
 * @param in_min El límite inferior del rango de entrada.
 * @param in_max El límite superior del rango de entrada.
 * @param out_min El límite inferior del rango de salida.
 * @param out_max El límite superior del rango de salida.
 * @return El valor asignado en el rango de salida.
 * 
 * Usada para simular valores ficticios con el fin de alimentar los graficos del broker
 */

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * @brief Manejador de eventos del cliente MQTT
 *
 * Maneja los eventos generados por el cliente MQTT, como la publicacion de un mensaje,
 * el error al conectarse con el broker, la perdida de la conexion, la reconexion exitosa,
 * el recibo de un mensaje, etc.
 *
 * @param arg puntero a la informacion de contexto adicional, no utilizado en este caso
 * @param event_base tipo de evento base, en este caso MQTT_EVENT
 * @param event_id identificador de evento especifico
 * @param event_data informacion adicional asociada con el evento
 */

void mqtt_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
esp_mqtt_event_handle_t event = event_data; /*Por medio de event_data FreeRTOS ingresa la direccion de la estructura que contiene el evento
emergente segun la comunicacion entre el cliente y el broker, esta se guarda en el puntero event para usarse en la ramificacion siguiente */
switch (event->event_id)
{

/** @warning Eventos No funcionales
 * 
 * Algunos eventos como MQTT_EVENT_DELETED y MQTT_EVENT_PUBLISHED son no funcionales y son netamente para interpretacion,
 * ya que no tuve la oportunidad de experimentar con el desencadenante de estos eventos
 */

/** @brief ESP_LOG
 * 
 * El macro ESP_LOG permite emitir mensajes por UART hacia el monitor, indicando el nivel de severidad del evento.
 * es muy similar a la funcion nativa printf, pero ESP_LOG puede marcar eventos normales, advertencias y errores criticos con
 * ESP_LOGI (normal), ESP_LOGW (advertencia) y ESP_LOGE (error critico), respectivamente.
 */

case MQTT_EVENT_DELETED:
ESP_LOGI(MQTT_TAG, "{Evento : DELETED}: La Publicacion del Mensaje [%s] ha Sido Eliminada", event->topic); 
break;

case MQTT_EVENT_PUBLISHED:
ESP_LOGI(MQTT_TAG, "{Evento : PUBLISHED}: La Publicacion del Mensaje [%s] ha Sido Exitosa", event->topic);
break;

case MQTT_EVENT_ERROR:
ESP_LOGW(MQTT_TAG, "{Evento : ERROR}: El Broker [%s] a Notificado de un Error Desconocido... ", THINGSBOARD_URL);
break;

case MQTT_EVENT_BEFORE_CONNECT:
ESP_LOGI(MQTT_TAG, "{Evento : BEFORE_CONNECT}: Estableciendo Conexion Con el Broker [%s]", THINGSBOARD_URL);
break;

/** @brief MQTT_EVENT_DISCONNECTED 
 * 
 * Los nombres de los eventos corresponden a un typedef enum (enumeracion) 
 * que representan valores enteros como 1, 2, 3 etc.
 * 
 * En este evento se intenta realizar una reconexion por ciclo for donde se itera la 
 * funcion esp_mqtt_client_reconnect(mqtt_client) hasta que sea exitosa con un maximo 
 * de 10 intentos antes de detener el cliente mqtt por completo y eliminarlo, 
 * posterior a eso el cliente se re-inicializa con mqtt_client_launch();, 
 * si la funcion esp_mqtt_client_stop o client_destroy 
 * fallan el sistema se reinicia a causa del macro ESP_ERROR_CHECK();
 * 
 * @note @return Las funciones generalmente devuelven un 0 si son exitosas y un -1 si fallan, 
 * En el caso de if(esp_mqtt_client_destroy(mqtt_client)) si la funcion logra destruir el cliente entonces
 * la condicion no se cumple y el if se salta, si no lo logra se produce un reinicio.
 * 
*/


case MQTT_EVENT_DISCONNECTED:
    ESP_LOGE(MQTT_TAG, "Conexion Con el Broker [%s] Perdida, Retomando la Conexion", THINGSBOARD_URL);
    for(uint8_t i = 0; esp_mqtt_client_reconnect(mqtt_client) == ESP_FAIL; i++){
 vTaskDelay(1000 / portTICK_PERIOD_MS); //Funcion de espera, el equivalente a delay(1000) de arduino
 if(i >= 10){
 ESP_LOGE(MQTT_TAG, "{Evento : DISCONNECT}: Reintentos de Reconexion Excedidos, Reiniciando Configuraciones Del MQTT Client...");
 ESP_ERROR_CHECK(esp_mqtt_client_stop(mqtt_client));
 if(esp_mqtt_client_destroy(mqtt_client)){
     ESP_LOGE(MQTT_TAG, "Reinicio Del MQTT Client Fallido, Reiniciando Sistema..."); 
     esp_restart();
 };
 mqtt_client_launch();
 }
    }
    ESP_LOGI(MQTT_TAG, "Conexion Con El Broker [%s] Restablecida", THINGSBOARD_URL);
    break;

case MQTT_EVENT_CONNECTED:
    ESP_LOGI(MQTT_TAG,"{Evento : CONNECT}: La Conexion Con el Broker [%s] es Estable y Funcional", THINGSBOARD_URL);
    break;

/** @brief Si el Broker emite algun mensaje como una respuesta a alertas este evento permite recibirlas
 * puede ser usado para manejar desiciones dentro de otras tareas o regular el sistema desde Thingsboard
 * para calibraciones sin necesidad de recompilar el codigo, similar a la idea de la pantalla tactil Wifi
 * pero desde el celular o computador
 */

case MQTT_EVENT_DATA:
ESP_LOGI(MQTT_TAG, "{Evento : DATA}: Mensaje Recibido:[\n Topic: %.*s\n Data: %.*s]", event->topic_len, event->topic, event->data_len, event->data);
    break;
    
default:
ESP_LOGW(MQTT_TAG, "[Warning]: Evento No Reconocido");
    break;
}

}

/**
 * @brief Inicializa el cliente MQTT y lo registra para eventos de MQTT
 *
 * Inicializa el cliente MQTT con la configuracion del broker y el token de ThingsBoard,
 * registra el manejador de eventos de MQTT y lanza el cliente MQTT.
 *
 * @return ESP_OK si se pudo inicializar el cliente MQTT, ESP_FAIL en caso contrario
 */
esp_err_t mqtt_client_launch(void){
    
esp_mqtt_client_config_t mqtt_cfg = { //Estructura que almacena las credenciales del broker para la conexion
    .broker.address.uri = THINGSBOARD_URL, 
    .credentials.username = THINGSBOARD_TOKEN
};

mqtt_client = esp_mqtt_client_init(&mqtt_cfg); //Se carga la configuracion al handler del cliente mqtt
if(mqtt_client == NULL){
ESP_LOGE(MQTT_TAG,"{Error}: No Se Pudo Cargar La Configuracion Del MQTT Client desde: [esp_mqtt_client_init(&mqtt_cfg)]");
return ESP_FAIL;
}
ESP_LOGI(MQTT_TAG, "Configuraciones De MQTT Inicializadas...");

ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL));
/*La funcion anterior adjunta el manejador de eventos a la serie de eventos que debe administrar, en este caso 
cualquier evento proveniente de la comunicacion MQTT se tomara en cuenta*/
ESP_LOGI(MQTT_TAG, "Manejador de Eventos de MQTT Client Inicializado...");

if(esp_mqtt_client_start(mqtt_client) == ESP_FAIL){ //funcion encargada de lanzar el cliente configurado
ESP_LOGE(MQTT_TAG, "{Error}: No Se Pudo Iniciar El MQTT Client"); 
return ESP_FAIL;
}
ESP_LOGI(MQTT_TAG, "Cliente MQTT Inicializado Con Exito...");

return ESP_OK;
}

/**
 * @brief Manejador de eventos de WiFi
 *
 * Maneja los eventos de WiFi incluyendo el inicio de la configuracion WiFi, la
 * conexion a la red, la obtencion de la IP y la desconexcion de la red.
 *
 * @param arg Puntero a la informacion de contexto adicional, no utilizado en este
 *            caso
 * @param event_base Conjunto o grupo de eventos utilizados, en este caso WIFI_EVENT o IP_EVENT
 * @param event_id Identificador de evento especifico dentro del grupo de eventos
 * @param event_data Informacion adicional asociada con el evento
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){

if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
ESP_LOGI(WIFI_TAG, "EL Modo WiFi STA ha Sido Inicializado Con Exito");
ESP_LOGI(WIFI_TAG, "Conectando...");
for(uint8_t i = 0; esp_wifi_connect() != ESP_OK; i++){ /*Mismo sistema usado en el manejador MQTT, itera hasta que la conexion se establezca*/
vTaskDelay(1000 / portTICK_PERIOD_MS);
ESP_LOGI(WIFI_TAG, "Conectando...");
if(i >= 10){
ESP_LOGE(WIFI_TAG, "Reintentos de Reconexion Excedidos, Reiniciando el Sistema...");
esp_restart();
}
} 
}

if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED){
ESP_LOGI(WIFI_TAG, "Estacion WiFi (STA) Conectada a la Red %s", WIFI_SSID);
ESP_LOGI(WIFI_TAG, "Iniciando Obtencion De Datos IP, Mask, Gateway...");
}

if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
ESP_LOGW(WIFI_TAG, "Conexion WiFi Interrumpida, Reconectando...");
for(uint8_t i = 0; esp_wifi_connect() == ESP_FAIL; i++){ 
/*Otro mecanismo de reconexion aunque este es redundante ya que existe una opcion 
que no habilite y que realiza la reconexion automatica*/
vTaskDelay(100 / portTICK_PERIOD_MS);
if(i >= 10){
ESP_LOGE(WIFI_TAG, "Reintentos de Reconexion Excedidos, Reiniciando el Sistema...");
esp_restart();    
}
}
ESP_LOGI(WIFI_TAG, "Reconexion Exitosa, Retomando la Comunicacion...");
}
if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){

/** @brief IPSTR es un macro que contiene un string que represeta
 *  el formato IP segun los prefijos de formateo de texto de printf (%d.%d.%d.%d)
 *  respecto a la netmask y al gateway lo mismo
 *  */    

ip_event_got_ip_t *ip_data = (ip_event_got_ip_t*) event_data;
ESP_LOGI(WIFI_TAG, "Datos de Red Obtenidos Con Exito: IP:" IPSTR "Mask:" IPSTR "Gateway:" IPSTR, 
    IP2STR(&ip_data->ip_info.ip),
    IP2STR(&ip_data->ip_info.netmask),
    IP2STR(&ip_data->ip_info.gw)
    );


}
}
/**
 * @brief Conecta a una red WiFi con credenciales especificadas.
 * Inicializa las configuraciones WiFi necesarias, crea un manejador de eventos WiFi
 * y configura el protocolo WiFi para conectarse a una red en modo STA.
 * @return ESP_OK en caso de exito, ESP_FAIL en caso de error.
 */
esp_err_t wifi_default_connect(void){

/*Sistema muy similar al MQTT pero mas directo y sencillo*/

ESP_LOGI(WIFI_TAG, "Iniciando Las Configuraciones WiFi Necesarias...");
ESP_ERROR_CHECK(esp_netif_init());
ESP_ERROR_CHECK(esp_event_loop_create_default());

esp_netif_t *wifi_netif = esp_netif_create_default_wifi_sta();
assert(wifi_netif);
ESP_LOGI(WIFI_TAG, "Interface WiFi Inicializada");
wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();

esp_wifi_init(&wifi_cfg);

esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &wifi_event_handler, NULL);
ESP_LOGI(WIFI_TAG,"Manejador De Eventos WiFi Iniciado");
wifi_config_t wifi_config = { /*Esta estructura contiene una gran cantidad de configuraciones y protocolos de seguridad no aprovechados aqui*/
    .sta = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
        .threshold.rssi = -120
    },

};
ESP_LOGI(WIFI_TAG, "Credenciales WiFi Configuradas...");

esp_wifi_set_mode(WIFI_MODE_STA);
esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
esp_wifi_start();
ESP_LOGI(WIFI_TAG, "Protocolo WiFi Inicializado Con Exito, Procediendo a la Conexion Con la Red: %s", WIFI_SSID);

return ESP_OK;
}

/**
 * @brief Lee el valor de los sensores y envia el valor leido a una cola
 * La tarea lee el valor del sensor digital en el pin GPIO_NUM_4 y el valor del
 * potenciometro en el pin ADC1_CHANNEL_6, y envia este valor a una cola.
 * @param pvParameters No se utiliza
 */
void sensor_task(void* pvParameters){
gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);

for(;;){ //lo mismo que while(1)
potenciometer_state = adc1_get_raw(ADC1_CHANNEL_6); //lee el valor del potenciometro   
sensor_state = !gpio_get_level(GPIO_NUM_4); //lee un boton con pull-up


/** @note Se envian los valores leidos de los sensores a la cola
 * 
 * Las colas en FreeRTOS permiten comunicar tareas entre si almacenando datos en fila y en orden,
 * En este caso los datos capturados por los perifericos son enviados por la cola "sensor_queue"
 * hacia la tarea que se encarga de enviar los datos al broker
 * 
 * @note portTICK_PERIOD_MS es un macro que se utiliza para calcular el tiempo de espera en milisegundos ya que el 
 * sistema trabaja por "ticks" de reloj, no comparables a tiempo comun como milisegundos
 */
    xQueueSend(sensor_queue, &sensor_state, 1000 / portTICK_PERIOD_MS);
    ESP_LOGI(ESP_TAG,"Sensor Real Value: %d", sensor_state);
    xQueueSend(potenciometer_queue, &potenciometer_state, 1000 / portTICK_PERIOD_MS);
    ESP_LOGI(ESP_TAG,"Potenciometer Real Value: %d", potenciometer_state);

vTaskDelay(2000 / portTICK_PERIOD_MS);
 }
}

/**
 * @brief Lee los valores de los sensores y envia los valores leidos a
 *        un servidor MQTT.
 * La tarea lee el valor del sensor digital en la cola sensor_queue y el valor del
 * potenciometro en la cola potenciometer_queue, y envia este valor a un servidor
 * MQTT con el topico especificado en la variable global topic.
 * @param pvParameters No se utiliza
 */
void mqtt_send_task(void* pvParameters){

int16_t Humidity = 0;
int16_t Temperature = 0;
int16_t dB = 0;

for(;;){



if(xQueueReceive(sensor_queue, &sensor_state, 1000 / portTICK_PERIOD_MS) == pdPASS){ //Funcion que recibe en orden los datos de la cola
    sprintf(payload, "{\"sensor\": %d}", sensor_state); //formatea la cadena de caracteres y la almacena en el payload
    ESP_LOGI(ESP_TAG,"Sensor Value: %d", sensor_state);
    esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), 0, 0); //Esta funcion publica la informacion por clave-valor (JSON)
}
else{ESP_LOGW(ESP_TAG,"Recibimiento fallido de sensor_state a la cola sensor_queue");}

if(xQueueReceive(potenciometer_queue, &potenciometer_state, 1000 / portTICK_PERIOD_MS) == pdPASS){
    /** @note Lo mismo de antes pero aqui se simulan datos ficticios del potenciometro y se envian de la misma forma */
    Humidity = map(potenciometer_state, 0, 4095, 0, 100);
    Temperature = map(potenciometer_state, 0, 4095, -30, 150);
    dB = map(potenciometer_state, 0, 4095, 0, 300);
    ESP_LOGI(ESP_TAG,"Potenciometer Value: %d", potenciometer_state);
    sprintf(payload, "{\"potenciometer\": %d}", potenciometer_state);
    esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), 0, 0);
    sprintf(payload, "{\"humidity\": %d}", Humidity);
    esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), 0, 0);
    sprintf(payload, "{\"temperature\": %d}", Temperature);
    esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), 0, 0);
    sprintf(payload, "{\"dB\": %d}", dB);
    esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), 0, 0);
}
else{ESP_LOGW(ESP_TAG,"Recibimiento fallido de potenciometer_state a la cola potenciometer_queue");}
vTaskDelay(2000 / portTICK_PERIOD_MS);

 }
}







/**
 * @brief Funcion principal de la aplicacion
 * Se encarga de inicializar el sistema, reiniciar en caso de fallas,
 * y lanzar las tareas necesarias para el envio de datos a un servidor MQTT.
 *
 */
void app_main(void)
{
esp_reset_reason_t reason_reset = esp_reset_reason();
    /* Configure the peripheral according to the LED type */
switch (reason_reset) //se utiliza para identificar el motivo del reinicio
{
case ESP_RST_PANIC:
    ESP_LOGW(ESP_TAG, "Reinicio Por Panico (Intentos o Errores Excesivos)");
    break;

case ESP_RST_SW:
    ESP_LOGW(ESP_TAG, "Reinicio Por Software");
    break;

case ESP_RST_PWR_GLITCH:
ESP_LOGW(ESP_TAG, "Reinicio Por Falla De Alimentacion (Caida de Voltaje)");
    break;

default:
ESP_LOGI(ESP_TAG, "Causa De Reinicio Desconocida");
    break;
}

adc1_channel_t adc = ADC1_CHANNEL_2;
adc_atten_t adc_config = ADC_ATTEN_DB_12; //configuraciones del canal 1 del ADC
adc_bits_width_t adc_width = ADC_WIDTH_BIT_12;

adc1_config_channel_atten(adc, adc_config);


ESP_LOGI(ESP_TAG, "Intentando Iniciar Memoria No Volatil (nvs)");
esp_err_t req = nvs_flash_init();
if (req == ESP_ERR_NVS_NOT_INITIALIZED || req == ESP_ERR_NVS_NO_FREE_PAGES) { //control para inicializar la memoria no volatil
ESP_LOGI(ESP_TAG, "Intento Fallido, Reacondicionando Memoria No Volatil (nvs)");    
ESP_ERROR_CHECK(nvs_flash_erase());
ESP_ERROR_CHECK(nvs_flash_init());
ESP_LOGI(ESP_TAG, "Memoria No Volatil (nvs) Iniciada Correctamente");
}
else{ESP_LOGI(ESP_TAG, "Memoria No Volatil (nvs) Iniciada Correctamente");}


ESP_LOGI(WIFI_TAG, "Inicializando WiFi...");
for(uint8_t i = 0; wifi_default_connect() != ESP_OK; i++){ //sistema de reintentos de conexion Wifi
ESP_LOGI(WIFI_TAG, "Intento Fallido, Reintentando Conexion Con WiFi...");    
vTaskDelay(100 / portTICK_PERIOD_MS);
esp_wifi_deinit();
if(i >= 10){
ESP_LOGE(WIFI_TAG, "Reintentos de Inicializacion Excedidos, Reiniciando el Sistema...");
esp_restart();    
}
}
vTaskDelay(10000 / portTICK_PERIOD_MS);

for(uint8_t i = 0; mqtt_client_launch() != ESP_OK; i++){ //sistema de reintentos de lanzamiento del cliente mqtt
vTaskDelay(100 / portTICK_PERIOD_MS);
esp_wifi_deinit();
if(i >= 5){
ESP_LOGE(WIFI_TAG, "Reintentos de Inicializacion Excedidos, Reiniciando el Sistema...");
esp_restart();    
}
}

/** @brief Creacion de las colas de datos
 * 
 * @param 5 = representa la cantidad de espacios de almacenamiento
 * @param sizeof(uint8_t) = representa el tamano individual de cada espacios de almacenamiento, en este caso 1 byte
 */

sensor_queue = xQueueCreate(5, sizeof(uint8_t)); 
potenciometer_queue = xQueueCreate(5, sizeof(uint32_t));


/** @brief Creacion de las tareas manejadas por FreeRTOS
 * 
 * @param sensor_task = funcion que la tarea se ejecutara en bucle
 * @param "sensor_task" = nombre de la tarea
 * @param 4*1024 = tamano de la pila de la tarea (usado para la cantidad de informacion que una tarea puede manejar y almacenar de forma aislada)
 * @param NULL = Aqui no se usa, pero permite pasar datos aislados a las tareas de forma modular, favoreciendo el uso de estructuras
 * @param 5 = prioridad de la tarea, la prioridad distribuye los recursos y las interrupciones de tareas ante otras mas importantes
 * @param NULL = Permite ingresar un manejador de la tarea para realizar seguimiento y control mas profundo
 * 
 * @note Apenas en este punto todo el codigo visto anteriormente se pone en marcha, por eso se inicializan las colas de datos aqui abajo
 * a pesar de ser llamadas arriba en las tareas
 */

xTaskCreate(sensor_task, "sensor_task", 4*1024, NULL, 5, NULL);
xTaskCreate(mqtt_send_task, "mqtt_task", 8*1024, NULL, 5, NULL);

}
