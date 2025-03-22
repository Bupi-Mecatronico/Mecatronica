#include <stdio.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#define PID1_Muestreo 5

TimerHandle_t get_pid_reference;
TimerHandle_t get_pid_output;
TimerHandle_t PID_Output_timer;

typedef struct {
float error;
float past_error;
float output_return;
float reference;
float Kp;
float Ki;
float Kd;
float Proportional;
float Integral;
float Derivative;
int Timer;
int u_system;
float filtered_derivative;
float filtered_reference;
float alpha;
const char* sensor_name;
}PID_Struct;


void PID_Initializing(PID_Struct *pid, float reference, float P_earning, float I_earning, float D_earning, int timer_interrupt, const char* name_sensor){
    pid->reference = reference;
    pid->Kp = P_earning;
    pid->Ki = I_earning;
    pid->Kd = D_earning;
    pid->Timer = timer_interrupt;
    pid->output_return = 0.0f;
    pid->Proportional = 0.0f;
    pid->Integral = 0.0f;
    pid->Derivative = 0.0f;
    pid->error = 0.0f;
    pid->past_error = 0.0f;
    pid->u_system = 0.0f;
    pid->sensor_name = name_sensor;
    pid->filtered_derivative = 0.0f;
    pid->filtered_reference = 0.0f;
}

float first_order_filter(PID_Struct *pid, float input, float filtered_value){
filtered_value = pid->alpha * input + (1 - pid->alpha) * filtered_value;
return filtered_value;
}

void PID_Output(PID_Struct *pid){
pid->filtered_reference = first_order_filter(pid, pid->reference, pid->filtered_reference);    
pid->error = pid->filtered_reference - pid->output_return;
pid->Proportional = pid->Kp*pid->error;
pid->Integral += pid->error * (pid->Timer / 1000.0f);
pid->Derivative = (pid->error - pid->past_error)/(pid->Timer / 1000.0f);
pid->filtered_derivative = first_order_filter(pid, pid->Derivative, pid->filtered_derivative);
pid->past_error = pid->error;
pid->u_system = pid->Proportional + (pid->Ki * pid->Integral) + (pid->Kd * pid->filtered_derivative);
}


void adc_get_pid_reference(PID_Struct *pid){
    pid->reference = adc1_get_raw(ADC1_CHANNEL_6);    
}

void adc_get_pid_output(PID_Struct *pid){

    static uint8_t lastStateA = 0;
    
    uint8_t stateA = gpio_get_level(GPIO_NUM_4);
    uint8_t stateB = gpio_get_level(GPIO_NUM_2);

    if (stateA != lastStateA) {  // Si hubo cambio en A
        if (stateA == stateB) {
            pid->output_return++;  // Dirección 1
        } else {
            pid->output_return--;  // Dirección opuesta
        }
    }
    lastStateA = stateA;  // Guardar estado anterior
}

void adc_get_pid_reference_callback(TimerHandle_t *xTimer){
PID_Struct* pid = (PID_Struct*)pvTimerGetTimerID(xTimer);
adc_get_pid_reference(pid);
}

void adc_get_pid_output_callback(TimerHandle_t *xTimer){
PID_Struct *pid = (PID_Struct*)pvTimerGetTimerID(xTimer);
adc_get_pid_output(pid);    
}

void PID_Output_callback(TimerHandle_t xTimer){
PID_Struct *pid = (PID_Struct*)pvTimerGetTimerID(xTimer);
PID_Output(pid);    
}

void motor_task(void *arg){
PID_Struct *internal_task_PID = (PID_Struct *)arg;

ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, internal_task_PID->u_system);
ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}



void app_main(void)
{

    

gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT);
gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);

ledc_timer_config_t pwm_timer_cfg = {
    .duty_resolution = LEDC_TIMER_12_BIT,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .freq_hz = 5000,
    .timer_num = LEDC_TIMER_0,
    .clk_cfg = LEDC_AUTO_CLK

};

ledc_timer_config(&pwm_timer_cfg);

ledc_channel_config_t pwm_channel_cfg = {
    .channel = LEDC_CHANNEL_0,
    .duty = 0,
    .gpio_num = GPIO_NUM_32,
    .intr_type = LEDC_INTR_DISABLE,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_sel = LEDC_TIMER_0,
    .hpoint = 0
};

ledc_channel_config(&pwm_channel_cfg);

adc1_config_width(ADC_WIDTH_BIT_12);


PID_Struct Motor1;
PID_Initializing(&Motor1, adc1_get_raw(ADC_CHANNEL_6), 3.0f, 0.001f, 0.01f, 5, "Enconder Cuadratura"); //falta completar argumentos

get_pid_reference = xTimerCreate("pid_reference", 
                                20 / portTICK_PERIOD_MS, 
                                pdTRUE,
                                (void*)&Motor1,
                                adc_get_pid_reference_callback);

get_pid_output = xTimerCreate("pid_output_read", 
                            PID1_Muestreo / portTICK_PERIOD_MS, 
                            pdTRUE, 
                            (void*)&Motor1, 
                            adc_get_pid_output_callback);                                

PID_Output_timer = xTimerCreate("pid_output", 
                                PID1_Muestreo / portTICK_PERIOD_MS, 
                                pdTRUE, 
                                (void*)&Motor1, 
                                PID_Output_callback);                                
    

xTimerStart(get_pid_reference, 0);
xTimerStart(get_pid_output, 0);
xTimerStart(PID_Output_timer, 0);

xTaskCreate(motor_task,"motor1_task", 1024,&Motor1,5,NULL);
}
