#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <string.h>

#define ET_UART_TXD (CONFIG_ET_UART_TXD)
#define ET_UART_RXD (CONFIG_ET_UART_RXD)
#define ET_UART_RTS (UART_PIN_NO_CHANGE)
#define ET_UART_CTS (UART_PIN_NO_CHANGE)

#define ET_UART_PORT_NUM (CONFIG_ET_UART_PORT_NUM)
#define ET_UART_BAUD_RATE (CONFIG_ET_UART_BAUD_RATE)
#define ET_UART_TASK_STACK_SIZE (CONFIG_ET_TASK_STACK_SIZE)

#define LED_1_PIN (CONFIG_LED_1_GPIO)
#define LED_2_PIN (CONFIG_LED_2_GPIO)
#define LED_3_PIN (CONFIG_LED_3_GPIO)
#define LED_4_PIN (CONFIG_LED_4_GPIO)
#define LED_5_PIN (CONFIG_LED_5_GPIO)

#define LOCK_1_PIN (CONFIG_LOCK_1_GPIO)
#define LOCK_2_PIN (CONFIG_LOCK_2_GPIO)
#define LOCK_3_PIN (CONFIG_LOCK_3_GPIO)
#define LOCK_4_PIN (CONFIG_LOCK_4_GPIO)
#define LOCK_5_PIN (CONFIG_LOCK_5_GPIO)

#define BUF_SIZE (1024)

TimerHandle_t _timer = NULL;

//Структура для передачи параметров из задачи приема команды в задачу, обрабатывающую каналы
struct channel_params
{   
    uint32_t package_id;
    uint8_t channel_id;
    uint8_t command;
    uint16_t unlock_time; 
    uint16_t flash_time;
    bool channel_permission; // флаг разрешения работы канала
};

//Структуры для каждого канала
struct channel_params LED_1_PARAM;
struct channel_params LED_2_PARAM;
struct channel_params LED_3_PARAM;
struct channel_params LED_4_PARAM;
struct channel_params LED_5_PARAM;

//Флаги создания задачи обработки канала
bool channel_1_task_created;
bool channel_2_task_created;
bool channel_3_task_created;
bool channel_4_task_created;
bool channel_5_task_created;

uint8_t recData[14]; // Массив для хранения принятой команды

static void configureGPIO(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_1_PIN) | (1ULL << LED_2_PIN) | (1ULL << LED_3_PIN)
        | (1ULL << LED_4_PIN) | (1ULL << LED_5_PIN) | (1ULL << LOCK_1_PIN) | (1ULL << LOCK_2_PIN)
        | (1ULL << LOCK_3_PIN) | (1ULL << LOCK_4_PIN) | (1ULL << LOCK_5_PIN),
    };

    gpio_config(&io_conf);
}

// После срабатывания таймера обнуляется флаг для работы канала
void timerCallback(TimerHandle_t xTimer){
    uint8_t *id = pvTimerGetTimerID(xTimer);

    if (*id == LED_1_PARAM.channel_id) {
        LED_1_PARAM.channel_permission = 0;
    }
    if (*id == LED_2_PARAM.channel_id) {
        LED_2_PARAM.channel_permission = 0;
    }
    if (*id == LED_3_PARAM.channel_id) {
        LED_3_PARAM.channel_permission = 0;
    }
    if (*id == LED_4_PARAM.channel_id) {
        LED_4_PARAM.channel_permission = 0;
    }
    if (*id == LED_5_PARAM.channel_id) {
        LED_5_PARAM.channel_permission = 0;
    }
}

void channelTask(void *pvParameters) {

    struct channel_params *ReceiveParam;

    ReceiveParam = (struct channel_params*) pvParameters;

    if (ReceiveParam->command == 0x02){

        uint8_t id = ReceiveParam->channel_id;
        char* timer_name = "timer_" + (char) ReceiveParam->channel_id;

        // Таймер для отслеживания времени работы канала
        _timer = xTimerCreate(timer_name,
            ReceiveParam->unlock_time / portTICK_PERIOD_MS,
            pdFALSE,
            &id,
            timerCallback
        );

        int led_pin = 0;
        int lock_pin = 0;

        if (ReceiveParam->channel_id == 0x01){
            led_pin = LED_1_PIN; lock_pin = LOCK_1_PIN;
        }
        if (ReceiveParam->channel_id == 0x02){
            led_pin = LED_2_PIN; lock_pin = LOCK_2_PIN;
        }
        if (ReceiveParam->channel_id == 0x03){
            led_pin = LED_3_PIN; lock_pin = LOCK_3_PIN;
        }
        if (ReceiveParam->channel_id == 0x04){
            led_pin = LED_4_PIN; lock_pin = LOCK_4_PIN;
        }
        if (ReceiveParam->channel_id == 0x05){
            led_pin = LED_5_PIN; lock_pin = LOCK_5_PIN;
        }

        //Проверка соответствия каналов, для исключения неправильного включения
        if ((ReceiveParam->channel_id >= 0x01) && (ReceiveParam->channel_id <= 0x05)){
        
            xTimerStart(_timer, 0);

            while (ReceiveParam->channel_permission) {
                gpio_set_level(lock_pin, 1);

                gpio_set_level(led_pin, 1);
            
                vTaskDelay(ReceiveParam->flash_time / portTICK_PERIOD_MS);

                gpio_set_level(led_pin, 0);

                vTaskDelay(ReceiveParam->flash_time / portTICK_PERIOD_MS);        
            }
        }
        
        // Как выйдет время таймера (unlock_time), реле выключится и задача удалится
        gpio_set_level(lock_pin, 0);
        channel_1_task_created = 0;
        channel_2_task_created = 0;
        channel_3_task_created = 0;
        channel_4_task_created = 0;
        channel_5_task_created = 0;
        vTaskDelete(NULL);
    }
}

//Передача в структуру данные в зависимости от канала
void createChannelTask(void){
    if (recData[4] == 0x01){
        LED_1_PARAM.package_id = recData[0] << 24 | recData[1] << 16 | recData[2] << 8 | recData[3];
        LED_1_PARAM.channel_id = recData[4];
        LED_1_PARAM.command =  recData[5];
        LED_1_PARAM.unlock_time = recData[6] << 8 | recData[7];
        LED_1_PARAM.flash_time = recData[8] << 8 | recData[9];
        LED_1_PARAM.channel_permission = 1;
        channel_1_task_created = 1;
        xTaskCreate(channelTask, "CH 1 TASK", 2048, (void*)&LED_1_PARAM, 1, NULL);
    }

    if (recData[4] == 0x02){
        LED_2_PARAM.package_id = recData[0] << 24 | recData[1] << 16 | recData[2] << 8 | recData[3];
        LED_2_PARAM.channel_id = recData[4];
        LED_2_PARAM.command =  recData[5];
        LED_2_PARAM.unlock_time = recData[6] << 8 | recData[7];
        LED_2_PARAM.flash_time = recData[8] << 8 | recData[9];
        LED_2_PARAM.channel_permission = 1;
        channel_2_task_created = 1;
        xTaskCreate(channelTask, "CH 2 TASK", 2048, (void*)&LED_2_PARAM, 1, NULL);
    }
    
    if (recData[4] == 0x03){
        LED_3_PARAM.package_id = recData[0] << 24 | recData[1] << 16 | recData[2] << 8 | recData[3];
        LED_3_PARAM.channel_id = recData[4];
        LED_3_PARAM.command =  recData[5];
        LED_3_PARAM.unlock_time = recData[6] << 8 | recData[7];
        LED_3_PARAM.flash_time = recData[8] << 8 | recData[9];
        LED_3_PARAM.channel_permission = 1;
        channel_3_task_created = 1;
        xTaskCreate(channelTask, "CH 3 TASK", 2048, (void*)&LED_3_PARAM, 1, NULL);
    }
    
    if (recData[4] == 0x04){
        LED_4_PARAM.package_id = recData[0] << 24 | recData[1] << 16 | recData[2] << 8 | recData[3];
        LED_4_PARAM.channel_id = recData[4];
        LED_4_PARAM.command =  recData[5];
        LED_4_PARAM.unlock_time = recData[6] << 8 | recData[7];
        LED_4_PARAM.flash_time = recData[8] << 8 | recData[9];
        LED_4_PARAM.channel_permission = 1;
        channel_4_task_created = 1;
        xTaskCreate(channelTask, "CH 4 TASK", 2048, (void*)&LED_4_PARAM, 1, NULL);
    }
    
    if (recData[4] == 0x05){
        LED_5_PARAM.package_id = recData[0] << 24 | recData[1] << 16 | recData[2] << 8 | recData[3];
        LED_5_PARAM.channel_id = recData[4];
        LED_5_PARAM.command =  recData[5];
        LED_5_PARAM.unlock_time = recData[6] << 8 | recData[7];
        LED_5_PARAM.flash_time = recData[8] << 8 | recData[9];
        LED_5_PARAM.channel_permission = 1;
        channel_5_task_created = 1;
        xTaskCreate(channelTask, "CH 5 TASK", 2048, (void*)&LED_5_PARAM, 1, NULL);
    }

    memset(recData, 0, 14);
}

static void UART_task(void *arg)
{

    uart_config_t uart_config = {
        .baud_rate = ET_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ET_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ET_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ET_UART_PORT_NUM, ET_UART_TXD, ET_UART_RXD, ET_UART_RTS, ET_UART_CTS));

    
    uint8_t data; // Буффер для принятия байта по UART

    int count = 0; //Счетчик для заполнения массива команды

    uint8_t shield_byte_f = 0x00; // Переменная для сохранения экранирующего сивола
    uint8_t start_byte_f = 0x00; // Переменная для сохранения первого байта начала команды
    uint8_t start_byte_s = 0x00; // Переменная для сохранения второго байта начала команды
    uint8_t stop_byte = 0x00; // Переменная для сохранени первого байта окончания команды

    while (1){

        int len = uart_read_bytes(ET_UART_PORT_NUM, &data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        
        if (len > 0)  {
            
            //Сохранение полученного экранирующего символа
            if (data == 0x02){
                shield_byte_f = data;
            }

            //Проверка на начало пакета данных
            if ((data == 0x33) && (shield_byte_f != 0x02)){
                start_byte_f = data;
            }

            //Если получили последовательность 0x33 0x01, то начинаем запись команды в массив
            if ((start_byte_f == 0x33) && (start_byte_s == 0x01)){

                //Проверка на наличие байта окончания команды
                if ((data == 0x33) && (shield_byte_f != 0x02)){
                    stop_byte = data;
                }
                
                //Проверка был ли предыдущий символ экранирующим, если да - перезаписываем предыдущее значение в массиве
                if ((shield_byte_f == 0x02) && (data == 0x33)){
                    recData[count-1] = data;
                    shield_byte_f = 0x00;
                } else {
                    recData[count] = data;
                    count++;
                }

                //Проверка последовательности байт об окончании пакета данных, если равно 0x33 0x03 заканчиваем запись данных
                if ((stop_byte == 0x33) && (data == 0x03)){
                    start_byte_f = 0x00;
                    start_byte_s = 0x00;
                    createChannelTask();
                }
            }

            //Проверка последовательности принятых байт для старта записи команды в массив
            if ((start_byte_f == 0x33) && (data == 0x01)){
                start_byte_s = data;
                stop_byte = 0x00;
                count = 0;
                memset(recData, 0, 14); //Очистка массива данных, для принятия новой команды
            }
        }
    }
}

void app_main(void)
{
    configureGPIO();
    xTaskCreate(UART_task, "uart_task", ET_UART_TASK_STACK_SIZE, NULL, 10, NULL);
}
