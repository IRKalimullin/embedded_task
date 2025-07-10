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

//Макросы для выбора конфигурации канала 
#define LED_PIN_NUM(n) CONFIG_LED_##n##_GPIO 
#define LOCK_PIN_NUM(n) CONFIG_LOCK_##n##_GPIO

int led_pins[] = {
    LED_PIN_NUM(1),
    LED_PIN_NUM(2),
    LED_PIN_NUM(3),
    LED_PIN_NUM(4),
    LED_PIN_NUM(5),
    LED_PIN_NUM(6),
    LED_PIN_NUM(7),
    LED_PIN_NUM(8),
    LED_PIN_NUM(9),
    LED_PIN_NUM(10),
    LED_PIN_NUM(11),
    LED_PIN_NUM(12),
    LED_PIN_NUM(13),
    LED_PIN_NUM(14),
    LED_PIN_NUM(15),
    LED_PIN_NUM(16),
    LED_PIN_NUM(17),
    LED_PIN_NUM(18),
    LED_PIN_NUM(19),
    LED_PIN_NUM(20),
    LED_PIN_NUM(21),
    LED_PIN_NUM(22),
    LED_PIN_NUM(23),
    LED_PIN_NUM(24),
    LED_PIN_NUM(25)
    };

int lock_pins[] = {
    LOCK_PIN_NUM(1),
    LOCK_PIN_NUM(2),
    LOCK_PIN_NUM(3),
    LOCK_PIN_NUM(4),
    LOCK_PIN_NUM(5),
    LOCK_PIN_NUM(6),
    LOCK_PIN_NUM(7),
    LOCK_PIN_NUM(8),
    LOCK_PIN_NUM(9),
    LOCK_PIN_NUM(10),
    LOCK_PIN_NUM(11),
    LOCK_PIN_NUM(12),
    LOCK_PIN_NUM(13),
    LOCK_PIN_NUM(14),
    LOCK_PIN_NUM(15),
    LOCK_PIN_NUM(16),
    LOCK_PIN_NUM(17),
    LOCK_PIN_NUM(18),
    LOCK_PIN_NUM(19),
    LOCK_PIN_NUM(20),
    LOCK_PIN_NUM(21),
    LOCK_PIN_NUM(22),
    LOCK_PIN_NUM(23),
    LOCK_PIN_NUM(24),
    LOCK_PIN_NUM(25)
    };

#define BUF_SIZE (1024)

TimerHandle_t _timer_lock = NULL; // Таймер для работы замка
TimerHandle_t _timer_flash = NULL; // Таймер для работы светодиода

//Структура для передачи параметров из задачи приема команды в задачу, обрабатывающую каналы
struct channel_params
{   
    uint32_t package_id;
    uint8_t channel_id;
    uint8_t command;
    uint16_t unlock_time; 
    uint16_t flash_time;
    bool channel_permission; // флаг разрешения работы канала
    bool flash_state; // Состояние светодиода для каждого канала
};

// Массив структур для хранения параметров для каждого канала
struct channel_params ch_parameters[sizeof(lock_pins) / sizeof(lock_pins[0])]; 

// Массив флагов создания задачи для каждого канала
bool channels_created_flags[sizeof(lock_pins) / sizeof(lock_pins[0])]; 

uint8_t recData[14]; // Массив для хранения принятой команды

uint8_t max_number_channel = sizeof(lock_pins) / sizeof(lock_pins[0]); // количество имеющихся каналов для управления


static void configureGPIO(void)
{
    for (int i = 0; i < max_number_channel; i++){
        gpio_reset_pin(led_pins[i]);
        gpio_set_direction(led_pins[i], GPIO_MODE_OUTPUT);
        gpio_reset_pin(lock_pins[i]);
        gpio_set_direction(lock_pins[i], GPIO_MODE_OUTPUT);
    }
}

// После срабатывания таймера обнуляется флаг для работы канала
void timerLockCallback(TimerHandle_t xTimer){
    uint8_t *id = pvTimerGetTimerID(xTimer);
    
    ch_parameters[*id-1].channel_permission = 0;
}

// При срабатывании таймера меняет состояние на противоположное
void timerFlashCallback(TimerHandle_t xTimer){
    uint8_t *id = pvTimerGetTimerID(xTimer);
    ch_parameters[*id-101].flash_state = !ch_parameters[*id-101].flash_state;
}

void channelTask(void *pvParameters) {

    struct channel_params *ReceiveParam;

    ReceiveParam = (struct channel_params*) pvParameters;

    if (ReceiveParam->command == 0x02){

        uint8_t id = ReceiveParam->channel_id;
        uint8_t id_flash = ReceiveParam->channel_id + 100;

        char* timer_name = "timer_" + (char) id;
        char* timer_name_flash = "timer_flash_" + (char) id;

        // Таймер для отслеживания времени работы канала
        _timer_lock = xTimerCreate(timer_name,
            ReceiveParam->unlock_time / portTICK_PERIOD_MS,
            pdFALSE,
            &id,
            timerLockCallback
        );

        // Таймер с перезапуском для моргания светодиодом
        _timer_flash = xTimerCreate(timer_name_flash,
            ReceiveParam->flash_time / portTICK_PERIOD_MS,
            pdTRUE,
            &id_flash,
            timerFlashCallback
        );

        int led_pin = led_pins[id-1];
        int lock_pin = lock_pins[id-1];
        
        xTimerStart(_timer_lock, 0);
        xTimerStart(_timer_flash, 0);

        while (ReceiveParam->channel_permission) {
            gpio_set_level(lock_pin, 1);
            gpio_set_level(led_pin, ReceiveParam->flash_state);      
        }

        // Как выйдет время таймера (unlock_time), реле выключится и задача удалится
        gpio_set_level(led_pin, 0);
        gpio_set_level(lock_pin, 0);
        xTimerStop(_timer_flash, 1);
        xTimerDelete(_timer_flash,1);
        channels_created_flags[id-1] = 0;
        vTaskDelete(NULL);
    }
}

//Передача в структуру данные в зависимости от канала
void createChannelTask(void){

    //Проверка чтобы id канала не был больше количества каналов, для исключения неправильного включения
    if ((recData[4] >= 0x01) && (recData[4] <= max_number_channel)){ 

        uint8_t ch_index = recData[4] - 1; // Получение индекса для записи в массив параметров

        ch_parameters[ch_index].package_id = recData[0] << 24 | recData[1] << 16 | recData[2] << 8 | recData[3];
        ch_parameters[ch_index].channel_id = recData[4];
        ch_parameters[ch_index].command = recData[5];
        ch_parameters[ch_index].unlock_time = recData[6] << 8 | recData[7];
        ch_parameters[ch_index].flash_time = recData[8] << 8 | recData[9];
        ch_parameters[ch_index].channel_permission = 1;
        ch_parameters[ch_index].flash_state = 0;
        
        channels_created_flags[ch_index] = 1;

        char* task_name = "TASK CH_" + (char)recData[4];

        xTaskCreate(channelTask, task_name, 2048, (void*)&ch_parameters[ch_index], 1, NULL);
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
