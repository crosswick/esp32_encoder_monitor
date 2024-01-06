#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define GPIO_INPUT_IO_1     12
#define GPIO_INPUT_IO_2     13
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))
#define ESP_INTR_FLAG_DEFAULT 0

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

QueueHandle_t encoderQueue;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    int MSB = gpio_get_level(GPIO_INPUT_IO_1); //MSB = most significant bit
    int LSB = gpio_get_level(GPIO_INPUT_IO_2); //LSB = least significant bit
    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

    lastEncoded = encoded; //store this value for next time

    // Send the new encoder value to the queue
    xQueueSendFromISR(encoderQueue, &encoderValue, NULL);
}

void encoderTask(void *pvParameter)
{
    long value;
    while(1) {
        if(xQueueReceive(encoderQueue, &value, portMAX_DELAY)) {
            printf("Encoder Value: %ld\n", value);
        }
    }
}

void app_main(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1; // Disable pull-up as we are using external pull-ups
    io_conf.pull_down_en = 0; // Disable pull-down
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, NULL);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, NULL);

    // Create the queue
    encoderQueue = xQueueCreate(10, sizeof(long));

    // Create the task that will handle the encoder values
    xTaskCreate(encoderTask, "encoderTask", 2048, NULL, 1, NULL);
}
