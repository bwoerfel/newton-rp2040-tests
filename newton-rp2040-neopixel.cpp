#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"

#include "Adafruit_NeoPixel.hpp"
#include "PwmIn.h"

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// GPIO defines
// Example uses GPIO 2
#define GPIO 2
#define NUMPIXELS 1
#define NEOPIXEL_PIN 21
#define NEOPIXEL_POWER 20

//#include "blink.pio.h"
#include "ppm.pio.h"
#include "PwmIn.pio.h"
#define NUM_OF_PINS 2

//Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/*
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
*/


int64_t alarm_callback(alarm_id_t id, __unused void *user_data) {
    printf("Timer %d fired!\n", (int) id);
    
    // Can return a value here in us to fire in the future
    return 0;
}

void set_value_and_log(uint channel, uint value_usec) {
    //printf("Channel %d is set to value %5.3f ms\r\n", channel, (float)value_usec / 1000.0f);
    ppm_set_value(channel, value_usec);

}


int main()
{
    stdio_init_all();
    sleep_ms(500);
    printf("##### ##### #####\n");

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart

    // GPIO initialisation.
    // We will make this GPIO an input, and pull it up by default
    gpio_init(GPIO);
    gpio_set_dir(GPIO, GPIO_IN);
    gpio_pull_up(GPIO);
    // See https://github.com/raspberrypi/pico-examples/tree/master/gpio for other gpio examples, including using interrupts

    // Example of using the HW divider. The pico_divider library provides a more user friendly set of APIs 
    // over the divider (and support for 64 bit divides), and of course by default regular C language integer
    // divisions are redirected thru that library, meaning you can just use C level `/` and `%` operators and
    // gain the benefits of the fast hardware divider.
    int32_t dividend = 123456;
    int32_t divisor = -321;
    // This is the recommended signed fast divider for general use.
    divmod_result_t result = hw_divider_divmod_s32(dividend, divisor);
    printf("%d/%d = %d remainder %d\n", dividend, divisor, to_quotient_s32(result), to_remainder_s32(result));
    // This is the recommended unsigned fast divider for general use.
    int32_t udividend = 123456;
    int32_t udivisor = 321;
    divmod_result_t uresult = hw_divider_divmod_u32(udividend, udivisor);
    printf("%d/%d = %d remainder %d\n", udividend, udivisor, to_quotient_u32(uresult), to_remainder_u32(uresult));
    // See https://github.com/raspberrypi/pico-examples/tree/master/divider for more complex use

    /*
    // PIO Blinking example
    PIO pio_led = pio1;
    uint offset_led = pio_add_program(pio_led, &blink_program);
    printf("Loaded program at %d\n", offset_led);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio_led, 0, offset_led, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio_led, 0, offset_led, 6, 3);
    #endif
    */
    uint ppm_pin = 4;
    ppm_program_init(pio1, ppm_pin);

    
    // set PwmIn
    uint pin_list[NUM_OF_PINS] = {14, 15};
    PwmIn my_PwmIn(pin_list, NUM_OF_PINS);  
        

    // the array with the results


    //pixels.rp2040Init(NEOPIXEL_PIN);    
    
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    // Watchdog example code
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // Whatever action you may take if a watchdog caused a reboot
    }
    
    // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(5000, 1);
    
    
    gpio_set_dir(NEOPIXEL_POWER, GPIO_OUT);
    gpio_set_drive_strength(NEOPIXEL_POWER, GPIO_DRIVE_STRENGTH_8MA);
    gpio_put(NEOPIXEL_POWER, 1);

    printf("NEOPIXEL: %u/n", PICO_DEFAULT_WS2812_PIN);

    // the instance of the PwmIn
    //

    while (true) {
        // You need to call this function at least more often than the 100ms in the enable call to prevent a reboot
        watchdog_update();
      
        //pixels.setPixelColor(0, 0xFF, 0x00, 0x80);
        //pixels.show();
        //set_value_and_log(5, 1100);
        //sleep_ms(1000);
        //set_value_and_log(5, 1150);
        //sleep_ms(1000);
                // translate pwm of input to output
        float PW_0 = my_PwmIn.read_PW(0);
        float PW_1 = my_PwmIn.read_PW(1);
        float PW_2 = my_PwmIn.read_PW(2);
        float PW_3 = my_PwmIn.read_PW(3);
        printf("PW_0=%f PW_1=%f PW_2=%f PW_3=%f\n", PW_0, PW_1, PW_2, PW_3);
        set_value_and_log(5, 1200);
        sleep_ms(100);
        

    }
}
