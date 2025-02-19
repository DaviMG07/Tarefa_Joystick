#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"

/* Definição de pinos e parâmetros */
#define I2C_PORT         i2c1
#define PIN_I2C_SDA      14
#define PIN_I2C_SCL      15
#define OLED_ADDRESS     0x3C
#define JOY_X_PIN        26 
#define JOY_Y_PIN        27  
#define JOY_BTN_PIN      22 
#define BTN_A_PIN        5 
#define BTN_B_PIN        6
#define LED_RED_PIN      13
#define LED_BLUE_PIN     12
#define LED_GREEN_PIN    11
#define PWM_FREQUENCY    5000

/* Variáveis globais para controle de estado */
static volatile uint32_t lastTimestamp = 0;
bool ledPWMEnabled = true;
bool showFrame = false;

/*
 * Inicializa o PWM em um pino específico com a frequência definida.
 */
void initializePWM(uint pin, uint freq) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint sliceIndex = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(sliceIndex, 255);
    pwm_set_clkdiv(sliceIndex, (float)48000000 / freq / 256);
    pwm_set_enabled(sliceIndex, true);
}

/*
 * Callback para tratamento de interrupções dos botões.
 * Executa debounce e realiza ações de acordo com o botão pressionado.
 */
void onButtonPress(uint gpio, uint32_t events) {
    printf("Interrupção do botão: GPIO %d\n", gpio);
    uint32_t currentTimestamp = to_us_since_boot(get_absolute_time());

    if (currentTimestamp - lastTimestamp > 300000) {  /* Debounce */
        lastTimestamp = currentTimestamp;

        if (gpio == JOY_BTN_PIN) {
            gpio_put(LED_GREEN_PIN, !gpio_get(LED_GREEN_PIN));
            showFrame = !showFrame;
        } else if (gpio == BTN_B_PIN) {
            reset_usb_boot(0, 0);  /* Reinicia no modo USB Boot */
        } else if (gpio == BTN_A_PIN) {
            ledPWMEnabled = !ledPWMEnabled;
        }

        /* Se o PWM estiver desabilitado, garante que os LEDs fiquem apagados */
        if (!ledPWMEnabled) {
            pwm_set_gpio_level(LED_RED_PIN, 0);
            pwm_set_gpio_level(LED_BLUE_PIN, 0);
        }
    }
}

/*
 * Converte a leitura do ADC em um valor de brilho PWM (0-255).
 */
uint8_t computePWMValue(uint16_t adcReading) {
    return (uint8_t)((abs(2048 - adcReading) / 2048.0) * 255);
}

/*
 * Configura um botão: inicializa, define como entrada, ativa pull-up e associa a interrupção.
 */
void initializeButton(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, &onButtonPress);
}

/*
 * Inicializa o barramento I2C e configura os pinos correspondentes.
 */
void setup_i2c(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

/*
 * Configura o display OLED usando a biblioteca ssd1306.
 */
void setupOLED(ssd1306_t *display) {
    ssd1306_init(display, WIDTH, HEIGHT, false, OLED_ADDRESS, I2C_PORT);
    ssd1306_config(display);
    ssd1306_send_data(display);
    ssd1306_fill(display, false);
    ssd1306_send_data(display);
}

/*
 * Inicializa o módulo ADC e os pinos para o joystick.
 */
void initializeADC(void) {
    adc_init();
    adc_gpio_init(JOY_X_PIN);
    adc_gpio_init(JOY_Y_PIN);
}

/*
 * Realiza a leitura dos valores ADC do joystick.
 */
void acquireJoystickData(uint16_t *adcX, uint16_t *adcY) {
    adc_select_input(0);
    *adcX = adc_read();
    adc_select_input(1);
    *adcY = adc_read();
}

/*
 * Calcula a posição do cursor no display com base nas leituras do joystick.
 */
void calculateCursorPos(uint16_t adcX, uint16_t adcY, float *cursorX, float *cursorY) {
    float targetX = 64 - ((float)adcX / 4095.0f) * 64;
    float targetY = ((float)adcY / 4095.0f) * 128;

    *cursorX = (targetX < 0) ? 0 : (targetX > 56 ? 56 : targetX);
    *cursorY = (targetY < 0) ? 0 : (targetY > 120 ? 120 : targetY);
}

/*
 * Atualiza os LEDs com base nas leituras do joystick.
 */
void refreshLEDs(uint16_t adcX, uint16_t adcY) {
    if (ledPWMEnabled) {
        uint8_t pwmRed = computePWMValue(adcY);
        uint8_t pwmBlue = computePWMValue(adcX);
        pwm_set_gpio_level(LED_RED_PIN, pwmRed);
        pwm_set_gpio_level(LED_BLUE_PIN, pwmBlue);

        if (adcX == 2048) {
            pwm_set_gpio_level(LED_RED_PIN, 0);
        }
        if (adcY == 2048) {
            pwm_set_gpio_level(LED_BLUE_PIN, 0);
        }
    } else {
        pwm_set_gpio_level(LED_RED_PIN, 0);
        pwm_set_gpio_level(LED_BLUE_PIN, 0);
    }
}

/*
 * Renderiza o display OLED: atualiza o fundo, desenha o contorno e o cursor.
 */
void renderOLED(ssd1306_t *display, float cursorX, float cursorY, bool colorFlag) {
    ssd1306_fill(display, !colorFlag);

    if (showFrame) {
        ssd1306_rect(display, 0, 0, 128, 64, colorFlag, !colorFlag);
    } else {
        ssd1306_rect(display, 0, 0, 128, 64, !colorFlag, colorFlag);
    }
    ssd1306_rect(display, (int)cursorX, (int)cursorY, 8, 8, colorFlag, colorFlag);
    ssd1306_send_data(display);
}

int main(void) {
    float cursorX = 0.0f, cursorY = 0.0f;
    uint16_t adcX = 0, adcY = 0;
    bool colorFlag = true;
    ssd1306_t display;

    /* Configura o PWM para os LEDs vermelho e azul */
    initializePWM(LED_RED_PIN, PWM_FREQUENCY);
    initializePWM(LED_BLUE_PIN, PWM_FREQUENCY);

    /* Configura o LED verde como saída */
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);

    /* Configura os botões com interrupções */
    initializeButton(BTN_B_PIN);
    initializeButton(JOY_BTN_PIN);
    initializeButton(BTN_A_PIN);

    /* Inicializa o barramento I2C e o display OLED */
    setup_i2c();
    setupOLED(&display);

    /* Inicializa o ADC para o joystick */
    initializeADC();

    while (true) {
        /* Adquire os valores do joystick */
        acquireJoystickData(&adcX, &adcY);

        /* Calcula a posição do cursor com base nas leituras */
        calculateCursorPos(adcX, adcY, &cursorX, &cursorY);

        /* Atualiza os LEDs de acordo com os valores do joystick */
        refreshLEDs(adcX, adcY);

        /* Renderiza o display OLED com o contorno e o cursor */
        renderOLED(&display, cursorX, cursorY, colorFlag);

        sleep_ms(10);  /* Delay para suavizar a animação */
    }

    return 0;
}

