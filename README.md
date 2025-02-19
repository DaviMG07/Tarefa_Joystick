# Projeto BitDogLab – Controle de LEDs e Display OLED com Joystick

**Aluno: [Davi Macêdo Gomes]**

Este projeto integra diversas funcionalidades do RP2040 para demonstrar o uso dos conversores analógico-digitais (ADC), do PWM e do protocolo I2C na placa BitDogLab. O código controla a intensidade de dois LEDs RGB (vermelho e azul) via PWM, de acordo com os valores lidos de um joystick, e representa graficamente a posição do joystick em um display OLED SSD1306. Adicionalmente, utiliza rotinas de interrupção com debouncing para tratar os botões e implementar funcionalidades extras.

---

## Objetivos e Requisitos da Tarefa

- **Leitura dos Valores do Joystick (ADC):**  
  - **Objetivo:** Capturar os valores analógicos dos eixos X e Y do joystick.  
  - **Implementação:**  
    - O ADC do RP2040 é inicializado e configurado para os pinos do joystick (GPIO 26 para o eixo X e GPIO 27 para o eixo Y).  
  - **Código:**  
    ```c
    adc_init();
    adc_gpio_init(JOY_X_PIN);
    adc_gpio_init(JOY_Y_PIN);
    ```

- **Controle dos LEDs RGB via PWM:**  
  - **Objetivo:** Ajustar a intensidade luminosa dos LEDs vermelho e azul conforme os valores do joystick.  
  - **Implementação:**  
    - São configurados dois canais PWM para os LEDs (GPIO 13 para o LED vermelho e GPIO 12 para o LED azul) com frequência definida (5000 Hz).  
    - A intensidade PWM é calculada a partir da diferença entre o valor central (2048) e a leitura ADC, permitindo uma variação suave do brilho.  
  - **Código:**  
    ```c
    initializePWM(LED_RED_PIN, PWM_FREQUENCY);
    initializePWM(LED_BLUE_PIN, PWM_FREQUENCY);
    
    // Cálculo da intensidade PWM com base no ADC
    uint8_t pwmRed = computePWMValue(adcY);
    uint8_t pwmBlue = computePWMValue(adcX);
    pwm_set_gpio_level(LED_RED_PIN, pwmRed);
    pwm_set_gpio_level(LED_BLUE_PIN, pwmBlue);
    ```

- **Exibição da Posição do Joystick no Display OLED:**  
  - **Objetivo:** Representar graficamente a posição atual do joystick utilizando um quadrado de 8x8 pixels.  
  - **Implementação:**  
    - O barramento I2C é inicializado (GPIO 14 e 15) para comunicação com o display OLED (endereço 0x3C).
    - O display é configurado e, em cada iteração do loop principal, a posição do cursor é calculada e renderizada com base nas leituras dos eixos X e Y.  
  - **Código:**  
    ```c
    setup_i2c();
    setupOLED(&display);
    
    // No loop principal:
    calculateCursorPos(adcX, adcY, &cursorX, &cursorY);
    renderOLED(&display, cursorX, cursorY, colorFlag);
    ```

- **Uso de Interrupções e Debouncing para Botões:**  
  - **Objetivo:** Implementar a resposta imediata aos acionamentos dos botões e evitar múltiplas detecções causadas pelo bouncing.  
  - **Implementação:**  
    - Os botões (joystick – GPIO 22, BTN_A – GPIO 5 e BTN_B – GPIO 6) são configurados com rotinas de interrupção.
    - Um mecanismo de debounce (300 ms) é aplicado para garantir a estabilidade das leituras.  
  - **Funcionalidades dos Botões:**  
    - **Botão do Joystick (GPIO 22):** Alterna o estado do LED verde e muda o estilo da borda do display.  
    - **Botão A (GPIO 5):** Habilita ou desabilita o controle PWM dos LEDs, garantindo que estes fiquem apagados quando desabilitados.  
    - **Botão B (GPIO 6):** Reinicia o sistema entrando no modo USB Boot.  
  - **Código:**  
    ```c
    // Configuração dos botões com interrupção e debounce
    initializeButton(JOY_BTN_PIN);
    initializeButton(BTN_A_PIN);
    initializeButton(BTN_B_PIN);
    
    // Função de callback para tratamento dos botões:
    void onButtonPress(uint gpio, uint32_t events) {
        // Verifica debounce e executa ações:
        if (gpio == JOY_BTN_PIN) {
            gpio_put(LED_GREEN_PIN, !gpio_get(LED_GREEN_PIN));
            showFrame = !showFrame;
        } else if (gpio == BTN_B_PIN) {
            reset_usb_boot(0, 0);
        } else if (gpio == BTN_A_PIN) {
            ledPWMEnabled = !ledPWMEnabled;
        }
    }
    ```

- **Atualização Contínua do Sistema:**  
  - **Objetivo:** Manter a aquisição dos dados do joystick, atualização dos LEDs e do display de forma contínua e suave.  
  - **Implementação:**  
    - O loop principal do programa realiza a leitura dos valores ADC, o cálculo da posição do cursor e a atualização dos LEDs e do display OLED a cada 10 ms, garantindo uma animação fluida.  
  - **Código:**  
    ```c
    while (true) {
        acquireJoystickData(&adcX, &adcY);
        calculateCursorPos(adcX, adcY, &cursorX, &cursorY);
        refreshLEDs(adcX, adcY);
        renderOLED(&display, cursorX, cursorY, colorFlag);
        sleep_ms(10);
    }
    ```

---

## Como Executar o Projeto

1. **Configuração do Ambiente Pico SDK:**  
   - Certifique-se de ter o ambiente de desenvolvimento do Pico SDK devidamente configurado, seguindo as instruções oficiais da Raspberry Pi Foundation.

2. **Compilação e Upload do Firmware:**  
   - Utilize o Visual Studio Code (ou outro editor de sua preferência) para compilar o código.
   - Faça o upload do firmware gerado para a placa BitDogLab.

3. **Verificação das Funcionalidades:**  
   - **Joystick:** Movimente o joystick para observar a variação da intensidade dos LEDs e o deslocamento do cursor no display OLED.
   - **Botões:**  
     - Pressione o botão do joystick para alternar o LED verde e o estilo da borda do display.
     - Utilize o botão A para ativar/desativar os LEDs controlados via PWM.
     - Pressione o botão B para reiniciar o microcontrolador no modo USB Boot.

---

## Considerações Finais

Este projeto demonstra a integração dos principais periféricos do RP2040, incluindo ADC, PWM e comunicação I2C, com uma aplicação prática utilizando a placa BitDogLab. Ao implementar o controle de LEDs e a renderização gráfica em um display OLED, o projeto consolida conceitos importantes de leitura analógica, controle por PWM e manipulação de interfaces gráficas, servindo como uma base para futuras expansões e experimentos.


