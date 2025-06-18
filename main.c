#include "stm32f1xx.h"
#include <stdio.h>
#include <string.h>

#define MQ2_ADC_CH     5
#define RGB_R_PIN      0
#define RGB_G_PIN      1
#define RGB_B_PIN      10
#define BUZZER_PIN     2
#define SW1_PIN        6
#define SW2_PIN        7
#define RELAY_LED_PIN  3

#define GAS_NONE       200
#define GAS_LOW        1000
#define GAS_HIGH       3000

volatile uint32_t msTicks = 0;
volatile uint8_t system_active = 1;
volatile uint8_t warn_state = 0;
volatile uint8_t blink_state = 0;
volatile uint16_t freeze_adc = 0;

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        msTicks++;
    }
}

void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = (SystemCoreClock / 1000000) - 1;
    TIM2->ARR = 1000 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
}

void delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) { ; }
}

void delay_us(uint32_t us) {
    uint32_t count = (SystemCoreClock / 1000000) * us / 5;
    while (count--) __NOP();
}

#define LCD_ADDR   0x27
#define BL         0x08
#define EN         0x04
#define RS         0x01

void I2C1_GPIO_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    GPIOB->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->CRL |=  ((0xB << (6 * 4)) | (0xB << (7 * 4)));
}

void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 8;
    I2C1->CCR = 40;
    I2C1->TRISE = 9;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_Write(uint8_t data) {
    while (I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = (LCD_ADDR << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

void LCD_Write4(uint8_t data) {
    I2C1_Write(data | BL);
    I2C1_Write(data | EN  | BL);
    delay_us(1);
    I2C1_Write((data & ~EN) | BL);
    delay_us(50);
}

void LCD_Cmd(uint8_t cmd) {
    LCD_Write4(cmd & 0xF0);
    LCD_Write4((cmd << 4) & 0xF0);
    delay_ms(2);
}

void LCD_Data(uint8_t data) {
    LCD_Write4((data & 0xF0) | RS);
    LCD_Write4(((data << 4) & 0xF0) | RS);
    delay_ms(1);
}

void LCD_Init(void) {
    delay_ms(50);
    LCD_Write4(0x30); delay_ms(5);
    LCD_Write4(0x30); delay_ms(1);
    LCD_Write4(0x30); LCD_Write4(0x20);
    LCD_Cmd(0x28);
    LCD_Cmd(0x08);
    LCD_Cmd(0x01); delay_ms(2);
    LCD_Cmd(0x06);
    LCD_Cmd(0x0C);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    LCD_Cmd(0x80 | (row ? 0x40 + col : col));
}

void LCD_Print(char *s) {
    while (*s)
        LCD_Data(*s++);
}

void LCD_Clear(void) {
    LCD_Cmd(0x01);
    delay_ms(2);
}

void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
    GPIOA->CRL &= ~(0xF << (MQ2_ADC_CH * 4));
    ADC1->SMPR2 |= (7 << (3 * MQ2_ADC_CH));
    ADC1->SQR3 = MQ2_ADC_CH;
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_ms(1);
}

uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_ADON;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void RGB_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    GPIOA->CRL &= ~(0xFF << (RGB_R_PIN * 4));
    GPIOA->CRL |=  (0x22 << (RGB_R_PIN * 4));
    GPIOB->CRH &= ~(0xF << ( (RGB_B_PIN - 8) * 4 ));
    GPIOB->CRH |=  (0x2 << ( (RGB_B_PIN - 8) * 4 ));
    GPIOA->BSRR = (1 << RGB_R_PIN) | (1 << RGB_G_PIN);
    GPIOB->BSRR = (1 << RGB_B_PIN);
}

void RGB_Set(uint8_t r, uint8_t g, uint8_t b) {
    if (r)
        GPIOA->BSRR = (1 << RGB_R_PIN);
    else
        GPIOA->BRR = (1 << RGB_R_PIN);

    if (g)
        GPIOA->BSRR = (1 << RGB_G_PIN);
    else
        GPIOA->BRR = (1 << RGB_G_PIN);

    if (b)
        GPIOB->BSRR = (1 << RGB_B_PIN);
    else
        GPIOB->BRR = (1 << RGB_B_PIN);
}

void Buzzer_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~(0xF << (BUZZER_PIN * 4));
    GPIOA->CRL |=  (0x2 << (BUZZER_PIN * 4));
    GPIOA->BSRR = (1 << BUZZER_PIN);
}

void Buzzer_Set(uint8_t on) {
    if (on)
        GPIOA->BRR = (1 << BUZZER_PIN);
    else
        GPIOA->BSRR = (1 << BUZZER_PIN);
}

void Buttons_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~(0xF << (SW1_PIN * 4));
    GPIOA->CRL |=  (0x8 << (SW1_PIN * 4));
    GPIOA->ODR |=  (1 << SW1_PIN);
    GPIOA->CRL &= ~(0xF << (SW2_PIN * 4));
    GPIOA->CRL |=  (0x8 << (SW2_PIN * 4));
    GPIOA->ODR |=  (1 << SW2_PIN);
}

void RelayLED_Init(void) {
    GPIOA->CRL &= ~(0xF << (RELAY_LED_PIN * 4));
    GPIOA->CRL |=  (0x1 << (RELAY_LED_PIN * 4));
    GPIOA->BRR = (1 << RELAY_LED_PIN);
}

int main(void) {
    uint16_t adc_val;
    char lcd_str[16];
    uint32_t lastBlinkTime = 0;
    uint32_t currentTime, blinkPeriod;

    SystemCoreClockUpdate();
    TIM2_Init();
    I2C1_GPIO_Init();
    I2C1_Init();
    LCD_Init();
    ADC1_Init();
    RGB_Init();
    Buzzer_Init();
    Buttons_Init();
    RelayLED_Init();

    freeze_adc = ADC1_Read();
    LCD_Clear();
    LCD_SetCursor(0, 1);
    LCD_Print("System Init");
    LCD_SetCursor(1, 1);
    LCD_Print("Please Wait");
    delay_ms(500);
    LCD_Clear();

    while (1) {
        if ((GPIOA->IDR & (1 << SW2_PIN)) == 0) {
            system_active = 1;
            warn_state    = 0;
            blink_state   = 0;
            freeze_adc    = 0;

            LCD_Clear();
            LCD_SetCursor(0, 1);
            LCD_Print("System Reset");
            LCD_SetCursor(1, 2);
            LCD_Print("Please Wait");

            delay_ms(500);

            freeze_adc = ADC1_Read();
            LCD_Clear();
            continue;
        }

        if ((GPIOA->IDR & (1 << SW1_PIN)) == 0) {
            system_active = !system_active;
            if (system_active)
                freeze_adc = ADC1_Read();
            delay_ms(200);
        }

        if (system_active) {
            adc_val = ADC1_Read();
            freeze_adc = adc_val;
        } else {
            adc_val = freeze_adc;
        }

        if (adc_val < GAS_NONE)
            warn_state = 0;
        else if (adc_val < GAS_LOW)
            warn_state = 1;
        else if (adc_val < GAS_HIGH)
            warn_state = 2;
        else
            warn_state = 3;

        snprintf(lcd_str, sizeof(lcd_str), "Gas:%4d", adc_val);
        LCD_SetCursor(0,0);
        LCD_Print(lcd_str);
        snprintf(lcd_str, sizeof(lcd_str), "Sys:%d Warn:%d", system_active, warn_state);
        LCD_SetCursor(1,0);
        LCD_Print(lcd_str);

        currentTime = msTicks;
        switch (warn_state) {
            case 0:
                RGB_Set(0, 0, 1);
                Buzzer_Set(0);
                break;
            case 1:
                RGB_Set(1, 1, 0);
                Buzzer_Set(0);
                break;
            case 2:
                blinkPeriod = 1000;
                if (currentTime - lastBlinkTime >= blinkPeriod / 2) {
                    blink_state = !blink_state;
                    lastBlinkTime = currentTime;
                }
                RGB_Set(blink_state, 0, 0);
                Buzzer_Set(0);
                break;
            case 3:
            {
                uint32_t freq = 2 + ((freeze_adc - GAS_HIGH) * (10 - 2)) / (4095 - GAS_HIGH);
                if (freq > 10)
                    freq = 10;
                blinkPeriod = 1000 / freq;
                if (currentTime - lastBlinkTime >= blinkPeriod / 2) {
                    blink_state = !blink_state;
                    lastBlinkTime = currentTime;
                }
                RGB_Set(blink_state, 0, 0);
                Buzzer_Set(!blink_state);
            }
                break;
        }

        if (warn_state == 3)
            GPIOA->BSRR = (1 << RELAY_LED_PIN);
        else
            GPIOA->BRR = (1 << RELAY_LED_PIN);

        delay_ms(50);
    }
}
