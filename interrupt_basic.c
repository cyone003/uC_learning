#include "stm32f4xx.h"
// Bare-metal register definitions
#define RCC_AHB1ENR     (*(volatile unsigned int*)0x40023830)
#define RCC_APB2ENR     (*(volatile unsigned int*)0x40023844)

#define GPIOA_MODER     (*(volatile unsigned int*)0x40020000)
#define GPIOA_ODR       (*(volatile unsigned int*)0x40020014)

#define SYSCFG_EXTICR1  (*(volatile unsigned int*)0x40013808)

#define EXTI_IMR        (*(volatile unsigned int*)0x40013C00)
#define EXTI_FTSR       (*(volatile unsigned int*)0x40013C0C)
#define EXTI_PR         (*(volatile unsigned int*)0x40013C14)

#define NVIC_ISER0      (*(volatile unsigned int*)0xE000E100) // IRQ 0–31

// --- GPIO and Interrupt Setup ---

void gpio_init(void) {
    // Enable GPIOA clock
    RCC_AHB1ENR |= (1 << 0);  // GPIOAEN
    RCC_AHB1ENR |= (1 << 2);  // GPIOAEN

    // Set PA6 as output
    GPIOA_MODER &= ~(0x3 << (6 * 2));  // Clear MODER6
    GPIOA_MODER |=  (0x1<< (6 * 2));  // Set MODER6 to output (01)

    GPIOA->ODR |= (1 << 6);

}

void exti_init(void) {
    // Enable SYSCFG clock
    RCC_APB2ENR |= (1 << 14);  // SYSCFGEN

    // Map EXTI1 to PA1 (EXTICR1, bits 7:4 = 0x0 for Port A)
    SYSCFG->EXTICR[3] &= ~(0xF << 4);  // Clear bits
    SYSCFG->EXTICR[3] |=  (0x2 << 4);  // 0x2 = Port C

    // Unmask EXTI1 line
    EXTI_IMR |= (1 << 13);

    // Trigger interrupt on falling edge of PA1
    EXTI->FTSR |= (1 << 13);
    EXTI->RTSR &= ~(1 << 13);
    // Enable IRQ7 (EXTI1_IRQn) in NVIC
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// --- Interrupt Handler ---

void EXTI15_10_IRQHandler(void) { //the name matters when using this irq handler function, the hardware needs to know what function to use so we can use our own name
    if (EXTI->PR & (1 << 13)) {
        EXTI->PR |= (1 << 13);      // Clear interrupt pending bit
        GPIOA_ODR &= ~(1 << 6);    // Set PA6 HIGH → LED OFF
    }
}

// --- Main Program ---

int main(void) {
    gpio_init();
    exti_init();

    while (1) {
        // Wait for interrupt to turn off the LED
    }
}
