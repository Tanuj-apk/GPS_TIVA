#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"

// SysTick ISR
void SysTick_Handler(void);

void UART6_Init(void)
{
    // Enable UART6 and GPIOP
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART6));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));

    // Configure pins PP0 = U6RX, PP1 = U6TX
    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);

    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure UART6: 115200 baud, 8N1
    UARTConfigSetExpClk(UART6_BASE,
                        SysCtlClockGet(),
                        115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}

void SysTick_Init(void)
{
    uint32_t sysclk = SysCtlClockGet();

    SysTickPeriodSet(sysclk);     // 1 second period
    SysTickIntEnable();
    SysTickEnable();
}

void SysTick_Handler(void)
{
    const char msg[] = "Hi Hello\n";
    int i;
    for ( i = 0; msg[i] != 0; i++)
    {
        UARTCharPut(UART6_BASE, msg[i]);
    }
}

int main(void)
{
    // Set system clock to 120 MHz using PLL
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                        SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL |
                        SYSCTL_CFG_VCO_480), 120000000);

    UART6_Init();
    SysTick_Init();

    while (1)
    {
        // Idle — UART TX handled by SysTick interrupt
    }
}
