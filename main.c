//*********************************************************************************
// UART7 Interrupt
//*********************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"

/* ------------------ Ring Buffer ------------------ */

#define UART7_BUFFER_SIZE 1024

volatile uint8_t  uart7_buffer[UART7_BUFFER_SIZE];
volatile uint16_t uart7_head = 0;
volatile uint16_t uart7_tail = 0;
volatile uint8_t  uart7_overflow = 0;

/* Push data into ring buffer (called from ISR) */
static inline void UART7_BufferPush(uint8_t data)
{
    uint16_t next = (uart7_head + 1) % UART7_BUFFER_SIZE;

    if (next != uart7_tail)
    {
        uart7_buffer[uart7_head] = data;
        uart7_head = next;
    }
    else
    {
        uart7_overflow = 1;   // Buffer full
    }
}

/* Pop data from ring buffer (called from main) */
int UART7_BufferPop(void)
{
    if (uart7_head == uart7_tail)
        return -1;

    uint8_t data = uart7_buffer[uart7_tail];
    uart7_tail = (uart7_tail + 1) % UART7_BUFFER_SIZE;

    return data;
}

/* ------------------ UART7 Interrupt Handler ------------------ */

void UART7IntHandler(void)
{
    uint32_t status = UARTIntStatus(UART7_BASE, true);
    UARTIntClear(UART7_BASE, status);

    while (UARTCharsAvail(UART7_BASE))
    {
        uint8_t c = (uint8_t)UARTCharGetNonBlocking(UART7_BASE);
        UART7_BufferPush(c);
    }
}

/* ------------------ UART7 Initialization ------------------ */

void UART7_Init(uint32_t sysClk, uint32_t baudrate)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART7));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    // PC4 -> U7RX, PC5 -> U7TX
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(UART7_BASE,
                        sysClk,
                        baudrate,
                        UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    UARTFIFOLevelSet(UART7_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);

    IntEnable(INT_UART7);
    IntMasterEnable();

    UARTEnable(UART7_BASE);
}

/* ------------------ MAIN ------------------ */

int main(void)
{
    // Set system clock to 50 MHz
    uint32_t sysClock = SysCtlClockFreqSet(
                            SYSCTL_USE_PLL |
                            SYSCTL_OSC_MAIN |
                            SYSCTL_XTAL_25MHZ |
                            SYSCTL_CFG_VCO_480,
                            50000000);

    // Initialize UART7 for GPS (change baudrate if your GPS uses different)
    UART7_Init(sysClock, 38400);

    while (1)
    {
        int data = UART7_BufferPop();

        if (data != -1)
        {
            uint8_t received_byte = (uint8_t)data;

            // GPS data received here (byte-by-byte)
            // Later: UBX parser will go here
            (void)received_byte;
        }

        // Optional: monitor overflow
        if (uart7_overflow)
        {
            // Handle overflow case here (log, LED, etc.)
            uart7_overflow = 0;
        }
    }
}
