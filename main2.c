//*********************************************************************************
// UART7 + UBX + TIMEGPS + PVT  (TM4C1294NCPDT)
// PC4  -> UART7 RX
// PC5  -> UART7 TX
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

/* ============================================================================
   TIMEGPS STRUCTURE
   ============================================================================ */

typedef struct
{
    uint32_t iTOW;     // ms
    int32_t  fTOW;     // ns
    int16_t  week;
    int8_t   leapS;
    uint8_t  valid;
    uint32_t tAcc;     // ns

} UBX_TIMEGPS_t;

volatile UBX_TIMEGPS_t timegps_data;
volatile uint8_t timegps_updated = 0;


/* ============================================================================
   PVT STRUCTURE
   ============================================================================ */

typedef struct
{
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t valid;
    uint8_t numSV;

} UBX_PVT_t;

volatile UBX_PVT_t pvt_data;
volatile uint8_t pvt_updated = 0;


/* ============================================================================
   TIMEGPS PARSER
   ============================================================================ */

void UBX_Parse_TIMEGPS(uint8_t *payload)
{
    memcpy((void*)&timegps_data.iTOW,  payload,      4);
    memcpy((void*)&timegps_data.fTOW,  payload + 4,  4);
    memcpy((void*)&timegps_data.week,  payload + 8,  2);
    memcpy((void*)&timegps_data.leapS, payload + 10, 1);
    memcpy((void*)&timegps_data.valid, payload + 11, 1);
    memcpy((void*)&timegps_data.tAcc,  payload + 12, 4);
}


/* ============================================================================
   PVT PARSER
   ============================================================================ */

void UBX_Parse_PVT(uint8_t *payload)
{
    pvt_data.valid   = payload[11];
    pvt_data.fixType = payload[20];
    pvt_data.flags   = payload[21];
    pvt_data.flags2  = payload[22];
    pvt_data.numSV   = payload[23];

    pvt_updated = 1;
}


/* ============================================================================
   UART RING BUFFER
   ============================================================================ */

#define UART7_BUFFER_SIZE 1024

volatile uint8_t  uart7_buffer[UART7_BUFFER_SIZE];
volatile uint16_t uart7_head = 0;
volatile uint16_t uart7_tail = 0;
volatile uint8_t  uart7_overflow = 0;

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
        uart7_overflow = 1;
    }
}

int UART7_BufferPop(void)
{
    if (uart7_head == uart7_tail)
        return -1;

    uint8_t data = uart7_buffer[uart7_tail];
    uart7_tail = (uart7_tail + 1) % UART7_BUFFER_SIZE;

    return data;
}


/* ============================================================================
   UART7 INTERRUPT
   ============================================================================ */

void UART7IntHandler(void)
{
    uint32_t status = UARTIntStatus(UART7_BASE, true);
    UARTIntClear(UART7_BASE, status);

    while (UARTCharsAvail(UART7_BASE))
    {
        uint8_t rx = (uint8_t)UARTCharGetNonBlocking(UART7_BASE);
        UART7_BufferPush(rx);
    }
}


/* ============================================================================
   UART7 INIT
   ============================================================================ */

void UART7_Init(uint32_t sysClk, uint32_t baudrate)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART7));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(UART7_BASE,
                        sysClk,
                        baudrate,
                        UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);

    IntEnable(INT_UART7);
    IntMasterEnable();

    UARTEnable(UART7_BASE);
}


/* ============================================================================
   UBX FRAME DETECTOR
   ============================================================================ */

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_MAX_PAYLOAD 256

typedef struct
{
    uint8_t class_id;
    uint8_t msg_id;
    uint16_t length;
    uint8_t payload[UBX_MAX_PAYLOAD];

} UBX_Message;

volatile UBX_Message ubx_message;
volatile uint8_t ubx_message_ready = 0;

typedef enum {
    UBX_WAIT_SYNC1,
    UBX_WAIT_SYNC2,
    UBX_WAIT_CLASS,
    UBX_WAIT_ID,
    UBX_WAIT_LEN1,
    UBX_WAIT_LEN2,
    UBX_WAIT_PAYLOAD,
    UBX_WAIT_CK_A,
    UBX_WAIT_CK_B
} UBX_State;

volatile UBX_State ubx_state = UBX_WAIT_SYNC1;

volatile uint8_t ubx_class, ubx_id;
volatile uint16_t ubx_len;
volatile uint16_t ubx_payload_index;
volatile uint8_t ubx_ck_a, ubx_ck_b;

static inline void UBX_Checksum(uint8_t byte)
{
    ubx_ck_a += byte;
    ubx_ck_b += ubx_ck_a;
}

void UBX_ProcessByte(uint8_t byte)
{
    switch (ubx_state)
    {
        case UBX_WAIT_SYNC1:
            if (byte == UBX_SYNC1)
                ubx_state = UBX_WAIT_SYNC2;
            break;

        case UBX_WAIT_SYNC2:
            if (byte == UBX_SYNC2)
                ubx_state = UBX_WAIT_CLASS;
            else
                ubx_state = UBX_WAIT_SYNC1;
            break;

        case UBX_WAIT_CLASS:
            ubx_class = byte;
            ubx_ck_a = 0;
            ubx_ck_b = 0;
            UBX_Checksum(byte);
            ubx_state = UBX_WAIT_ID;
            break;

        case UBX_WAIT_ID:
            ubx_id = byte;
            UBX_Checksum(byte);
            ubx_state = UBX_WAIT_LEN1;
            break;

        case UBX_WAIT_LEN1:
            ubx_len = byte;
            UBX_Checksum(byte);
            ubx_state = UBX_WAIT_LEN2;
            break;

        case UBX_WAIT_LEN2:
            ubx_len |= ((uint16_t)byte << 8);
            UBX_Checksum(byte);

            if (ubx_len > UBX_MAX_PAYLOAD)
                ubx_state = UBX_WAIT_SYNC1;
            else
            {
                ubx_payload_index = 0;
                ubx_state = UBX_WAIT_PAYLOAD;
            }
            break;

        case UBX_WAIT_PAYLOAD:
            ubx_message.payload[ubx_payload_index++] = byte;
            UBX_Checksum(byte);

            if (ubx_payload_index >= ubx_len)
                ubx_state = UBX_WAIT_CK_A;
            break;

        case UBX_WAIT_CK_A:
            if (byte == ubx_ck_a)
                ubx_state = UBX_WAIT_CK_B;
            else
                ubx_state = UBX_WAIT_SYNC1;
            break;

        case UBX_WAIT_CK_B:
            if (byte == ubx_ck_b)
            {
                ubx_message.class_id = ubx_class;
                ubx_message.msg_id   = ubx_id;
                ubx_message.length   = ubx_len;
                ubx_message_ready = 1;
            }

            ubx_state = UBX_WAIT_SYNC1;
            break;
    }
}


/* ============================================================================
   MAIN
   ============================================================================ */

int main(void)
{
    uint32_t sysClock = SysCtlClockFreqSet(
                            SYSCTL_USE_PLL |
                            SYSCTL_OSC_MAIN |
                            SYSCTL_XTAL_25MHZ |
                            SYSCTL_CFG_VCO_480,
                            50000000);

    UART7_Init(sysClock, 38400);

    while (1)
    {
        int data = UART7_BufferPop();

        if (data != -1)
        {
            UBX_ProcessByte((uint8_t)data);
        }

        /* Handle UART overflow */
        if (uart7_overflow)
        {
            uart7_overflow = 0;
            ubx_state = UBX_WAIT_SYNC1;
        }

        if (ubx_message_ready)
        {
            ubx_message_ready = 0;

            /* -------- TIMEGPS -------- */
            if (ubx_message.class_id == 0x01 &&
                ubx_message.msg_id   == 0x20 &&
                ubx_message.length   == 16)
            {
                UBX_Parse_TIMEGPS(ubx_message.payload);

                // Check TIMEGPS validity
                if ((timegps_data.valid & 0x07) == 0x07 &&
                    timegps_data.tAcc < 50000)
                {
                    timegps_updated = 1;
                }
            }

            /* -------- PVT -------- */
            else if (ubx_message.class_id == 0x01 &&
                     ubx_message.msg_id   == 0x07 &&
                     ubx_message.length   == 92)
            {
                UBX_Parse_PVT(ubx_message.payload);
            }
        }

        /* -------- Process TIMEGPS -------- */
        if (timegps_updated)
        {
            timegps_updated = 0;

            uint32_t current_sec = timegps_data.iTOW / 1000;

            // Future: sync with PPS here
            (void)current_sec;
        }

        /* -------- Process PVT -------- */
        if (pvt_updated)
        {
            pvt_updated = 0;

            uint8_t fix_valid  = (pvt_data.flags & 0x01);
            uint8_t time_valid = (pvt_data.valid & 0x02);
            uint8_t confirmed  = (pvt_data.flags2 & 0x20);

            if (fix_valid && time_valid && confirmed && (pvt_data.numSV >= 6))
            {
                // GPS is healthy and reliable
            }
            else
            {
                // GPS health degraded
            }
        }
    }
}
