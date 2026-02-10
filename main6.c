// Parsing and PPS ISR, sending it to CPU

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
#include "driverlib/systick.h"

/* ============================================================
   SYSTEM TICK (1 ms) FOR PPS WATCHDOG
   ============================================================ */

volatile uint32_t system_ms = 0;
volatile uint32_t pps_missing_ms = 0;

volatile uint8_t periodic_tx_flag = 0;

void SysTick_Handler(void)
{
    system_ms++;
    pps_missing_ms++;   // increments until PPS ISR resets it

    static uint32_t last_tx_ms = 0;

    // 1 Hz periodic trigger
    if ((system_ms - last_tx_ms) >= 1000)
    {
        last_tx_ms = system_ms;
        periodic_tx_flag = 1;
    }
}

/* ============================================================
   PPS CONFIGURATION
   ============================================================ */

#define PPS_PORT_BASE GPIO_PORTB_BASE
#define PPS_PIN       GPIO_PIN_2

volatile uint8_t  gps_pps_present = 0;   // mainly for debug
volatile uint32_t gps_system_seconds = 0;
volatile uint8_t  pps_missed = 0;
volatile uint8_t  gps_fix_ok = 0;
volatile uint32_t gps_pps_count = 0;
volatile uint8_t gps_tx_request = 0;

/* ============================================================
   TRANSMIT STRUCTURE
   ============================================================ */
#pragma pack(1)
typedef struct
{
    uint8_t  sync1;
    uint8_t  sync2;
    uint8_t  length;

    uint8_t  card_id;

    uint16_t gps_week;
    uint32_t gps_tow_ms;
    uint32_t gps_system_seconds;

    uint32_t tAcc_ns;

    uint8_t  numSV;
    uint8_t  gps_fix_ok;
    uint8_t  pps_ok;

    uint16_t status_flags;
    uint16_t crc16;

} GPS_To_CPU_Frame;
#pragma pack()

#define GPS_FRAME_LEN 24

uint16_t CRC16_Modbus(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    for ( i = 0; i < len; i++)
    {
        crc ^= buf[i];
        uint8_t j;
        for ( j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}

/* ============================================================
   TIMEGPS STRUCTURE
   ============================================================ */

typedef struct
{
    uint32_t iTOW;
    int32_t  fTOW;
    int16_t  week;
    int8_t   leapS;
    uint8_t  valid;
    uint32_t tAcc;

} UBX_TIMEGPS_t;

volatile UBX_TIMEGPS_t timegps_data;
volatile uint8_t timegps_updated = 0;

/* ============================================================
   PVT STRUCTURE
   ============================================================ */

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

/* ============================================================
   PPS INTERRUPT HANDLER
   ============================================================ */

void PPS_GPIO_IntHandler(void)
{
    GPIOIntClear(PPS_PORT_BASE, PPS_PIN);

    gps_pps_present = 1;
    gps_pps_count++;
    pps_missing_ms = 0;

    IntMasterDisable();
    if (gps_system_seconds != 0)
    {
        gps_system_seconds++;
    }
    IntMasterEnable();
}

void GPIOBIntHandler(void)
{
    PPS_GPIO_IntHandler();
}

/* ============================================================
   PPS INITIALIZATION
   ============================================================ */

void PPS_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinTypeGPIOInput(PPS_PORT_BASE, PPS_PIN);
    GPIOPadConfigSet(PPS_PORT_BASE, PPS_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    GPIOIntTypeSet(PPS_PORT_BASE, PPS_PIN, GPIO_RISING_EDGE);
    GPIOIntEnable(PPS_PORT_BASE, PPS_PIN);

    IntEnable(INT_GPIOB);
}

/* ============================================================
   UART + UBX STACK
   ============================================================ */

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

void UART7IntHandler(void)
{
    uint32_t status = UARTIntStatus(UART7_BASE, true);
    UARTIntClear(UART7_BASE, status);

    while (UARTCharsAvail(UART7_BASE))
    {
        UART7_BufferPush((uint8_t)UARTCharGetNonBlocking(UART7_BASE));
    }
}

void UART7_Init(uint32_t sysClk, uint32_t baudrate)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART7));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(UART7_BASE, sysClk, baudrate,
                        UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);

    IntEnable(INT_UART7);
    IntMasterEnable();
    UARTEnable(UART7_BASE);
}
void UART6_Init(uint32_t sysClk, uint32_t baudrate)
{
    /* Enable peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART6));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));

    /* Configure PP0, PP1 for UART6 */
    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);
    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* UART6 Config */
    UARTConfigSetExpClk(UART6_BASE,
                        sysClk,
                        baudrate,
                        UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);

    UARTEnable(UART6_BASE);
}

/* ============================================================
   UBX FRAME PARSER
   ============================================================ */

#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_MAX_PAYLOAD 256

volatile uint8_t  ubx_class, ubx_id;
volatile uint16_t ubx_len, ubx_payload_index;
volatile uint8_t  ubx_ck_a, ubx_ck_b;
volatile uint8_t  ubx_message_ready = 0;

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
volatile uint8_t ubx_payload[UBX_MAX_PAYLOAD];

static inline void UBX_Checksum(uint8_t byte)
{
    ubx_ck_a += byte;
    ubx_ck_b += ubx_ck_a;
}

void UBX_ProcessByte(uint8_t byte)
{
    switch(ubx_state)
    {
        case UBX_WAIT_SYNC1:
            if (byte == UBX_SYNC1) ubx_state = UBX_WAIT_SYNC2;
            break;

        case UBX_WAIT_SYNC2:
            if (byte == UBX_SYNC2) ubx_state = UBX_WAIT_CLASS;
            else ubx_state = UBX_WAIT_SYNC1;
            break;

        case UBX_WAIT_CLASS:
            ubx_class = byte;
            ubx_ck_a = ubx_ck_b = 0;
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
            {
                ubx_state = UBX_WAIT_SYNC1;
            }
            else
            {
                ubx_payload_index = 0;
                ubx_state = UBX_WAIT_PAYLOAD;
            }
            break;

        case UBX_WAIT_PAYLOAD:
            ubx_payload[ubx_payload_index++] = byte;
            UBX_Checksum(byte);

            if (ubx_payload_index >= ubx_len)
                ubx_state = UBX_WAIT_CK_A;
            break;

        case UBX_WAIT_CK_A:
            if (byte == ubx_ck_a) ubx_state = UBX_WAIT_CK_B;
            else ubx_state = UBX_WAIT_SYNC1;
            break;

        case UBX_WAIT_CK_B:
            if (byte == ubx_ck_b) ubx_message_ready = 1;
            ubx_state = UBX_WAIT_SYNC1;
            break;
    }
}

/* ============================================================
   UBX DATA PARSERS
   ============================================================ */

void UBX_Parse_TIMEGPS(uint8_t *payload)
{
    memcpy((void*)&timegps_data.iTOW,  payload,      4);
    memcpy((void*)&timegps_data.fTOW,  payload + 4,  4);
    memcpy((void*)&timegps_data.week,  payload + 8,  2);
    memcpy((void*)&timegps_data.leapS, payload + 10, 1);
    memcpy((void*)&timegps_data.valid, payload + 11, 1);
    memcpy((void*)&timegps_data.tAcc,  payload + 12, 4);
}

void UBX_Parse_PVT(uint8_t *payload)
{
    pvt_data.valid   = payload[11];
    pvt_data.fixType = payload[20];
    pvt_data.flags   = payload[21];
    pvt_data.flags2  = payload[22];
    pvt_data.numSV   = payload[23];

    if ((pvt_data.flags & 0x01) && (pvt_data.valid & 0x02) && // GNSS fix OK and Date valid
        (pvt_data.flags2 & 0x20) && (pvt_data.numSV >= 4)) // Confirmed time and 6 or more sat
        gps_fix_ok = 1;
    else
        gps_fix_ok = 0;
}
uint16_t Build_Status_Flags(void)
{
    uint16_t flags = 0;

    if (gps_fix_ok)               flags |= (1 << 0);
    if (!pps_missed)              flags |= (1 << 1);
    if ((timegps_data.valid & 0x07) == 0x07) flags |= (1 << 2);
    if (pps_missed)               flags |= (1 << 3);
    if (timegps_data.tAcc > 100000) flags |= (1 << 4);  // accuracy bad

    return flags;
}
void GPS_Send_Frame(void)
{
    GPS_To_CPU_Frame frame;

    frame.sync1  = 0xA5;
    frame.sync2  = 0x5A;
    frame.length = GPS_FRAME_LEN;

    frame.card_id = 1;   // GPS card ID

    frame.gps_week = timegps_data.week;
    frame.gps_tow_ms = timegps_data.iTOW;
    frame.gps_system_seconds = gps_system_seconds;

    frame.tAcc_ns = timegps_data.tAcc;

    frame.numSV = pvt_data.numSV;
    frame.gps_fix_ok = gps_fix_ok;
    frame.pps_ok = (pps_missed == 0);

    frame.status_flags = Build_Status_Flags();

    frame.crc16 = CRC16_Modbus((uint8_t *)&frame,
                               GPS_FRAME_LEN - 2);

    uint8_t *ptr = (uint8_t*)&frame;
    uint8_t i;
    for (i = 0; i < GPS_FRAME_LEN; i++)
    {
        UARTCharPut(UART6_BASE, ptr[i]);
    }
}

/* ============================================================
   MAIN TIME ENGINE
   ============================================================ */

int main(void)
{
    uint32_t sysClock = SysCtlClockFreqSet(
                            SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                            SYSCTL_XTAL_25MHZ | SYSCTL_CFG_VCO_480,
                            50000000);

    // 1 ms SysTick for PPS watchdog
    SysTickPeriodSet(sysClock / 1000);
    SysTickIntEnable();
    SysTickEnable();

    UART7_Init(sysClock, 38400);
    UART6_Init(sysClock, 115200);   // <-- don't forget this!
    PPS_Init();

    while(1)
    {
        int data = UART7_BufferPop();
        if (data != -1)
            UBX_ProcessByte((uint8_t)data);

        if (uart7_overflow)
        {
            uart7_overflow = 0;
            ubx_state = UBX_WAIT_SYNC1;
        }

        if (ubx_message_ready)
        {
            ubx_message_ready = 0;

            if (ubx_class == 0x01 && ubx_id == 0x20 && ubx_len == 16)
            {
                UBX_Parse_TIMEGPS(ubx_payload);

                if ((timegps_data.valid & 0x07) == 0x07 &&
                    timegps_data.tAcc < 50000)
                {
                    uint32_t tow_sec     = timegps_data.iTOW / 1000;
                    uint32_t ubx_abs_sec = (timegps_data.week * 604800u) + tow_sec;

                    // Only realign near PPS edge to avoid jumps mid-second
                    if ((timegps_data.iTOW % 1000) < 10u ||
                        (timegps_data.iTOW % 1000) > 990u)
                    {
                        IntMasterDisable();
                        gps_system_seconds = ubx_abs_sec;
                        IntMasterEnable();
                    }
                }
            }

            if (ubx_class == 0x01 && ubx_id == 0x07 && ubx_len == 92)
            {
                UBX_Parse_PVT(ubx_payload);
            }
        }

        /* ===== PPS timeout detection using ms timer ===== */
        if (pps_missing_ms > 1500)   // > 1.5 seconds without PPS
        {
            pps_missed = 1;
        }
        else
        {
            pps_missed = 0;
        }

        /* ===== HERE: send frame periodically ===== */
        if (periodic_tx_flag)
        {
            periodic_tx_flag = 0;
            GPS_Send_Frame();
        }

        /* gps_system_seconds is now your synchronized time base */
    }
}

