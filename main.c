#include "uart.h"

#define DELAY_SECONDS               1000
#define NUM_OF_BYTES                40
#define DELIMITER                   "-"
#define SYNC_TIME_HOUR              4

typedef struct SunTimes
{
    uint8_t u8_Sunrise_Hour;
    uint8_t u8_Sunrise_Minute;
    uint8_t u8_Sunset_Hour;
    uint8_t u8_Sunset_Minute;
} SunTimes_T;

typedef enum ParserState
{
    DATE_YEAR = 0x01,
    DATE_MONTH,
    DATE_DAY,
    DATE_HOUR,
    DATE_MINUTE,
    DATE_SECOND,
    DATE_SUNRISE_HOUR,
    DATE_SUNRISE_MINUTE,
    DATE_SUNSET_HOUR,
    DATE_SUNSET_MINUTE,
    DATE_CRC,
    DATE_FINISH
} ParserState_T;

static char u8_UART_DataReceiveBuffer[ MAX_STR_LENGTH ];
static uint8_t u8_UART_Index = 0;
static bool b_UART_DataReceived = false;
static bool b_RTC_SynchronizationDone = false;
static uint8_t u8_RTC_ActualDay = 0;
static char s_u8Crc;

Calendar calendar;
SunTimes_T sunTimes;

void Init_RTC( void );
void Init_GPIO( void );
void Init_Clock( void );
void sendRTCtoPC( void );
void Init_UART_A0( void );
void Init_UART_A3( void );
void delayMilis( uint32_t u32NumOfMiliseconds );
bool UART_DataExtract( uint16_t u16_UART_Buffer, uint8_t* u8_UART_DataBuffer );
uint8_t u8_Atoi( uint8_t* u8_Buffer, uint16_t* p_u16_Num );
uint8_t u8_DateParser( void );
void ClearFlags( void );
void RTC_ESP8266_Synchronization( void );
unsigned char u8_Crc8( unsigned char* strData, uint8_t u8Length );

/**
 * main.c
 */
void main( void )
{
    // Stop watchdog timer
    WDT_A_hold( WDT_A_BASE );
    uint8_t u8_Error = 0x00;

    //Initialization
    Init_GPIO();
    Init_Clock();
    Init_UART_A0();
    Init_UART_A3();
    //Init_RTC();
    //RTC_ESP8266_Synchronization();

    while( 1 )
    {
        //Toggle P1.0 output
        //GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );

        //sendRTCtoPC();
        //delayMilis( 1000 );
        if( b_UART_DataReceived )
        {
            GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
            u8_Error = u8_DateParser();
            GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0 );
            ClearFlags();
        }
        if( ( calendar.Hours == SYNC_TIME_HOUR ) || ( u8_Error != 0x00 ) )
        {
            //RTC_ESP8266_Synchronization();
            u8_RTC_ActualDay = calendar.DayOfMonth;
        }
//        if( ( calendar.DayOfMonth != u8_RTC_ActualDay ) && ( calendar.Hours == 0x00 ) && ( calendar.Minutes == 0x00 ) && ( calendar.Seconds ) >= 0x00 )
//        {
//            b_RTC_SynchronizationDone = false;
//        }

    }
}

/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    // Set all GPIO pins to output low for low power
    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );
    GPIO_setOutputHighOnPin( GPIO_PORT_P4, GPIO_PIN0 );

    GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );

    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION );

    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P6, GPIO_PIN1,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_PJ,
    GPIO_PIN4 + GPIO_PIN5,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq( CS_DCORSEL_0, CS_DCOFSEL_6 );
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource( 32768, 0 );
    //Set ACLK=LFXT
    CS_initClockSignal( CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    //Start XT1 with no time out
    CS_turnOnLFXT( CS_LFXT_DRIVE_3 );
}

/*
 * UART Communication Initialization
 */
void Init_UART_A0()
{
    // Configure UART 9600 bit per second at 8MHz clock source
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource |= EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar |= ( UCBR5 ) ^ ( UCBR4 ) ^ ( UCBR2 );
    param.firstModReg |= ( UCBRF0 );
    param.secondModReg |= ( UCBRS6 ) ^ ( UCBRS3 ) ^ ( UCBRS0 );
    param.parity |= EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst |= EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits |= EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode |= EUSCI_A_UART_MODE;
    param.overSampling |= EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if( STATUS_FAIL == EUSCI_A_UART_init( EUSCI_A0_BASE, &param ) )
    {
        return;
    }

    EUSCI_A_UART_enable( EUSCI_A0_BASE );

    EUSCI_A_UART_clearInterrupt( EUSCI_A0_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT );

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt( EUSCI_A0_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT ); // Enable interrupt

    // Enable global interrupt
    __enable_interrupt();
}

void Init_UART_A3()
{
    // Configure UART 9600 bit per second at 8MHz clock source
    EUSCI_A_UART_initParam param = { 0 };
    param.selectClockSource |= EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar |= ( UCBR5 ) ^ ( UCBR4 ) ^ ( UCBR2 );
    param.firstModReg |= ( UCBRF0 );
    param.secondModReg |= ( UCBRS6 ) ^ ( UCBRS3 ) ^ ( UCBRS0 );
    param.parity |= EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst |= EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits |= EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode |= EUSCI_A_UART_MODE;
    param.overSampling |= EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if( STATUS_FAIL == EUSCI_A_UART_init( EUSCI_A3_BASE, &param ) )
    {
        return;
    }

    EUSCI_A_UART_enable( EUSCI_A3_BASE );

    EUSCI_A_UART_clearInterrupt( EUSCI_A3_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT );

    // Enable USCI_A3 RX interrupt
    EUSCI_A_UART_enableInterrupt( EUSCI_A3_BASE,
    EUSCI_A_UART_RECEIVE_INTERRUPT ); // Enable interrupt

    // Enable global interrupt
    __enable_interrupt();
}

/*
 * Real Time Clock Initialization
 */
void Init_RTC()
{
    //Setup Current Time for Calendar
    calendar.Seconds |= 0x00;
    calendar.Minutes |= 0x00;
    calendar.Hours |= 0x00;
    calendar.DayOfWeek |= 0x00;
    calendar.DayOfMonth |= 0x00;
    calendar.Month |= 0x00;
    calendar.Year |= 0x000;

    // Initialize RTC with the specified Calendar above
    RTC_C_initCalendar( RTC_C_BASE, &calendar,
    RTC_C_FORMAT_BINARY );

    //Start RTC Clock
    RTC_C_startClock( RTC_C_BASE );
}

void sendRTCtoPC()
{
    uint8_t cnt;
    char buffer[ 100 ];
    memset( buffer, 0, sizeof( buffer ) );
    calendar = RTC_C_getCalendarTime( RTC_C_BASE );

    snprintf( buffer, sizeof( buffer ), "%04d%02d%02d%02d%02d%02d\r\n", calendar.Year, calendar.Month, calendar.DayOfMonth, calendar.Hours, calendar.Minutes, calendar.Seconds );
    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin(
    GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION );

    for( cnt = 0; cnt < strlen( buffer ); cnt++ )
    {
        // Send Ackknowledgement to Host PC
        EUSCI_A_UART_transmitData( EUSCI_A0_BASE, buffer[ cnt ] );
    }

    while( EUSCI_A_UART_queryStatusFlags( EUSCI_A0_BASE, EUSCI_A_UART_BUSY ) )
        ;

    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
}

void RTC_ESP8266_Synchronization()
{
    const char wakeUpCharacter = '!';
    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P6, GPIO_PIN0,
    GPIO_PRIMARY_MODULE_FUNCTION );
    // Send Ackknowledgement to Host PC
    EUSCI_A_UART_transmitData( EUSCI_A3_BASE, wakeUpCharacter );

    while( EUSCI_A_UART_queryStatusFlags( EUSCI_A3_BASE, EUSCI_A_UART_BUSY ) )
    {
        ;
    }

    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN0 );
}

void delayMilis( uint32_t u32NumOfMiliseconds )
{
    uint32_t u32pomMilis = u32NumOfMiliseconds * 8;

    do
    {
        _delay_cycles( 1000 );
    }
    while( u32pomMilis-- > 0 );
}

uint8_t u8_Atoi( uint8_t* u8_Buffer, uint16_t* p_u16_Num )
{
    uint8_t u8_Error = 0xFF;
    uint16_t u16_Final_mun = 0;
    uint8_t u8_Cnt;
    *p_u16_Num = 0;

    for( u8_Cnt = 0; u8_Buffer[ u8_Cnt ] != 0x00; ++u8_Cnt )
    {
        if( u8_Buffer[ u8_Cnt ] >= 0x30 && u8_Buffer[ u8_Cnt ] <= 0x39 )
        {
            u16_Final_mun = u16_Final_mun * 10 + ( u8_Buffer[ u8_Cnt ] - 0x30 );
            u8_Error = 0x00;
        }
        else
        {
            u8_Error = 0x01;
            break;
        }
    }
    if( u8_Error == 0x00 )
    {
        *p_u16_Num = u16_Final_mun;
    }

    return u8_Error;
}

void ClearFlags()
{
    memset( u8_UART_DataReceiveBuffer, 0, sizeof( u8_UART_DataReceiveBuffer ) );
    b_UART_DataReceived = FALSE;
    u8_UART_Index = 0x00;
}

uint8_t u8_DateParser()
{
    uint8_t u8_Error = 0xFF;
    uint8_t u8_ParseState = DATE_YEAR;
    uint16_t u16_Number = 0;
    char pszDateArray[ NUM_OF_BYTES ];
    char* pszSubDateArray;
    unsigned char u8Crc;

    memset( pszDateArray, 0, sizeof( pszDateArray ) );
    memcpy( pszDateArray, u8_UART_DataReceiveBuffer, strlen( u8_UART_DataReceiveBuffer ) );
//    if( sizeof( pszDateArray ) != NUM_OF_BYTES )
//    {
//        return u8_Error = 0x02;
//    }

    pszSubDateArray = strtok( pszDateArray, DELIMITER );
    u8_Error = u8_Atoi( (uint8_t*) pszSubDateArray, &u16_Number );
    if( u8_Error != 0x00 )
    {
        return u8_Error;
    }
    calendar.Year = u16_Number;
    u8_ParseState = DATE_MONTH;

    while( pszSubDateArray != NULL )
    {
        pszSubDateArray = strtok( NULL, DELIMITER );
        if( pszSubDateArray != NULL )
        {
            if( u8_ParseState == DATE_CRC )
            {
                unsigned char u8DataBuffer[NUM_OF_BYTES];
                unsigned char u8Crc = 0x00;
                memset(u8DataBuffer, 0, sizeof(u8DataBuffer));
                memcpy( u8DataBuffer, u8_UART_DataReceiveBuffer, strlen( u8_UART_DataReceiveBuffer ) - 3 );
                u8Crc = u8_Crc8(u8DataBuffer, 32);
                if(u8Crc != 0x00)
                {
                    u8_ParseState = DATE_FINISH;
                }


            }
            if( u8_ParseState == DATE_FINISH )
            {
                u8_Error = 0x00;
                Init_RTC();
                b_RTC_SynchronizationDone = true;
                break;
            }
            u8_Error = u8_Atoi( (uint8_t*) pszSubDateArray, &u16_Number );
            if( u8_Error != 0x00 )
            {
                break;
            }
            switch( (ParserState_T) u8_ParseState )
            {
                case DATE_MONTH :
                {
                    calendar.Month = (uint8_t) u16_Number;
                    u8_ParseState = DATE_DAY;
                    break;
                }
                case DATE_DAY :
                {
                    calendar.DayOfMonth = (uint8_t) u16_Number;
                    u8_ParseState = DATE_HOUR;
                    break;
                }
                case DATE_HOUR :
                {
                    calendar.Hours = (uint8_t) u16_Number;
                    u8_ParseState = DATE_MINUTE;
                    break;
                }
                case DATE_MINUTE :
                {
                    calendar.Minutes = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SECOND;
                    break;
                }
                case DATE_SECOND :
                {
                    calendar.Seconds = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNRISE_HOUR;
                    break;
                }
                case DATE_SUNRISE_HOUR :
                {
                    sunTimes.u8_Sunrise_Hour = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNRISE_MINUTE;
                    break;
                }
                case DATE_SUNRISE_MINUTE :
                {
                    sunTimes.u8_Sunrise_Minute = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNSET_HOUR;
                    break;
                }
                case DATE_SUNSET_HOUR :
                {
                    sunTimes.u8_Sunset_Hour = (uint8_t) u16_Number;
                    u8_ParseState = DATE_SUNSET_MINUTE;
                    break;
                }
                case DATE_SUNSET_MINUTE :
                {
                    sunTimes.u8_Sunset_Minute = (uint8_t) u16_Number;
                    u8_ParseState = DATE_CRC;
                    break;
                }
//                case DATE_CRC :
//                {
//                    unsigned char* u8DataBuffer;
//                    memcpy( u8DataBuffer, pszDateArray, sizeof( pszDateArray ) - 2 );
//                    u8
//                    break;
//                }
                default :
                {
                    u8_Error = 0x03;
                    break;
                }
            }
        }
        else
        {
            break;
        }
    }
    return u8_Error;
}

bool UART_DataExtract( uint16_t u16_UART_Buffer, uint8_t* u8_UART_DataBuffer )
{
    bool b_UART_BufferIsReceived = FALSE;

    u8_UART_DataBuffer[ u8_UART_Index++ ] = u16_UART_Buffer;
    if( u8_UART_DataBuffer[ u8_UART_Index - 1 ] == 0x0A )
    {
        return b_UART_BufferIsReceived = TRUE;
    }

    return b_UART_BufferIsReceived;
}

unsigned char u8_Crc8( unsigned char* strData, uint8_t u8Length )
{
    unsigned char crc = 0x00;
    uint8_t u8Cnt;

    if( strData == NULL )
    {
        return 0;
    }
    crc &= 0xff;
    for( u8Cnt = 0; u8Cnt < u8Length; u8Cnt++ )
    {
        crc = crc8_table[ crc ^ strData[ u8Cnt ] ];
    }
    return crc;
}

/*
 * USCI_A0 Interrupt Service Routine that receives PC GUI's commands
 */
#pragma vector = USCI_A3_VECTOR
__interrupt void USCI_A3_ISR( void )
{
    //char data;
    switch( __even_in_range( UCA3IV, USCI_UART_UCTXCPTIFG ) )
    {
        case USCI_NONE :
            break;
        case USCI_UART_UCRXIFG :
            b_UART_DataReceived = UART_DataExtract( UCA3RXBUF, (uint8_t*) &u8_UART_DataReceiveBuffer );
            __bic_SR_register_on_exit( LPM3_bits ); // Exit active CPU
            break;
        case USCI_UART_UCTXIFG :
            break;
        case USCI_UART_UCSTTIFG :
            break;
        case USCI_UART_UCTXCPTIFG :
            break;
    }
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR( void )
{
    //char data;
    switch( __even_in_range( UCA0IV, USCI_UART_UCTXCPTIFG ) )
    {
        case USCI_NONE :
            break;
        case USCI_UART_UCRXIFG :
            RTC_ESP8266_Synchronization();
            __bic_SR_register_on_exit( LPM3_bits ); // Exit active CPU
            break;
        case USCI_UART_UCTXIFG :
            break;
        case USCI_UART_UCSTTIFG :
            break;
        case USCI_UART_UCTXCPTIFG :
            break;
    }
}

