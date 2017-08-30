/*****************************************************************************
 * FILE NAME:                                                                *
 *  main.c                                                                   *
 *                                                                           *
 * PURPOSE:                                                                  *
 *                                                                           *
 * FILE REFERENCES:                                                          *
 *  Name I/O Description                                                     *
 *  ---- --- -----------                                                     *
 *  none                                                                     *
 *                                                                           *
 * EXTERNAL VARIABLES:                                                       *
 *                                                                           *
 *  Name Type I/O Description                                                *
 *  ---- ---- --- -----------                                                *
 *  none                                                                     *
 *                                                                           *
 * EXTERNAL REFERENCES:                                                      *
 *  Name Description                                                         *
 *  ---- -----------                                                         *
 *  none                                                                     *
 *                                                                           *
 * ABNORMAL TERMINATION CONDITIONS, ERROR AND WARNING MESSAGES:              *
 *  none                                                                     *
 *                                                                           *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:                                   *
 *  none                                                                     *
 *                                                                           *
 * NOTES:                                                                    *
 *                                                                           *
 * REQUIREMENTS/FUNCTIONAL SPECIFICATIONS REFERENCES:                        *
 *                                                                           *
 * MISRA NOTES:                                                              *
 *  The file 'options.lnt' disables MISRA violations for compiler library    *
 *  files.                                                                   *
 *                                                                           *
 * DEVELOPMENT HISTORY:                                                      *
 *  Date	Author	Change Id	Release	Description Of Change        *
 *  --------	------	---------	-------	---------------------        *
 *                                                                           *
 * ALGORITHM (PDL)                                                           *
 *  none                                                                     *
 *                                                                           *
 *****************************************************************************/

/*****************************************************************************
 * Includes
 *****************************************************************************/
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include "LIN_DRIVER/lin_driver_api.h"
#include "CRC_Tables7bit.h"

/*****************************************************************************
 * PIC configuration words
 *****************************************************************************/
/*lint -save -e975 */
/* CONFIG1 */
#pragma config FOSC = INTOSC    /* Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin) */
/*#pragma config WDTE = SWDTEN*/    /* Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register) */
#pragma config WDTE = NSLEEP    /* Watchdog Timer Enable (WDT enabled while running and disabled in Sleep) */
#pragma config PWRTE = OFF      /* Power-up Timer Enable (PWRT disabled) */
#pragma config MCLRE = ON       /* MCLR Pin Function Select (MCLR/VPP pin function is MCLR) */
#pragma config CP = OFF         /* Flash Program Memory Code Protection (Program memory code protection is disabled) */
#pragma config CPD = OFF        /* Data Memory Code Protection (Data memory code protection is disabled) */
#pragma config BOREN = ON       /* Brown-out Reset Enable (Brown-out Reset enabled) */
#pragma config CLKOUTEN = OFF   /* Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin) */
#pragma config IESO = OFF       /* Internal/External Switchover (Internal/External Switchover mode is disabled) */
#pragma config FCMEN = OFF      /* Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled) */

/* CONFIG2 */
#pragma config WRT = OFF        /* Flash Memory Self-Write Protection (Write protection off) */
#pragma config VCAPEN = OFF     /* Voltage Regulator Capacitor Enable (All VCAP pin functionality is disabled) */
#pragma config PLLEN = OFF      /* PLL Enable (4x PLL disabled) */
#pragma config STVREN = ON      /* Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset) */
#pragma config BORV = LO        /* Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.) */
#pragma config LVP = OFF        /* Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming) */

/* User ID */
#if 0
#pragma config IDLOC0 = 0x7Fu;
#pragma config IDLOC1 = 0x7Fu;
#pragma config IDLOC2 = 0x7Fu;
#pragma config IDLOC3 = 0x7Fu;
#endif
/*lint -restore */

/*****************************************************************************
 * Local Defines
 *****************************************************************************/
/* MCU_RESET defined per Errata 11.1 - "after executing a RESET instruction...the device might
 * remain in Reset"  This is workaround method 1: use the watchdog
 */
#define MCU_RESET		WDTCON = 0x01u; \
				for(;;) \
				{ \
				    ; \
				}

#define MCU_SLEEP		asm("sleep")

/* Hardware specific defines */
#define LIN_XCVR_EN             (LATC3)
#define LIN_XCVR_TX_EN          (LATC5)

#define BOOST_CONVERTER_ENABLE  (LATA7 = 1)
#define BOOST_CONVERTER_DISABLE (LATA7 = 0)

#define SET_CLOSE_COIL		(LATC1 = 1)
#define SET_OPEN_COIL		(LATC2 = 1)
#define CLR_CLOSE_COIL		(LATC1 = 0)
#define CLR_OPEN_COIL		(LATC2 = 0)
#define TOG_CLOSE_COIL		(PORTC ^= 0x02u)

#define HAZARD_HOLD		(RA4)

#define input_Keep_Alive_1	((uint8_t) ((inputsPortB & 0x02u) >> 1u)) /* RB1 */
#define input_Keep_Alive_2	((uint8_t) ((inputsPortB & 0x04u) >> 2u)) /* RB2 */
#define input_All_Keep_Alives	((uint8_t) (inputsPortB & 0x06u))
#define input_Wake_Up_1		((uint8_t) ((inputsPortB & 0x20u) >> 5u)) /* RB5 */
#define input_Wake_Up_2		((uint8_t) ((inputsPortB & 0x10u) >> 4u)) /* RB4 */
#define input_Wake_Up_3		((uint8_t) ((inputsPortB & 0x08u) >> 3u)) /* RB3 */
#define input_All_Wake_Ups	((uint8_t) (inputsPortB & 0x38u))

#define ALL_INPUTS              (PORTB & 0x3Eu)
#define HIB_WAKE_UPS            (PORTB & 0x38u)
#define INTERRUPT_KEEP_ALIVES   (PORTB & 0x06u)

/* Hardware capabilities */
#define ADC_NUM			3u /* Number of ADC channels to monitor */

/* Functionality defines */
#define SOL_ON_PULSE		10u /* 10mS - bus = 1MHz, /64 timer prescale */
#define SOL_OFF_PULSE		40u /* 40mS */
#define SOL_ON_COOL		2000u /* 2S */
#define SOL_OFF_COOL		1000u /* 1S */
#define SOL_INIT_DELAY		200u /* 200mS - Initial delay from power-on to first solenoid action */

#define SOL_RETRY		5u /* How many times to retry if the solenoid fails to change state */

/* Solenoid phases */
#define SOL_OPEN_COIL		0u
#define SOL_CLOSE_COIL		1u

#define SOLENOID_INIT		0xFFu

/* Solenoid positions */
#define SOLENOID_OPEN		0u
#define SOLENOID_CLOSED		1u

/* Solenoid names */
#define SOLENOID_MAIN1		0u

/* ADC channel names */
/* Used as a/d sample array indexes, update carefully */
#define ADC_VBAT		0u
#define ADC_VLOAD		1u
#define ADC_VBOOST              2u

/* ADC channels */
#define ADC_CHAN_VBAT		0u /* AN0 is input voltage */
#define ADC_CHAN_VLOAD		1u /* AN1 is bus bar voltage */
#define ADC_CHAN_VBOOST         2u /* AN2 is boost converter voltage*/

/* Function return values */
#define SUCCESS			0x00u
#define FAIL			0x01u

#define CLEAR                   0
#define SET                     1

#define WDT_STAGE_MAIN_TOP	0u
#define WDT_STAGE_MAIN_BOT	1u
#define WDT_STAGE_ISR		2u

/* JD specific LIN value defines */
#if 0
#define INPUT_INACTIVE          0u
#define INPUT_ACTIVE            1u
#define INPUT_ERROR             2u
#define INPUT_NA                3u
#endif

#define FAULT_INACTIVE		0u
#define FAULT_ACTIVE		1u

#define E_NONE              0u /* No error */
#define E_STACK             1u /* Stack underflow/overflow */
#define E_WDT_RESET         2u /* Watchdog reset */
#define E_OVERVOLTAGE_16V   3u /* Overvoltage (> 16V) detected */
#define E_OVERVOLTAGE_26_5V 4u /* Overvoltage (> 26.5V) detected */
#define E_UNDERVOLTAGE      5u /* Undervoltage (< 9V) */
#define E_BROWNOUT_RESET    6u /* Brownout reset (< 5V) */
                             /* 7u skipped */
#define E_OVERVOLTAGE_DISC  8u /* Overvoltage (> 16V) disconnect */

/* TODO: Make this a timer driven delay, not a main loop counter */
#define E_DELAY_LOOP_COUNTS 750u /* How many main loops to delay before making a fault active */

#define INTERNAL_STATUS_NORMAL           0u /* Normal */
#define INTERNAL_STATUS_PREV_ERROR       1u /* previous Error */
#define INTERNAL_STATUS_DOOR_TIMER       2u /* Open Door timer running */
#define INTERNAL_STATUS_DOOR_DISC        3u /* Open Door disconnect */
#define INTERNAL_STATUS_OVERV_TIMER      4u /* Overvoltage timer running */
#define INTERNAL_STATUS_OVERV_DISC       5u /* Overvoltage disconnect */
#define INTERNAL_STATUS_ACTIVE_ERROR     14u /* Active Error */
#define INTERNAL_STATUS_NA               15u /* N/A */

#define SW_STATUS_OFF_OFF       0u /* CommandOff_OutputOff */
#define SW_STATUS_ON_OFF        1u /* CommandOn_OutputOff */
#define SW_STATUS_OFF_ON        2u /* CommandOff_OutputOn */
#define SW_STATUS_ON_ON         3u /* CommandOn_OutputOn */
#define SW_STATUS_ERROR         14u /* Error */
#define SW_STATUS_NA            15u /* NotAvailable */

/* Timeout values, 50mS interrupts */
#define TIMEOUT_WU1		6000u /* 5 minutes */
#define TIMEOUT_HIBERNATE	160u  /* 8 seconds */
#define TIMEOUT_OVERV		6000u /* 5 minutes */
#define TIMEOUT_40HR		2400u /* 40 hours, updated in 60 second intervals */
#define TIMEOUT_HAZARD		7u    /* 350ms */
#define TIMEOUT_OPEN            60u   /* 3 seconds */

/* Voltage thresholds */
#define ADC_1V                  33u /* 1.0V: 100k/15k voltage divider, 4.096 ref */
#define ADC_5V			163u /* 5.0V: 100k/15k voltage divider, 4.096 ref */
#define ADC_6V			195u /* 6.0V: 100k/15k voltage divider, 4.096 ref */
				     /* 6.0V is the lowest the LIN transciever can run */
#define ADC_9V			293u /* 9.0V: 100k/15k voltage divider, 4.096 ref */
#define ADC_OVERVOLTAGE		521u /* 16.0V: 100k/15k voltage divider, 4.096 ref */
#define ADC_MAX_VOLTAGE		863u /* 26.5V: 100k/15k voltage divider, 4.096 ref */
#define BATTV_HYSTERESIS	8u   /* 0.25V hysterisis */
#define ADC_BOOST_MIN		206u /* 20.0V: 232k/10k voltage divider, 4.096 ref */
#define ADC_BOOST_INIT		279u /* 27.0V: 232k/10k voltage divider, 4.096 ref */

/* Digital low pass filter constant.  Warning, too big a value will overflow int16_t */
#define LPF_K			32 /* K = (T * SPS), 3mS timer = 333 SPS.  16 = 50mS */

#define LCYCLIC_INT_DISABLE	(TMR2IE = 0)
#define LCYCLIC_INT_ENABLE	(TMR2IE = 1)

#define CAL_TARGET_12V	0u
/*#define CAL_TARGET_24V	1u*/

#define NUM_CAL_SAMPLES 	64u

#define MAX_CAL_VALUE		((int16_t) 25)

#define CAL_SUCCESS		0u
#define CAL_VAL_OUT_OF_RANGE	1u
#define CAL_WRITE_FAIL		2u
/*#define CAL_DATA_ERROR		3u*/

#define CAL_DISABLE_CHAN	0xFFFFu

/*****************************************************************************
 * Types
 *****************************************************************************/
typedef struct
{
    uint8_t position;
    uint8_t requestedPosition;
    uint8_t solenoidFail;
    uint8_t retry;
} SOLENOID_DATA;

typedef struct
{
    uint16_t inputVoltage;
    uint8_t wuInputStatus1;
    uint8_t wuInputStatus2;
    uint8_t wuInputStatus3;
    uint16_t outputVoltage;
    uint8_t kaInputStatus1;
    uint8_t kaInputStatus2;
    uint8_t errorCounter;
    uint8_t errorState;
    uint8_t internalStatus;
    uint8_t switchStatus;
    /*uint8_t reserved;*/
    /*uint8_t heartbeatCounter;*/
    /*unit8_t reserved2;*/
    /*uint8_t CRC;*/
} LIN_DATA;

typedef struct
{
    uint16_t WakeUp1;
    uint16_t Hibernate;
    uint16_t Overvoltage;
    uint16_t Hazard;
    uint16_t Open;
} TIMEOUT_DATA;

typedef struct
{
    uint8_t Active;
    uint16_t faultTimeout;
    uint16_t Delay;
} FAULT_DATA;

typedef struct
{
    FAULT_DATA StackUnderOverflow;
    FAULT_DATA WatchdogReset;
    FAULT_DATA OverVoltage16VDetected;
    FAULT_DATA OverVoltageDisconnect;
    FAULT_DATA OverVoltage26VDetected;
    FAULT_DATA UnderVoltage9V;
    FAULT_DATA BrownoutReset;
    FAULT_DATA OpenDoorTimerRunning;
    FAULT_DATA OpenDoorDisconnect;
} E_STATE_DATA;

typedef struct
{
    int16_t adcCal[8];
} CAL_DATA;

typedef struct
{
    uint8_t SerialNumber[4];
    uint8_t WU1Disable;
    CAL_DATA Calibration;
    /*const uint8_t pad[224];*/ /* Pad the struct out to 256 bytes */
} NVM_EEPROM_MAP;

/*****************************************************************************
 * Prototypes
 *****************************************************************************/
void UpdateLINStatus(LIN_DATA *LINData);
uint8_t UpdateErrorData(uint8_t errors[], uint8_t active[]);
uint8_t GetSwitchStatus(void);
uint16_t ReadTimeout(const volatile uint16_t *timeout);
void DecErrorTimeouts(void);
void WriteTimeout(volatile uint16_t *timeout, uint16_t value);
static uint8_t GetSolenoidPosition(uint8_t solenoid);
static uint8_t SolenoidIsActive(uint8_t solenoid);
static uint8_t SetSolenoid(uint8_t state);
static void SolenoidCheck(uint8_t solenoid);
uint16_t ADCRead(uint8_t chan);
static void ADCReadAll(void);
/*static void ADCReadNext(void);*/
static void ADCApplyCal(void);
void LowPassInit(const volatile int16_t value, volatile int16_t *accum, int16_t K);
int16_t LowPass(const volatile int16_t value, volatile int16_t *accum, int16_t K);
static void DeviceInit(void);
static void wdtKick(uint8_t phase);
static void Hibernate(void);
static void UpdatePowerupFlags(E_STATE_DATA *eData);
void PowerDown(void);
uint8_t AutoCal(uint8_t target);
void C1ISR(void);
void interrupt ISR(void);

/*****************************************************************************
 * Onboard EEPROM memory - 256 bytes
 *****************************************************************************/
static eeprom NVM_EEPROM_MAP NVMData = { {0u, 0u, 0u, 0u},	    /* Serial Number */
					  0u,			    /* WU1Disable */
					 {{0, 0, 0, 0, 0, 0, 0, 0}} /* CAL_DATA */
				       };
static eeprom E_STATE_DATA ErrorData;

/*****************************************************************************
 * External Globals
 *****************************************************************************/
/* Backup copy of STATUS register from startup assembly code */
extern uint8_t __resetbits;

/* The compiler defines the assembly symbols ___powerdown and ___timeout to represent
 * the bit address of the Power-down and Time-out bits within the STATUS register
 * and can be used if required.*/
/*extern bit __powerdown;*/
/*extern bit __timeout;*/

/*****************************************************************************
 * Globals
 *****************************************************************************/
static const uint8_t FirmwareVersion[4] = { 0u, 0u, 0u, 7u };

static volatile SOLENOID_DATA Solenoid;
static volatile uint8_t ISRCoil = SOL_CLOSE_COIL; /* Which coil for solenoid timer ISR to update */
static volatile uint16_t TMR4Counter = SOL_INIT_DELAY;
static volatile TIMEOUT_DATA Timeouts;

static volatile uint8_t Timer1msTick = 0u;
static volatile uint8_t ErrorTimeoutUpdate = 0u;

static volatile uint16_t ADCRaw[ADC_NUM] = {0u, 0u, 0u};
static volatile uint16_t ADClpf[ADC_NUM] = {0u, 0u, 0u};
static volatile int16_t ADCAccum[ADC_NUM];

static volatile uint16_t wdtState = 0u;

/*****************************************************************************
 *
 * FUNCTION NAME:
 * main
 *
 * DESCRIPTION:
 * main
 *
 * RETURN VALUE:
 * main does not return, infinite loop
 *
 *****************************************************************************/
void main(void)
{
    /* LOCAL VARIABLES:
     *
     * Variable			Type		Description
     * --------			----		-----------
     */
    uint16_t tmpStatus = 0u;
    uint8_t heartBeat = 0x00u;  /*0x0Fu; TODO: The JD spec is oddly specific about initalizing this to 0x0F */
    static LIN_DATA LINData;
    uint8_t PanicOpen = 0u;

    uint8_t inputsPortB;

    uint16_t ADCbuf[ADC_NUM] = {0u, 0u, 0u};

    uint8_t u8temp;
    uint16_t u16temp;
    uint8_t WU1_TEMP, WU2_TEMP, WU3_TEMP, COUNT = 0u;
    uint8_t lastWU2 = 0u;

    GIE = CLEAR; /* Disable global interrupts */

    DeviceInit();

     /*  Debounce for IO interrupt */
    WU1_TEMP = input_Wake_Up_1;
    WU2_TEMP = input_Wake_Up_2;
    WU3_TEMP = input_Wake_Up_3;
    
    /* insert a short delay on startup */
    while(COUNT < 2u)
    {
        if(TMR0IF == 1)
        {
            COUNT++;
            TMR0 = 0u;
            TMR0IF = 0;
        }
        
        if((input_Wake_Up_1 != WU1_TEMP)||(input_Wake_Up_2 != WU2_TEMP)||(input_Wake_Up_3 != WU3_TEMP) || (input_Keep_Alive_1 != 0u))
        {
            Hibernate();
        }
        
        if(input_All_Wake_Ups == 0u || (input_Keep_Alive_1 == 0u)) //TODO: change keep alive 1 to reverse pol
        {
           Hibernate();
        }
    }
    
    /* Enable fixed voltage reference to be used by DAC */
    FVRCON = 0x87u; /* FVR enabled, 1.024V for comparator and DAC, 4.096 for ADC */
    u16temp = 0xFFFFu;

    /*
     * When the Fixed Voltage Reference module is enabled, it requires time to stabilize.
     * Once they are stable, FVRRDY will be set. 
     */
    while (FVRRDY == 0) /* comparing bit to integer, the bit is promoted to a signed integer for the operation */
    {
        /* Simple timeout, reset if it gets to zero */
        u16temp--;
        if (u16temp == 0u)
        {
          MCU_RESET
        }
    }

    /*
     * Enable onboard DAC
     */
    DACCON0 = 0x88u; /* Enable DAC, use FVR as source */
    DACCON1 = 21u; /* set the DAC output voltage. This will be the Low Panic Open Voltage */ 

    /*
     * Turn on the boost converter
     */
    BOOST_CONVERTER_ENABLE;

    /*
     * Init LIN
     */
    (void) l_sys_init(); /* always returns 0, ignore */

    /* Init solenoid data */
    Solenoid.position = SOLENOID_INIT;
    Solenoid.requestedPosition = SOLENOID_INIT;
    Solenoid.retry = 0u;
    Solenoid.solenoidFail = 0u;

    /* Init LIN data*/
    LINData.inputVoltage = 0u;
    LINData.wuInputStatus1 = 0u;
    LINData.wuInputStatus2 = 0u;
    LINData.wuInputStatus3 = 0u;
    LINData.outputVoltage = 0u;
    LINData.kaInputStatus1 = 0u;
    LINData.kaInputStatus2 = 0u;
    LINData.errorCounter = 0u;
    LINData.errorState = E_NONE;
    LINData.internalStatus = INTERNAL_STATUS_NA;
    LINData.switchStatus = 0u;

    LowPassInit(0, &ADCAccum[ADC_VBAT], LPF_K);
    LowPassInit(0, &ADCAccum[ADC_VLOAD], LPF_K);
    LowPassInit(0, &ADCAccum[ADC_VBOOST], LPF_K);

    /* Fill in the cause of the last reset for LIN data */
    UpdatePowerupFlags(&ErrorData);

    /* Init device timeouts */
    if (NVMData.WU1Disable != 0u)
    {
        Timeouts.WakeUp1 = 0u;
    }
    else
    {
	Timeouts.WakeUp1 = TIMEOUT_WU1;
    }
    Timeouts.Hibernate = TIMEOUT_HIBERNATE;
    Timeouts.Overvoltage = TIMEOUT_OVERV;

    /* Init Fault data */
    ErrorData.BrownoutReset.Delay = 0u;
    ErrorData.UnderVoltage9V.Delay = 0u;

    GIE = SET; /* Enable global interrupts */

    /* Wait until boost voltage is up */
    u16temp = 0xFFFFu;
    while (ADCRaw[ADC_VBOOST] < ADC_BOOST_INIT) /* 289 = 28V, 299 = 29V */
    {
        ADCRaw[ADC_VBOOST] = ADCRead(ADC_CHAN_VBOOST);

        /* Simple timeout, reset if it gets to zero */
        u16temp--;
        if (u16temp == 0u)
        {
            MCU_RESET
        }
    }

    /* Wait until input voltage is up */
    u16temp = 0xFFFFu;
    while (ADCRaw[ADC_VBAT] < ADC_6V)
    {
        ADCRaw[ADC_VBAT] = ADCRead(ADC_CHAN_VBAT);

        /* Simple timeout, reset if it gets to zero */
        u16temp--;
        if (u16temp == 0u)
        {
            MCU_RESET
        }
    }

    LIN_XCVR_TX_EN = 1; /* TXE, enable transmitter */

    CM1CON0 = 0x86u; /* Enable comparator, high speed mode with hysteresis */
    CM1CON1 = 0x90u; /* Comparator Control Register 1: Interrupt on positive edge, use DAC output as voltage reference, CxVN connects to C12IN0 pin.  */
    C1IF = CLEAR; /* Precautionary: Clear the interrupt flag. */
    C1IE = 1; /* Enable comparator peripheral interrupt */ /* comparing bit to integer, the bit is promoted to a signed integer for the operation */

    
    
    /* buffer PORTB for Hibernate detection*/
    inputsPortB = ALL_INPUTS; /*RB1, RB2, RB3, RB4, RB5*/
    /* 
     * If there is a reverse polarity on the input, the reverse polarity detection input 
     * will be high. If that happens set the comparitor interrupt flag high to immediately 
     * open the relay.
     */
  
    
    if(input_Keep_Alive_1 == 1u) //TODO:update to reverse polarity input
    {
            C1IF = SET;    
    }
    /* check wu and ka, if all inactive go back to hibernate */
    if (input_Wake_Up_2 == 1u) 
    {
        WriteTimeout(&Timeouts.Hazard, TIMEOUT_HAZARD);
    }
    else if ((input_All_Keep_Alives == 0u) && (input_All_Wake_Ups == 0u))
    {
        Hibernate();
    }
    else
    {
        WriteTimeout(&Timeouts.Hazard, 0u);
    }



    /****************************************
     * Main loop
     ****************************************/
    for (;;)
    {
	/* Top stage of WDT kick process */
	wdtState = 0x5555u;
	wdtKick(WDT_STAGE_MAIN_TOP);

        /* buffer PORTB for For loop*/
        inputsPortB = ALL_INPUTS; /* RB1, RB2, RB3, RB4, RB5 */

	/*need to set PORTA4 to an output and low in DeviceInit(); */
	/*poll wu2, 350ms output on PORTA4 on low->high transition */
	if ((Timeouts.Hazard == 0u) && ((input_Wake_Up_2 == 1u) && (lastWU2 == 0u)))
	{
	    WriteTimeout(&Timeouts.Hazard, TIMEOUT_HAZARD);
	}
	
        lastWU2 = input_Wake_Up_2;

	if ((Timeouts.Hazard != 0u)) /* && (HAZARD_HOLD != 1)) */
	{
	    HAZARD_HOLD = 1;
	}

	else
	{
	    HAZARD_HOLD = 0;
	}

	/* Update ADC values */
	ADCReadAll();
	/*ADCReadNext();*/ /* Read 1 ADC channel per loop to improve LIN latency */
	ADCApplyCal();

	/* Buffer ADC values so each main loop uses the same data for calculations */
	LCYCLIC_INT_DISABLE; /* ADC low pass filter is in the cyclic interrupt timer */
	ADCbuf[ADC_VBAT] = ADClpf[ADC_VBAT];
	ADCbuf[ADC_VLOAD] = ADClpf[ADC_VLOAD];
	ADCbuf[ADC_VBOOST] = ADClpf[ADC_VBOOST];
	LCYCLIC_INT_ENABLE;


	/*
	 * Check for over voltage events
	 */
	if (SolenoidIsActive(SOLENOID_MAIN1) == 0u) /* Don't sample when solenoid is active to avoid noise */
	{
	    /* If input voltage is less than 5V, set the error code.  This error
	     * code is also set if the uC brownout was detected after boot */
	    if (ADCbuf[ADC_VBAT] < ADC_5V)
	    {
            if (ErrorData.BrownoutReset.Delay < E_DELAY_LOOP_COUNTS)
            {
             ErrorData.BrownoutReset.Delay++;
            }
        	else
            {
               if (ErrorData.BrownoutReset.Active != FAULT_ACTIVE)
               {
                    ErrorData.BrownoutReset.Active = FAULT_ACTIVE;
               }

               if (ErrorData.BrownoutReset.faultTimeout != TIMEOUT_40HR)
               {
                    ErrorData.BrownoutReset.faultTimeout = TIMEOUT_40HR;
		       }
            }
	    }
	    else
	    {
            if (ErrorData.BrownoutReset.Delay != 0u)
            {
                ErrorData.BrownoutReset.Delay = 0u;
        	}
            if (ErrorData.BrownoutReset.Active != FAULT_INACTIVE)
            {
              ErrorData.BrownoutReset.Active = FAULT_INACTIVE;
            }
	    }

	    /* If input voltage is less than 9V, set the error code */
	    if (ADCbuf[ADC_VBAT] < ADC_9V)
	    {
            if (ErrorData.UnderVoltage9V.Delay < E_DELAY_LOOP_COUNTS)
            {
                ErrorData.UnderVoltage9V.Delay++;
            }
            else
        	{
                if (ErrorData.UnderVoltage9V.Active != FAULT_ACTIVE)
               {
            	ErrorData.UnderVoltage9V.Active = FAULT_ACTIVE;
               }

               if (ErrorData.UnderVoltage9V.faultTimeout != TIMEOUT_40HR)
              {
                ErrorData.UnderVoltage9V.faultTimeout = TIMEOUT_40HR;
              }
            }
	    }
	    else
	    {
            if (ErrorData.UnderVoltage9V.Delay != 0u)
            {
                ErrorData.UnderVoltage9V.Delay = 0u;
            }
            if (ErrorData.UnderVoltage9V.Active != FAULT_INACTIVE)
            {
               ErrorData.UnderVoltage9V.Active = FAULT_INACTIVE;
            }
	    }

	    /* If input voltage is greater than 16V and less than 26.5V, open in 5 minutes */
	    if (ADCbuf[ADC_VBAT] >= ADC_OVERVOLTAGE)
	    {
            if (ErrorData.OverVoltage16VDetected.Active != FAULT_ACTIVE)
            {
                ErrorData.OverVoltage16VDetected.Active = FAULT_ACTIVE;
            }

            if (ErrorData.OverVoltage16VDetected.faultTimeout != TIMEOUT_40HR)
            {
                ErrorData.OverVoltage16VDetected.faultTimeout = TIMEOUT_40HR;
            }
	    }
	    else if (ADCbuf[ADC_VBAT] < (ADC_OVERVOLTAGE - BATTV_HYSTERESIS))
	    {
            WriteTimeout(&Timeouts.Overvoltage, TIMEOUT_OVERV);
            if (ErrorData.OverVoltage16VDetected.Active != FAULT_INACTIVE)
            {
               ErrorData.OverVoltage16VDetected.Active = FAULT_INACTIVE;
            }
	    }
	    else
	    {
            /* Hysteresis zone, if there's no fault active, reset the timeout */
            if (ErrorData.OverVoltage16VDetected.Active == FAULT_INACTIVE)
            {
               WriteTimeout(&Timeouts.Overvoltage, TIMEOUT_OVERV);
            }
	    }

	    /* If input voltage is greater than 26.5V, open immediately */
	    if (ADCbuf[ADC_VBAT] >= ADC_MAX_VOLTAGE)
	    {
            PanicOpen = 1u;

            if (ErrorData.OverVoltage26VDetected.Active != FAULT_ACTIVE)
            {
                ErrorData.OverVoltage26VDetected.Active = FAULT_ACTIVE;
            }

            if (ErrorData.OverVoltage26VDetected.faultTimeout != TIMEOUT_40HR)
            {
                ErrorData.OverVoltage26VDetected.faultTimeout = TIMEOUT_40HR;
            }

	    }
	    else if (ADCbuf[ADC_VBAT] < (ADC_MAX_VOLTAGE - BATTV_HYSTERESIS))
	    {
            PanicOpen = 0u;
            if (ErrorData.OverVoltage26VDetected.Active != FAULT_INACTIVE)
            {
                ErrorData.OverVoltage26VDetected.Active = FAULT_INACTIVE;
            }
        }
	    else
	    {
            ; /* Don't change in hysteresis zone */
	    }
	}

	/* If boost voltage is sagging, open immediately */
	if (ADClpf[ADC_VBOOST] < ADC_BOOST_MIN)
	{
	    PanicOpen = 1u;
	}

	/*
	 * Check for wakeup1 (door switch input) timeout
	 */
	if (input_Wake_Up_1 == 0u)
	{
	    /* wakeup1 is inactive, make sure it's not disabled and reset the timeout */
	    if (NVMData.WU1Disable != 0u)
	    {
		WriteTimeout(&Timeouts.WakeUp1, TIMEOUT_WU1);
		NVMData.WU1Disable = 0u;
	    }

	    if (ErrorData.OpenDoorTimerRunning.Active != FAULT_INACTIVE)
	    {
		ErrorData.OpenDoorTimerRunning.Active = FAULT_INACTIVE;
	    }
	    if (ErrorData.OpenDoorTimerRunning.faultTimeout != TIMEOUT_40HR)
	    {
		ErrorData.OpenDoorTimerRunning.faultTimeout = TIMEOUT_40HR;
	    }
	}
	else
	{
	    /* wakeup1 is active, update the error codes */
	    if (ErrorData.OpenDoorTimerRunning.Active != FAULT_ACTIVE)
	    {
		ErrorData.OpenDoorTimerRunning.Active = FAULT_ACTIVE;
	    }
	    if (ErrorData.OpenDoorTimerRunning.faultTimeout != TIMEOUT_40HR)
	    {
		ErrorData.OpenDoorTimerRunning.faultTimeout = TIMEOUT_40HR;
	    }
	    if (ErrorData.OpenDoorDisconnect.Active != FAULT_INACTIVE)
	    {
		ErrorData.OpenDoorDisconnect.Active = FAULT_INACTIVE;
	    }

	    u16temp = ReadTimeout(&Timeouts.WakeUp1);
	    if (u16temp == 0u)
	    {
		/* WU1 has timed out, and should be disabled */
		if (NVMData.WU1Disable != 1u)
		{
		    NVMData.WU1Disable = 1u;
		}

		if (ErrorData.OpenDoorDisconnect.Active != FAULT_ACTIVE)
		{
		    ErrorData.OpenDoorDisconnect.Active = FAULT_ACTIVE;
		}
		if (ErrorData.OpenDoorDisconnect.faultTimeout != TIMEOUT_40HR)
		{
		    ErrorData.OpenDoorDisconnect.faultTimeout = TIMEOUT_40HR;
		}
		if (ErrorData.OpenDoorTimerRunning.Active != FAULT_INACTIVE)
		{
		    ErrorData.OpenDoorTimerRunning.Active = FAULT_INACTIVE;
		}
	    }
	}

	/*
	 * Determine current solenoid position
	 */
	u8temp = GetSolenoidPosition(SOLENOID_MAIN1);
	if (u8temp != 0xFFu)
	{
	    Solenoid.position = u8temp;
	}
	else
	{
	    /* If the return value came back as 0xFF, GetSolenoidPosition failed */
	    Solenoid.position = SOLENOID_INIT;
	}

	/* Update requested solenoid position.
	 *
	 * Wake-ups are active low, Keep-alives are active high, hardware
	 * handles the bit inversion so 0 = inactive and 1 = active.
	 *
	 * Wakeup1 is the door position detect, and has a 5 minute timeout
	 * specified before it is ignored (in case the door is not installed).
	 */
	u16temp = ReadTimeout(&Timeouts.Overvoltage);
	if (u16temp == 0u)
	{
	    if (ErrorData.OverVoltageDisconnect.Active != FAULT_ACTIVE)
	    {
		ErrorData.OverVoltageDisconnect.Active = FAULT_ACTIVE;
	    }
	    if (ErrorData.OverVoltageDisconnect.faultTimeout != TIMEOUT_40HR)
	    {
		ErrorData.OverVoltageDisconnect.faultTimeout = TIMEOUT_40HR;
	    }
	}
	else
	{
	    if (ErrorData.OverVoltageDisconnect.Active != FAULT_INACTIVE)
	    {
		ErrorData.OverVoltageDisconnect.Active = FAULT_INACTIVE;
	    }
	}

	if ((PanicOpen == 1u) || ((u16temp == 0u) && (input_Wake_Up_3 == 0u)))
	{
	    Solenoid.requestedPosition = SOLENOID_OPEN;

	    WriteTimeout(&Timeouts.Hibernate, TIMEOUT_HIBERNATE);
	}

	else if (((ReadTimeout(&Timeouts.WakeUp1) > 0u) && (input_Wake_Up_1 == 1u)) ||
        (input_Keep_Alive_1 == 1u) || (input_Keep_Alive_2 == 1u) ||
		(input_Wake_Up_2 == 1u) || (input_Wake_Up_3 == 1u))
	{
	    Solenoid.requestedPosition = SOLENOID_CLOSED;
	    WriteTimeout(&Timeouts.Hibernate, TIMEOUT_HIBERNATE);
        WriteTimeout(&Timeouts.Open, TIMEOUT_OPEN);
	}

	else if (ReadTimeout(&Timeouts.Open) == 0u)
	{
        Solenoid.requestedPosition = SOLENOID_OPEN;
	}

    else
    {
                /* MIRSA else statement */
    }

	/* Scan solenoid position, retry if the bus bar is in the
	 * wrong position, set the error flag if the bus bar is still in the
	 * wrong position after the retries
	 */

 
    if(input_Keep_Alive_1 == 1u) //TODO: Update KA1 input to reverse pol 
    {
        Solenoid.requestedPosition = SOLENOID_OPEN;
 
    }

    SolenoidCheck(SOLENOID_MAIN1);
	/*
	 *  Update LIN status data
	 *
	 * data[0][1]...[7]
	 * [AAAA AAAA] [DDCC BBAA] [EEEE EEEE] [HHGG FFEE] [JJJJ JJJJ] [LLLL KKKK] [NNNN MMMM] [QQQ QQQP]
	 *
	 * A: input voltage
	 * B: wake up 1
	 * C: wake up 2
	 * D: wake up 3
	 * E: output voltage
	 * F: keep alive 1
	 * G: keep alive 2
	 * H: error counter
	 * J: error state
	 * K: internal status
	 * L: switch status
	 * M: reserved
	 * N: heartbeat counter
	 * P: reserved
	 * Q: CRC
	 */
	/* Do these calculations outside the 1ms timer loop for better performance */
	u16temp = ADCbuf[ADC_VBAT];
	LINData.inputVoltage = (uint16_t)(((uint32_t)u16temp * (uint32_t)307u) / (uint32_t)500u) & 0x3FFu;
	LINData.wuInputStatus1 = input_Wake_Up_1 & 0x03u;
	LINData.wuInputStatus2 = input_Wake_Up_2 & 0x03u;
	LINData.wuInputStatus3 = input_Wake_Up_3 & 0x03u;
	u16temp = ADCbuf[ADC_VLOAD];
	LINData.outputVoltage = (uint16_t)(((uint32_t)u16temp * (uint32_t)307u) / (uint32_t)500u) & 0x3FFu;
	LINData.kaInputStatus1 = input_Keep_Alive_1 & 0x03u;
	LINData.kaInputStatus2 = input_Keep_Alive_2 & 0x03u;
	LINData.switchStatus = GetSwitchStatus();

	/*
	 * 1ms Task Cycle
	 */
	if (Timer1msTick)
	{
	    Timer1msTick = 0u;

	    /* Get status of lin driver from status word, see LIN specification
	     * for more detailed information */
	    tmpStatus = l_ifc_read_status();

	    /* Check for successful transmission (reception or transmission of a frame) */
	    if (tmpStatus & 0x02u)
	    {
		/* Every time a frame is received, increment heartBeat */
		if ((tmpStatus & 0x3F00u) == 0x2100u)
		{
		    heartBeat++;
		    if (heartBeat > 0x0Eu)
		    {
			heartBeat = 0u;
		    }

		    /* These values need to be updated each time a frame is received */
		    UpdateLINStatus(&LINData);
		}
	    }
	    /* Update LIN data*/
	    LCYCLIC_INT_DISABLE; /* Don't allow l_cyclic_com_task() interrupt during LIN packet data update */            
	    l_u16_wr_inputVoltage(LINData.inputVoltage);	/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_wu_Input_Status1(LINData.wuInputStatus1);	/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_wu_Input_Status2(LINData.wuInputStatus2);	/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_wu_Input_Status3(LINData.wuInputStatus3);	/*lint !e717 !e734 !e835 !e960 */
	    l_u16_wr_outputVoltage(LINData.outputVoltage);	/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_ka_Input_Status1(LINData.kaInputStatus1);	/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_ka_Input_Status2(LINData.kaInputStatus2);	/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_errorCounter(LINData.errorCounter);		/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_errorState(LINData.errorState);		/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_internalStatus(LINData.internalStatus);	/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_SwitchStatus(LINData.switchStatus);		/*lint !e717 !e734 !e835 !e960 */
	    l_u8_wr_Reserved(0x00u);				/*lint !e717 !e734 !e835 !e845 !e960 */
	    l_u8_wr_Heartbeat_Counter(heartBeat & 0x0Fu);	/*lint !e717 !e734 !e835 !e960 */
	    l_bool_wr_Reserved2(0x00u);				/*lint !e717 !e734 !e835 !e845 !e960 */
	    l_u8_wr_CRC(calcJD_CRC(l_LinData.frames.l_frm_BatteryCutOffStatus.data) & 0x7Fu);	/*lint !e717 !e734 !e835 !e960 */
	    LCYCLIC_INT_ENABLE;

	    /* Call the cyclic task according to the configured value
	     * (field "LIN Task Cycletime" in the LDC-Tool) */
	    ld_task(); /*lint !e960 */
	}

	/*
	 * Check on Error timeouts
	 */
	if (ErrorTimeoutUpdate != 0u)
	{
	    ErrorTimeoutUpdate = 0u;

	    DecErrorTimeouts();
	}

	/* Bottom stage of WDT kick process */
	wdtState += 0x2222u;
	wdtKick(WDT_STAGE_MAIN_BOT);

	/* Check for hibernate */
	u16temp = ReadTimeout(&Timeouts.Hibernate);

	if (u16temp == 0u)
	{
	    Hibernate();
	}
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void UpdateLINStatus(LIN_DATA *LINData)
{
    static uint8_t eCnt = 0u;
    static uint8_t eArray[4] = { 0u, 0u, 0u, 0u};
    static uint8_t eStatus[4] = { 0u, 0u, 0u, 0u};
    static uint8_t intStateIdx = 0u;
    uint8_t intStatus = INTERNAL_STATUS_NORMAL;
    uint8_t errState = E_NONE;
    uint8_t errCounter = 0u;
    uint16_t u16temp;

    uint8_t intStateCnt = 0u;

    /* Loop through the switch statement searching for InternalStatus other than normal */
    do
    {
	switch (intStateIdx)
	{
	case 0:
	    if (ErrorData.OpenDoorTimerRunning.faultTimeout != 0u)
	    {
		errState = E_NONE;

		/* Check if it's active or previous */
		if (ErrorData.OpenDoorTimerRunning.Active != FAULT_INACTIVE)
		{
		    intStatus = INTERNAL_STATUS_DOOR_TIMER;
		}
	    }
	    else if ((ErrorData.OpenDoorDisconnect.faultTimeout != 0u))
	    {
		/*  */
		if ((Solenoid.position == SOLENOID_OPEN) && (ErrorData.OpenDoorDisconnect.Active == FAULT_ACTIVE) && (ErrorData.OverVoltageDisconnect.Active == FAULT_INACTIVE))
		{
		    errState = E_NONE;
		    /* Only set open door disconnect if the solenoid is opened */
		    intStatus = INTERNAL_STATUS_DOOR_DISC;
		}
	    }
	    else
	    {
		; /* Do nothing */
	    }

	    intStateIdx++;
	    intStateCnt++;
	    break;

	case 1:
	    u16temp = ReadTimeout(&Timeouts.Overvoltage); /* Read the overvoltage timer value */
	    if (u16temp < (TIMEOUT_OVERV - 50u))
	    {
		if (u16temp != 0u)
		{
		    errState = E_NONE;
		    intStatus = INTERNAL_STATUS_OVERV_TIMER;
		}
	    }

	    intStateIdx++;
	    intStateCnt++;
	    break;

	case 2:
	    /* Stay in this state until all detected (up to 4) errors are transmitted */
	    if (eCnt == 0u)
	    {
		eCnt = UpdateErrorData(eArray, eStatus);
	    }

	    if (eCnt > 0u)
	    {
		eCnt--;
		errCounter = (eCnt & 0x03u);
		errState = eArray[3u - eCnt];
		intStatus = eStatus[3u - eCnt];
	    }

	    if (eCnt == 0u)
	    {
		errCounter = 0u;
		intStateIdx = 0u;
	    }
	    intStateCnt++;
	    break;

	default:
	    intStateIdx = 0u;
	    break;
	}
    } while ((intStatus == INTERNAL_STATUS_NORMAL) && (intStateCnt < 3u));

    LINData->internalStatus = intStatus;
    LINData->errorState = errState;
    LINData->errorCounter = errCounter;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
uint8_t UpdateErrorData(uint8_t errors[], uint8_t active[])
{
    uint8_t eIdx = 4u;
    uint16_t u16temp;

    /* "Stack underflow/overflow"
     * This shall be reported if the ?Stack underflow/overflow? condition has
     * occurred
     */
    if (ErrorData.StackUnderOverflow.faultTimeout != 0u)
    {
	eIdx--;
	errors[eIdx] = E_STACK;
	active[eIdx] = INTERNAL_STATUS_PREV_ERROR;
    }

    /* "Watchdog reset"
     * This shall be reported if a watchdog reset has occurred, this can never be
     * an active error. See chapter 2.4.8 - Hardware Watchdog for definition of
     * watchdog reset.
     */
    if ((ErrorData.WatchdogReset.faultTimeout != 0u) && (eIdx > 0u))
    {
	eIdx--;
	errors[eIdx] = E_WDT_RESET;
	active[eIdx] = INTERNAL_STATUS_PREV_ERROR;
    }

    /* "Overvoltage (> 16 V) detected"
     * This shall be reported, if overvoltage (>16V) has been detected on Power
     * Input.
     */
    if ((ErrorData.OverVoltage16VDetected.faultTimeout != 0u) && (eIdx > 0u))
    {
	eIdx--;
	errors[eIdx] = E_OVERVOLTAGE_16V;

	/* Check if it's active or previous */
	if (ErrorData.OverVoltage16VDetected.Active != FAULT_INACTIVE)
	{
	    active[eIdx] = INTERNAL_STATUS_ACTIVE_ERROR;
	}
	else
	{
	    active[eIdx] = INTERNAL_STATUS_PREV_ERROR;
	}
    }

    /* "Overvoltage (> 26,5 V) detected"
     * This shall be reported, if overvoltage (>26,5V) has been detected on Power
     * Input.
     */
    if ((ErrorData.OverVoltage26VDetected.faultTimeout != 0u) && (eIdx > 0u))
    {
	eIdx--;
	errors[eIdx] = E_OVERVOLTAGE_26_5V;

	/* Check if it's active or previous */
	if (ErrorData.OverVoltage26VDetected.Active != FAULT_INACTIVE)
	{
	    active[eIdx] = INTERNAL_STATUS_ACTIVE_ERROR;
	}
	else
	{
	    active[eIdx] = INTERNAL_STATUS_PREV_ERROR;
	}
    }

    /* "Undervoltage (U < 9V)"
     * This shall be reported, if undervoltage (<9V) has been detected on Power
     * Input.
     */
    if ((ErrorData.UnderVoltage9V.faultTimeout != 0u) && (eIdx > 0u))
    {
	eIdx--;
	errors[eIdx] = E_UNDERVOLTAGE;

	/* Check if it's active or previous */
	if (ErrorData.UnderVoltage9V.Active != FAULT_INACTIVE)
	{
	    active[eIdx] = INTERNAL_STATUS_ACTIVE_ERROR;
	}
	else
	{
	    active[eIdx] = INTERNAL_STATUS_PREV_ERROR;
	}
    }

    /* "Brown out reset (U < 5V)"
     * This shall be reported, if undervoltage (<5V) has been detected on Power
     / Input.
     */
    if ((ErrorData.BrownoutReset.faultTimeout != 0u) && (eIdx > 0u))
    {
	eIdx--;
	errors[eIdx] = E_BROWNOUT_RESET;
	active[eIdx] = INTERNAL_STATUS_PREV_ERROR;
    }

    /* "Overvoltage (> 16 V) disconnect"
     * This shall be reported, if the Battery Disconnect Relay switched off Power
     * Output due to expiration of the ?Overvoltage timer? previously.
     */
    if ((ErrorData.OverVoltageDisconnect.faultTimeout != 0u) && (eIdx > 0u))
    {
	eIdx--;

	u16temp = ReadTimeout(&Timeouts.Overvoltage); /* Read the overvoltage timer value */
	if (u16temp == 0u)
	{
	    if (ErrorData.OverVoltage26VDetected.Active == FAULT_INACTIVE)
	    {
		errors[eIdx] = E_OVERVOLTAGE_16V;
	    }
	    else
	    {
		errors[eIdx] = E_OVERVOLTAGE_26_5V;
	    }
	    active[eIdx] = INTERNAL_STATUS_OVERV_DISC;
	}
	else
	{
	    errors[eIdx] = E_OVERVOLTAGE_DISC;
	    active[eIdx] = INTERNAL_STATUS_PREV_ERROR;
	}
    }

    return (4u - eIdx); /* return ErrorCount */
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void DecErrorTimeouts(void)
{
    /* Brownout can never be an active error */
    if (ErrorData.BrownoutReset.faultTimeout != 0u)
    {
	ErrorData.BrownoutReset.faultTimeout--;
    }

    if (ErrorData.StackUnderOverflow.faultTimeout != 0u)
    {
	ErrorData.StackUnderOverflow.faultTimeout--;
    }

    if (ErrorData.WatchdogReset.faultTimeout != 0u)
    {
	ErrorData.WatchdogReset.faultTimeout--;
    }

    if (ErrorData.OverVoltage16VDetected.Active == FAULT_INACTIVE)
    {
	if (ErrorData.OverVoltage16VDetected.faultTimeout != 0u)
	{
	    ErrorData.OverVoltage16VDetected.faultTimeout--;
	}
    }

    if (ErrorData.OverVoltage26VDetected.Active == FAULT_INACTIVE)
    {
	if (ErrorData.OverVoltage26VDetected.faultTimeout != 0u)
	{
	    ErrorData.OverVoltage26VDetected.faultTimeout--;
	}
    }

    if (ErrorData.OverVoltageDisconnect.Active == FAULT_INACTIVE)
    {
	if (ErrorData.OverVoltageDisconnect.faultTimeout != 0u)
	{
	    ErrorData.OverVoltageDisconnect.faultTimeout--;
	}
    }

    if (ErrorData.UnderVoltage9V.Active == FAULT_INACTIVE)
    {
	if (ErrorData.UnderVoltage9V.faultTimeout != 0u)
	{
	    ErrorData.UnderVoltage9V.faultTimeout--;
	}
    }

    if (ErrorData.OpenDoorDisconnect.Active == FAULT_INACTIVE)
    {
	if (ErrorData.OpenDoorDisconnect.faultTimeout != 0u)
	{
	    ErrorData.OpenDoorDisconnect.faultTimeout--;
	}
    }

    if (ErrorData.OpenDoorTimerRunning.Active == FAULT_INACTIVE)
    {
	if (ErrorData.OpenDoorTimerRunning.faultTimeout != 0u)
	{
	    ErrorData.OpenDoorTimerRunning.faultTimeout--;
	}
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
uint8_t GetSwitchStatus(void)
{
    uint8_t retval = SW_STATUS_NA;

    if (Solenoid.requestedPosition == SOLENOID_OPEN)
    {
	if (Solenoid.position == SOLENOID_OPEN)
	{
	    retval = SW_STATUS_OFF_OFF;
	}
	else
	{
	    retval = SW_STATUS_OFF_ON;
	}
    }
    else if (Solenoid.requestedPosition == SOLENOID_CLOSED)
    {
	if (Solenoid.position == SOLENOID_CLOSED)
	{
	    retval = SW_STATUS_ON_ON;
	}
	else
	{
	    retval = SW_STATUS_ON_OFF;
	}
    }
    else
    {
	if (Solenoid.requestedPosition != SOLENOID_INIT)
	{
	    retval = SW_STATUS_ERROR;
	}
    }

    return retval;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Reading a uint16 in main can be corrupted when a timer interrupt occurs.
 *  This function disables the timer, reads the value, then re-enables the
 *  timer to ensure a valid read.
 *
 *  Timeout is read in 4 opcodes, an interrupt in the middle would corrupt
 *  the returned value (compiler version 1.20):
 *  	moviw	[0]fsr1
 *	movwf	(ReadTimeout@retval)
 *	moviw	[1]fsr1
 *	movwf	(ReadTimeout@retval+1)
 *
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
uint16_t ReadTimeout(const volatile uint16_t *timeout)
{
    uint16_t retval;

    TMR6IE = 0; /* Multi-instruction variable update, don't let a timer interrupt mess it up */
    retval = *timeout;
    TMR6IE = 1; /* Variable updated, turn the interrupt back on */

    return retval;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Writing a uint16 in main can be corrupted when a timer interrupt occurs.
 *  This function disables the timer, writes the value, then re-enables the
 *  timer to ensure a valid write.
 *
 *  Timeout is written in 4 opcodes, an interrupt in the middle would corrupt
 *  the written data (compiler version 1.20):
 * 	movf	(WriteTimeout@value),w
 *	movwi	[0]fsr1
 *	movf	(WriteTimeout@value+1),w
 *	movwi	[1]fsr1
 *
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void WriteTimeout(volatile uint16_t *timeout, uint16_t value)
{
    TMR6IE = 0; /* Multi-instruction variable update, don't let a timer interrupt mess it up */
    *timeout = value;
    TMR6IE = 1; /* Variable updated, turn the interrupt back on */
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 * GetSolenoidPosition
 *
 * DESCRIPTION:
 * This function gets the current position of the solenoid
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 * solenoid	uint8_t		I	name of which solenoid to check
 *
 * RETURN VALUE:
 * uint8_t - solenoid position, either opened or closed
 *
 *****************************************************************************/
static uint8_t GetSolenoidPosition(uint8_t solenoid)
{
    /* LOCAL VARIABLES:
     *
     * Variable     Type	Description
     * --------     ----	-----------
     * retval       uint8_t	temporary variable to hold return value
     * load         uint16_t    temporary varibale to hold load voltage
     * batt         uint16_t    temporary varibale to hold batt voltage
     */
    uint8_t retval = 0xFFu;
    uint16_t load = ADCRaw[ADC_VLOAD];
    uint16_t batt = ADCRaw[ADC_VBAT];

    switch (solenoid)
    {
    case SOLENOID_MAIN1:

        /* This method checks the voltage to determine state */
        if (load >= batt)  /*TODO: improved position detection */
        {
            if ((load - batt) <= ADC_1V)
            {
                retval = SOLENOID_CLOSED;
            }
            else
            {
                retval = SOLENOID_OPEN;
            }
        }

        else
        {
            if ((batt - load) <= ADC_1V)
            {
                retval = SOLENOID_CLOSED;
            }
            else
            {
                retval = SOLENOID_OPEN;
            }
        }

	break;

    default:
        break;
    }

    return retval;

}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *  SolenoidIsActive
 *
 * DESCRIPTION:
 *  Determines if the solenoid timer is active or not
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *  uint8_t - 0 if inactive, 1 if active
 *
 *****************************************************************************/
static uint8_t SolenoidIsActive(uint8_t solenoid)
{
    uint8_t retval = 1u; /* Initialize to active */

    switch (solenoid)
    {
    case SOLENOID_MAIN1:
        if (TMR4ON == 0)
        {
            retval = 0u;
        }
        break;

    default:
        retval = 0u;
        break;
    }
    return retval;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 * SetSolenoid
 *
 * DESCRIPTION:
 * This function sets the current position of the solenoid
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 * state	uint8_t		I	which state to set the solenoid to
 *
 * RETURN VALUE:
 * uint8_t indicating success or fail (timer busy)
 *
 *****************************************************************************/
static uint8_t SetSolenoid(uint8_t state)
{
    /* LOCAL VARIABLES:
     *
     * Variable			Type		Description
     * --------			----		-----------
     * retval			uint8_t		temporary variable to hold return value
     */
    uint8_t retval = FAIL;

    if (state == SOLENOID_CLOSED)
    {
        if (SolenoidIsActive(SOLENOID_MAIN1) == 0u) /* Don't execute if the timer is active */
        {
            ISRCoil = SOL_CLOSE_COIL; /* Tell the ISR which coil to energize */
            SET_CLOSE_COIL;

            /* The coil is now energized, set the timer to de-energize it */
            TMR4Counter = SOL_ON_PULSE;
            TMR4ON = 1;

            retval = SUCCESS;
        }
    }
    else if (state == SOLENOID_OPEN)
    {
        if (SolenoidIsActive(SOLENOID_MAIN1) == 0u) /* Don't execute if the timer is active */
        {
            ISRCoil = SOL_OPEN_COIL; /* Tell the ISR which coil to energize */
            SET_OPEN_COIL;

            /* The coil is now energized, set the timer to de-energize it */
            TMR4Counter = SOL_OFF_PULSE;
            TMR4ON = 1;

            retval = SUCCESS;
        }
    }
    else
    {
        /* Do nothing if the state is not open or closed */
    }

    return retval;
}
/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
static void SolenoidCheck(uint8_t solenoid)
{
    uint8_t pos = Solenoid.position;
    uint8_t reqpos = Solenoid.requestedPosition;
         
    if((PORTB & 0x02) == 0x02) //TODO: Update KA1 input to reverse pol
 {
    if (SolenoidIsActive(solenoid) == 0u) /* Don't do anything if the solenoid timer is busy */
      {
            (void) SetSolenoid(SOLENOID_OPEN);
       }
}
 else
 {
    if (pos != reqpos)
    {
        if (SolenoidIsActive(solenoid) == 0u) /* Don't do anything if the solenoid timer is busy */
        {
            /* The control switch position and solenoid position don't match, change the solenoid position */
            if (Solenoid.requestedPosition == SOLENOID_CLOSED)
            {
                /* Change the solenoid to closed.  SolenoidRetry is incremented each time the control switch
                 * and solenoid position don't match.  If retry is zero, this is the first time this loop has
                 * been run since a successful solenoid position change, if it's nonzero, the solenoid failed
                 * to change state and this statement has been run again.  Cycle the solenoid off/on until max
                 * number of retries has been hit.  One retry in this phase is 2 cycles (one open, one close),
                 * so execute this loop 2*retry times.
                 */
                if (Solenoid.retry <= (SOL_RETRY * 2u))
                {
                    /* If the solenoid didn't close, the contacts might be dirty.  Try to hammer the
                     * solenoid off/on.  'retry' is incremented each cycle, so if the lower bit is zero, close
                     * it.  If the lower bit is 1, try to open it until max retries are hit
                     */
                    if ((Solenoid.retry & 0x01u) == 0u)
                    {
                        (void) SetSolenoid(SOLENOID_CLOSED);
                    }
                    else
                    {
                        (void) SetSolenoid(SOLENOID_OPEN);
                    }
                }
                else
                {
                    Solenoid.solenoidFail = 1u;
                }
            }
            else if (Solenoid.requestedPosition == SOLENOID_OPEN)
            {
                /* If the solenoid didn't open, the contacts might be welded together.  Try to open it
                 * until max retries are hit.
                 */
                if (Solenoid.retry <= SOL_RETRY)
                {
                    (void) SetSolenoid(SOLENOID_OPEN);
                }
                else
                {
                    Solenoid.solenoidFail = 1u;
                }
            }
            else if (Solenoid.requestedPosition == SOLENOID_INIT)
            {
                Solenoid.solenoidFail = 0u;
                Solenoid.retry = 0u;
            }
            else
            {
                /* Do nothing if requestedPostion is not open, close, or init. */
            }

            /* Increment retry every time the control switch and solenoid position don't match
             * Solenoid close retry has two counts per cycle, so increment until 2*retry
             */
            if (Solenoid.retry <= (SOL_RETRY * 2u))
            {
                Solenoid.retry++;
            }
        }
    }
    else
    {
        /* Control switch and solenoid position match, clear the retry counter */
        Solenoid.retry = 0u;
        Solenoid.solenoidFail = 0u;
    }
 }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *  ADCReadAll
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument			Type		IO	Description
 * --------			----		--	-----------
 *
 * RETURN VALUE: void
 *
 *****************************************************************************/
uint16_t ADCRead(uint8_t chan)
{
    uint16_t retval;
    uint16_t u16temp = 0xFFFFu;
    uint8_t i;

    ADCON0 &= ~0x7Cu; /* Clear channel bits */
    chan <<= 2;
    ADCON0 |= chan;

    for(i = 25u; i > 0u; i--)
    {
        ;
    }

    ADCON0 |= 0x02u; /* Start the conversion */
    while ((ADCON0 & 0x02u) == 0x02u)
    {
        /* Wait for conversion complete */

	/* Simple timeout, reset if it gets to zero */
	u16temp--;
	if (u16temp == 0u)
	{
	    MCU_RESET
	}
    }

    retval = ADRES;

    return retval;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *  ADCReadAll
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument			Type		IO	Description
 * --------			----		--	-----------
 *
 * RETURN VALUE: void
 *
 *****************************************************************************/
static void ADCReadAll(void)
{
    static const uint8_t adcReadList[] = {ADC_CHAN_VBAT, ADC_CHAN_VLOAD, ADC_CHAN_VBOOST};
    static const uint8_t readListSize = 3u;

    uint8_t chanidx;
    uint8_t chan;

    for (chanidx = 0u; chanidx < readListSize; chanidx++)
    {
        /* Read the channel */
        chan = adcReadList[chanidx];
        ADCRaw[chan] = ADCRead(chan); /* ADCRead returns the adc value */
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *  ADCReadAll
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument			Type		IO	Description
 * --------			----		--	-----------
 *
 * RETURN VALUE: void
 *
 *****************************************************************************/
#if 0
static void ADCReadNext(void)
{
    static const uint8_t adcReadList[] = {ADC_CHAN_VBAT, ADC_CHAN_VLOAD, ADC_CHAN_VBOOST};
    static const uint8_t readListSize = 3u;

    static uint8_t chanidx = 0u;
    uint8_t chan;

    /* Read the channel */
    chan = adcReadList[chanidx];
    ADCRaw[chan] = ADCRead(chan); /* ADCRead returns the adc value */

    chanidx++;
    if (chanidx >= readListSize)
    {
	chanidx = 0u;
    }
}
#endif

/*****************************************************************************
 *
 * FUNCTION NAME:
 * ADCApplyCal
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *
 * Argument			Type		IO	Description
 * --------			----		--	-----------
 *
 * RETURN VALUE: void
 *
 *****************************************************************************/
static void ADCApplyCal(void)
{
    uint8_t i;
    int32_t val;

    for (i = 0u; i < ADC_NUM; i++)
    {
        val = (int32_t) ADCRaw[i];

        val += (int16_t) NVMData.Calibration.adcCal[i];

        if (val > 1023)
        {
            val = 1023;
        }
        else if (val < 0)
        {
            val = 0;
        }
        else
        {
	    ; /* No change when cal value is 0 */
        }

        ADCRaw[i] = (uint16_t) val;
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Filter concept from http://www.radiolocman.com/shem/schematics.html?di=36513
 *  http://www.edn.com/article/464637-Simple_digital_filter_cleans_up_noisy_data.php
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void LowPassInit(volatile int16_t value, volatile int16_t *accum, int16_t K)
{
    *accum = value * K;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Filter concept from http://www.radiolocman.com/shem/schematics.html?di=36513
 *  http://www.edn.com/article/464637-Simple_digital_filter_cleans_up_noisy_data.php
 *
 *  "Calculate the value of the constant, K, based on the sampling rate of the system and the desired time constant
 *  for the filter as follows: K=TxSPS, where K>1, and SPS is the system's sampling rate. For example, for a
 *  system-sampling rate of 200 samples/sec and a desired time constant of 30 sec, the constant K would equal 6000
 *  samples. Applying a step change to the routine's input requires 6000 samples to reach approximately 63% of its
 *  final value at the output."
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
int16_t LowPass(volatile int16_t value, volatile int16_t *accum, int16_t K)
{
	int16_t LPOut, LPAcc, LPIn;

	/* LPOUT = LPACC/K */
	/* LPACC = LPACC + LPIN - LPOUT */
	LPIn = value;
	LPAcc = *accum;

	LPOut = LPAcc / K;
	LPAcc = LPAcc + (LPIn - LPOut);
	*accum = LPAcc;


	if(LPOut == 0)
	{
	    LPOut++;
	}


	return LPOut;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
static void DeviceInit(void)
{
    /*
     * Init oscillator
     */
    OSCCON = 0x7Au;/* PLL disabled, 16MHz, Internal oscillator block */
    OSCTUNE = 0x00u;

    /*
     *  Init watchdog
     */
    WDTCON = 0x0Fu; /* 128mS typical timer period, SWDTEN set */

    /*
     *  Init GPIO
     */
    LATA = 0x00u;
    TRISA = 0x0Fu; /* 0 = output, 1 = input */

    LATB = 0x00u;
    TRISB = 0xFEu;

    LATC = 0x48u; /* Set LIN TX to recessive state, CS_WK high */
    TRISC = 0x80u; /* Set PORTC1 to output */

    LATE  = 0x00u;
    TRISE = 0x00u;

    IOCBF = 0u; /* Clear interrupt-on-change flags */

    /*
     * Init ADC hardware
     */
    ADCON0 = 0x01u; /* A2D on */
    ADCON1 = 0xE3u; /* right justified, FOSC/64, N-REF=VSS, P-REF=FVR */
    ANSELA = 0x07u; /* AN0=analog, AN1=analog, AN2=analog, all others digital */
    ANSELB = 0x00u; /* All digital I/O on port B */

    /*
     *  Init timers
     */
    OPTION_REG = 0x93u; /*  pullups=OFF, int=falling edge, T0=timer, T0prescale=1:16 */

    /* Init Timer 2 for 200µs Tick for LIN timing */
    T2CON = 0x1Cu; /* pre-scalar = 1:1, T2 on, post-scalar = 1:4 */
    PR2 = 199u; /* 100 for 8MHz, 200 for 16MHz */
    TMR2IF = 0;
    TMR2IE = 1;

    /* Init Timer 4 for solenoid timing */
    T4CON = 0x7Cu; /* 1:16 postscalar, 1:1 prescaler*/
    PR4 = 249u; /* 1mS interrupts */
    TMR4IF = 0;
    TMR4IE = 1;

    /* Init Timer 6 for various timeout counting */
    T6CON = 0x7Fu; /* 1:16 postscalar, 1:64 prescalar */
    PR6 = 200u; /* 50mS interrupts */
    TMR6IF = 0;
    TMR6IE = 1;

    PEIE = 1; /* Enable peripheral interrupts */
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  http://www.ganssle.com/watchdogs.htm
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
static void wdtKick(uint8_t phase)
{
    switch (phase)
    {
    case WDT_STAGE_MAIN_TOP:
	if (wdtState != 0x5555u)
	{
	    MCU_RESET
	}

	wdtState += 0x1111u;

	break;

    case WDT_STAGE_MAIN_BOT:
	if (wdtState != 0x8888u)
	{
	    MCU_RESET
	}

	CLRWDT();

	if (wdtState != 0x8888u)
	{
	    MCU_RESET
	}

	wdtState = 0u;

	break;

    case WDT_STAGE_ISR:
	/*TODO: add counters to each timer ISR, have them incremement.
	 *confirm in the previous stage that the ISR timers are >0
	 */
	break;

    default:
	MCU_RESET /* Invalid phase, reset */
	/*break; This break statement is unreachable due to infinite loop in MCU_RESET*/
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
static void Hibernate(void)
{
    /* It's possible a WU went high before this line and the flag is about to be
     * cleared.  The if() statement below is intended to catch this case, clear
     * the flags and enable the interrupt before the if() statement manually 
     * checks the WakeUp states
     */

    /* The uC datasheet, section 3.4, notes that only AND operations should
     * be used when clearing IOCBF flags to avoid missing a detected edge.  This
     * is not a concern in this case, just clear all the flags */
    IOCBF = 0u; /* Clear all edge detect interrupt flags */
    /* Race condition here, if any WU goes high in between these 2 opcodes, the interrupt is missed */
    IOCBP = 0x3E; //0x38u; /* Enable positive edge detect on WU1, WU2, WU3 pins */ TODO: change to reverse pol input 0x3A should = 0x39
    IOCIE = 1; /* Enable interrupt on change */

    /* Wakeup interrupts are now enabled, make sure they are still inactive
     * before hibernating (this catches the race condition noted above).  From
     * here on, if a WakeUp goes active, the interrupt will be called which will
     * reset the device.  It's possible contact bounce went high and then low
     * before this check, in which case the device should still hibernate
     * (basically ignoring the contact bounce).
     */
    if (HIB_WAKE_UPS == 0u)
    {
	/* It's possible the above port state read happened during a glitch
	 * where all the WakeUps read low for a clock cycle, in which case
	 * the interrupt-on-change should fire and reset the device when any
	 * of the WakeUps go back high.
	 */
	
	/* WakeUp interrupts are now enabled, and the WakeUp pin states have
	 * been verified as inactive, power down.
	 */
	
	/* Place peripherals in low power mode, enable wakeup interrupts */
	PowerDown();

	/* Make sure both solenoid drivers are off */
	CLR_CLOSE_COIL;
	CLR_OPEN_COIL;

	OSCCON = 0x02u; /* Cut clock speed back to 31KHz */
	/*WDTCON = 0x00u;*/ /* Kill the watchdog */

	/* Sleep */
	MCU_SLEEP;

	/* If the chip wakes from sleep, force a reset  */
	MCU_RESET
    }
    else
    {
	/* One or more of the wakeups went active before the interrupts were
	 * enabled, disable the interrupt and resume normal operation
	 */
	IOCIE = 0;
	IOCBP = 0u;

	/* Reset the hibernate timeout to*/
	WriteTimeout(&Timeouts.Hibernate, TIMEOUT_HIBERNATE);
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *  none
 *
 *****************************************************************************/
static void UpdatePowerupFlags(E_STATE_DATA *eData)
{
    /* Set JD error state flags related to the last reset.  Note: STATUS
     * register is backed up to the variable __resetbits in startup.as.  This
     * must be enabled in the project linker settings.
     *
     * STKOVF STKUNF RMCLR RI POR BOR TO PD Condition
     *    0      0     1    1  0   x   1  1 Power-on Reset
     *    0      0     1    1  u   0   1  1 Brown-out Reset
     *    u      u     u    u  u   u   0  u WDT Reset
     *    u      u     u    0  u   u   u  u RESET Instruction Executed
     *    1      u     u    u  u   u   u  u Stack Overflow Reset (STVREN = 1)
     *    u      1     u    u  u   u   u  u Stack Underflow Reset (STVREN = 1)
     */
    if ((PCON & 0x80u) == 0x80u)
    {
	eData->StackUnderOverflow.Active = FAULT_INACTIVE;
	eData->StackUnderOverflow.faultTimeout = TIMEOUT_40HR;
    }
    if ((PCON & 0x40u) == 0x40u)
    {
	eData->StackUnderOverflow.Active = FAULT_INACTIVE;
	eData->StackUnderOverflow.faultTimeout = TIMEOUT_40HR;
    }
    if ((__resetbits & 0x10u) == 0x00u)
    {
	/* The watchdog timer is required to reset the chip, and every wakeup
	 * from hiberate will have the watchdog bit set.  The chip errata
	 * mandates that the watchdog be used in lieu of asm("reset").
	 */
#if 0
	eData->WatchdogReset.Active = 0u;
	eData->WatchdogReset.faultTimeout = TIMEOUT_40HR;
#endif
    }
    if ((PCON & 0xCFu) == 0x0Du) /* If /POR is high, brownout occurred */
    {
	if ((__resetbits & 0x18u) == 0x18u)
	{
	    /* Brownout reset is manually monitored in main(), the hardware
	     * brownout bits sometimes get set during a normal power-down
	     */
#if 0
	    eData->BrownoutReset.Active = 0u;
	    eData->BrownoutReset.faultTimeout = TIMEOUT_40HR;
#endif
	}
    }
    if ((PCON & 0x04u) == 0x00u)
    {
	/* This was added for debug, and not required in the JD spec */
	/*data->errorState = ERROR_RESET_INSTR;*/
	/*data->errorCounter++;*/
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *  none
 *
 *****************************************************************************/
void PowerDown(void)
{
    /* Turn off interrupts while re-configuring the hardware */
    GIE = CLEAR; /* Disable global interrupts */

    CLRWDT(); /* Kick the watchdog to get a fresh timeout until reset */
    BOOST_CONVERTER_DISABLE; /* Disable the boost converter to save power */

    /* Disable interrupts */
    RCIE = 0;
    TMR2IE = 0;
    TMR4IE = 0;
    TMR6IE = 0;
    C1IE = 0;

    /* Clear any pending flags */
    RCIF = CLEAR;
    TMR2IF = CLEAR;
    TMR4IF = CLEAR;
    TMR6IF = CLEAR;

    /* Shut down onboard peripherals */
    ANSELA = 0x3Fu; /* Enable very high input impedance analog inputs */
    FVRCON = 0u; /* Disable fixed voltage reference */
    DACCON0 = 0u; /* Disable DAC */
    ADCON0 = 0u; /* Disable ADC */
    CM1CON0 = 0u; /* Disable Comparator */
    RCSTA = 0u; /* Disable EUSART */
    T2CON = 0u; /* Disable Timer 2 */
    T4CON = 0u; /* Disable Timer 4 */
    T6CON = 0u; /* Disable Timer 6 */

    /* Set the I/O pins to low power states */
    TRISA = 0x0Fu; /* 0 = output */
    LATA = 0x00u;
    TRISB = 0x3Eu; /* Keep-Alives and Wake-ups as input */
    LATB = 0x00u;
    TRISC = 0x80u; /* LIN Rx is pulled up externally */
    LATC = 0x00u;

    /* Wakeup interrupts are still enabled, let them happen if they're pending */
    GIE = SET;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *  none
 *
 *****************************************************************************/
uint8_t AutoCal(uint8_t target)
{
    /* LOCAL VARIABLES:
     *
     * Variable			Type		Description
     * --------			----		-----------
     * cal				CAL_DATA	struct to be written to NVM
     */
    static const uint16_t calTarget[][8] =
    {
	{ 392u, 392u, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN }, /* 12V cal targets */
	{ 782u, 782u, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN, CAL_DISABLE_CHAN }  /* 24V cal targets */
    };

    CAL_DATA cal;
    uint32_t accum[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    uint8_t i, j;
    uint8_t retval = CAL_SUCCESS;

    /* Initialize cal data */
    for (i = 0u; i < 8u; i++)
    {
	cal.adcCal[i] = 0;
    }

    for (i = 0u; i < NUM_CAL_SAMPLES; i++)
    {
	/* Loop through each channel and read its value */
	for (j = 0u; j < 8u; j++)
	{
	    /* Read the channel */
	    accum[j] += ADCRead(j);
	}

	/*delaySysTick(10u); TODO: add delay? */ /* each tick is 10mS */
    }

    /* Average each accumulated A/D value */
    for (i = 0u; i < 8u; i++)
    {
	accum[i] /= NUM_CAL_SAMPLES;
    }

    /* Calculate the calibration value */
    for (i = 0u; i < 8u; i++)
    {
	if (calTarget[target][i] != CAL_DISABLE_CHAN)
	{
	    cal.adcCal[i] = (int16_t) ((int16_t) calTarget[target][i] - (int16_t) accum[i]);
	    if (abs(cal.adcCal[i]) > MAX_CAL_VALUE)
	    {
		retval = CAL_VAL_OUT_OF_RANGE;
	    }
	}
	else
	{
	    cal.adcCal[i] = 0;
	}
    }

    /* Only write cal data to nvm if there's been no error so far */
    if (retval == CAL_SUCCESS)
    {
	for (i = 0u; i < 8u; i++)
	{
	    NVMData.Calibration.adcCal[i] = cal.adcCal[i];
	}

	/* Verify it was written */
	for (i = 0u; i < 8u; i++)
	{
	    if (NVMData.Calibration.adcCal[i] != cal.adcCal[i])
	    {
		retval = CAL_WRITE_FAIL;
	    }
	}

    }

    return retval;
}

/*****************************************************************************
 *
 * FUNCTION NAME: C1ISR: Comparator 1 Interrupt Service Routine 
 *
 * DESCRIPTION: Low Panic Open Voltage. Set Open Coil when interrupt hits. 
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *  none
 *
 *****************************************************************************/
void C1ISR(void)
{
    uint8_t counter = 0u;

    /* Initiate Power Down sequence */
    PowerDown();

    /* ensure close coil is not energized. At no time should both coils ever be
     * powered at one time. 
     */
    CLR_CLOSE_COIL;
    SET_OPEN_COIL;

    OSCCON = 0x02u; /* Cut clock speed back to 31KHz */

    /* Spin until the watchdog kicks */
    for (;;)
    {
	WDTCON = 0x0Fu; /* Make sure the watchdog is enabled */

    /* toggle the coil at approximately 85% Duty */
	SET_OPEN_COIL;
	for (counter = 14u; counter > 0u; counter--)
	{
	    ;
	}

	CLR_OPEN_COIL;
	for (counter = 1u; counter > 0u; counter--)
	{
	    ;
	}
    }

    /*C1IF = CLEAR;*/ /* Clear the interrupt */
}
/*****************************************************************************
 *
 * FUNCTION NAME:
 *  ISR
 *
 * DESCRIPTION:
 *  Interrupt service routine
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *  none
 *
 *****************************************************************************/
void interrupt ISR(void)
{
    /* LOCAL VARIABLES:
     *
     * Variable		Type		Description
     * --------		----		-----------
     * seq		uint8_t		which phase of solenoid action to execute
     */
    static uint8_t seq = 1u; /* Initialized to cooldown sequence.  DeviceInit() starts this on power-up to insert a cooldown delay */
    static uint8_t lpfIdx = 0u;
    static uint16_t T6Counter = 0u;
    static uint8_t cntr1msTick = 0u;

    /****************************************
     * Interrupt-On-Change interrupt
     ****************************************/
    if (IOCBF != 0u)
    {
        /*IOCBF = 0u;*/

	MCU_RESET
    }

    /****************************************
     * Comparator C1 interrupt
     ****************************************/
    else if (C1IF)
    {
	C1IF = CLEAR; /* Clear the interrupt */
	C1ISR();
    }

    /****************************************
     * USART receive interrupt
     ****************************************/
    else if (RCIF)
    {
	/* LIN-API Call "l_ifc_rx()" to get received byte into LIN Driver */
	l_ifc_rx();
	RCIF = CLEAR; /* Clear the interrupt */
    }

    /****************************************
     * Timer2 to PR2 interrupt
     ****************************************/
    else if (TMR2IF) /* interrupt every 200uSec */
    {
	TMR2IF = CLEAR; /* Clear the interrupt */
	l_cyclic_com_task(); /*lint !e960 */

	cntr1msTick++;
	if (cntr1msTick >= 5u) /* 200us x5 = 1ms */
	{
	    cntr1msTick = 0u;
	    Timer1msTick = 1u;

	    /* Update ADC low pass filter data, one channel per interrupt */
	    lpfIdx++;
	    if (lpfIdx > 2u)
	    {
		lpfIdx = 0u;
	    }
	    ADClpf[lpfIdx] = (uint16_t) (LowPass((int16_t) ADCRaw[lpfIdx], &ADCAccum[lpfIdx], LPF_K));
	}
    }

    /****************************************
     * Timer4 to PR4 Match interrupt
     ****************************************/
    else if (TMR4IF) /* 1mS interrupt period */
    {
	TMR4IF = CLEAR; /* Clear the interrupt */

	/* If the delay counter is non-zero, decrement it but do not run the
	 * solenoid control logic .
	 */
	if (TMR4Counter > 0u)
	{
	    TMR4Counter--;

	    /* Toggle the Close coil at 50% duty to reduce the amount of
	     * charge depleted from the boost voltage storage capacitor
	     */
	    if (seq == 0u)
	    {
            if (ISRCoil == SOL_CLOSE_COIL)
            {
              TOG_CLOSE_COIL;
            }

            if (ISRCoil == SOL_OPEN_COIL)
            {
               if ((TMR4Counter & 1u) == 0u)
               {
            	SET_OPEN_COIL;
               }
            }
	    }
	}
	else
	{
	    /* This sequence ends the solenoid coil pulse and sets up the cooldown delay */
	    if (seq == 0u)
	    {
		/* Defensively clear both coil signals each time this sequence executes.  It doesn't
		 * have any undesirable side effects, and can help prevent solenoid melting if a
		 * coil is accidentally energized.  At no point in device operation should both coils
		 * ever be energized at the same time.
		 */
		CLR_CLOSE_COIL;
		CLR_OPEN_COIL;

		if (ISRCoil == SOL_CLOSE_COIL)
		{
		    TMR4Counter = SOL_ON_COOL;
		    seq = 1u;
		}
		else if (ISRCoil == SOL_OPEN_COIL)
		{
		    TMR4Counter = SOL_OFF_COOL;
		    seq = 1u;
		}
		else
		{
		    /* Reset sequence and disable the timer if ISRCoil is unknown */
		    seq = 0u;
		    TMR4ON = 0;
		}
	    }
	    /* This sequence is the end of the cooldown, disable the timer */
	    else
	    {
		/* Cooldown is over, reinit back to solenoid actions */
		seq = 0u;
		TMR4ON = 0;
	    }
	}
    }

    /****************************************
     * Timer6 to PR6 Match interrupt, 50mS
     ****************************************/
    else if (TMR6IF)
    {
	TMR6IF = CLEAR; /* Clear the interrupt */
	if (Timeouts.WakeUp1 > 0u)
	{
	    Timeouts.WakeUp1--;
	}

	if (Timeouts.Hibernate > 0u)
	{
	    Timeouts.Hibernate--;
	}

	if (Timeouts.Overvoltage > 0u) /* nest if statements instead of && to prevent side effects */
	{
        if(INTERRUPT_KEEP_ALIVES == 0u)
        {
          Timeouts.Overvoltage--;  
        }
	}

	if (Timeouts.Hazard > 0u)
	{
	    Timeouts.Hazard--;
	}

        if (Timeouts.Open > 0u)
        {
            Timeouts.Open--;
        }

	if (T6Counter > 0u)
	{
	    T6Counter--;
	}
	else
	{
	    T6Counter = 1200u; /* Tell main() to update every 60 seconds */
	    ErrorTimeoutUpdate = 1u;
	}
    }

    else
    {
	/* Unimplemented interrupt, force a reset */
	MCU_RESET
    }
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Disable LIN interrupts
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
l_irqmask l_sys_irq_disable(void)
{
    /* Disable Interrupts */
    /*GIE = 0u;*/

    return 0u;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Restore LIN interrupts
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void l_sys_irq_restore(l_irqmask mask)
{
    /* Enable Interrupts */
    /*GIE = 1u;*/

    (void) mask;

    return;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void LIN_Enable_Transceiver(void)
{
    LIN_XCVR_EN = SET;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *
 * ARGUMENTS:
 *  none
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void LIN_Disable_Transceiver(void)
{
    LIN_XCVR_EN = CLEAR;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Example for user defined Callback functions from LIN driver.
 *  Driver Callback to get Serial Number of ECU.
 *  Argument 1:  Output: pointer to data buffer for serial number
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void l_callback_RBI_ReadEcuSerialNumber(l_u8 SerialnumberResult[]) /*lint !e960 identifier exceeds 31 characters */
{
    /* LIN has requested the serial number */
    SerialnumberResult[0] = NVMData.SerialNumber[0];
    SerialnumberResult[1] = NVMData.SerialNumber[1];
    SerialnumberResult[2] = NVMData.SerialNumber[2];
    SerialnumberResult[3] = NVMData.SerialNumber[3];
}


/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Example for user defined Callback functions from LIN driver.
 *  Driver Callback to get User defined answer for specific id.
 *  Argument 1: Input: special id from request.
 *  Argument 2: Output: pointer to data buffer response.
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
unsigned char l_callback_RBI_UserDefined(l_u8 id, l_u8 UserDefinedResult[]) /*lint !e970 */
{
    uint8_t tmp = (uint8_t) LD_NO_RESPONSE;

    switch (id)
    {
    case 62:
	/* None of these values get sent, AutoCal takes longer than the LIN timeout */
        UserDefinedResult[0] = id;
        UserDefinedResult[1] = 0u;
        UserDefinedResult[2] = 0u;
        UserDefinedResult[3] = 0u;
        UserDefinedResult[4] = 0u;
	(void) AutoCal(CAL_TARGET_12V);
	tmp = (uint8_t) LD_POSTIVE_RESPONSE;
      break;

    case 63:
        UserDefinedResult[0] = id;
        UserDefinedResult[1] = FirmwareVersion[0];
        UserDefinedResult[2] = FirmwareVersion[1];
        UserDefinedResult[3] = FirmwareVersion[2];
        UserDefinedResult[4] = FirmwareVersion[3];
        tmp = (uint8_t) LD_POSTIVE_RESPONSE;
        break;
#if 0
    case 33:
        /*define cell serial number here */
        UserDefinedResult[0] = id;
        UserDefinedResult[1] = id;
        UserDefinedResult[2] = 0x01;
        UserDefinedResult[3] = 0x09;
        UserDefinedResult[4] = 0xFF;
        tmp = 1;
        break;
    case 34:
        /*define cell serial number here */
        UserDefinedResult[0] = id;
        UserDefinedResult[1] = id;
        UserDefinedResult[2] = 0x02;
        UserDefinedResult[3] = 0x09;
        UserDefinedResult[4] = 0xFF;
        tmp = 1;
        break;
    case 35:
        /*define cell serial number here */
        UserDefinedResult[0] = id;
        UserDefinedResult[1] = id;
        UserDefinedResult[2] = 0x03;
        UserDefinedResult[3] = 0x09;
        UserDefinedResult[4] = 0xFF;
        tmp = 1;
        break;
#endif
    default:
        tmp = (uint8_t) LD_NEGATIVE_RESPONSE;
        break;
    }

    return tmp;
}

/*****************************************************************************
 *
 * FUNCTION NAME:
 *
 * DESCRIPTION:
 *  Example for user defined Callback functions from LIN driver.
 *  Driver Callback for data dump request.
 *  Argument 1: Input: pointer to data buffer with data from request.
 *  Argument 2: Output: pointer to data buffer to make response.
 *
 *  From the LIN 2.1 spec: "This service is reserved for initial configuration
 *  of a slave node by the slave node supplier and the format of this message is
 *  supplier specific. This service shall only be used by supplier diagnostics
 *  and not in a running cluster, e.g at the car OEM."
 *
 * ARGUMENTS:
 *
 * Argument	Type		IO	Description
 * --------	----		--	-----------
 *
 * RETURN VALUE:
 *
 *****************************************************************************/
void l_callback_DataDump(l_u8* ReceivedData, l_u8* TransmitData)
{
    uint8_t temp;

    switch (ReceivedData[0])
    {
    case 0:
               (void) AutoCal(CAL_TARGET_12V);
        TransmitData[0] = (uint8_t) LD_POSTIVE_RESPONSE;    /*TransmitData[0] = acknowledge byte */
               break;

    case 1:
        /* Set serial number  per 4.2.5.3
         3c 14 06 b4 01 XX XX XX XX to be sent by EOL test  */
               temp = ReceivedData[1];
        NVMData.SerialNumber[0] = temp;
               temp = ReceivedData[2];
        NVMData.SerialNumber[1] = temp;
               temp = ReceivedData[3];
        NVMData.SerialNumber[2] = temp;
               temp = ReceivedData[4];
        NVMData.SerialNumber[3] = temp;

               /* Verify the serial number was written */
               if ((NVMData.SerialNumber[0] == ReceivedData[1]) &&
                   (NVMData.SerialNumber[1] == ReceivedData[2]) &&
                   (NVMData.SerialNumber[2] == ReceivedData[3]) &&
                   (NVMData.SerialNumber[3] == ReceivedData[4]))
               {
                              TransmitData[0] = (uint8_t) LD_POSTIVE_RESPONSE;    /*TransmitData[0] = acknowledge byte */
               }
               else
               {
                              TransmitData[0] = (uint8_t) LD_NEGATIVE_RESPONSE;
               }
               break;

    default:
               break;
    }
}

