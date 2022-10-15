#include <atmel_start.h>

#include <pwm_basic_example.h>
#include <pwm_basic.h>

/***************************** Start of Note Section. ***************************************/


/***************************** End of Note Section. *****************************************/
/***************************** Start of Define section. *************************************/


/***************************** End of Define section. ***************************************/
/***************************** Start of Macro Definitions Section. **************************/


/***************************** End of Macro Definitions Section. ****************************/
/*************** Start of Global Variable Declaration Section. ******************************/
volatile uint16_t ms_counter = 0;

uint8_t period = 0;
uint8_t duty_cycle = 0;

/*************** End of Global Variable Declaration Section. ********************************/
/***************************** Start of Prototype Function Declaration Section. *************/

void GPIOs_Init(void);
void TCA0_Init(uint16_t time_in_ms);
void WDT_Init(WDT_PERIOD_t wdt_period);
void WDT_Reset(void);
bool TCB2_Initialize_As_PWM(uint8_t period, uint8_t duty_cycle);
bool TCB2_Setup_As_PWM_In_Hz(uint32_t period_in_hertz, uint8_t duty_cycle_in_percent);

/***************************** End of Prototype Function Declaration Section. ***************/
/***************************** Start of Prototype Function Definitions Section. *************/
/*
* \brief	This function initializes GPIOs not used by peripherals.
*			GPIOs used by peripheral are initialized in peripheral functions
*			located in file "driver_init.c".
*
* \return	Nothing
*/
void GPIOs_Init(void) {
	
	// Pin PC5.
	// Set pin direction to Output.
	PORTC.DIR |= (1 << 5);
	// Set level to low because is active high.
	PORTC.OUT |= (1 << 5);
	// Enable pull-up.
	PORTC.PIN5CTRL |= (1 << PORT_PULLUPEN_bp);
	
	
	
}

/*
* \brief	This function enables the watchdog In Normal mode operation.
*			In this mode a single time-out period is set for the WDT. If the WDT is not reset from
*			software using the WDR any time before the time-out occurs, the WDT will issue a system Reset.
*			A new WDT time-out period will be started each time the WDT is reset by WDR.
*
* \return	Nothing.
*/
void WDT_Init(WDT_PERIOD_t wdt_period) {
	// Disable global interrupts.
	cli();
	// Reset Watchdog Timer
	asm("wdr");
	// Bits 3:0 – PERIOD[3:0] Period
	// Writing a non-zero value to this bit enables the WDT,
	// and selects the time-out period in Normal mode accordingly.
	//ccp_write_io((void*)&(WDT.CTRLA), WDT_PERIOD_OFF_gc | WDT_WINDOW_OFF_gc);
	ccp_write_io((void*)&(WDT.CTRLA), wdt_period | WDT_WINDOW_OFF_gc);
	// Enable global interrupts.
	sei();
}

/*
* \brief	This function reset the watchdog, and a new WDT time-out period
*			will be started each time the WDT is reset by WDR.
*
* \return	Nothing.
*/
void WDT_Reset(void) {
	// Disable global interrupts.
	cli();
	// Reset Watchdog Timer
	asm("wdr");
	// Enable global interrupts.
	sei();
}


/**
 * \brief		This function setup the 16-bit Timer/Counter Type A (TCA) in normal operation
 *				to perform an overflow interrupt every "time_in_ms" mS.	
 *				This timer will be used as our House Keeper Timer.
 * 
 * \return		Nothing
 */
void TCA0_Init(uint16_t time_in_ms) {
	// Main Clock Pre-scaler = 4
	// Timer/Counter Pre-scaler = 1, 
	// 1000 is because we want to count in mS.	
	TCA0.SINGLE.PER = (register16_t)((float)((time_in_ms * F_CPU) / (4 * 1 * 1000)));
	
	// Bit 0 – OVF Timer Overflow/Underflow Interrupt Enable
	// Enable overflow interrupt
	TCA0.SINGLE.INTCTRL = 0 << TCA_SINGLE_CMP0_bp /* Compare 0 Interrupt: disabled */
		| 0 << TCA_SINGLE_CMP1_bp /* Compare 1 Interrupt: disabled */
		| 0 << TCA_SINGLE_CMP2_bp /* Compare 2 Interrupt: disabled */
		| 1 << TCA_SINGLE_OVF_bp; /* Overflow Interrupt: enabled */
		
	// Bits 2:0 – WGMODE[2:0] Waveform Generation Mode.
	// Bits 4, 5, 6 – CMPEN Compare n Enable.
	TCA0.SINGLE.CTRLB = 0 << TCA_SINGLE_ALUPD_bp /* Auto Lock Update: disabled */
		| 0 << TCA_SINGLE_CMP0EN_bp		/* Compare 0 Enable: disabled */
		| 0 << TCA_SINGLE_CMP1EN_bp		/* Compare 1 Enable: disabled */
		| 0 << TCA_SINGLE_CMP2EN_bp		/* Compare 2 Enable: disabled */
		| TCA_SINGLE_WGMODE_NORMAL_gc;	/* Normal Mode */	
	
	// Bit 0 – ENABLE Enable
	// Enable the peripheral by writing a '1' to the ENABLE bit in the Control A register (TCAn.CTRLA).
	// Bits 3:1 – CLKSEL[2:0] Clock Select
	// Select the clock frequency to fCLK_PER/1 by clearing bits 3:1	
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc      /* System Clock */
		| 1 << TCA_SINGLE_ENABLE_bp    /* Module Enable: enabled */
		| 0 << TCA_SINGLE_RUNSTDBY_bp; /* RUN STANDBY: disabled */	
		
}

/**
 * \brief	  This function setup the timer TCB2 in 8-Bit PWM mode.
 *
 *            The counter will continuously count from BOTTOM to CCMPL, and the output
 *            will be set at BOTTOM and cleared when the counter reaches CCMPH.
 *            CCMPH is the number of cycles for which the output will be driven high.
 *            CCMPL+1 is the period of the output pulse.
 * 
 * \return	  True is function is executed.
 */
bool TCB2_Initialize_As_PWM(uint8_t period, uint8_t duty_cycle) {
	
	// Sanity check. The period must be greater than the duty cycle.
	if(period < duty_cycle) {
		return false;
	}	
	
	// Setup the port pin used to output the PWM signal (PB4).	
	PORTB.DIR |= (1 << 4);					/* Set pin direction to Output. */	
	PORTB.OUT &= ~(1 << 4);					/* Set pin to low state. */
	
	// Use the register PORTMUX.TCBROUTEA to set the alternative pin for TCB2.
	// This register controls the pin position for TCB2 output.
	// If the Bit 2 of this register is "1", them the timer output will be on PB4.
	PORTMUX.TCBROUTEA |= PORTMUX_TCB2_bm;	/* set the alternate pin mux */	
	
	TCB2.CCMPL = period;//0x0f;				/* Capture/Compare Register Value Low Byte.
											/* These bits controls the PWM period when use in 8-Bit PWM mode. */
	
	TCB2.CCMPH = duty_cycle;//0x0c;			/* Capture/Compare Register Value High Byte. 
											/* These bits controls the PWM duty cycle when use in 8-Bit PWM mode. */

	TCB2.CNT = 0x0;							/* Counter Register. */

	TCB2.CTRLB = 0 << TCB_ASYNC_bp			/* Asynchronous Enable: disabled */
		| 1 << TCB_CCMPEN_bp				/* Pin Output Enable: enabled */
		| 0 << TCB_CCMPINIT_bp				/* Pin Initial State: disabled */
		| TCB_CNTMODE_PWM8_gc;				/* 8-bit PWM */

	TCB2.DBGCTRL = 0 << TCB_DBGRUN_bp;		/* Debug Run: disabled */

	TCB2.EVCTRL = 0 << TCB_CAPTEI_bp		/* Event Input Enable: disabled */
		| 0 << TCB_EDGE_bp					/* Event Edge: disabled */
		| 0 << TCB_FILTER_bp;				/* Input Capture Noise Cancellation Filter: disabled */

	TCB2.INTCTRL = 1 << TCB_CAPT_bp			/* Setting: enabled */;

	TCB2.CTRLA = TCB_CLKSEL_DIV1_gc			/* CLK_PER */
		| 1 << TCB_ENABLE_bp				/* Enable:enabled */
		| 0 << TCB_RUNSTDBY_bp				/* Run Standby: disabled */
		| 0 << TCB_SYNCUPD_bp				/* Synchronize Update: disabled */;			
	
	return true;	
}

/**
 * \brief	   This function setup the timer TCB2 in 8-Bit PWM mode.
 *			   using the given frequency and duty cycle in percent.
 *
 * \param[in] freq_in_hertz. PWM frequency in hertz.
 *
 * \param[in] duty_cycle_in_percent. Duty cycle in percent.
 * 
 * \return	  True is function is executed.
 */
bool TCB2_Setup_As_PWM_In_Hz(uint32_t freq_in_hertz, uint8_t duty_cycle_in_percent) {	
	// Calculate the period from the frequency in hertz.
	period = (uint8_t)(F_CPU / freq_in_hertz) - 1;
	// Calculate the duty as a function of duty cycle in percent and the period.
	duty_cycle = (float)(((period + 1) * duty_cycle_in_percent) / 100);
	// Setup the timer TCB2 in 8-Bit PWM mode.
	if(TCB2_Initialize_As_PWM(period, duty_cycle)) {
		return true;
	}	
	return false;
}

/***************************** End of Prototype Function Definitions Section. ***************/
/***************************** Start of Interrupts Section **********************************/
/*
* \brief	Overflow interrupt service routine for TCA.
*			This routine is called every time an overflow
*			takes place.
*			This routine is used as a house keeping timer
*			to count milliseconds.
*			The counters is this routine are consumed by
*			other functions.
*
* \return	Nothing
*/
ISR(TCA0_OVF_vect) {
	//
	ms_counter++;
	
	// to test the house keeping timer.
	PORTC.OUTTGL |= (1 << 5);
	
	// Clear overflow interrupt flag. Bit 0 – OVF Overflow/Underflow Interrupt Flag.
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

/*
* \brief	Capture Interrupt Service Routine for Timer TCB2.
*
* \return	Nothing
*/
ISR(TCB2_INT_vect) {
	
	// Clear the interrupt flag.
	TCB2.INTFLAGS = TCB_CAPT_bm;
}

/***************************** End of Interrupts Section ************************************/


// Entry point.
int main(void) {
	// Initializes MCU drivers.
	atmel_start_init();
	// Initialize GPIOs.
	GPIOs_Init();
	// Setup the timer TCA0 in normal mode. 
	// This timer act as a house keeper timer.
	// Initializes the TCA1 to overflow every 1 milliseconds.
	TCA0_Init(1);
	// Setup the timer TCB2 in 8-Bit PWM mode.
	// Set the frequency to be 373kHz and a 25% duty cycle.
	TCB2_Setup_As_PWM_In_Hz(250000, 25);		
	// Clear the Overflow Interrupt for TCA0.
	TCA1.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm;
	// Clear the interrupt flag
	TCB2.INTFLAGS |= TCB_CAPT_bm | TCB_OVF_bm;	
	// Enable global interrupts.
	sei();	
	
	// Initializes the Watch Dog to 1K cycles (1.0s).
	WDT_Init(WDT_PERIOD_1KCLK_gc);
	// Main loop.
	while (1) {
		// Code here.		
		// Reset Watch Dog.
		WDT_Reset();
	}
}
