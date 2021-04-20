/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


#include "derivative.h" /* include peripheral declarations S32K144 */

typedef enum { false, true } bool;

/////////////////////////////
////  GLOBAl VARIABLES    ///
/////////////////////////////

int flag=0;
int toggle=1;
int flagMask=0;
uint16_t var1, var2;
uint16_t CH0_edge1, CH0_edge2, CH2_edge1, CH2_edge2, temp;
uint16_t FTM0_CH0_pulse_width, FTM0_CH2_pulse_width, FTM0_CH0_period;
uint8_t ADC0_RA;


/////////////////////////////////////////
//// SELECT FLEXTIMER MODULE FEATURE ////
/////////////////////////////////////////

#define SW 9

/* SW=0 - Edge-Align PWM Mode
 * 		- PWM0 -> FTM0_CH0 -> PTD15
 * 		- PWM1 -> FTM0_CH1 -> PTD16
 * 		- PWM2 -> FTM0_CH2 -> PTD0
 * 		- PWM3 -> FTM0_CH3 -> PTD1
 *
 * SW=1 - Center-Align PWM Mode
 * 		- PWM0 -> FTM0_CH0 -> PTD15
 * 		- PWM1 -> FTM0_CH1 -> PTD16
 * 		- PWM2 -> FTM0_CH2 -> PTD0
 * 		- PWM3 -> FTM0_CH3 -> PTD1
 *
 * SW=2 - Combine Mode for Phase Shifted PWM
 *  	- PWM0 -> FTM1_CH0 -> PTB2
 * 		- PWM1 -> FTM1_CH1 -> PTB3
 * 		- PWM4 -> FTM1_CH4 -> PTD8
 * 		- PWM5 -> FTM1_CH5 -> PTD9
 *
 * SW=3	- ADC triggering by FTM through PDB module
 * 		- FTM0 Init trigger signal triggers PDB through TRGMUX
 * 		  and PDB0 pre-trigger signal triggers ADC0 module directly
 * 		- check timing between FTM0, PDB0 and ADC0:
 * 			- PTD15 - PWM signal
 * 			- PTD2  - FTM0 Init triggers
 * 			- PTD3  - ADC0 conversions
 *
 * SW=4 - Single-Edge Capture Mode
 *		- FTM1 is used to generate tested input signal
 * 		  for FTM0 that works in Single-Edge Capture Mode
 * 		- Interconnect: PTB2 and PTD15 externally
 *
 * SW=5 - Dual-Edge Capture Mode
 * 		- FTM1 is used to generate tested input signals
 * 		  for FTM0 that works in Dual-Edge Capture Mode
 * 		- Interconnect: PTB2 and PTD15 externally
 * 		                PTB3 and PTD0 externally
 *
 * SW=6 - Quadrature Decoder Mode
 * 		- FTM1 is used to imitate Phase A and B signals
 * 		  for FTM2 that works in Quadrature Decoder mode
 * 		- Interconnect: PTB2 and PTD11 - signal A externally
 * 		  				PTD8 and PTD10 - signal B externally
 * 		- Check FTM2 timer overflow on PTD0 pin
 *
 * SW=7 - Center-Align PWM and Half Cycle Reload
 * 		- Check the reload opportunities on PTD3
 *
 * SW=8 - Center-Align PWM and Full Cycle Reload
 * 		- Check the reload opportunities on PTD3
 *
 * SW=9 - Center-Align PWM Synchronized by Software Trigger
 *		- Check software trigger events on PTD3 pin
 *
 * SW=10 - Center-Align PWM Synchronized by Hardware Trigger
 *       - Hardware trigger signal is generated from FTM1 through PTB2 pin
 *         and enter through PTA14 on fault pin FTM0_FLT0
 *       - Interconnect PTB2 and PTA14 externally
 *
 * SW=11 - Center-Align PWM and Fault Control
 * 		 - FTM1 is used to generate fault signal to stop PWM output of FTM0
 * 		 - Interconnect: PTB2 and PTA14 externally
 *
 * SW=12 - Center-Align PWM and Global Time Base (GTB)
 * 		 - Global time base applied to FTM0 and FTM2 counters
 * 		 - Compare PWM signals on PTD15 and PTD0 pins
 *
 *  */

///////////////////////
////   FUNCTIONS    ///
///////////////////////

/* SW=0 - Edge-Align PWM Mode */
void Edge_Align_PWM_Init()
{
	/* Enable clock for PORTD */
	PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
	/* Select and enable clock for FTM0 */
	PCC->PCCn[PCC_FLEXTMR0_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
	/* Set PORTD pins for FTM0 */
	PORTD->PCR[15] = PORT_PCR_MUX(2); 	// FTM0, Channel0
	PORTD->PCR[16] = PORT_PCR_MUX(2); 	// FTM0, Channel1
	PORTD->PCR[0] = PORT_PCR_MUX(2); 	// FTM0, Channel2
	PORTD->PCR[1] = PORT_PCR_MUX(2); 	// FTM0, Channel3
	/* Enable registers updating from write buffers */
	FTM0->MODE = FTM_MODE_FTMEN_MASK;
	/* Enable sync, combine mode and dead-time for pair channel n=1 and n=2 */
	FTM0->COMBINE = FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK
	| FTM_COMBINE_COMP1_MASK | FTM_COMBINE_DTEN1_MASK;
	/* Set Modulo in initialization stage (10kHz PWM frequency @112MHz system clock) */
	FTM0->MOD = FTM_MOD_MOD(11200-1);
	/* Set CNTIN in initialization stage */
	FTM0->CNTIN = FTM_CNTIN_INIT(0);
	/* Enable high-true pulses of PWM signals */
	FTM0->CONTROLS[0].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[1].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[2].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[3].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	/* Set channel value in initialization stage */
	FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(5600); // 50% duty cycle
	FTM0->CONTROLS[1].CnV=FTM_CnV_VAL(5600); // 50% duty cycle
	FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800); // 25% duty cycle
	FTM0->CONTROLS[3].CnV=FTM_CnV_VAL(2800); // 25% duty cycle
	/* Reset FTM counter */
	FTM0->CNT = 0;
	/* Insert deadtime (1us) */
	FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
	/* Clock selection and enabling PWM generation */
	FTM0->SC = FTM_SC_CLKS(1) | FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK | FTM_SC_PWMEN2_MASK |
	FTM_SC_PWMEN3_MASK;
}

/* SW=1 - Center-Align PWM Mode */
void Center_Align_PWM_Init()
{
	/* Enable clock for PORTD */
	PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
	/* Select and enable clock for FTM0 */
	PCC->PCCn[PCC_FLEXTMR0_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
	/* Set PORTD pins for FTM0 */
	PORTD->PCR[15] = PORT_PCR_MUX(2);
	PORTD->PCR[16] = PORT_PCR_MUX(2);
	PORTD->PCR[0] = PORT_PCR_MUX(2);
	PORTD->PCR[1] = PORT_PCR_MUX(2);
	/* Select up-down counter for Center-Align PWM */
	FTM0->SC = FTM_SC_CPWMS_MASK;
	/* Combine mode and dead-time enable for channel0 and channel1 */
	FTM0->COMBINE = FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK;
	/* Combine mode and dead-time enable for channel2 and channel3 */
	FTM0->COMBINE |= FTM_COMBINE_COMP1_MASK | FTM_COMBINE_DTEN1_MASK;
	/* Set Modulo (10kHz PWM frequency @112MHz system clock) */
	FTM0->MOD = FTM_MOD_MOD(5600-1);
	/* Set CNTIN */
	FTM0->CNTIN = FTM_CNTIN_INIT(0);
	/* High-true pulses of PWM signals */
	FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
	/* Set Channel Value */
	FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800); // 50% duty cycle
	FTM0->CONTROLS[1].CnV=FTM_CnV_VAL(2800); // 50% duty cycle
	FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(1400); // 25% duty cycle
	FTM0->CONTROLS[3].CnV=FTM_CnV_VAL(1400); // 25% duty cycle
	/* FTM counter reset */
	FTM0->CNT = 0;
	/* Insert DeadTime (1us) */
	FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
	/* Clock selection and enabling PWM generation */
	FTM0->SC |= FTM_SC_CLKS(1) | FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK
			 | FTM_SC_PWMEN2_MASK | FTM_SC_PWMEN3_MASK;
}

/* SW=2 - Combine Mode for Phase Shifted PWM */
void Phase_Shifted_PWM()
{
    /* Enable clock for FTM1 */
    PCC->PCCn[PCC_FLEXTMR1_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTB */
    PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;

    PORTB->PCR[2] = PORT_PCR_MUX(2);				// Set PTB2 for FTM1 – Channel0
    PORTB->PCR[3] = PORT_PCR_MUX(2);				// Set PTB3 for FTM1 – Channel1
    PORTD->PCR[8] = PORT_PCR_MUX(6);				// Set PTD8 for FTM1 – Channel4
    PORTD->PCR[9] = PORT_PCR_MUX(6);				// Set PTd9 for FTM1 – Channel5

    /* Enable combine, complementary mode and dead-time for channel pair CH0/CH1 and CH4/CH5 */
    FTM1->COMBINE = FTM_COMBINE_COMBINE0_MASK | FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK
    		      | FTM_COMBINE_COMBINE2_MASK | FTM_COMBINE_COMP2_MASK | FTM_COMBINE_DTEN2_MASK;

    FTM1->CONTROLS[0].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[1].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[4].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[5].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses

    /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
    FTM1->MOD = FTM_MOD_MOD(11200-1);				// Set modulo
    FTM1->CONTROLS[0].CnV=FTM_CnV_VAL(2800);		// Set channel Value
    FTM1->CONTROLS[1].CnV=FTM_CnV_VAL(8400);		// Set channel Value
    FTM1->CONTROLS[4].CnV=FTM_CnV_VAL(5600);		// Set channel Value
    FTM1->CONTROLS[5].CnV=FTM_CnV_VAL(11200);		// Set channel Value
    FTM1->CNT = 0;				       				// Counter reset
	/* Insert DeadTime (1us) */
	FTM1->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
    FTM1->SC|=FTM_SC_CLKS(1)|FTM_SC_PWMEN0_MASK|FTM_SC_PWMEN1_MASK|FTM_SC_PWMEN4_MASK
    	    |FTM_SC_PWMEN5_MASK; 					// Select clock and enable PWM
}

/* SW=3, TRGMUX initialization to route FTM Init trigger signal into PDB0 trigger input */
void TRGMUX_Init()
{
	PCC->PCCn[PCC_TRGMUX_INDEX] = PCC_PCCn_CGC_MASK;				// Enable clock for TRGMUX module
	TRGMUX->TRGMUXn[TRGMUX_PDB0_INDEX] = TRGMUX_TRGMUXn_SEL0(0x16);	// Set FTM as a trigger source for PDB0
}

/* SW=3, PDB0 initialization to trigger ADC0 at the center of PWM by pre-trigger signal */
void PDB0_Init()
{
    PCC->PCCn[PCC_PDB0_INDEX] = PCC_PCCn_CGC_MASK;				// Enable clock for PDB0
    FSL_NVIC->ISER[PDB0_IRQn / 32] |= (1 << (PDB0_IRQn % 32));	// Enable interrupt
	PDB0->MOD = 11200;											// Set Modulo
	PDB0->CH[0].C1 = PDB_C1_TOS(1) | PDB_C1_EN(1);				// Select and enable Channel0
	PDB0->CH[0].DLY[0] = 5600;									// Set delay
	PDB0->IDLY = 0;												// Set interrupt delay
	PDB0->SC  = PDB_SC_PRESCALER(0) | PDB_SC_MULT(0);			// Select clock prescaler and mult factor
	PDB0->SC |= PDB_SC_TRGSEL(0) | PDB_SC_PDBIE_MASK;			// Select trigger input source and enable interrupt
	PDB0->SC |= PDB_SC_PDBEN_MASK | PDB_SC_LDOK_MASK;			// Enable PDB and update PDB registers
}

/* SW=3, ADC0 initialized to be triggered by FTM0 Init triggers */
void ADC0_Init()
{
    PCC->PCCn[PCC_ADC0_INDEX] = 0;										// Disable clock for ADC0
    PCC->PCCn[PCC_ADC0_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;	// Enable clock for ADC0
    FSL_NVIC->ISER[ADC0_IRQn / 32] |= (1 << (ADC0_IRQn % 32));			// Enable interrupt

    /* Set divide ratio to 1 and select 8-bit conversion */
	ADC0->CFG1 = ADC_CFG1_ADIV(0) | ADC_CFG1_MODE(0) | ADC_CFG1_ADICLK(0);
	/* Select hardware trigger */
    ADC0->SC2 = ADC_SC2_ADTRG_MASK;
    /* Select channel 12 as an input and enable conversion complete interrupt */
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(12);
}

/* SW=3, FTM0 initialized to generate PWM and to trigger ADC0 through PDB0 */
void FTM0_Init()
{
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    /* Select and enable clock for FTM0 */
    PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

    /* Set PORTD pins for FTM0 */
    PORTD->PCR[15] = PORT_PCR_MUX(2);	// Set PTD15 for FTM0 – Channel0
    PORTD->PCR[16] = PORT_PCR_MUX(2);	// Set PTD16 for FTM0 – Channel1
    PORTD->PCR[0] = PORT_PCR_MUX(2);	// Set PTD0  for FTM0 – Channel2
    PORTD->PCR[1] = PORT_PCR_MUX(2);	// Set PTD1  for FTM0 – Channel3
    PORTD->PCR[2] = PORT_PCR_MUX(1);	// Set PTD2  as a PTD2
    PORTD->PCR[3] = PORT_PCR_MUX(1);	// Set PTD2  as a PTD2
    PTD->PDDR = (1<<2) | (1<<3);		// Set PTD2 and PTD3 as an output

    /* Select up-down counter for Center-Align PWM */
    FTM0->SC = FTM_SC_CPWMS_MASK;
    /* Enable combine, complementary mode and dead-time for channels pair CH0/CH1 and CH2/CH3 */
    FTM0->COMBINE = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK;
    FTM0->COMBINE |= FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_COMP1_MASK | FTM_COMBINE_DTEN1_MASK;
    /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
    FTM0->MOD = FTM_MOD_MOD(5600-1);
    /* Set CNTIN */
    FTM0->CNTIN = FTM_CNTIN_INIT(0);
    /* High-true pulses of PWM signals */
    FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
    /* Set Channel Value */
    FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800); // 50% duty-cycle
    FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800); // 50% duty-cycle
    /* FTM counter reset */
    FTM0->CNT = 0;
    /* Insert DeadTime (1us) */
    FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
    /* Enable trigger generation when FTM counter = CNTIN */
    FTM0->EXTTRIG = FTM_EXTTRIG_INITTRIGEN_MASK;
    /* Enable clock and PWM */
    FTM0->SC |= FTM_SC_CLKS(1) | FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK | FTM_SC_PWMEN2_MASK | FTM_SC_PWMEN3_MASK;
}


/* SW=4 - Single-Edge Capture Mode */
void FTM0_Single_Edge_Capture_Mode()
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    /* Select and enable clock for FTM0 */
    PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

    PORTD->PCR[15] = PORT_PCR_MUX(2);								// Set PTD15 for FTM0 - Channel0
    FSL_NVIC->ISER[FTM0_IRQn / 32] |= (1 << (FTM0_IRQn % 32));		// Enable FTM0 interrupt

    /* Input capture mode sensitive on rising edge to measure period of tested signal */
    FTM0->CONTROLS[0].CnSC =  FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK;
    /* Reset counter */
    FTM0->CNT = 0;
    /* Select clock */
    FTM0->SC = FTM_SC_CLKS(1);
}

/* SW=5 - Dual-Edge Capture Mode */
void FTM0_Dual_Edge_Capture_Mode()
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    /* Select and enable clock for FTM0 */
    PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

    PORTD->PCR[15] = PORT_PCR_MUX(2);		// Set PTD15 for FTM0 - Channel0
    PORTD->PCR[0] = PORT_PCR_MUX(2);		// Set PTD0  for FTM0 - Channel2

	FSL_NVIC->ISER[FTM0_IRQn / 32] |= (1 << (FTM0_IRQn % 32));		// Enable interrupt

	/* Enable dual-edge capture mode */
	FTM0->COMBINE= FTM_COMBINE_DECAPEN0_MASK | FTM_COMBINE_DECAP0_MASK
			     | FTM_COMBINE_DECAPEN1_MASK | FTM_COMBINE_DECAP1_MASK;

	/* Select positive polarity pulse width measurement and enable continuous mode for FTM0_CH0/CH2 */
	FTM0->CONTROLS[0].CnSC =  FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK;
    FTM0->CONTROLS[1].CnSC =  FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK;
	FTM0->CONTROLS[2].CnSC =  FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK;
    FTM0->CONTROLS[3].CnSC =  FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK;

    /* Reset counter */
    FTM0->CNT = 0;
    /* Select clock */
    FTM0->SC = FTM_SC_CLKS(1);
}

/* SW=6 - Quadrature Decoder Mode */
void FTM2_Quadrature_Decoder_Mode()
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    /* Select and enable clock for FTM2 */
    PCC->PCCn[PCC_FLEXTMR2_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

    PORTD->PCR[10] = PORT_PCR_MUX(3);								// Set PTD10 for FTM2 - Phase B input
    PORTD->PCR[11] = PORT_PCR_MUX(3);								// Set PTD11 for FTM2 - Phase A input
    PORTD->PCR[0] = PORT_PCR_MUX(1);								// Set PTD0 as a GPIO
    PTD->PDDR=1<<0;													// Set PTD0 as an output

    FSL_NVIC->ISER[FTM2_IRQn / 32] |= (1 << (FTM2_IRQn % 32));		// Enable interrupt

    /* Encoder simulation with totally 10 rising/falling edges */
    FTM2->MOD = FTM_MOD_MOD(10);
    FTM2->CNTIN = FTM_CNTIN_INIT(0);
    FTM2->QDCTRL= FTM_QDCTRL_QUADEN_MASK;

    /* Reset counter */
    FTM2->CNT = 0;
    /* Select clock */
    FTM2->SC = FTM_SC_CLKS(1) | FTM_SC_TOIE_MASK;
}

/* SW=7 - Center-Align PWM and Half Cycle Reload */
void Half_Cycle_PWM_Reload()
{
   /* Enable clock for PORTD */
   PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
   /* Select and enable clock for FTM0 */
   PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

   FSL_NVIC->ISER[FTM0_IRQn / 32] |= (1 << (FTM0_IRQn % 32));		// Enable interrupt

   PORTD->PCR[15] = PORT_PCR_MUX(2);	// Set PTD15 for FTM0 - Channel0
   PORTD->PCR[16] = PORT_PCR_MUX(2);	// Set PTD16 for FTM0 - Channel1
   PORTD->PCR[0] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel2
   PORTD->PCR[1] = PORT_PCR_MUX(2);		// Set PTD1 for FTM0  - Channel3
   PORTD->PCR[3] = PORT_PCR_MUX(1);		// Set PTD3 as a GPIO
   PTD->PDDR = 1<<3;					// Set PTD3 as an output
   PTD->PSOR |= 1<<3;					// Set PTD3 to high logic

   /* Select up-down counter for Center-Align PWM */
   FTM0->SC = FTM_SC_CPWMS_MASK;
   /* Enable registers updating from write buffers */
   FTM0->MODE = FTM_MODE_FTMEN_MASK;
   /* Enable sync, combine mode and dead-time for pair channel n=1 and n=2 */
   FTM0->COMBINE = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK
                 | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_COMP1_MASK | FTM_COMBINE_DTEN1_MASK;
   /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
   FTM0->MOD = FTM_MOD_MOD(5600-1);
   /* Set CNTIN */
   FTM0->CNTIN = FTM_CNTIN_INIT(0);
   /* High-true pulses of PWM signals */
   FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
   /* Set Channel Value */
   FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   /* FTM counter reset */
   FTM0->CNT = 0;
   /* Insert DeadTime (1us) */
   FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
   /* Enable half cycle reload */
   FTM0->SYNC = FTM_SYNC_CNTMAX_MASK | FTM_SYNC_CNTMIN_MASK;
   /* Select clock, enable reload opportunity interrupt and PWM generation */
   FTM0->SC |= FTM_SC_CLKS(1) | FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK
		    |  FTM_SC_PWMEN2_MASK | FTM_SC_PWMEN3_MASK  | FTM_SC_RIE_MASK;
}

/* SW=8 - Center-Align PWM and Full Cycle Reload */
void Full_Cycle_PWM_Reload()
{
   /* Enable clock for PORTD */
   PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
   /* Select and enable clock for FTM0 */
   PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

   FSL_NVIC->ISER[FTM0_IRQn / 32] |= (1 << (FTM0_IRQn % 32));		// Enable interrupt

   PORTD->PCR[15] = PORT_PCR_MUX(2);	// Set PTD15 for FTM0 - Channel0
   PORTD->PCR[16] = PORT_PCR_MUX(2);	// Set PTD16 for FTM0 - Channel1
   PORTD->PCR[0] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel2
   PORTD->PCR[1] = PORT_PCR_MUX(2);		// Set PTD1 for FTM0  - Channel3
   PORTD->PCR[3] = PORT_PCR_MUX(1);		// Set PTD3 as a GPIO
   PTD->PDDR = 1<<3;					// Set PTD3 as an output
   PTD->PSOR |= 1<<3;

   /* Select up-down counter for Center-Align PWM */
   FTM0->SC = FTM_SC_CPWMS_MASK;
   /* Enable registers updating from write buffers */
   FTM0->MODE = FTM_MODE_FTMEN_MASK;
   /* Enable sync, combine mode and dead-time for pair channel n=1 and n=2 */
   FTM0->COMBINE = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK
                 | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_COMP1_MASK | FTM_COMBINE_DTEN1_MASK;
   /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
   FTM0->MOD = FTM_MOD_MOD(5600-1);
   /* Set CNTIN */
   FTM0->CNTIN = FTM_CNTIN_INIT(0);
   /* High-true pulses of PWM signals */
   FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
   /* Set Channel Value */
   FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   /* FTM counter reset */
   FTM0->CNT = 0;
   /* Insert DeadTime (1us) */
   FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
   /* Enable full cycle reload */
   FTM0->SYNC = FTM_SYNC_CNTMAX_MASK;
   /* Select clock, enable reload opportunity interrupt and PWM generation */
   FTM0->SC |= FTM_SC_CLKS(1) | FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK
		    |  FTM_SC_PWMEN2_MASK | FTM_SC_PWMEN3_MASK  | FTM_SC_RIE_MASK;
}


/* SW=9 - Center-Align PWM Synchronized by Software Trigger */
void CPWM_and_Software_Synchronization()
{
   /* Enable clock for PORTD */
   PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
   /* Select and enable clock for FTM0 */
   PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

   FSL_NVIC->ISER[FTM0_IRQn / 32] |= (1 << (FTM0_IRQn % 32));		// Enable interrupt

   PORTD->PCR[15] = PORT_PCR_MUX(2);	// Set PTD15 for FTM0 - Channel0
   PORTD->PCR[16] = PORT_PCR_MUX(2);	// Set PTD16 for FTM0 - Channel1
   PORTD->PCR[0] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel2
   PORTD->PCR[1] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel3
   PORTD->PCR[3] = PORT_PCR_MUX(1);		// Set PTD3 as a GPIO
   PTD->PDDR = 1<<3;					// Set PTD3 as an output

   /* Select up-down counter for Center-Align PWM */
   FTM0->SC = FTM_SC_CPWMS_MASK;
   /* Enable registers updating from write buffers */
   FTM0->MODE = FTM_MODE_FTMEN_MASK;
   /* Enable sync, combine mode and dead-time for channels pair n=1 and n=2 */
   FTM0->COMBINE = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK
                 | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_COMP1_MASK | FTM_COMBINE_DTEN1_MASK;
   /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
   FTM0->MOD = FTM_MOD_MOD(5600);
   /* Set CNTIN */
   FTM0->CNTIN = FTM_CNTIN_INIT(0);
   /* High-true pulses of PWM signals */
   FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
   FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
   /* Set Channel Value */
   FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   FTM0->CONTROLS[1].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   FTM0->CONTROLS[3].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
   /* FTM counter reset */
   FTM0->CNT = 0;
   /* Insert DeadTime (1us) */
   FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
   /* Select clock, enable reload opportunity interrupt */
   FTM0->SC |= FTM_SC_CLKS(1) | FTM_SC_RIE_MASK;
   /* Enable reload opportunity interrupt when FTM counter reach CNTMAX value */
   FTM0->SYNC |= FTM_SYNC_SYNCHOM_MASK | FTM_SYNC_CNTMAX_MASK;
   /* Allow each fourth reload opportunity interrupt */
   FTM0->CONF = FTM_CONF_LDFQ(3);
   /* Enable software synchronization */
   FTM0->SYNCONF = FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_SWWRBUF_MASK | FTM_SYNCONF_SWOM_MASK | FTM_SYNCONF_SWRSTCNT_MASK;
   /* Enable PWM */
   FTM0->SC |= FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK | FTM_SC_PWMEN2_MASK | FTM_SC_PWMEN3_MASK;
}

/* SW=10 - Center-Align PWM Synchronized by Hardware Trigger */
void CPWM_and_Hardware_Synchronization()
{
	   /* Enable clock for PORTA */
	   PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;
	   /* Enable clock for PORTD */
	   PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
	   /* Select and enable clock for FTM0 */
	   PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

	   PORTD->PCR[15] = PORT_PCR_MUX(2);	// Set PTD15 for FTM0 - Channel0
	   PORTD->PCR[16] = PORT_PCR_MUX(2);	// Set PTD16 for FTM0 - Channel1
	   PORTD->PCR[0] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel2
	   PORTD->PCR[1] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel3
	   PORTA->PCR[14] = PORT_PCR_MUX(2);	// Set PTA14 as fault pin for hardware PWM sync
	   PORTD->PCR[3] = PORT_PCR_MUX(1);		// Set PTD3 as a GPIO
	   PTD->PDDR = 1<<3;					// Set PTD3 as an output

       SIM->FTMOPT0 &= ~SIM_FTMOPT0_FTM0CLKSEL(1);

	   /* Select up-down counter for Center-Align PWM */
	   FTM0->SC = FTM_SC_CPWMS_MASK;
	   /* Enable registers updating from write buffers */
	   FTM0->MODE = FTM_MODE_FTMEN_MASK;
	   /* Enable sync, combine mode and dead-time for pair channel n=1 and n=2 */
	   FTM0->COMBINE = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_COMP0_MASK | FTM_COMBINE_DTEN0_MASK
	                 | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_COMP1_MASK | FTM_COMBINE_DTEN1_MASK;
	   /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
	   FTM0->MOD = FTM_MOD_MOD(5600);
	   /* Set CNTIN */
	   FTM0->CNTIN = FTM_CNTIN_INIT(0);
	   /* High-true pulses of PWM signals */
	   FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
	   FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
	   FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
	   FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
	   /* Set Channel Value */
	   FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
	   FTM0->CONTROLS[1].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
	   FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
	   FTM0->CONTROLS[3].CnV=FTM_CnV_VAL(2800);		// 50% duty cycle
	   /* FTM counter reset */
	   FTM0->CNT = 0;
	   /* Insert DeadTime (1us) */
	   FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
	   /* Select and enable clock */
	   FTM0->SC |= FTM_SC_CLKS(1);
	   /* Enable hardware synchronization */
	   FTM0->SYNCONF = FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_HWWRBUF_MASK;
	   /* Enable FTM counter reset after hardware trigger */
	   FTM0->SYNCONF |= FTM_SYNCONF_HWRSTCNT_MASK | FTM_SYNCONF_HWTRIGMODE_MASK;
	   /* Select FTM0_FLT0 pin as a hardware trigger input */
	   FTM0->SYNC = FTM_SYNC_TRIG2_MASK;
	   /* Enable PWM */
	   FTM0->SC |= FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK | FTM_SC_PWMEN2_MASK | FTM_SC_PWMEN3_MASK;
}

/* SW=11 - Center-Align PWM and Fault Control */
void CPWM_and_Fault_Control()
{
    /* Enable clock for PORTA */
    PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    /* Select and enable clock for FTM0 */
    PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

    PORTD->PCR[15] = PORT_PCR_MUX(2);		// Set PTD15 for FTM0 - Channel0
    PORTD->PCR[16] = PORT_PCR_MUX(2);		// Set PTD16 for FTM0 - Channel1
    PORTD->PCR[0] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel2
    PORTD->PCR[1] = PORT_PCR_MUX(2);		// Set PTD0 for FTM0  - Channel3
    PORTA->PCR[14] = PORT_PCR_MUX(2);		// Set PTD14 as a fault pin

    /* Enable registers updating from write buffers */
    FTM0->MODE = FTM_MODE_FTMEN_MASK;
    /* Enable combine, complementary mode and dead-time for pair channel n=1 and n=2 */
    FTM0->COMBINE=FTM_COMBINE_COMBINE0_MASK|FTM_COMBINE_COMP0_MASK|FTM_COMBINE_DTEN0_MASK
                 |FTM_COMBINE_FAULTEN0_MASK|FTM_COMBINE_COMBINE1_MASK|FTM_COMBINE_COMP1_MASK
                 |FTM_COMBINE_DTEN1_MASK|FTM_COMBINE_FAULTEN1_MASK;
    /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
    FTM0->MOD = FTM_MOD_MOD(11200);
    /* Set CNTIN */
    FTM0->CNTIN = FTM_CNTIN_INIT(0);
    /* High-true pulses of PWM signals */
    FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
    /* Set Channel Value */
    FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(1120);
    FTM0->CONTROLS[1].CnV=FTM_CnV_VAL(11200-1120);
    FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);
    FTM0->CONTROLS[3].CnV=FTM_CnV_VAL(11200-2800);
    /* FTM counter reset */
    FTM0->CNT = 0;
    /* Insert DeadTime (1us) */
    FTM0->DEADTIME = FTM_DEADTIME_DTPS(3) | FTM_DEADTIME_DTVAL(7);
    /* Set PTA14 pin as a fault input FTM0_FLT0 */
    FTM0->FLTCTRL = FTM_FLTCTRL_FAULT0EN_MASK;
    /* Enable fault control for all channels and select automatic fault clearing mode */
    FTM0->MODE |= FTM_MODE_FAULTM(3);
    /* Safe value is set as a low after fault input is detected */
    FTM0->POL = (~FTM_POL_POL0_MASK) & (~FTM_POL_POL1_MASK);
    /* A 1 at the fault input indicates the fault */
    FTM0->FLTPOL &= ~FTM_FLTPOL_FLT0POL_MASK;
    /* Select clock and enable PWM */
    FTM0->SC = FTM_SC_CLKS(1) | FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK | FTM_SC_PWMEN2_MASK
             |FTM_SC_PWMEN3_MASK;
}

/* SW=12 - Center-Align PWM and Global Time Base (GTB) applied to FTM0 and FTM2 */
void CPWM_and_Global_Time_Base()
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;

    /* Select and enable clock for FTM0 */
    PCC->PCCn[PCC_FLEXTMR0_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
    /* Select and enable clock for FTM2 */
    PCC->PCCn[PCC_FLEXTMR2_INDEX] =  PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;

    PORTD->PCR[15] = PORT_PCR_MUX(2);		// Set PTD15 for FTM0 - Channel0
    PORTD->PCR[16] = PORT_PCR_MUX(2);		// Set PTD16 for FTM0 - Channel1
    PORTD->PCR[0] = PORT_PCR_MUX(4);		// Set PTD0  for FTM2 - Channel0
    PORTD->PCR[1] = PORT_PCR_MUX(4);		// Set PTD1  for FTM2 - Channel1

    /* Select up-down counter for Center-Align PWM */
    FTM0->SC = FTM_SC_CPWMS_MASK;
    FTM2->SC = FTM_SC_CPWMS_MASK;
    /* Enable registers updating from write buffers */
    FTM0->MODE = FTM_MODE_FTMEN_MASK;
    FTM2->MODE = FTM_MODE_FTMEN_MASK;
    /* Enable complementary mode for channels pair n=1 */
    FTM0->COMBINE = FTM_COMBINE_COMP0_MASK;
    FTM2->COMBINE = FTM_COMBINE_COMP0_MASK;
    /* Set Modulo (10kHz PWM frequency @112MHz system clock) */
    FTM0->MOD = FTM_MOD_MOD(5600);
    FTM2->MOD = FTM_MOD_MOD(5600);
    /* Set CNTIN */
    FTM0->CNTIN = FTM_CNTIN_INIT(0);
    FTM2->CNTIN = FTM_CNTIN_INIT(0);
    /* High-true pulses of PWM signals */
    FTM0->CONTROLS[0].CnSC =  FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[1].CnSC =  FTM_CnSC_ELSB_MASK;
    FTM2->CONTROLS[0].CnSC =  FTM_CnSC_ELSB_MASK;
    FTM2->CONTROLS[1].CnSC =  FTM_CnSC_ELSB_MASK;
    /* Set Channel Value – 1% duty cycle */
    FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(56);
    FTM2->CONTROLS[0].CnV=FTM_CnV_VAL(56);

    /* FTM counter reset */
    FTM0->CNT = 0;
    FTM2->CNT = 0;

    /* Enable global time base to control FTM0 and FTM2 */
    FTM0->CONF = FTM_CONF_GTBEEN_MASK;
    FTM2->CONF = FTM_CONF_GTBEEN_MASK;

    /* Select clock */
    FTM0->SC |= FTM_SC_CLKS(1);
    FTM2->SC |= FTM_SC_CLKS(1);

    /* Synchronization signal for FTM0 and FTM2 */
    FTM0->CONF |= FTM_CONF_GTBEOUT_MASK;

    /* Enable PWM */
    FTM0->SC |= FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK;
    FTM2->SC |= FTM_SC_PWMEN0_MASK | FTM_SC_PWMEN1_MASK;
}

/* FTM1 initialized to simulate encoder signals for Quadrature Decoder Mode of FTM2 */
void Encoder_Generator()
{
    /* Enable clock for FTM1 */
    PCC->PCCn[PCC_FLEXTMR1_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTB */
    PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;

    PORTB->PCR[2] = PORT_PCR_MUX(2);				// Set PTB2 for FTM1 – Channel0
    PORTB->PCR[3] = PORT_PCR_MUX(2);				// Set PTB3 for FTM1 – Channel1
    PORTD->PCR[8] = PORT_PCR_MUX(6);				// Set PTD8 for FTM1 – Channel4
    PORTD->PCR[9] = PORT_PCR_MUX(6);				// Set PTD9 for FTM1 – Channel5

    FTM1->COMBINE = FTM_COMBINE_COMBINE0_MASK | FTM_COMBINE_COMBINE2_MASK;
    FTM1->CONTROLS[0].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[1].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[4].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[5].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->MOD = FTM_MOD_MOD(11200);					// Set modulo
    FTM1->CONTROLS[0].CnV=FTM_CnV_VAL(2800);		// Set channel Value
    FTM1->CONTROLS[1].CnV=FTM_CnV_VAL(8400);		// Set channel Value
    FTM1->CONTROLS[4].CnV=FTM_CnV_VAL(5600);		// Set channel Value
    FTM1->CONTROLS[5].CnV=FTM_CnV_VAL(11200);		// Set channel Value
    FTM1->CNT = 0;				       				// Counter reset
    FTM1->SC|=FTM_SC_CLKS(1)|FTM_SC_PWMEN0_MASK|FTM_SC_PWMEN1_MASK|FTM_SC_PWMEN4_MASK
    	    |FTM_SC_PWMEN5_MASK; 					// Select clock and enable PWM
}

/* FTM1 is initialized to generate signals for Single-Edge Capture Mode of FTM0
 * and Dual-Edge Capture Mode of FTM0  */
void Signals_Generator()
{
    /* Enable clock for FTM1 */
    PCC->PCCn[PCC_FLEXTMR1_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTB */
    PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTC */
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;

    PORTB->PCR[2] = PORT_PCR_MUX(2);									// Set PTB2 for FTM1 – Channel0
    PORTB->PCR[3] = PORT_PCR_MUX(2);									// Set PTB3 for FTM1 – Channel1
    PORTD->PCR[8] = PORT_PCR_MUX(6);									// Set PTD8 for FTM1 – Channel4
    PORTD->PCR[9] = PORT_PCR_MUX(6);									// Set PTd9 for FTM1 – Channel5

    FTM1->CONTROLS[0].CnSC=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[1].CnSC=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[4].CnSC=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->CONTROLS[5].CnSC=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;		// Select high-true pulses
    FTM1->MOD = FTM_MOD_MOD(11200 - 1);									// Set Modulo (10kHz PWM frequency @112MHz system clock)
    FTM1->CONTROLS[0].CnV=FTM_CnV_VAL(2000);							// Set channel Value
    FTM1->CONTROLS[1].CnV=FTM_CnV_VAL(5600);							// Set channel Value
    FTM1->CONTROLS[4].CnV=FTM_CnV_VAL(1000);							// Set channel Value
    FTM1->CONTROLS[5].CnV=FTM_CnV_VAL(5600);							// Set channel Value
    FTM1->CNT = 0;				       									// Counter reset
    FTM1->SC|=FTM_SC_CLKS(1)|FTM_SC_PWMEN0_MASK|FTM_SC_PWMEN1_MASK|FTM_SC_PWMEN4_MASK
    	    |FTM_SC_PWMEN5_MASK; 										// Select clock and enable PWM
}

/* FTM1 initialized to generate fault signals for Center-Align PWM with Fault Control of FTM0 */
void Fault_Signal_Generator()
{
    /* Enable clock for FTM1 */
    PCC->PCCn[PCC_FLEXTMR1_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
    /* Enable clock for PORTB */
    PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;

    PORTB->PCR[2] = PORT_PCR_MUX(2);				// Set PTB2 for FTM1 – Channel0

    FTM1->SC=FTM_SC_CPWMS_MASK;						// Select up-down counter for Center-Align PWM
    FTM1->CONTROLS[0].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses

    /* Set Modulo (2240Hz PWM frequency @112MHz system clock) */
    FTM1->MOD = FTM_MOD_MOD(50000);					// Set modulo
    FTM1->CONTROLS[0].CnV=FTM_CnV_VAL(25000);		// Set Channel Value
    FTM1->CNT = 0;				       				// Counter reset
    FTM1->SC|=FTM_SC_CLKS(1)|FTM_SC_PWMEN0_MASK; 	// Select clock and enable PWM
}

/* FTM1 initialized to generate hardware trigger signal to synchronize Center-Align PWM of FTM0 */
void Hardware_Triggers_Generator()
{
	 /* Enable clock for FTM1 */
	 PCC->PCCn[PCC_FLEXTMR1_INDEX] = PCC_PCCn_PCS(6) | PCC_PCCn_CGC_MASK;
	 /* Enable clock for PORTB */
	 PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;

	 PORTB->PCR[2] = PORT_PCR_MUX(2);				// PTB2 is used for FTM1-Channel0

	 FSL_NVIC->IP[FTM1_IRQn] = 0x01;								// Set priority interrupt
	 FSL_NVIC->ISER[FTM1_IRQn / 32] |= (1 << (FTM1_IRQn % 32));		// Enable interrupt

	 FTM1->SC=FTM_SC_CPWMS_MASK;					// Select up-down counter for Center-Align PWM
	 FTM1->CONTROLS[0].CnSC=FTM_CnSC_ELSB_MASK;		// Select high-true pulses
	 FTM1->MOD = FTM_MOD_MOD(15000);				// Set modulo
	 FTM1->CONTROLS[0].CnV=FTM_CnV_VAL(7500);		// Set Channel Value
	 FTM1->CNT = 0;									// Counter reset
	 /* Select clock, enable reload opportunity interrupt and PWM generation */
	 FTM1->SC|=FTM_SC_CLKS(1)|FTM_SC_RIE_MASK|FTM_SC_PWMEN0_MASK;
	 /* Reload opportunity interrupt occurs when FTM counter reach CNTMAX value */
	 FTM1->SYNC |= FTM_SYNC_CNTMAX_MASK;
}

/* Transmit from RUN mode to HSRUN mode */
void HSRUN_Init()
{
    SMC->PMPROT=SMC_PMPROT_AHSRUN_MASK	// Allows High Speed Run
    		   |SMC_PMPROT_AVLP_MASK;	// Allows Very Low Power Modes
    SMC->PMCTRL=SMC_PMCTRL_RUNM(3);		// Entry to High Speed Run
    /* Wait for High Speed Run mode */
    while(SMC->PMSTAT != SMC_PMSTAT_PMSTAT(128))
    {
    }
}

/* System oscillator and HSRUN configuration */
void SCG_Init()
{
	/* SOSC Configuration
	 * OSC_OUT = 8MHz, 8MHz EXTAL */

	SCG->SOSCDIV=SCG_SOSCDIV_SOSCDIV1(1)	// System OSC DIV1=1
	      	    |SCG_SOSCDIV_SOSCDIV2(1);	// System OSC DIV2=1
	SCG->SOSCCFG=SCG_SOSCCFG_RANGE(2) 		// Medium frequency range OSC 1-8MHz
				|SCG_SOSCCFG_EREFS(1);		// Select internal OSC
	while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK);	// Wait for SOSCCSR unlocked state
	SCG->SOSCCSR=SCG_SOSCCSR_SOSCLPEN_MASK	// Enable OSC in very low power modes
			    |SCG_SOSCCSR_SOSCEN_MASK;	// Enable OSC
	while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK));	// Wait for OSC enabling and valid output

	/* SPLL Configuration
	 * PLL_OUT = 112MHz, 8MHz EXTAL */

	SCG->SPLLDIV=SCG_SPLLDIV_SPLLDIV1(1)	// PLL DIV1=1
	 			|SCG_SPLLDIV_SPLLDIV2(1);	// PLL DIV2=1
	SCG->SPLLCFG=SCG_SPLLCFG_PREDIV(0)      // PLL PREDIV=0
	     		|SCG_SPLLCFG_MULT(12);      // PLL MULT=28
	while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK);	// Wait for SPLLCSR unlocked state
	SCG->SPLLCSR=SCG_SPLLCSR_SPLLEN_MASK;	// Enable PLL
	while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK));	// Wait for PLL enabling and valid output

	/* HCCR Configuration
	 * PLL_OUT = 112MHz, 8MHz EXTAL */

    SCG->HCCR=SCG_HCCR_SCS(6)		// PLL source clock
			 |SCG_HCCR_DIVCORE(0)	// DIVCORE=1, Core=112MHz
			 |SCG_HCCR_DIVBUS(1)	// DIVBUS=2, BUS=56MHz
			 |SCG_HCCR_DIVSLOW(3);	// DIVSLOW=4, Flash=28MHz
}




//////////////////
////   MAIN   ////
//////////////////

int main(void)
{
	SCG_Init();
	HSRUN_Init();

/* Edge-Align PWM Mode */
#if SW == 0
	Edge_Align_PWM_Init();

/* Center-Align PWM Mode */
#elif SW == 1
	Center_Align_PWM_Init();

/* Combine Mode for Phase Shifted PWM */
#elif SW == 2
	Phase_Shifted_PWM();

/* ADC triggering by FTM through PDB module */
#elif SW == 3
	ADC0_Init();
	TRGMUX_Init();
	PDB0_Init();
	FTM0_Init();

/* Single-Edge Capture Mode */
#elif SW == 4
	Signals_Generator();
	FTM0_Single_Edge_Capture_Mode();

/* Dual-Edge Capture Mode */
#elif SW == 5
	Signals_Generator();
	FTM0_Dual_Edge_Capture_Mode();

/* Quadrature Decoder Mode */
#elif SW == 6
	Encoder_Generator();
	FTM2_Quadrature_Decoder_Mode();

/* Center-Align PWM and Half Cycle Reload */
#elif SW == 7
	Half_Cycle_PWM_Reload();

/* Center-Align PWM and Full Cycle Reload */
#elif SW == 8
	Full_Cycle_PWM_Reload();

/* Center-Align PWM Synchronized by Software Trigger */
#elif SW == 9
	CPWM_and_Software_Synchronization();

/* Center-Align PWM Synchronized by Hardware Trigger */
#elif SW == 10
	CPWM_and_Hardware_Synchronization();
	Hardware_Triggers_Generator();

/* Center-Align PWM and Fault Control */
#elif SW == 11
	CPWM_and_Fault_Control();
	Fault_Signal_Generator();

/* Center-Align PWM and Global Time Base (GTB) */
#elif SW == 12
	CPWM_and_Global_Time_Base();


#else
#error "Undefined FlexTimer Feature!"
#endif

	for(;;)
	{

	}
	
	return 0;
}

///////////////////////
////   INTERRUPTS   ///
///////////////////////


/* FTM0 Interrupt routine */
void FTM0_IRQHandler()
{

/* Interrupt routine to determine period of tested signal */
#if SW == 4
	FTM0_CH0_period = FTM0->CONTROLS[0].CnV - temp;		// Period calculation
    temp = FTM0->CONTROLS[0].CnV;						// Save C0V value into the variable
    FTM0->CONTROLS[0].CnSC &= ~FTM_CnSC_CHF_MASK;		// clear channel flag

/* Interrupt routine to measure positive polarity pulse width of tested input signals */
#elif SW == 5

	if(FTM0->CONTROLS[1].CnSC & FTM_CnSC_CHF_MASK)		// wait for falling edge of FTM0_CH0 signal
	{
		CH0_edge1 = FTM0->CONTROLS[0].CnV;
		CH0_edge2 = FTM0->CONTROLS[1].CnV;
		FTM0_CH0_pulse_width =  CH0_edge2 - CH0_edge1;	// pulse width calculation of FTM0_CH0 signal
		FTM0->CONTROLS[1].CnSC &= ~FTM_CnSC_CHF_MASK;	// clear FTM0_CH1 capture flag
	}
	if(FTM0->CONTROLS[3].CnSC & FTM_CnSC_CHF_MASK)		// wait for falling edge of FTM0_CH2 signal
	{
		CH2_edge1 = FTM0->CONTROLS[2].CnV;
		CH2_edge2 = FTM0->CONTROLS[3].CnV;
		FTM0_CH2_pulse_width =  CH2_edge2 - CH2_edge1;	// pulse width calculation of FTM0_CH2 signal
		FTM0->CONTROLS[3].CnSC &= ~FTM_CnSC_CHF_MASK;	// clear FTM0_CH3 capture flag
	}

/* Half cycle reload opportunity interrupt routine */
#elif SW == 7

    if(flag)
    {
        FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(5600);
        FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);
    }
    else
    {
        FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800);
        FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(5600);
    }
	flag = !flag;
	PTD->PTOR = 1<<3;									// Toggle PTD3 to show reload opportunities
	FTM0->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;				// Set LDOK bit
	FTM0->SC &= ~FTM_SC_RF_MASK;						// Clear Reload Flag

/* Full cycle reload opportunity interrupt routine */
#elif SW == 8

    if(flag)
    {
        FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(5600);
        FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);
    }
    else
    {
        FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800);
        FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(5600);
    }
	flag = !flag;
	PTD->PTOR = 1<<3;									// Toggle PTD3 to show reload opportunities
	FTM0->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;				// Set LDOK bit
	FTM0->SC &= ~FTM_SC_RF_MASK;						// Clear Reload Flag

/* Interrupt routine for updating FTM registers - Software synchronization */
#elif SW == 9

    if(flag)
    {
        /* Enable FTM0_CH0/FTM0_CH1 mask */
        FTM0->OUTMASK = FTM_OUTMASK_CH0OM_MASK | FTM_OUTMASK_CH1OM_MASK;
        FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(1120);
        FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(1120);
        if(flagMask)
        {
            /* Set FTM0_CH0/FTM0_CH1 output polarity to high */
            FTM0->POL = FTM_POL_POL0_MASK | FTM_POL_POL1_MASK;
        }
	else
        {
            /* Set FTM0_CH0/FTM0_CH1 output polarity to low */
            FTM0->POL &= (~FTM_POL_POL0_MASK) & (~FTM_POL_POL1_MASK);
        }
	flagMask = !flagMask;
	}
    else
    {
        /* Set FTM0_CH0/FTM0_CH1 output polarity to low */
        FTM0->POL &= (~FTM_POL_POL0_MASK) & (~FTM_POL_POL1_MASK);
        /* Disable FTM0_CH0/FTM0_CH1 mask */
        FTM0->OUTMASK &= (~FTM_OUTMASK_CH0OM_MASK) & (~FTM_OUTMASK_CH1OM_MASK);
        FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(2800);
        FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(2800);
    }
	flag = !flag;
	PTD->PTOR |= 1<<3;										// Toggle PTD3 to show reload opportunities
	FTM0->SYNC |= FTM_SYNC_SWSYNC_MASK;						// Software sync
	FTM0->SC &= ~FTM_SC_RF_MASK;							// Clear Reload Flag

#endif

}

/* FTM1 interrupt routine to change values of FTM0 registers */
void FTM1_IRQHandler()
{
	if(flag)
		{
		    FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(1000);
			FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(4000);
		}
	else
		{
			FTM0->CONTROLS[0].CnV=FTM_CnV_VAL(4000);
			FTM0->CONTROLS[2].CnV=FTM_CnV_VAL(1000);
		}
	flag = !flag;
	FTM1->SC &= ~FTM_SC_RF_MASK;							// Clear Reload Flag bit
}

/* FTM2 interrupt routine */
void FTM2_IRQHandler()
{
	PTD->PTOR|=1<<0;										// Toggle PTD0
	FTM2->SC &= ~FTM_SC_TOF_MASK;							// Clear timer overflow flag
}

/* ADC0 interrupt routine */
void ADC0_IRQHandler()
{
	PTD->PTOR |= 1<<3;										// Toggle PTD3
	ADC0_RA = ADC0->R[0];									// Read ADC result register
}

/* PDB0 interrupt routine */
void PDB0_IRQHandler()
{
	PTD->PTOR |= 1<<2;										// Toggle PTD2
    PDB0->SC &= ~PDB_SC_PDBIF_MASK; 						// Clear PDB interrupt flag
}
