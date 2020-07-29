/***************************************************************************//**
 * @file main_pdm_stereo_ldma.c
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Labs, Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_pdm.h"
#include "retargetserialconfig.h"
#include "stdio.h"
#include "retargetserial.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_timer.h"

#include "gpiointerrupt.h"

// Global variables used to set top value and duty cycle of the timer
#define PWM_FREQ          20000
//Changed: 20000 to 32986.111
#define DUTY_CYCLE_STEPS  0.05
#define TARGET_DUTY_CYCLE	0.55

// DMA channel used for the example
#define LDMA_CHANNEL        0
#define LDMA_CH_MASK        (1 << LDMA_CHANNEL)

// Left/right buffer size
#define BUFFER_SIZE         2048
#define SOUND_LENGTH		BUFFER_SIZE / 16

// Ping-pong buffer size
#define PP_BUFFER_SIZE      128

// Buffers for left/right PCM data
int16_t left[BUFFER_SIZE];
//int16_t right[BUFFER_SIZE];

// Descriptor linked list for LDMA transfer
LDMA_Descriptor_t descLink[2];

// Buffers for ping-pong transfer
uint32_t pingBuffer[PP_BUFFER_SIZE];
uint32_t pongBuffer[PP_BUFFER_SIZE];

#define LENC (BUFFER_SIZE + SOUND_LENGTH - 1)
int corrL[LENC];
//int corrR[LENC];

// Keeps track of previously written buffer
bool prevBufferPing;

//Speaker definitions
#define OUT_FREQ 32986.11111111111

static uint32_t topValue = 0;
static volatile float dutyCycle = 0;
volatile int pulse_width=PWM_FREQ / 400;  // This sets the pulse width to 2.5mS (1/400)

volatile bool on_off = true;

/***************************************************************************//**
 * @brief
 *   Sets up PDM microphones
 ******************************************************************************/
void initPdm(void)
{
  // Set up clocks
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PDM, true);
  CMU_ClockSelectSet(cmuClock_PDM, cmuSelect_HFRCODPLL); // 19 MHz

  // Config GPIO and pin routing
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);    // MIC_EN
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 0);    // PDM_CLK
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput, 0);       // PDM_DATA

  GPIO_SlewrateSet(gpioPortC, 7, 7);

  GPIO->PDMROUTE.ROUTEEN = GPIO_PDM_ROUTEEN_CLKPEN;
  GPIO->PDMROUTE.CLKROUTE = (gpioPortC << _GPIO_PDM_CLKROUTE_PORT_SHIFT)
                            | (6 << _GPIO_PDM_CLKROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT0ROUTE = (gpioPortC << _GPIO_PDM_DAT0ROUTE_PORT_SHIFT)
                            | (7 << _GPIO_PDM_DAT0ROUTE_PIN_SHIFT);
  GPIO->PDMROUTE.DAT1ROUTE = (gpioPortC << _GPIO_PDM_DAT1ROUTE_PORT_SHIFT)
                            | (7 << _GPIO_PDM_DAT1ROUTE_PIN_SHIFT);

  // Config PDM
  PDM->CFG0 = PDM_CFG0_STEREOMODECH01_CH01ENABLE
              | PDM_CFG0_CH0CLKPOL_NORMAL
              | PDM_CFG0_CH1CLKPOL_INVERT
              | PDM_CFG0_FIFODVL_FOUR
              | PDM_CFG0_DATAFORMAT_DOUBLE16
              | PDM_CFG0_NUMCH_TWO
              | PDM_CFG0_FORDER_FIFTH;

  PDM->CFG1 = (5 << _PDM_CFG1_PRESC_SHIFT);
  //Changed: 5 to 20 to 5

  // Enable module
  PDM->EN = PDM_EN_EN;

  // Start filter
  while(PDM->SYNCBUSY != 0);
  PDM->CMD = PDM_CMD_START;

  // Config DSR/Gain
  while(PDM->SYNCBUSY != 0);
  PDM->CTRL = (8 << _PDM_CTRL_GAIN_SHIFT) | (32 << _PDM_CTRL_DSR_SHIFT);
  //change 3 to 8 once thing works
}

/**************************************************************************//**
 * @brief
 *    Interrupt handler for TIMER0 that changes the duty cycle
 *
 * @note
 *    This handler doesn't actually dynamically change the duty cycle. Instead,
 *    it acts as a template for doing so. Simply change the dutyCycle
 *    global variable here to dynamically change the duty cycle.
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER0);
  static int count=0;
  TIMER_IntClear(TIMER0, flags);
  if ((dutyCycle<TARGET_DUTY_CYCLE)&&(on_off)) {
	  dutyCycle += DUTY_CYCLE_STEPS;
  }
  if ((dutyCycle>0)&&(!on_off)) {
	  dutyCycle -= DUTY_CYCLE_STEPS;
  }
  if (on_off) {
	  count++;
	  if (count==pulse_width) {
		  on_off = false;
		  count=0;
	  }
  }
  // Update CCVB to alter duty cycle starting next period
  TIMER_CompareBufSet(TIMER0, 0, (uint32_t)(topValue * dutyCycle));
}


/***************************************************************************//**
 * @brief
 *   Initialize the LDMA controller for ping-pong transfer
 ******************************************************************************/
void initLdma(void)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // LDMA transfers trigger on PDM Rx Data Valid
  LDMA_TransferCfg_t periTransferTx =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_PDM_RXDATAV);

  // Link descriptors for ping-pong transfer
  descLink[0] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&PDM->RXDATA, pingBuffer,
                                     PP_BUFFER_SIZE, 1);
  descLink[1] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&PDM->RXDATA, pongBuffer,
                                     PP_BUFFER_SIZE, -1);

  // Next transfer writes to pingBuffer
  prevBufferPing = false;

  LDMA_Init(&init);

  LDMA_StartTransfer(LDMA_CHANNEL, (void*)&periTransferTx, (void*)&descLink);
}

/***************************************************************************//**
 * @brief
 *   LDMA IRQ handler.
 ******************************************************************************/
void LDMA_IRQHandler(void)
{
  uint32_t pending;

  // Read interrupt source
  pending = LDMA_IntGet();

  // Clear interrupts
  LDMA_IntClear(pending);

  // Check for LDMA error
  if(pending & LDMA_IF_ERROR) {
    // Loop here to enable the debugger to see what has happened
    while(1);
  }

  // Keep track of previously written buffer
  prevBufferPing = !prevBufferPing;
}

//Speaker Functions

/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
//void initGpio(void)
//{
//	  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
//}


void initGpio(void)
{
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief
 *    CMU initialization
 *****************************************************************************/
void initCmu(void)
{
  // Enable clock to GPIO and TIMER0
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
}

/**************************************************************************//**
 * @brief TIMER initialization
 *****************************************************************************/

//void initTIMER(void)
//{
//  uint32_t timerFreq = 0;
//  // Initialize the timer
//  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
//  // Configure TIMER0 Compare/Capture for output compare
//  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
//
//  // Use PWM mode, which sets output on overflow and clears on compare events
//  timerInit.prescale = timerPrescale64;
//  timerInit.enable = false;
//  timerCCInit.mode = timerCCModePWM;
//
//  // Configure but do not start the timer
//  TIMER_Init(TIMER0, &timerInit);
//
//  // Route Timer0 CC0 output to PA6
//  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
//  // GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortA << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
//  //   							     | (6 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
//  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
//  								  | (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
//
//  // Configure CC Channel 0
//  TIMER_InitCC(TIMER0, 0, &timerCCInit);
//
//  // Start with 10% duty cycle
//  dutyCycle = DUTY_CYCLE_STEPS;
//
//  // set PWM period
//  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
//  topValue = (timerFreq / PWM_FREQ);
//
//  //int topValue = timerFreq / (2*OUT_FREQ) - 1;
//  // Set top value to overflow at the desired PWM_FREQ frequency
//  TIMER_TopSet(TIMER0, topValue);
//
//  // Set compare value for initial duty cycle
//  TIMER_CompareSet(TIMER0, 0, (uint32_t)(topValue * dutyCycle));
//
//  // Start the timer
//  TIMER_Enable(TIMER0, true);
//
//  // Enable TIMER0 compare event interrupts to update the duty cycle
//  TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);
//  NVIC_EnableIRQ(TIMER0_IRQn);
//}

/////////////////////////////////////////////////////////////////////////////////////////


void initTIMER()
{
	//uint32_t OUT_FREQ= 1000;
	uint32_t timerFreq = 0;
  // Initialize the timer
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Configure TIMER0 Compare/Capture for output compare
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  timerInit.prescale = timerPrescale1;
  timerInit.enable = false;
  timerCCInit.mode = timerCCModeCompare;
  timerCCInit.cmoa = timerOutputActionToggle;

  // configure, but do not start timer
  TIMER_Init(TIMER0, &timerInit);

  // Route Timer0 CC0 output to PA6
  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
  								  | (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  // Set Top value
  // Note each overflow event constitutes 1/2 the signal period
  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0)/(timerInit.prescale + 1);
  int topValue = timerFreq / (2*OUT_FREQ) - 1;
  TIMER_TopSet (TIMER0, topValue);

  /* Start the timer */
  // TIMER_Enable(TIMER0, true);
}

void stopTimer()
{
	TIMER_Enable(TIMER0, false);
	GPIO_PinOutClear(gpioPortD, 3);
}
void startTimer()
{
    TIMER_Enable(TIMER0, true);
}




/***************************************************************************//**
 * @brief
 *   Main function
 ******************************************************************************/


int gen(int index)
{
	// 0 1 1
	switch (index % 3)
	{
	case 0:
		return -1;
	case 1:
		return 1;
	case 2:
		return 1;
	}
	return 0;
}



void correlateL()
{
	//Correlation
	for(int i=0; i<LENC; i++)
	{
		int i1 = i;
		int tmp = 0.0;
		for (int j=0; j<BUFFER_SIZE; j++)
		{
			if(i1>=0 && i1<SOUND_LENGTH)
				tmp = tmp + (left[j]*gen(i1));

			i1 = i1-1;
			corrL[i] = tmp;
		}
	}
}

//void correlateR()
//{
//	//Correlation
//	for(int i=0; i<LENC; i++)
//	{
//		int i1 = i;
//		int tmp = 0.0;
//		for (int j=0; j<BUFFER_SIZE; j++)
//		{
//			if(i1>=0 && i1<SOUND_LENGTH)
//				tmp = tmp + (right[j]*gen(i1));
//
//			i1 = i1-1;
//			corrR[i] = tmp;
//		}
//	}
//}


int main(void)
{
	//Microphone code

	//volatile bool start_record = false;
	int i;
	int offset = 0;
	bool done = false;
	 // Chip errata
	CHIP_Init();

	//initialize speaker
	initCmu();//speaker
	initGpio();//speaker

	 // Initialize LDMA and PDM
	initLdma();
	initPdm();
	initTIMER();

	printf("Hello World\r\n");
	RETARGET_SerialFlush();
	 //test
	//int count = 0;

	 while (1)
	 {

		 int8_t c = RETARGET_ReadChar();
		if (c > 0)
		{
			printf("Character detected. Recording\r\n");
			startTimer();//speaker
			//initTimer();
			while (!done)
			{
				EMU_EnterEM1();
				// After LDMA transfer completes and wakes up device from EM1,

				// convert data from ping-pong buffers to left/right PCM data
				if(prevBufferPing) {
				      for(i = 0; i < PP_BUFFER_SIZE; i++) {
				        left[i + offset] = pingBuffer[i] & 0x0000FFFF;
				        //right[i + offset] = (pingBuffer[i] >> 16) & 0x0000FFFF;
				      }
				    }
				else {
				      for(i = 0; i < PP_BUFFER_SIZE; i++) {
				        left[offset + i] = pongBuffer[i] & 0x0000FFFF;
				        //right[offset + i] = (pongBuffer[i] >> 16) & 0x0000FFFF;
				      }
				}
				offset += PP_BUFFER_SIZE;

				if(offset >= SOUND_LENGTH)
				{
					stopTimer();
				}
				if (offset >= BUFFER_SIZE) {
					done = true;
				}
			}

			//stopTimer();

			printf("left\r\n");
			for (i = 0; i < BUFFER_SIZE; i++)
			{
				printf("%d ", left[i]);
			}
			printf("\r\n");
			RETARGET_SerialFlush();

//			printf("right\r\n");
//			for (i = 0; i < BUFFER_SIZE; i++) {
//				printf("%d ", right[i]);
//			}

			printf("\r\n");
			RETARGET_SerialFlush();
			//RETARGET_SerialFlush();

			correlateL();
			//correlateR();

			printf("lcorrelation\r\n");
			for (i = 0; i < LENC; i++)
			{
				printf("%d ", corrL[i]);
			}
			printf("\r\n");
			RETARGET_SerialFlush();

//			printf("rcorrelation\r\n");
//			for (i = 0; i < LENC; i++)
//			{
//				printf("%d ", corrR[i]);
//			}
//			printf("\r\n");
//			RETARGET_SerialFlush();

			done = false;
			offset = 0;
			printf("done\r\n");
		}
	}
}

