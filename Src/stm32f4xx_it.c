 /*
 * stm32f4xx_it.c
 *
 *  Created on: Sep 4, 2025
 *      Author: benbr
 */

#include "stm32f407xx.h"

extern void HT_Complete_callback(void);
extern void FT_Complete_callback(void);
extern void TE_error_callback(void);
extern void DME_error_callback(void);
extern void FE_error_callback(void);

#define is_it_HT() 	DMA1->HISR & ( 1 << 20)
#define is_it_FT() 	DMA1->HISR & ( 1 << 21)
#define is_it_TE() 	DMA1->HISR & ( 1 << 19)
#define is_it_FE() 	DMA1->HISR & ( 1 << 16)
#define is_it_DME() DMA1->HISR & ( 1 << 18)

void clear_EXTI_Pending_bit(void)
{
	EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;

	if (pEXTI->PR & (1 << 0))
	{
		//clear pending bit
		pEXTI->PR |= (1 << 0);
	}
}


/**********************************IRQ Handlers*****************************************/
void EXTI0_IRQHandler(void)
{
	USART_TypeDef *pUART2;
	pUART2 = USART2;

	//here we will send the uart2_tx DMA request to DMA1 controller
	pUART2->CR3 |= (1 << 7);

	clear_EXTI_Pending_bit();

}


// IRQ handler for DMA1 stream 6 global interrupt
void DMA1_Stream6_IRQHandler (void)
{
	//Half-transfer
	if( is_it_HT() )
	{
		DMA1->HIFCR |= ( 1 << 20);
		HT_Complete_callback();
	}else if(is_it_FT() )
	{
		DMA1->HIFCR |= ( 1 << 21);
		FT_Complete_callback();
	}else if ( is_it_TE() )
	{
		DMA1->HIFCR |= ( 1 << 19);
		TE_error_callback();

	}else if (is_it_FE() )
	{
		DMA1->HIFCR |= ( 1 << 16);
		FE_error_callback();
	}else if( is_it_DME() )
	{
		DMA1->HIFCR |= ( 1 << 18);
		DME_error_callback();
	}else{
		  ;
	}
}





