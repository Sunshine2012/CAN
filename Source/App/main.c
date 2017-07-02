#include "stm32f10x.h"
#include "GLCD.h"

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
vu32 ret; /* for return of the interrupt handling */
volatile TestStatus TestRx;
ErrorStatus HSEStartUpStatus;

TestStatus CAN_Polling(void);
TestStatus CAN_Interrupt(void);
void MyCANTransmit (void);
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);


void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  	/* Configure PD.02, PD.03, PD.04 and PD.07 as Output push-pull */
  	// For STM3210B-LK1 use PD.02 -PC.07
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
    
#if 0
  	/* Configure CAN pin: RX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  	/* Configure CAN pin: TX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE );	   //重影射CAN IO脚到 PD0，PD1
#endif
  	/* Configure CAN pin: RX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	/* Configure CAN pin: TX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE );	   //重影射CAN IO脚到 PD0，PD1
}

//系统中断管理
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

  	/* Configure the NVIC Preemption Priority Bits */  
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	/* enabling interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

//配置系统时钟,使能各外设时钟
void RCC_Configuration(void)
{
	SystemInit();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
                           |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
                           |RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
						   |RCC_APB2Periph_ADC1  | RCC_APB2Periph_AFIO 
                           |RCC_APB2Periph_SPI1, ENABLE );
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL ,ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 
                           |RCC_APB1Periph_USART3|RCC_APB1Periph_TIM2	                           
                           , ENABLE );
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* CAN Periph clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

void InitDis(void) 
{
   /* LCD Module init */
   GLCD_init();
   GLCD_clear(White);
   GLCD_setTextColor(Blue);
   GLCD_displayStringLn(Line1, "     GoldBull");
   GLCD_displayStringLn(Line2, "   CAN example");
   GLCD_setTextColor(Red);
}

//配置所有外设
void Init_All_Periph(void)
{
	RCC_Configuration();	
//	InitDis();
//	GLCD_Test();
	GPIO_Configuration();
	NVIC_Configuration();
}

int main(void)
{  
    int i  = 0;
	Init_All_Periph();

	/* CAN transmit at 100Kb/s and receive by polling in loopback mode*/
  	//TestRx = CAN_Polling();
    //GPIO_SetBits (GPIOD, GPIO_Pin_0);
    
    GPIO_SetBits (GPIOE, GPIO_Pin_0);
    GPIO_SetBits (GPIOE, GPIO_Pin_1);
    GPIO_SetBits (GPIOE, GPIO_Pin_2);
    GPIO_SetBits (GPIOE, GPIO_Pin_3);

    GPIO_ResetBits (GPIOD, GPIO_Pin_2);

//  	if (TestRx == FAILED)
//  	{
//    	/* Turn on led connected to PD.04 pin (LD3) */
//    	GPIO_SetBits(GPIOD, GPIO_Pin_4);
//  	}
//  	else
//  	{
//    	/* Turn on led connected to PD.02 pin (LD1) */
//   	 GPIO_SetBits(GPIOD, GPIO_Pin_2);
//  	}

  	/* CAN transmit at 500Kb/s and receive by interrupt in loopback mode*/
  	TestRx = CAN_Interrupt();

//  	if (TestRx == FAILED)
//  	{
//   	 	/* Turn on led connected to PD.07 pin (LD4) */
//    	GPIO_SetBits(GPIOD, GPIO_Pin_7); 
//  	}
//  	else
//  	{
//    	/* Turn on led connected to PD.03 pin (LD2) */
//    	GPIO_SetBits(GPIOD, GPIO_Pin_3);
// 	}


 	while(1)
  	{
        // CAN_Interrupt();
        // CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
        MyCANTransmit ();
        for (i = 100000; i > 0; i--)
        ;
  	}
}



void MyCANTransmit (void)
{  
    u32 i = 0;
    u8 TransmitMailbox;
    CanTxMsg TxMessage;
    CanRxMsg RxMessage;



    /* transmit */
    
    TxMessage.StdId = 0x00;
    TxMessage.ExtId = 0x7800;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_EXT;;
    TxMessage.DLC = 8;
    TxMessage.Data[0] = 0x55;
    TxMessage.Data[1] = 0xA0;
    TxMessage.Data[2] = 0x50;
    TxMessage.Data[3] = 0xA0;
    TxMessage.Data[4] = 0x50;
    TxMessage.Data[5] = 0xA0;
    TxMessage.Data[6] = 0x50;
    TxMessage.Data[7] = 0xA0;
    TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);
    i = 0;
    while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
    {
        i++;
    }

    i = 0;
    while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
    {
        i++;
    }
}


/*******************************************************************************
* Function Name  : CAN_Polling
* Description    : Configures the CAN and transmit and receive by polling
* Input          : None
* Output         : None
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
TestStatus CAN_Polling(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  /* CAN register init */
  CAN_DeInit(CAN1);
  // aiwesky 20140506
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_7tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;
  CAN_InitStructure.CAN_Prescaler=5;
  CAN_Init(CAN1,&CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  //
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  //CAN_FilterInitStructure.CAN_FilterActivation=DISABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

#if 0 //  aiwesky 20140504
  /* receive */
  RxMessage.StdId=0x00;
  RxMessage.IDE=CAN_ID_STD;
  RxMessage.DLC=0;
  RxMessage.Data[0]=0x00;
  RxMessage.Data[1]=0x00;
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

  if (RxMessage.StdId!=0x11) return FAILED;

  if (RxMessage.IDE!=CAN_ID_STD) return FAILED;

  if (RxMessage.DLC!=2) return FAILED;

  if ((RxMessage.Data[0]<<8|RxMessage.Data[1])!=0xCAFE) return FAILED;
#endif  
  //return !PASSED; /* Test Passed */

}

/*******************************************************************************
* Function Name  : CAN_Interrupt
* Description    : Configures the CAN and transmit and receive by interruption
* Input          : None
* Output         : None
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
TestStatus CAN_Interrupt(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CanTxMsg TxMessage;
  u32 i = 0;

  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler=5;
  CAN_Init(CAN1,&CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* CAN FIFO0 message pending interrupt enable */ 
  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);


#if 1
  /* transmit 1 message */
  TxMessage.StdId=0x00;
  TxMessage.ExtId=0xF00;
  TxMessage.IDE=CAN_ID_EXT;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.DLC=2;
  TxMessage.Data[0]=0xDE;
  TxMessage.Data[1]=0xCA;
  CAN_Transmit(CAN1,&TxMessage);

  /* initialize the value that will be returned */
  ret = 0xFF;
       
  /* receive message with interrupt handling */
  i=0;
  while((ret == 0xFF) && (i < 0xFFF))
  {
    i++;
  }
  
  if (i == 0xFFF)
  {
    ret=0;  
  }

  /* disable interrupt handling */
  // aiwesky 20140503
  CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
#endif // aiwesky 20140506
  return (TestStatus)ret;
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    
    MyCANTransmit ();
    
    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
    RxMessage.IDE=0;
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

    if((RxMessage.ExtId == 0x7800) && (RxMessage.IDE == CAN_ID_EXT)
     && (RxMessage.DLC ==2 ) && ((RxMessage.Data[1] | RxMessage.Data[0]<<8) == 0xDECA))
    {
        ret = 1; 
        GPIO_ResetBits (GPIOC, GPIO_Pin_1);
    }
    else
    {
        ret = 0; 
    }
}
