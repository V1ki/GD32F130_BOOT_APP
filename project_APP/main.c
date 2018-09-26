/**
  ******************************************************************************
  * @file    ADC/AnalogWatchdog/main.c
  * @author  MCU SD
  * @version V1.0
  * @date    6-Sep-2014
  * @brief   Main program body.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
//#include "simplequeue.h"
//#include "systick.h"
#include "usart.h"
#include "fmc.h"

/* Private variables ---------------------------------------------------------*/
ADC_InitPara  ADC_InitStructure;
uint8_t b_onoffflag=0,flag_notACCON=0; 
int presspowerkeytime=0,b_isenscontflag=0,vbatuse=0,isensuse=0,report_volt=0;
bool maskshake=false;
uint16_t vbattmp=0,isenstmp =0;
uint32_t tmp;


#define FMC_PAGE_SIZE           ((uint16_t)0x400)


#define GPIO_WKUP	GPIO_PIN_0
#define PORT_WKUP	GPIOA

#define GPIO_ACCDET GPIO_PIN_3
#define PORT_ACCDET GPIOA

#define GPIO_PWKEY GPIO_PIN_7
#define PORT_PWKEY GPIOA

#define GPIO_OPVEN GPIO_PIN_5
#define PORT_OPVEN GPIOA

#define GPIO_PW5VEN GPIO_PIN_8
#define PORT_PW5VEN GPIOA

#define GPIO_PW4VEN GPIO_PIN_6
#define PORT_PW4VEN GPIOA

#define GPIO_ISENSADC GPIO_PIN_2
#define PORT_ISENSADC GPIOA
#define CHANN_ISENSADC ADC_CHANNEL_2

#define GPIO_VBATADC GPIO_PIN_1
#define PORT_VBATADC GPIOA
#define CHANN_VBATADC ADC_CHANNEL_1


uint32_t before_start_time = 30;// 重启后等待时间
uint32_t between_time = 30; // 喂狗后等待时间

uint32_t current_between_time = 0 ; // 当前喂狗等待时间
uint32_t current_start_time = 0 ; // 重启后等待时间

uint32_t boot_version = 0x04030201 ;
#define ADDRESS_APP_VERSION 0x08009C0C  // App 程序的版本

uint32_t * mptrd;

uint8_t data[DATA_LEN];	//data
/* Private function prototypes -----------------------------------------------*/
void clearData(void) {
    uint8_t i = 0 ;
    for(i = 0 ; i<32 ; i++) {
        data[i] = 0x00 ;
    }
}


void delay1us(int us)
{
    int i,j;
    for(i=0; i<us; i++)
        for(j=0; j<1; j++)
        {}
}

void delay_us(uint16_t delay)
{
    TIMER_Enable(TIMER2, ENABLE);
    TIMER_SetCounter(TIMER2, delay);
    while(delay > 1)
    {
        delay = TIMER_GetCounter(TIMER2);
    }
    TIMER_Enable(TIMER2, DISABLE);
}


void delay_ms(uint16_t delay)
{
    while(delay--) {
        delay_us(1000);
    }
}




void RCC_Configuration(void)
{

    /* ADCCLK = PCLK2/6 */
    RCC_ADCCLKConfig(RCC_ADCCLK_APB2_DIV6);
    /* Enable GPIOC clock */
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB, ENABLE);
    /* Enable ADC1 and GPIO_LED clock */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_ADC1, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_CFG | RCC_APB2PERIPH_USART1, ENABLE);

    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_PWR, ENABLE);
    RCC_USARTCLKConfig(RCC_USART1CLK_HSI);


    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB,ENABLE );
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_USART2,ENABLE);
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_USART1,ENABLE);
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_SPI1,ENABLE);

}

void GPIO_BootConfiguration(void)
{
    GPIO_InitPara GPIO_InitStructure;

    /* Configure PC1 (ADC Channel11) as analog input -------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_VBATADC | GPIO_ISENSADC;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_WKUP ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;
    GPIO_Init(PORT_WKUP, &GPIO_InitStructure);

	/*
	  GPIO_InitStructure.GPIO_Pin = GPIO_WKUP ;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_2MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
    GPIO_Init(PORT_WKUP, &GPIO_InitStructure);
	
	*/
	
    GPIO_InitStructure.GPIO_Pin = GPIO_ACCDET ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
    GPIO_Init(PORT_ACCDET, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_PWKEY ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
    GPIO_Init(PORT_PWKEY, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_PW5VEN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_2MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
    GPIO_Init(PORT_PW5VEN, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_PW4VEN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_2MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
    GPIO_Init(PORT_PW4VEN, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_OPVEN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_2MHZ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
    GPIO_Init(PORT_OPVEN, &GPIO_InitStructure);

    //USART1
    GPIO_InitStructure.GPIO_Pin     = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode    = GPIO_MODE_IN;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PUPD_PULLDOWN;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    //GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE13,GPIO_AF_0);
    //GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE14,GPIO_AF_0);
    /*
        GPIO_InitStructure.GPIO_Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    */
}



void NVIC_Configuration(void)
{
    NVIC_InitPara NVIC_InitStructure;
    EXTI_InitPara EXTI_InitStructure;

    /* Configure and enable ADC interrupt */
    NVIC_InitStructure.NVIC_IRQ = ADC1_CMP_IRQn;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PRIGroup_Enable(NVIC_PRIGROUP_1);
    NVIC_InitStructure.NVIC_IRQ = EXTI2_3_IRQn;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Connect EXTI3 Line to PA3 pin */
    SYSCFG_EXTILine_Config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN3);
    EXTI_InitStructure.EXTI_LINE = EXTI_LINE3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LINEEnable = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Connect EXTI7 Line to PA7 pin */
    SYSCFG_EXTILine_Config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN7);
    EXTI_InitStructure.EXTI_LINE = EXTI_LINE7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LINEEnable = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_PRIGroup_Enable(NVIC_PRIGROUP_1);
    NVIC_InitStructure.NVIC_IRQ = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge*/
    EXTI_ClearIntBitState(EXTI_LINE17);
    EXTI_InitStructure.EXTI_LINE = EXTI_LINE17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LINEEnable = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* NVIC configuration */
    NVIC_InitStructure.NVIC_IRQ = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART1 */
    /*EXTI_ClearIntBitState(EXTI_LINE25);
    EXTI_InitStructure.EXTI_LINE = EXTI_LINE25;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LINEEnable = ENABLE;
    EXTI_Init(&EXTI_InitStructure);*/
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQ = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStructure.NVIC_IRQSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void ADC_Configuration(void)
{
    /* Config ADC1  ----------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode_Scan = DISABLE;
    ADC_InitStructure.ADC_Mode_Continuous = DISABLE;
    ADC_InitStructure.ADC_Trig_External = ADC_EXTERNAL_TRIGGER_MODE_NONE;
    ADC_InitStructure.ADC_Data_Align = ADC_DATAALIGN_RIGHT;
    ADC_InitStructure.ADC_Channel_Number = 1;
    ADC_Init(&ADC_InitStructure);

    /* Enable ADC1 */
    ADC_Enable(ENABLE);

    /* ADC1 calibration */
    ADC_Calibration();
    delay_ms(5);
}

int RTC_ALARM_PreConfig(bool RST)
{
    int i,j;
    /* Allow access to RTC */
    PWR_BackupAccess_Enable(ENABLE);

    if(RST)
    {
        RCC_BackupReset_Enable(ENABLE);
        RCC_BackupReset_Enable(DISABLE);
    }

    /* Enable the LSE OSC */
    RCC_LSEConfig(ENABLE);
    /* Wait till LSE is ready */
    for(j=0; j<200; j++)
    {
        for(i=0; i<20000; i++)
        {
            if(!(RCC_GetBitState(RCC_FLAG_LSESTB) == RESET))
                break;
        }
        if(i<20000)
            break;
    }
    if(j>=200)
    {
        return 0x01;
    }
    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable the RTC Clock */
    RCC_RTCCLK_Enable(ENABLE);

    /* Wait for RTC APB registers synchronisation */
    //RTC_WaitRSF_ToSetAgain();

    return 0;
}


void Delayus_Configuration(void)
{
    TIMER_BaseInitPara  TIMER_TimeBaseStructure;

    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_TIMER2,ENABLE);

    TIMER_TimeBaseStructure.TIMER_Period = 1;
    TIMER_TimeBaseStructure.TIMER_Prescaler = ((SystemCoreClock / 1000000) - 1);
    TIMER_TimeBaseStructure.TIMER_ClockDivision = TIMER_CDIV_DIV1;
    TIMER_TimeBaseStructure.TIMER_CounterMode = TIMER_COUNTER_DOWN;
    TIMER_BaseInit(TIMER2, &TIMER_TimeBaseStructure);
}



/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    app_version = 0x01000104 ;

    /*Config System clocks --------------------------------------------------*/
    RCC_Configuration();

    /*Config GPIO ports -----------------------------------------------------*/
    GPIO_BootConfiguration();

    /*NVIC configuration ----------------------------------------------------*/
    NVIC_Configuration();
    //NVIC_VectTableSet(NVIC_VECTTAB_FLASH,0x08006400);
    NVIC_VectTableSet(NVIC_VECTTAB_FLASH,0x6400);



    GPIO_SetBits(PORT_PW5VEN,GPIO_PW5VEN);
    GPIO_SetBits(PORT_PW4VEN,GPIO_PW4VEN);

    /* Enable write access to IWDG_PSR and IWDG_RLDR registers */
    IWDG_Write_Enable(IWDG_WRITEACCESS_ENABLE);

    /* IWDG counter clock: 40KHz(LSI) / 256 = 0.156 KHz */
    IWDG_SetPrescaler(IWDG_PRESCALER_256);

    /* Set counter reload value to 3120 */
    IWDG_SetReloadValue(3120);  //156=1S //3120

    /* Reload IWDG counter */
    IWDG_ReloadCounter();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();

    ADC_Configuration();

    USART_Configure();

    USART_INT_Set(USART1,USART_INT_RBNE,ENABLE);

    //USART1_Sendstr("now App1");

    Delayus_Configuration();

    //delay_ms(500);
    clearData();
    data[0] = 0 ;
    usart_send_ack(RESP_START_APP,data,1);
    /*
    mptrd = (uint32_t *)(ADDRESS_APP_VERSION) ;
    tmp = *mptrd ;

    if( tmp != app_version) {

    	readParamFromData();
    	app_version = 0x01000104 ;
    	writeParamToData();

    }
    */


    while (1)
    {
        GPIO_SetBits(PORT_OPVEN,GPIO_OPVEN);
        /* Start ADC1 Software Conversion */
        ADC_RegularChannel_Config(CHANN_VBATADC, 1, ADC_SAMPLETIME_13POINT5);
        ADC_SoftwareStartConv_Enable(ENABLE);
        delay_ms(2);

        vbattmp=ADC_GetConversionValue();
        vbatuse=(vbatuse+vbattmp)/2;


        ADC_RegularChannel_Config(CHANN_ISENSADC, 1, ADC_SAMPLETIME_13POINT5);
        ADC_SoftwareStartConv_Enable(ENABLE);
        delay_ms(2);

        isenstmp=ADC_GetConversionValue();
        isensuse=(isensuse+isenstmp)/2;

        GPIO_ResetBits(PORT_OPVEN,GPIO_OPVEN);

        // 从GPIO 中读取 ACC的 状态
        flag_notACCON = GPIO_ReadInputBit(PORT_ACCDET,GPIO_ACCDET);

        IWDG_ReloadCounter();
        delay_ms(1000);

        if(!flag_notACCON) {
						// ACC  ON 

            if( current_start_time < before_start_time) {
                current_start_time++ ;
            }
            //before_start_time 20  c= 20 before_start_time = 10 c= 20
            else if(current_start_time >= before_start_time) {
                current_between_time ++ ;
            }


            if(current_between_time > between_time) {
                clearData();
                usart_send_ack(RESP_REBOOT,data,0);
                //USART1_Sendstr("--reboot--\r");

                // 重启机器
                GPIO_ResetBits(PORT_PW5VEN,GPIO_PW5VEN);
                GPIO_ResetBits(PORT_PW4VEN,GPIO_PW4VEN);
                delay_ms(5000);
							  usart_send_ack(RESP_BEFORE_PW_SET,data,0);
                GPIO_SetBits(PORT_PW5VEN,GPIO_PW5VEN);
                GPIO_SetBits(PORT_PW4VEN,GPIO_PW4VEN);

                delay_ms(2000);
								usart_send_ack(RESP_BEFORE_PWKEY_RESET,data,0);
                GPIO_ResetBits(PORT_PWKEY,GPIO_PWKEY);

                //GPIO_ResetBits(PORT_ACCDET,GPIO_ACCDET);
                delay_ms(3000);
							  usart_send_ack(RESP_BEFORE_PWKEY_SET,data,0);
                GPIO_SetBits(PORT_PWKEY,GPIO_PWKEY);
                //GPIO_SetBits(PORT_ACCDET,GPIO_ACCDET);

                current_start_time = 0 ;
                current_between_time = 0 ;

            }

        }
				else {
					
            clearData();

            data[0] = current_between_time >> 24 ;
            data[1] = current_between_time >> 16 ;
            data[2] = current_between_time >> 8 ;
            data[3] = current_between_time;

            // app 程序版本
            data[4] = current_start_time >> 24 ;
            data[5] = current_start_time >> 16 ;
            data[6] = current_start_time >> 8 ;
            data[7] = current_start_time ;



            usart_send_ack(RESP_BEFORE_PW_RESET,data,8);
				}

        //USART1_Sendstr("--go --on---\r");


        if(u_rx_flag == 0x01) {

            if(u_rx.cmd == CMD_CHECK_VERSION) {
                // 检查版本
                // 首先清空数据
                clearData();
                // 从Flash 中读取设置的参数
                mptrd = (uint32_t *)(ADDRESS_APP_VERSION) ;
                app_version = *mptrd ;

                // boot 程序版本
                data[0] = boot_version >> 24 ;

                data[1] = boot_version >> 16 ;

                data[2] = boot_version >> 8 ;

                data[3] = boot_version;

                // app 程序版本
                data[4] = app_version >> 24 ;

                data[5] = app_version >> 16 ;

                data[6] = app_version >> 8 ;

                data[7] = app_version ;

                delay_ms(500);
                usart_send_ack(RESP_CHECK_VERSION,data,8);
            } else if(u_rx.cmd == CMD_FETCH_DOG) {
                // 检查版本
                // 首先清空数据
                clearData();
                tmp = (u_rx.data[3] << 24) + (u_rx.data[2] << 16) + (u_rx.data[1] << 8) + u_rx.data[0] ;
                if (tmp == 0) {
                    before_start_time = 30 ;
                }
                else {
                    before_start_time = tmp ;
                }
                tmp = (u_rx.data[7] << 24) + (u_rx.data[6] << 16) + (u_rx.data[5] << 8) + u_rx.data[4] ;

                if (tmp == 0) {
                    between_time = 30 ;
                }
                else {
                    between_time = tmp ;
                }
                //before_start_time = (u_rx.data[3] << 24) + (u_rx.data[2] << 16) + (u_rx.data[1] << 8) + u_rx.data[0] ;
                //between_time = (u_rx.data[7] << 24) + (u_rx.data[6] << 16) + (u_rx.data[5] << 8) + u_rx.data[4] ;

                current_between_time = 0 ;
                // boot 程序版本




                data[0] = between_time >> 24 ;
                data[1] = between_time >> 16 ;
                data[2] = between_time >> 8 ;
                data[3] = between_time;

                // app 程序版本
                data[4] = before_start_time >> 24 ;
                data[5] = before_start_time >> 16 ;
                data[6] = before_start_time >> 8 ;
                data[7] = before_start_time ;


                delay_ms(500);
                usart_send_ack(RESP_FETCH_DOG,data,8);
            } else if(u_rx.cmd == CMD_GOTO_BOOT ) {
                //clearData();

                //data[0] = 0 ;
                //usart_send_ack(RESP_GOTO_BOOT,data,1);
                delay_ms(500);
                JumpToBoot();
            }
            else if(u_rx.cmd == CMD_START_APP ) {
                delay_ms(500);
                data[0] = 0 ;
                usart_send_ack(RESP_START_APP,data,1);
            }


            u_rx_flag = 0 ;


        }
        else {
            clearData();

            data[0] = current_between_time >> 24 ;
            data[1] = current_between_time >> 16 ;
            data[2] = current_between_time >> 8 ;
            data[3] = current_between_time;

            // app 程序版本
            data[4] = current_start_time >> 24 ;
            data[5] = current_start_time >> 16 ;
            data[6] = current_start_time >> 8 ;
            data[7] = current_start_time ;



            usart_send_ack(STATUS_APP,data,8);
        }



    }
}
