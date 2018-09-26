#ifndef __USART_H
#define __USART_H	

#include "gd32f1x0.h"



#define CMD_CHECK_VERSION (uint8_t)0xb0  /* ���汾 */
#define RESP_CHECK_VERSION (uint8_t)0xb1 /* ���汾����Ӧ��� */

#define CMD_CHECK_STATUS (uint8_t)0xb2 /* ������״̬ */
#define RESP_CHECK_STATUS (uint8_t)0xb3 /* ������״̬����Ӧ��� */

#define CMD_START_APP (uint8_t)0xb4 /* ��ת��App */
#define RESP_START_APP (uint8_t)0xb5 /* ��ת��App����Ӧ��� */

// BOOT ������Ӧ�ò����õ��������
#define CMD_GOTO_BOOT (uint8_t)0xb6 /* ��ת��BOOT */
#define RESP_GOTO_BOOT (uint8_t)0xb7 /* ��ת��BOOT ����Ӧ��� */

#define CMD_EARSE_APP (uint8_t)0xb8 /* ���� APP ��FLASH ���� */
#define RESP_EARSE_APP (uint8_t)0xb9 /* ����APP ��FLASH ����ķ��ؽ�� */

#define CMD_WRITE_DATA (uint8_t)0xba /* ����д���� */
#define RESP_WRITE_DATA (uint8_t)0xbb /* ����д���ݵ���Ӧ��� */

#define CMD_SET_PARAM (uint8_t)0xbc /* ���ò���( App �Ĵ�С,checksum ,App �汾 )*/
#define RESP_SET_PARAM (uint8_t)0xbd /* ���� ��������Ӧ��� */

#define CMD_FETCH_DOG (uint8_t)0xbe  /* ι��,ͬʱ����ι��ʱ�� */
#define RESP_FETCH_DOG (uint8_t)0xbf  /* ι����Ӧ */

#define STATUS_COUNTER_DOWN (uint8_t)0xa0  /* counter_down ״̬ */
#define STATUS_APP (uint8_t)0xa1  /* counter_down ״̬ */

#define RESP_REBOOT (uint8_t)0xa2  /* counter_down ״̬ */



#define RESP_BEFORE_PW_RESET (uint8_t)0xa3  /* counter_down ״̬ */

#define RESP_BEFORE_PW_SET (uint8_t)0xa4  /* counter_down ״̬ */


#define RESP_BEFORE_PWKEY_RESET (uint8_t)0xa5  /* counter_down ״̬ */

#define RESP_BEFORE_PWKEY_SET (uint8_t)0xa6  /* counter_down ״̬ */


#define DATA_LEN 32 
#define RX_DATA_LEN sizeof(USART_RX) 

typedef struct {
	
	// �ȶ�40���ֽ�
	// ��������
	uint16_t header ; // header
	uint8_t dev_type ; // device type
	uint8_t cmd ; // cmd
	uint8_t len ; //valid data len 

	uint8_t data[DATA_LEN];	//data
	
	uint8_t sum ; // checksum
	
	uint16_t footer ;// footer
	
} USART_RX ;




extern USART_RX u_rx;


extern	uint8_t u_rx_flag  ; // ����Ҫ��һ��flag
	
void USART_Configure(void);


//����1����һ���ֽ�
void USART1_Sendstr(char *p);

void usart_send(char *usartdat,int length) ;

/* 
	�򴮿ڷ��ͻ�Ӧ.
	@param cmd  ��ʾ USART_RX �е� cmd ,һ��Ϊ���������һ
	@param data ��ʾ USART_RX �е� data 
  @param len  ��ʾ USART_RX �е� len ,Ϊ��Ч���ݵĳ��� 
*/
void usart_send_ack(uint8_t cmd,uint8_t data[],uint8_t len);


#endif
