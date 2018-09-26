#ifndef __FMC_H
#define __FMC_H	


#include "gd32f1x0.h"


#define FMC_PAGE_SIZE           ((uint16_t)0x400)

#define AppAddress 0x08006400      // APP Address
#define AppDataArea   0x08009C00			// App Data Area

#define BootAddress   0x08000000      // BootAddress

#define ADDRESS_WRITE_NOW_APP_LEN 0x08009C00 // APP д����� ���ݵı����ַ
#define ADDRESS_WRITE_EXCEPTED_APP_LEN  0x08009C04 // APP �����ܳ��� �����ݱ����ַ
#define ADDRESS_WRITE_APP_CHECK_SUM 0x08009C08 // APP �����checksum �����ݱ����ַ
#define ADDRESS_APP_VERSION 0x08009C0C  // App ����İ汾

extern uint32_t write_now_app_len, write_app_len, write_app_checksum,app_version ;
/* ���App �����Flash �е����� */
void clearAppArea(void);

void readParamFromData(void) ;

void writeParamToData(void) ;

/*
   ����App
 */
//void startApplication(void) ;

void JumpToBoot(void) ;

/*
		���App ��״̬,ʵ���ǱȽ�App��checksum 
		1.�Ƚ�д�볤�Ⱥ��ڴ�����
		2.�Ƚ�ʵ��checksum �� �ڴ�checksum
		@return 0 : ��ʾ App ��״̬���� 
		@return 1 : ��ʾ д�볤�� �� �ڴ����� ��һ��
		@return 2 : ��ʾ ʵ��checksum �� �ڴ�checksum ��һ��
		@return 3 : ��ʾ App�������λ��ַ����ȷ.
*/
uint8_t checkAppStatus(void) ;

/*
		��Flash ��д������
		@param offset ����� AppAddress ��ƫ��,��ָ��������д���λ��
		@param datas  ��Ҫд�뵽 Flash �е����� 
		@param len    ��Ҫд�뵽Flash �е����ݵĳ���
		@return 	0: 	д��ɹ�
		@return 	1: 	д��ʧ��
		
 */
uint8_t writeData(uint32_t offset,uint32_t datas[],uint8_t len) ;

#endif
