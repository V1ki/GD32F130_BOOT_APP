#include "fmc.h"
#include "usart.h"


volatile FMC_State FMCStatus = FMC_READY ;
__IO uint32_t NbrOfPage = 0x00;
uint32_t EraseCounter = 0x00,address = 0x0;
uint32_t *ptrd;
uint32_t write_now_app_len, write_app_len, write_app_checksum,app_version ;

void clearAppArea() {
    /* Unlock the Flash Bank1 Program Erase controller */
    FMC_Unlock();

    /* Define the number of page to be erased */
    NbrOfPage =  24 ;
    /* Clear All pending flags */
    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );

    /* Erase the FLASH pages */
    for(EraseCounter = 0; EraseCounter < NbrOfPage; EraseCounter++)
    {
        FMC_ErasePage(AppAddress + (FMC_PAGE_SIZE * EraseCounter));
        FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );
    }

    FMC_Lock();
}


void readParamFromData() {
    // ʵ��д�볤��
    ptrd = (uint32_t*)(ADDRESS_WRITE_NOW_APP_LEN);
    write_now_app_len = *ptrd ;
    //�ڴ�д�볤��
    ptrd = (uint32_t*)(ADDRESS_WRITE_EXCEPTED_APP_LEN);
    write_app_len = *ptrd ;
    // �ڴ� checksum
    ptrd = (uint32_t*)(ADDRESS_WRITE_APP_CHECK_SUM);
    write_app_checksum = *ptrd ;
//
    ptrd = (uint32_t *)(ADDRESS_APP_VERSION) ;
    app_version = *ptrd ;

}

void writeParamToData() {

    /* Unlock the Flash Bank1 Program Erase controller */
    FMC_Unlock();

    /* Clear All pending flags */
    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );

    FMC_ErasePage(AppDataArea);
    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );

    FMCStatus = FMC_ProgramWord(ADDRESS_WRITE_NOW_APP_LEN,write_now_app_len);

    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );


    FMCStatus = FMC_ProgramWord(ADDRESS_WRITE_EXCEPTED_APP_LEN,write_app_len);

    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );

    FMCStatus = FMC_ProgramWord(ADDRESS_WRITE_APP_CHECK_SUM,write_app_checksum);

    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );
    FMCStatus = FMC_ProgramWord(ADDRESS_APP_VERSION,app_version);

    FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );

    FMC_Lock();


}

/*
		���App ��״̬,ʵ���ǱȽ�App��checksum
		1.�Ƚ�д�볤�Ⱥ��ڴ�����
		2.�Ƚ�ʵ��checksum �� �ڴ�checksum
		@return 0 : ��ʾ App ��״̬����
		@return 1 : ��ʾ д�볤�� �� �ڴ����� ��һ��
		@return 2 : ��ʾ ʵ��checksum �� �ڴ�checksum ��һ��
		@return 3 : ��ʾ App�������λ��ַ����ȷ.
*/
uint8_t checkAppStatus(void) {

    uint32_t sum_c = 0x0,address = AppAddress;
	
	
    // ���ȴ� flash �ж�ȡ ����
    readParamFromData() ;
	
    // ��� ʵ��д�볤�� �� �ڴ�д�볤�� ��ͬ�Ļ�.���ü�����
    if(write_app_len != write_now_app_len || write_app_len == 0xFFFFFFFF || write_now_app_len == 0xFFFFFFFF) {

        return 1 ;
    }
		
    // ѭ��ȡֵ,Ȼ���ۼ�
    for (address = AppAddress; address < (AppAddress + write_app_len) ; address+=4 ) {
        ptrd = (uint32_t*)(address);
        sum_c += *ptrd ;
    }


    // ��� checksum ���ԵĻ�...�ǾͲ�����
    if(sum_c != write_app_checksum || write_app_checksum == 0xFFFFFFFF) {

        return 2;
    }
		

		// ��� App ����λ��ַû������Ļ�
		if (((*(__IO uint32_t*)AppAddress) & 0x20000000 ) != 0x20000000) {

        return 3 ;
    }
		
    return 0 ;
}



/*
typedef void (*pFonction)(void);
pFonction startApp;
uint32_t JumpAddress;

void startApplication(void)
{
    if (((*(__IO uint32_t*)AppAddress) & 0x20000000 ) == 0x20000000) {
        JumpAddress = *(__IO uint32_t*) (AppAddress + 4);
        startApp = (pFonction) JumpAddress;
        __set_MSP(*(__IO uint32_t*) AppAddress);
        __enable_irq();     //PRIMASK=1,��ȫ���ж�
			
			
				//USART1_Sendstr(" -- jump success \r\n");
			
        startApp();
    }
		else {
			USART1_Sendstr(" -- jump failed \r\n");
			
		}
}
*/
typedef void (*pFonction)(void);
pFonction JumpToApp;
uint32_t JumpAddress;

void JumpToBoot(void)
{
    if (((*(__IO uint32_t*)BootAddress) & 0x20000000 ) == 0x20000000) {
        JumpAddress = *(__IO uint32_t*) (BootAddress + 4);
        JumpToApp = (pFonction) JumpAddress;
        __set_MSP(*(__IO uint32_t*) BootAddress);
        __enable_irq();     //PRIMASK=1,��ȫ���ж�

        //USART1_Sendstr("Jump to Boot application\n\r");
        JumpToApp();
    }
}



/*
		��Flash ��д������
		@param offset ����� AppAddress ��ƫ��,��ָ��������д���λ��
		@param datas  ��Ҫд�뵽 Flash �е����� 
		@param len    ��Ҫд�뵽Flash �е����ݵĳ���
		@return 	0: 	д��ɹ�
		@return 	1: 	д��ʧ��
		
 */
uint8_t writeData(uint32_t offset,uint32_t datas[],uint8_t len) {
    uint32_t temp0 = 0x0,temp1 = 0x0 ;
    uint8_t i = 0x0,writecount = 0 ;

    // д������
    FMC_Unlock();
    //Address = 0x00 ;
    address = AppAddress + offset  ;
    for (i= 0 ; i < len ; i++ ) {
        temp0 = datas[i] ;

        //FMC_ProgramWord(AppAddress + writeIndex + Address * ((uint16_t)4),tmp);
        FMCStatus = FMC_ProgramWord(address,temp0);
        address = address + 4 ;
        FMC_ClearBitState(FMC_FLAG_EOP | FMC_FLAG_WERR | FMC_FLAG_PERR );

    }

    FMC_Lock();

    // У������ �Ƿ�д��ɹ�
    ptrd = (uint32_t*)(AppAddress + offset );
    for (i= 0 ; i < len ; i++ ) {

        temp0 = datas[i] ;
        temp1 = *ptrd ;
        if(temp0 != temp1)
        {
            writecount = 1 ;
            break;
        }
        ptrd++;
    }
		
		//����ǰд�Ľ��ȸ���
		readParamFromData();
		
		write_now_app_len = ( offset + len * 4 );
		
		writeParamToData();
		
		
    return writecount ;


}





