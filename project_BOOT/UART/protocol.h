
#if !defined( _PROTOCOL_H )

#define _PROTOCOL_H

#include <stdint.h>
#include "gd32f1x0.h"

#define RECEIVE_BUFF_SIZE (1024)   //���� �������ݰ�����


#define PROTOCOL_DATA_SIZE (40)


typedef  struct
{
    uint16_t header ; // Э��ͷ
    uint16_t cmd ;	  // ����
    uint16_t dLen ;   // Э�������

    uint8_t  buf[PROTOCOL_DATA_SIZE]; // Э���а�����

    uint16_t chk ; //Э��� У��
    uint16_t footer ; // Э��β

} protocol;

typedef struct {
    uint8_t volatile start ;

    uint8_t  buf[RECEIVE_BUFF_SIZE];

} data_receive ;



#endif