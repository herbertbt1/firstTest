#include "stdio.h"
#include "../inc/ZH_DHA_intf.h"

//#pragma pack(2)

typedef struct
{
	char bb3; //1���ֽڶ��� 1
	char dd3; //1���ֽڶ��� 1
	int aa3; //2���ֽڶ��� 11 11
	short cc23;//2���ֽڶ��� 11

} testlength3;


//#pragma pack()
int main(void)
{
	
	int length3 = sizeof(testlength3); //2���ֽڶ��룬ռ���ֽ�11 11 11 11,length = 8

	int m = 0;
}