#include "stdio.h"
#include "../inc/ZH_DHA_intf.h"

//#pragma pack(2)

typedef struct
{
	char bb3; //1个字节对齐 1
	char dd3; //1个字节对齐 1
	int aa3; //2个字节对齐 11 11
	short cc23;//2个字节对齐 11

} testlength3;


//#pragma pack()
int main(void)
{
	
	int length3 = sizeof(testlength3); //2个字节对齐，占用字节11 11 11 11,length = 8

	int m = 0;
}