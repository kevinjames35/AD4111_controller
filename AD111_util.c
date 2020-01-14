//=================AD4111 MCU Command table===========================
//  Command: [I] 			-> identify AD4111 module.
//						return list:	1.[0x1]:4 channel diff voltage mode
//								2.[0x2]:4 channel current mode
//								3.[0x3]:8 channel singled-end voltage mode
//  Command: [M]-[ch] 			-> set reading channel
//  Command: [C] 			-> continue read back data
//  Command: [T] 			-> stop read back data
//  Command: [S] 			-> read back data once
//  Command: [K]-[O]-[ch]-[value_6] 	-> set calibration offset value
//  Command: [K]-[G]-[ch]-[value_6] 	-> set calibration gain value
//  Command: [W]-[len]-[addr]-[value]	-> write data to specific address
//  Command: [R]-[len]-[addr]	 	-> read data from specific address
//  Command: [U]-[ch] 			-> save channel calibration value into flash
//  Command: [F]-[value_4]		-> set filter and sample per second for every channel
//========================================================================
//note: this table is just for record how to communicate mcu using minicom or putty.
//	only need to type the capital character in [], [ch] in the table represent channel number, which should be around 0~3 or 0~7.
//	[value_6] represent six number for three byte hex data, and [value_4] represent four number for two bye data.
//	[len] represent how many byte trying to read or write, [addr] represent the address value.
  

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>




#define BUFFER_SIZE	1024
int fcom = -1; 


u_int8_t *pWritePtr, *pReadPtr;
u_int8_t *pRxBufferStart, *pRxBufferEnd;
u_int16_t wRxCounter;
u_int32_t dwBaudrate =0 ;
struct termios old_tios,SerialPortSettings;

pthread_t InQueueID;
pthread_t CheckingThread;
pthread_mutex_t buf_mut;

u_int8_t RxBuffer[BUFFER_SIZE];

int ascii_to_hex(char ch) 
{ 
char ch_tmp; 
int hex_val = -1; 

	ch_tmp = tolower(ch); 

	if ((ch_tmp >= '0') && (ch_tmp <= '9')) { 
		hex_val = ch_tmp - '0'; 
	} 
	else if ((ch_tmp >= 'a') && (ch_tmp <= 'f')) { 
		hex_val = ch_tmp - 'a' + 10; 
	} 

	return hex_val; 
} 
/********************************************************/
int32_t str_to_hex(char *hex_str) 
{
//receive string sample
//: [0x7FFD8B]
int i, len; 
int32_t hex_tmp, hex_val; 
char *bptr;
	bptr = strstr(hex_str, "0x");
	if ( bptr != NULL ) bptr+=2;
	else 	bptr=hex_str;

	len = (int)strlen(bptr); 
	hex_val = 0; 
 	for (i=0; i<len-1;i++) { 
		hex_tmp = ascii_to_hex(bptr[i]); 
		if (hex_tmp == -1) 
		{ return -1; } 

		hex_val = (hex_val) * 16 + hex_tmp; 
	} 
	return hex_val; 
} 
/**********************************************/
float receive_data_analysis(char* buffer)
{
//: [0x800000]-> 0V
//: [0xFFFFFF]-> +10V
	float data;
	data = (( str_to_hex(buffer) - 0x800000 )*(0.000001192));
	//data_buffer = (data_temp - 0x800000)*(0.000001192);
	return data;
	
}
/***********************************************************/
int set_interface_attribs (int speed)
{

tcgetattr(fcom, &SerialPortSettings);
        cfsetospeed (&SerialPortSettings, speed);
        cfsetispeed (&SerialPortSettings, speed);
		// 8N1 Mode 
		SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity   
		SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
		SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size             
		SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8                                 
		
		SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
		
		
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p 
		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                            

		SerialPortSettings.c_oflag |= OPOST;//No Output Processing
		SerialPortSettings.c_oflag |= (ONOCR);//No Output CR
       SerialPortSettings.c_oflag &= ~(ONLCR | OCRNL | ONLRET);//No Output CR
//printf("c flag = %X\n",SerialPortSettings.c_cflag);
//printf("i flag = %X\n",SerialPortSettings.c_iflag);
//printf("o flag = %X\n",SerialPortSettings.c_oflag);
//printf("l flag = %X\n",SerialPortSettings.c_lflag);
        if (tcsetattr (fcom, TCSANOW, &SerialPortSettings) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

/**********************************************/

void* _IncomeInQueueThread(void* object)
{
unsigned char tmpBuffer[60];
int xi,n;
u_int8_t xch;
//printf("incoming\n");
	while(1)
	{

		n=read(fcom, tmpBuffer, 60);
//printf("receive n=%d, \n", n);
//fflush(stdout);
		if ( n > 0 && n < 60 ) {
//printf("receive n=%d, \n", n);
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
//printf("%c", xch);
					pthread_mutex_lock(&buf_mut);				
					*pWritePtr = xch;
					wRxCounter++;
					if ( pWritePtr == pRxBufferEnd ) pWritePtr = pRxBufferStart;
					else				    pWritePtr++;
					pthread_mutex_unlock(&buf_mut);
			}
//printf("\n");
		}
//		else{
//			usleep(5000);
//		}
	}
	return NULL;
}
/**********************************************/

void* _PrintIncomeInQueueThread(void* object)
{
u_int8_t tmpBuffer[60];
int xi,n;
u_int8_t xch;
int flag = 0;

//printf("print incoming\n");
	while(1)
	{
		n=read(fcom, tmpBuffer, 60);
		if ( n > 0 && n < 60 ) {
			for ( xi=0 ; xi<n ; xi++) {
				xch = tmpBuffer[xi];
			
				if ( xch == 0x3a)  {
					flag = 1;
					pReadPtr = pRxBufferStart;
				}
				if ( xch == 0x5D){
					flag = 0;
					*pReadPtr = xch;		
					printf("%8.6f V %s\n",receive_data_analysis(pRxBufferStart),pRxBufferStart);
				}
				if ( flag == 1){
					*pReadPtr = xch;
					pReadPtr++;
				}
			}
//printf("\n");
		}

	}
	return NULL;
}
/******************************************/

int _SendBufferLength(u_int8_t* buffer, int32_t length)
{
u_int8_t *bptr;
	//_ReadBuffer_clear();
	//if ( !fIgnOpened ) return 0;
	if ( length < 1 ) return 0;
	bptr = buffer;
//printf("send CMD=%s\n", bptr);
	write(fcom, bptr, length);
	tcdrain(fcom);
	return 1;
}
/**************/
int _ReadBufferLength(u_int8_t* buffer, int32_t length)
{
int32_t iResult = 0;
int32_t fEnding =0;
int waitCnt, rxCnt;
u_int8_t *bptr, xch;

	if ( !fcom ) return iResult;
	if ( length < 1 ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wRxCounter == 0 ) 
		{
			usleep(2000); //2ms
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut);
			xch = *pReadPtr;
			*bptr++ = xch;
			wRxCounter--;
			if ( pReadPtr == pRxBufferEnd) pReadPtr = pRxBufferStart;
			else				  pReadPtr++;
			pthread_mutex_unlock(&buf_mut);
			rxCnt++;
			if ( rxCnt == length )  {
				iResult =1;
				fEnding = 1;
			}
		}
	}

	return iResult;
}

/**************/
int _ReadBuffer(int8_t* buffer)
{
int iResult = 0;
int fEnding =0;
int waitCnt, rxCnt;
int8_t *bptr, xch;

	//if ( !fIgnOpened ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wRxCounter == 0 ) 
		{
			sleep(5);
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut);
			xch = *pReadPtr;
			if ( xch != 0x3e && xch != 0x0a  && xch != 0x00 ) {
				//maybe ignore all CTRL code 
//printf("%c",xch);
				*bptr++ = xch;
				rxCnt++;
			}
			wRxCounter--;
			if ( pReadPtr == pRxBufferEnd) pReadPtr = pRxBufferStart;
			else				  pReadPtr++;
			pthread_mutex_unlock(&buf_mut);
			
			if ( (xch == 0x0a) | (xch ==0x3e) )  {

				*bptr = 0x00;
				iResult =1;
				fEnding = 1;
			}
			
		}
	}

	return iResult;
}
/*****************************/
/**** Open COM Port      *****/
/*****************************/
int32_t _OpenPort(char* pComPath)
{
int iResult = 0;

	//fcom = open("/dev/ttyUSB0", O_RDWR | O_NDELAY);
fcom = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (fcom < 0) return 0;
	//else printf("fcom=%x",fcom);
	//Buffer pointer initial
	wRxCounter =0 ;	
	pWritePtr = &RxBuffer[0];
	pReadPtr = &RxBuffer[0];
	pRxBufferStart = &RxBuffer[0];
	pRxBufferEnd = &RxBuffer[BUFFER_SIZE-1];
	//hook receiver thread
	//fcom=1;
	pthread_mutex_init(&buf_mut, NULL);
	

	set_interface_attribs(B115200); 
//pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));


	iResult = 1;
	
	
	return iResult;
}
/****************************************************************/
void __printf_usage(char *argv0)
{
		printf("./AD4111_util -continue\n");
		printf("./AD4111_util -single\n");
		printf("./AD4111_util -identify\n");
		printf("./AD4111_util -select [channel]\n");
		printf("./AD4111_util -gain [channel] [value]\n");
		printf("./AD4111_util -offset [channel] [value]\n");
		printf("./AD4111_util -filter [mode] [speed]\n");
		printf("./AD4111_util -save [channel]\n");
}
/***************************************************************/
int main(int argc, char **argv) 
{
int iResult = 0;
char *com_path = "/dev/ttyUSB0";

char rdData[100];
int rcvCnt=0;
int count=0;
char* filter="0500";
	if(argc<2)
	{
		printf("./AD4111_util -continue\n");
		printf("./AD4111_util -single\n");
		printf("./AD4111_util -identify\n");
		printf("./AD4111_util -select [channel]\n");
		printf("./AD4111_util -gain [channel] [value]\n");
		printf("./AD4111_util -offset [channel] [value]\n");
		printf("./AD4111_util -filter [mode] [speed]\n");
		printf("./AD4111_util -save [channel]\n");

		return 0;
	}
	//iResult = _OpenPort(com_path);
	//if(iResult == 1)
	//{
		if(strcmp("-continue",argv[1])==0)
		{
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _PrintIncomeInQueueThread, (void*)(0));

				iResult = _SendBufferLength("C",1);
				if(iResult == 1)
				{

					usleep(10000000);
					iResult = _SendBufferLength("T",1);
					if(iResult == 1)
					{
						printf("Stop reading\n");
	
					}
				}
			}
		}
		else if(strcmp("-single",argv[1])==0)
		{
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
				iResult = _SendBufferLength("S",1);
				if(iResult == 1)
				{
					//printf("send success\n");
					usleep(10000);
					iResult = _ReadBuffer(rdData);
					if(iResult == 1)
					{
						//printf("read success\n");
						printf("%s\n",rdData);
						//usleep(50000);
					}
					iResult = _ReadBuffer(rdData);
					if(iResult == 1)
					{
						//printf("read success\n");
						printf("%s\n",rdData);
						//usleep(50000);
					}
					iResult = _ReadBuffer(rdData);
					if(iResult == 1)
					{
						//printf("read success\n");
						//printf("%s\n",rdData);
						printf("%8.6f V %s\n", receive_data_analysis(rdData),rdData);
						//usleep(50000);
					}
				}
			}
		}
		else if(strcmp("-identify",argv[1])==0)
		{			
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
			
				iResult = _SendBufferLength("I",1);
				if(iResult == 1)
				{
					//printf("send success\n");
					
					usleep(50000);
					iResult = _ReadBuffer(rdData);
					if(iResult == 1)
					{
						//printf("read success\n");
						printf("%s\n",rdData);
						//usleep(50000);
					}
					iResult = _ReadBuffer(rdData);
					if(iResult == 1)
					{
						//printf("read success\n");
						printf("%s\n",rdData);
						//usleep(50000);
					}
					iResult = _ReadBuffer(rdData);
					if(iResult == 1)
					{
						//printf("read success\n");
						printf("%s\n",rdData);
						//usleep(50000);
					}
	
				}
			}
		}			
		else if(strcmp("-select",argv[1])==0)
		{
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
			
				iResult = _SendBufferLength("M",1);
				if(iResult == 1)
				{
					//printf("send success\n");
					//usleep(100000);
					
					iResult = _SendBufferLength(argv[2],1);
					if(iResult == 1)
					{
						printf("select channel %s success\n",argv[2]);
						//usleep(1000000);
					}
				}
			}
		}
		else if(strcmp("-gain",argv[1])==0)
		{
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
			
				iResult = _SendBufferLength("K",1);
				if(iResult == 1)
				{
					iResult = _SendBufferLength("G",1);
					if(iResult == 1)
					{
						iResult = _SendBufferLength(argv[2],1);
						if(iResult == 1)
						{

						iResult = _SendBufferLength(argv[3],6);
						if(iResult == 1)
						{
							printf("set channel %s gain 0x%s success\n",argv[2],argv[3]);
							//usleep(1000000);
						}							
						
						}
					}
				}
			}
		}
		else if(strcmp("-offset",argv[1])==0)
		{
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
			
				iResult = _SendBufferLength("K",1);
				if(iResult == 1)
				{
					iResult = _SendBufferLength("O",1);
					if(iResult == 1)
					{
						iResult = _SendBufferLength(argv[2],1);
						if(iResult == 1)
						{

						iResult = _SendBufferLength(argv[3],6);
						if(iResult == 1)
						{
							printf("set channel %s offset 0x%s success\n",argv[2],argv[3]);
							//usleep(1000000);
						}							
						
						}
					}
				}
			}
		}
		else if(strcmp("-save",argv[1])==0)
		{
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
			
				iResult = _SendBufferLength("U",1);
				if(iResult == 1)
				{
					iResult = _SendBufferLength(argv[2],1);
					if(iResult == 1)
					{
						printf("save channel %s calibration into flash\n",argv[2]);
					}
					
				}
			}

		}
		else if(strcmp("-filter",argv[1])==0)
		{
			iResult = _OpenPort(com_path);
			if(iResult == 1)
			{
				pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
			
				iResult = _SendBufferLength("F",1);
				if(iResult == 1)
				{
					if(strcmp("0",argv[2])==0)
					{
						switch(atoi(argv[3])){
						case '0':filter="050F";
						break;
						case '1':filter="0510";
						break;						
						case '2':filter="0511";
						break;
						case '3':filter="0512";
						break;
						case '4':filter="0513";
						break;
						case '5':filter="0514";
						break;
						case '6':filter="0515";
						break;
						case '7':filter="0516";
						break;
						default:
						__printf_usage(argv[0]);
						return -1;
						}

					}
					else if(strcmp("1",argv[2])==0)
					{
						switch(atoi(argv[3])){
						case '0':filter="056F";
						break;
						case '1':filter="0570";
						break;						
						case '2':filter="0571";
						break;
						case '3':filter="0572";
						break;
						case '4':filter="0573";
						break;
						case '5':filter="0574";
						break;
						case '6':filter="0575";
						break;
						case '7':filter="0576";
						break;
						default:
						__printf_usage(argv[0]);
						return -1;
						}
					}
					else
					{						
						__printf_usage(argv[0]);
						return -1;
					}
					iResult = _SendBufferLength(filter,4);
					if(iResult == 1)
					{
						printf("set module filter success\n",argv[2]);
					}
					
				}
			}

		}
		else
		{printf("7\n");}

	//}
close(fcom);

return 0;

}
