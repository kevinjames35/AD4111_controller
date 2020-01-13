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
printf("incoming\n");
	while(1)
	{

		n=read(fcom, tmpBuffer, 60);
//printf("receive n=%d, \n", n);
//fflush(stdout);
		if ( n > 0 && n < 60 ) {
//printf("receive n=%d, \n", n);
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
printf("%c", xch);
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

u_int8_t rdData[10];
u_int32_t xi;
int32_t iret;

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
	pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));

	set_interface_attribs(B115200); 



	iResult = 1;
	
	
	return iResult;
}

/***************************************************************/
int main(int argc, char **argv) 
{
int iResult = 0;
char *com_path = "/dev/ttyUSB0";

char rdData[100];
int rcvCnt=0;

	if(argc<2)
	{
		printf("./AD4111_util -continue\n");
		printf("./AD4111_util -single\n");
		printf("./AD4111_util -identify\n");
		printf("./AD4111_util -single\n");
		printf("./AD4111_util -select [channel]\n");
		printf("./AD4111_util -gain\n");
		printf("./AD4111_util -offset\n");
		printf("./AD4111_util -save\n");

		return 0;
	}
	iResult = _OpenPort(com_path);
	if(iResult == 1)
	{
		if(strcmp("-continue",argv[1])==0)
		{printf("1\n");}
		else if(strcmp("-single",argv[1])==0)
		{
			iResult = _SendBufferLength("S",1);
			if(iResult == 1)
			{
				printf("send success\n");
				usleep(10000);
				iResult = _ReadBuffer(rdData);
				if(iResult == 1)
				{
					printf("read success\n");
					printf("%s\n",rdData);
					usleep(50000);
				}
				iResult = _ReadBuffer(rdData);
				if(iResult == 1)
				{
					printf("read success\n");
					printf("%s\n",rdData);
					usleep(50000);
				}
				iResult = _ReadBuffer(rdData);
				if(iResult == 1)
				{
					printf("read success\n");
					printf("%s\n",rdData);
					usleep(50000);
				}
			}
		}
		else if(strcmp("-identify",argv[1])==0)
		{
			
			iResult = _SendBufferLength("I",1);
			if(iResult == 1)
			{
				//printf("send success\n");
				
				usleep(50000);
				iResult = _ReadBuffer(rdData);
				if(iResult == 1)
				{
					printf("read success\n");
					printf("%s\n",rdData);
					usleep(50000);
				}
				iResult = _ReadBuffer(rdData);
				if(iResult == 1)
				{
					printf("read success\n");
					printf("%s\n",rdData);
					usleep(50000);
				}
				iResult = _ReadBuffer(rdData);
				if(iResult == 1)
				{
					printf("read success\n");
					printf("%s\n",rdData);
					usleep(50000);
				}

			}
		}			
		else if(strcmp("-select",argv[1])==0)
		{
			iResult = _SendBufferLength("M",1);
			if(iResult == 1)
			{
				printf("send success\n");
				usleep(100000);
				//*wrCmd = atoi(argv[2]);
				iResult = _SendBufferLength(argv[2],1);
				if(iResult == 1)
				{
					printf("send success\n");
					usleep(1000000);
				}
			}

		}
		else if(strcmp("-gain",argv[1])==0)
		{printf("4\n");}
		else if(strcmp("-offset",argv[1])==0)
		{printf("5\n");}
		else if(strcmp("-save",argv[1])==0)
		{printf("6\n");}
		else
		{printf("7\n");}

	}
close(fcom);

return 0;


}
