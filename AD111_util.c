#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#define BUFFER_SIZE	1024
int fcom = -1; 
char strLcmPath[30];

u_int8_t *pWritePtr, *pReadPtr;
u_int8_t *pRxBufferStart, *pRxBufferEnd;
u_int16_t wRxCounter;
u_int32_t dwBaudrate =0 ;
struct termios old_tios;

pthread_t InQueueID;
pthread_t CheckingThread;
pthread_mutex_t buf_mut;
pthread_mutex_t port_mut;
u_int8_t	RxBuffer[BUFFER_SIZE];

/*****************/
//void* _IncomeInQueueThread(void* object);
//int32_t _OpenLcmPort(char* pLsmPath);
//void _CloseLcmPort(void);
//int32_t _SendBufferLength(u_int8_t* buffer, int32_t length);
//int _ReadBufferLength(u_int8_t* buffer, int32_t length);
//void _search_LCM_port(void);


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls,
                                        	// enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}
/**********************************************/

void* _IncomeInQueueThread(void* object)
{
unsigned char tmpBuffer[32];
int xi,n;
u_int8_t xch;
//printf("incoming queue\n");
	while(1)
	{
printf("XXX n=, \n");
		n=read(fcom, tmpBuffer, 32);
printf("receive n=%d, \n", n);
fflush(stdout);
		if ( n > 0 && n < 32 ) {
//printf("receive n=%d, \n", n);
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
printf("Rev. 0x%02X \n", xch);
					pthread_mutex_lock(&buf_mut);	
//printf("-%s-\n",xch);				
					*pWritePtr = xch;
					wRxCounter++;
					if ( pWritePtr == pRxBufferEnd ) pWritePtr = pRxBufferStart;
					else				    pWritePtr++;
					pthread_mutex_unlock(&buf_mut);
			}
printf("\n");
		}
//		else{
//			usleep(5000);
//		}
	}
	return NULL;
}
/*
void* _IncomeInQueueThread(void* object)
{
unsigned char tmpBuffer[512];
int xi;
uint8_t xch;

	while(1)
	{
		int n=read(fcom, tmpBuffer, 512);
		if ( n > 0 && n < 512 ) {
			pthread_mutex_lock(&buf_mut);
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
	
							
			}
			pthread_mutex_unlock(&buf_mut);
		}
		else {
			//printf("sleep\n");
			usleep(5000);	//delay 1ms
			
		}
	}
	return NULL;
}
*/
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



/**************************************************
int _ReadBuffer_LF(int8_t* buffer)
{
int iResult = 0;
int fEnding =0;
int waitCnt, rxCnt;
int8_t *bptr, xch;

	//if ( !fIgnOpened_p2 ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wIgnRxCounter == 0 ) 
		{
			_ign_msdelay(4); //4ms
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut);
			xch = *pIgnReadPtr;
			if ( xch != 0x0d && xch != 0x0a  && xch != 0x00 ) {
				//maybe ignore all CTRL code 
				*bptr++ = xch;
				rxCnt++;
			}
			wIgnRxCounter_p2--;
			if ( pIgnReadPtr_p2 == pIgnRxBufferEnd_p2) pIgnReadPtr_p2 = pIgnRxBufferStart_p2;
			else				  pIgnReadPtr_p2++;
			pthread_mutex_unlock(&ign_buf_mut_p2);
			
			if ( xch == 0x0a || rxCnt >= MAX_STR_LENGTH )  {
				*bptr = 0x00;
				iResult =1;
				fEnding = 1;
			}
			
		}
	}

	return iResult;
}*/
/**************/

//************************************************************/
/*****************************/
/**** Close COM Port     *****/
/*****************************/
void _ClosePort(void)
{
	pthread_cancel(InQueueID);	
	pthread_mutex_destroy(&buf_mut);
	pthread_mutex_destroy(&port_mut);
	tcsetattr(fcom, TCSANOW, &old_tios);	//restore setting
	close(fcom);
	dwBaudrate = 0;
	fcom = 0;
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
fcom = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fcom < 0) return 0;
	tcgetattr(fcom, &old_tios);		//backup setting
//printf("open pass");
	//Buffer pointer initial
	wRxCounter =0 ;	
	pWritePtr = &RxBuffer[0];
	pReadPtr = &RxBuffer[0];
	pRxBufferStart = &RxBuffer[0];
	pRxBufferEnd = &RxBuffer[BUFFER_SIZE-1];
	//hook receiver thread
	fcom=1;
	pthread_mutex_init(&port_mut, NULL);
	pthread_mutex_init(&buf_mut, NULL);
	//pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
 
//	set_interface_attribs(fcom, B115200, 0); 
		//not checking device when assign baudrate
		//iret = _ReadBufferLength(rdData, 2);
		//if ( rdData[0]==0x10 && rdData[1]==0x02 )iResult = 1;
	iResult = 1;
	
	
	return iResult;
}

/***************************************************************/
int main(int argc, char **argv) 
{
int iResult = 0;
	//int fcom=0;
char *wrCmd = "I";
	char *com_path = "/dev/ttyUSB0";

char rdData[100];
int rcvCnt=0;

	if(argc<2)
	{
		printf("./AD411_util -continue\n");
		printf("./AD411_util -single\n");
		printf("./AD411_util -identify\n");
		printf("./AD411_util -single\n");
		printf("./AD411_util -gain\n");
		printf("./AD411_util -offset\n");
		printf("./AD411_util -save\n");

return 0;
	}

	if(strcmp("-continue",argv[1])==0)
	{printf("1\n");}
	else if(strcmp("-single",argv[1])==0)
	{printf("2\n");}
	else if(strcmp("-identify",argv[1])==0)
	{
		printf("3\n");
		iResult = _OpenPort(com_path);
		sleep(2);
		if(iResult == 1)
		{
		printf("open done\n");
			iResult = _SendBufferLength(wrCmd,1);
printf("Wait <enter>\n");
rcvCnt = read(fcom, rdData, 100);
printf("rcvCnt = %d",rcvCnt);
getchar();
			if(iResult == 1)
			{printf("send success\n");

				//iResult = _ReadBufferLength(rdData, 20000);
				//if ( iResult ) {
				//		printf("receive data");
				//}	
			}
		}
		else
		{	
			printf("fail to open : %s\n",com_path);
		}		//wrCmd[1] =;
	}
	else if(strcmp("-gain",argv[1])==0)
	{printf("4\n");}
	else if(strcmp("-offset",argv[1])==0)
	{printf("5\n");}
	else if(strcmp("-save",argv[1])==0)
	{printf("6\n");}
	else
	{printf("7\n");}




return 0;


}
