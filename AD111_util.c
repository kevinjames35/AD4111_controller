#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>


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
unsigned char tmpBuffer[512];
int xi;
uint8_t xch;

	while(1)
	{
		int n=read(fcom, tmpBuffer, 512);
		if ( n > 0 && n < 512 ) {
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
					pthread_mutex_lock(&buf_mut);					
					*pWritePtr = xch;
					wRxCounter++;
					if ( pWritePtr == pSASRxBufferEnd ) pWritePtr = pSASRxBufferStart;
					else				    pWritePtr++;
					pthread_mutex_unlock(&buf_mut);
							
			}
		}
		else {
			usleep(5000);	//5ms
		}
	}
	return NULL;
}
/******************************************/
int32_t _SendBufferLength(uint8_t* buffer, int32_t length)
{
uint8_t *bptr;

	if ( !fcom ) return 0;
	if ( length < 1 ) return 0;
	bptr = buffer;
	write(fcom, bptr, length);
	tcdrain(fcom);
	return 1;
}
/**************/
int _ReadBufferLength(uint8_t* buffer, int32_t length)
{
int32_t iResult = 0;
int32_t fEnding =0;
int waitCnt, rxCnt;
uint8_t *bptr, xch;

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
			if ( pReadPtr == pSASRxBufferEnd) pReadPtr = pSASRxBufferStart;
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
//************************************************************/
/*****************************/
/**** Close LCM Port     *****/
/*****************************/
void _CloseLcmPort(void)
{
	pthread_cancel(SASInQueueID);	
	pthread_mutex_destroy(&buf_mut);
	pthread_mutex_destroy(&port_mut);
	tcsetattr(fcom, TCSANOW, &old_tios);	//restore setting
	close(fcom);
	dwBaudrate = 0;
	fcom=-1;
	fcom = 0;
}
/*****************************/
/**** Open LCM Port      *****/
/*****************************/
int32_t _OpenLcmPort(char* pLcmPath)
{
int iResult = 0;
uint32_t baudrate[]={B115200, B57600, B38400, B19200, B9600 };
uint32_t brValue[] ={ 115200,  57600,  38400,  19200,  9600 };
uint8_t wrCmd[3] = {0xFE, 0x30, 0xFD };
uint8_t rdData[10];
uint32_t xi;
int32_t iret;

	fcom = open(pLcmPath, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fcom < 0) return 0;
	tcgetattr(fcom, &old_tios);		//backup setting

	//Buffer pointer initial
	wRxCounter =0 ;	
	pWritePtr = &SASRxBuffer[0];
	pReadPtr = &SASRxBuffer[0];
	pSASRxBufferStart = &SASRxBuffer[0];
	pSASRxBufferEnd = &SASRxBuffer[BUFFER_SIZE-1];
	//hook receiver thread
	fcom=1;
	pthread_mutex_init(&port_mut, NULL);
	pthread_mutex_init(&buf_mut, NULL);
	pthread_create(&SASInQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
 
	if ( dwBaudrate == 0 ) { //search 
		for (xi=0 ; xi<(sizeof(baudrate)/4) ; xi++ ) {
			//open LCM Communciaction port 
			set_interface_attribs(fcom, baudrate[xi], 0); //baudrate 2400 ~ 115200, parity none
			//write Read Model Number command "FE 30 FD"	
			wrCmd[1] = 0x30;
			write(fcom, wrCmd, 3);
			// read model number 2bytes
			iret = _ReadBufferLength(rdData, 2);
			if ( iret ) {
				dwBaudrate = brValue[xi];
				wrCmd[1] = 0x58;
				write(fcom, wrCmd, 3); //clear, because serch will effect LCD screen display
				xi = 1000;
				iResult = 1;	
			}
		}
	}
	else {
		switch (dwBaudrate) {
		case 57600:
			xi=1;
			break;
		case 38400:
			xi=2;
			break;
		case 19200:
			xi=3;
			break;
		case 9600:
			xi=4;
			break;
		case 115200:
		default:
			xi=0;
			break;

		}
		set_interface_attribs(fcom, baudrate[xi], 0); 
		//not checking device when assign baudrate
		//wrCmd[1] = 0x30;
		//write(fcom, wrCmd, 3);
		//iret = _ReadBufferLength(rdData, 2);
		//if ( rdData[0]==0x10 && rdData[1]==0x02 )iResult = 1;
		iResult = 1;
	}
	
	return iResult;
}

/***************************************************************/
int main(int argc, char **argv) 
{

	int fcom=0;
	char *com_path="/dev/ttyS1";
	fcom = open(com_path,O_RDWR|O_NOCTTY|O_SYNC);
	if(fcom==1)
	{
		
			
	//Buffer pointer initial
	wRxCounter =0 ;	
	pWritePtr = &SASRxBuffer[0];
	pReadPtr = &SASRxBuffer[0];
	pSASRxBufferStart = &SASRxBuffer[0];
	pSASRxBufferEnd = &SASRxBuffer[BUFFER_SIZE-1];
	//hook receiver thread

	pthread_mutex_init(&port_mut, NULL);
	pthread_mutex_init(&buf_mut, NULL);
	pthread_create(&SASInQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));
 
	set_interface_attribs(fcom,B115200,0);//set baud rate 115200

	
	
	}
	else
	{	
		printf("fail to open : %s\n",*com_path);
	}


	if(argc<2)
	{
		printf("./AD411_util -continue");
		printf("./AD411_util -single");
		printf("./AD411_util -identify");
		printf("./AD411_util -single");
		printf("./AD411_util -gain");
		printf("./AD411_util -offset");
		printf("./AD411_util -save");
	}
	if(strcmp(argv[1],"-continue"))
	{}
	else if(strcmp(argv[1],"-continue"))
	{}
	else if(strcmp(argv[1],"-continue"))
	{}
	else if(strcmp(argv[1],"-continue"))
	{}
	else if(strcmp(argv[1],"-continue"))
	{}
	else if(strcmp(argv[1],"-continue"))
	{}
	else if(strcmp(argv[1],"-continue"))
	{}
	else if(strcmp(argv[1],"-continue"))
	{}
	else
	{}




}
