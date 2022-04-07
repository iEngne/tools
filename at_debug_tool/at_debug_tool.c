
#include "stdio.h"
#include "string.h"
#include "unistd.h"
#include "termios.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <dirent.h>
/* 内存操作*/
#define OSA_clear(ptr)		            memset((ptr), 0, sizeof(*(ptr)))	
#define OSA_clearSize(ptr, size)        memset((ptr), 0, (size))
#define OSA_memCpy(dst, src)            memcpy((dst), (src), sizeof(*(src)))
#define OSA_memCpySize(dst, src, size)  memcpy((dst), (src), (size))
#define OSA_memCmp(dst, src)            memcmp((dst), (src), sizeof(*(src)))
#define OSA_memCmpSize(dst, src, size)  memcmp((dst), (src), (size))
#define OSA_strstr(sStrA,sStrB)          \
                strstr(sStrA, sStrB)

#define READ_BUF_SIZE 128
#define ATCMD_RES_BUF_SIZE   512
/* 4G网络AT命令发送设备节点 EC20 AT口*/
///#define   MOBILE_4G_AT_CMD_DEV     "/dev/ttyUSB2"
/*EC200T AT口*/
///#define   MOBILE_4G_AT_CMD_DEV     "/dev/ttyUSB1"
#define   AT_MSG_MAX  256
#define   MSG_BUFFER_MAX  512
#define OSA_SOK        0        /* 成功*/
#define OSA_EFAIL     -1        /* 通用错误*/
#define OSA_ETIMEOUT  -2        /* 等待超时*/
#define OSA_EINTR     -3        /* 被信号中断*/
#define OSA_EBUSY     -4        /* 对象忙 */
/* 判断返回值 */
#define OSA_isSuccess(status)  (OSA_SOK == (status))
#define OSA_isFail(status)     (OSA_SOK != (status))
    
#define OSA_isTrue(status)     ((status))
#define OSA_isFalse(status)    (!(status))   
    
#define OSA_isNull(ptr)        (NULL == (ptr))
#define OSA_isNotNull(ptr)     (NULL != (ptr))
#ifndef OSA_RETURN_IFFALSE
#define OSA_RETURN_IFFALSE(p) if (!(p))\
    { \
        printf("%s check fails, return OSA_EFAIL\n", #p); \
        return OSA_EFAIL; \
    }
#endif
typedef enum AT_MSG_RETURN_TYPE
{
    AT_MSG_RETURN_OK=0,
    AT_MSG_RETURN_ERR,
    AT_MSG_RETURN_TIMEOUT,    
    AT_MSG_RETURN_BUTT,
}AT_MSG_RETURN_TYPE_E;

/* 布尔量定义 */
#ifndef DEFINED_Bool
#define DEFINED_Bool
typedef unsigned char      Bool;        /* 通用布尔类型 */
#endif

/* 有符号类型定义 */
#ifndef DEFINED_Int32
#define DEFINED_Int32
typedef int                Int32;       /* 有符号32位整形数类型 */   
#endif

#ifndef DEFINED_Int16
#define DEFINED_Int16
typedef short              Int16;       /* 有符号16位整形数类型 */   
#endif

#ifndef DEFINED_Int8
#define DEFINED_Int8
typedef char               Int8;        /* 有符号8位整形数类型 */    
#endif

/* 指针类型定义 */
#ifndef DEFINED_Ptr
#define DEFINED_Ptr
typedef void *             Ptr;         /* 指针类型 */
#endif

/* 字符类型定义 */
#ifndef DEFINED_String
#define DEFINED_String
typedef char *             String;      /* 字符串类型，以NUL结尾。*/
#endif

#ifndef DEFINED_Char
#define DEFINED_Char
typedef char               Char;        /* 字符类型 */
#endif

/* 无符号类型定义 */ 
#ifndef DEFINED_Uint32
#define DEFINED_Uint32
typedef unsigned int       Uint32;      /* 无符号32位整形数类型 */ 
#endif

#ifndef DEFINED_Uint16
#define DEFINED_Uint16
typedef unsigned short     Uint16;      /* 无符号16位整形数类型 */ 
#endif

#ifndef DEFINED_Uint8
#define DEFINED_Uint8
typedef unsigned char      Uint8;       /* 无符号8位整形数类型 */ 
#endif

#ifndef DEFINED_Int64
#define DEFINED_Int64
typedef long long          Int64;       /* 有符号64位整形数类型 */
#endif

#ifndef DEFINED_Bool16
#define DEFINED_Bool16
typedef unsigned short     Bool16;      /* 16位布尔类型 */
#endif

#ifndef DEFINED_Bool32
#define DEFINED_Bool32
typedef unsigned int       Bool32;      /* 32位布尔类型 */
#endif

#ifndef DEFINED_Int32L
#define DEFINED_Int32L
typedef long               Int32L;      /* 有符号32位长整形数类型 */ 
#endif

#ifndef DEFINED_Uint32L
#define DEFINED_Uint32L
typedef unsigned long      Uint32L;     /* 无符号32位长整形数类型 */   
#endif

#ifndef DEFINED_Uint64
#define DEFINED_Uint64
typedef unsigned long long Uint64;      /* 无符号64位整形数类型 */
#endif


/* 浮点类型定义 */
#ifndef DEFINED_Float32
#define DEFINED_Float32
typedef float              Float32;		/* 32位浮点数类型 */
#endif

#ifndef DEFINED_Float64
#define DEFINED_Float64
typedef double             Float64;		/* 64位浮点数类型 */
#endif

/* 句柄类型 */
#ifndef DEFINED_Handle
#define DEFINED_Handle
typedef void *             Handle;      /* 统用句柄类型 */
#endif

/* 空类型定义 */
#ifndef DEFINED_Empty
#define DEFINED_Empty
typedef void               Empty;        /* 空类型 */
#endif

/* size_t类型定义 */
#ifndef DEFINED_Sizet
#define DEFINED_Sizet
typedef size_t             Sizet;       /* size_t类型 */
#endif
static Int32 g_atCmdHdl = -1;

#define EC200T_NODE  "/dev/ttyUSB1"
#define EC20_NODE   "/dev/ttyUSB2"


/*******************************************************************************
*  函数名  :   net_4g_send_atcmd
*  功  能 :   发送AT CMD
*  输  入 :   - cmd  AT命令
*  返  回 :   OSA_EFAIL  发送失败
*		    OSA_SOK    发送成功
*******************************************************************************/
static int mobile_send_atcmd(char * pCmd)
{
	int n;

    int length = strlen(pCmd);
    pCmd[length] = '\r';
    pCmd[length+1] = '\n';
	n = write(g_atCmdHdl, pCmd, strlen(pCmd));
	if( n < 0 )
	{
		printf(" send cmd failed!\n");
		return OSA_EFAIL;		
	}
    printf("start to send:%s\n", pCmd);
    usleep(3*1000);
	return OSA_SOK;
}


/* /dev/ttyUSB2的初始化*/
Int32 mobile_serial_init(void)
{
	struct termios newtio,oldtio;
	//save old setting
	if ( tcgetattr(g_atCmdHdl,&oldtio) != 0) 
	{ 
		printf("tcgetattr error");
		return OSA_EFAIL;
	}
	//extern void bzero(void *s, int n); set s zero 
	OSA_clear(&newtio);
	//set the size of char
	newtio.c_cflag |= CLOCAL | CREAD; 
	newtio.c_cflag &= ~CSIZE;
	//set data mode(CS8)  //CS7
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;
	//set speed B115200 B2400 B4800 B9600
	cfsetispeed(&newtio, B115200);
	cfsetospeed(&newtio, B115200);
	//set stop CSTOPB or ~CSTOPB
	newtio.c_cflag &= ~CSTOPB;
	//set wait time and less recv char 
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	//deal the char not recv
	tcflush(g_atCmdHdl,TCIFLUSH);
	//new setting 
	if((tcsetattr(g_atCmdHdl,TCSANOW,&newtio))!=0)
	{
		printf("com set error");//
		return OSA_EFAIL;
	}
	printf("serial_init done!\n");
	
	return OSA_SOK;
}

int getTtyUSBNum(void)
{
    DIR *dir;
    int index = 0;
    struct dirent *ptr;
    char path[32];
	dir = opendir("/dev");
	if (dir == NULL)
	{
        printf("Dir is not exist!\n");
        return OSA_EFAIL;
	}
	while ((ptr=readdir(dir)) != NULL)
	{
		if (strcmp(ptr->d_name, ".") == 0 ||strcmp(ptr->d_name, "..") == 0)
		{
			continue;
		}
		sprintf(path,"/dev/%s",ptr->d_name);
		/* Do get sub partition device. */
        if (strstr(ptr->d_name, "ttyUSB"))
        {
			printf("Get block device path=%s\n",path);
			index++;
        }
	}
    return index;
}


/* 打开/dev/ttyUSB2*/
Int32 mobile_at_handle_init(void)
{
    int i = 0;
	long flags = 0;
	char nodePath[32] = {0};

	if (g_atCmdHdl != -1)
	{
		printf(" at cmd handle have been inited!\n");
		return OSA_SOK;
	}
    int ttyUSBNum = getTtyUSBNum();
    if (ttyUSBNum == 3)
    {
        memcpy(nodePath, EC200T_NODE,strlen(EC200T_NODE));
    }
    else if (ttyUSBNum > 3)
    {
        memcpy(nodePath, EC20_NODE,strlen(EC20_NODE));
    }
    printf("nodePath:%s\n", nodePath);
	for(i = 0; i < 10;i++)
	{
	    if(access(nodePath,F_OK) == 0)
	    {
	        break;
	    }
	    else
	    {
	       usleep(1000*1000);
	    }

	}
	
	if(i == 10)
	{
		printf(" access %s Failed\n", nodePath);		
		return OSA_EFAIL;
	}
	
	g_atCmdHdl = open(nodePath, O_RDWR | O_NOCTTY | O_NDELAY );
	if (g_atCmdHdl < 0 )
	{
		printf(" open %s failed!!\n", nodePath);
		return OSA_EFAIL;
	}

	flags &= ~O_NONBLOCK;
	fcntl(g_atCmdHdl, F_SETFL, flags); //set to block

	return OSA_SOK;
}

/* 初始化at命令相关设备*/
Int32 mobile_at_init(void)
{
	Int32 ret = OSA_EFAIL;

	ret = mobile_at_handle_init();
	if(OSA_isFail(ret))
	{
		return OSA_EFAIL;
	}

	return mobile_serial_init();
}


static int mobile_send_cmd(int atcmdfd,char *pmsg_buf)
{
	int ret = -1;
	if(OSA_isNull(pmsg_buf) || (atcmdfd < 0))
	{
		printf(" args is null\n");
		return OSA_EFAIL;
	}

	ret = write(atcmdfd, pmsg_buf, strlen(pmsg_buf));
	if( ret < 0 )
	{
		printf(" send cmd failed!\n");
		return OSA_EFAIL;
	}
	
    return OSA_SOK;
}



static AT_MSG_RETURN_TYPE_E mobile_get_msg(int atcmdfd, char *pmsg_buf, 
							char *pmsg_expect_start, char *pmsg_expect_ok_end, 
							char *pmsg_expect_err_end, int repone_ms_time_out)
{
    
	char  msg_buf[MSG_BUFFER_MAX]; //单次读取数据缓存
	unsigned int remain_size = sizeof(msg_buf);
	unsigned char flagEnd = 0;		
	unsigned char flagHead = 0;	
	unsigned char expect_start_cnt = strlen(pmsg_expect_start);	
	unsigned char expect_ok_end_cnt = 0;	
	unsigned char expect_err_end_cnt = 0;
	unsigned char expect_index = 0;	
	unsigned char expect_ok_end_index = 0;
	unsigned char expect_err_end_index = 0;
	char msg_byte = 0;
	int ret=-1;
	AT_MSG_RETURN_TYPE_E MsgRet = AT_MSG_RETURN_BUTT;

    int time_ms_use=0;
    struct timeval start;
    struct timeval end;
    gettimeofday(&start,NULL);
	
	if(atcmdfd < 3)
	{
		printf(" args is atcmdfd < 3\n");
	}
	
	if(OSA_isNull(pmsg_buf) || OSA_isNull(pmsg_expect_start) || (atcmdfd < 3))
	{
		printf(" args is null\n");
		return AT_MSG_RETURN_BUTT;
	}

    if(NULL != pmsg_expect_ok_end)
	{
	    
        expect_ok_end_cnt = strlen(pmsg_expect_ok_end);
	}
	
    if(NULL != pmsg_expect_err_end)
	{
	    
        expect_err_end_cnt = strlen(pmsg_expect_err_end);
	}

	OSA_clearSize(msg_buf, sizeof(msg_buf));

	while (remain_size > 0)
	{		
		ret = read(atcmdfd, &msg_byte, 1);		
		
        //printf("%c", msg_byte);
		if ( ret < 0 )
		{
			printf(" read at cmd result failed!\n");
			return OSA_EFAIL;
		}
		else if(ret == 0)
		{
		    break;
		}
		else
		{
    		if(msg_byte == *(pmsg_expect_start+expect_index) && (flagHead == 0)) {
    		    if(expect_index < sizeof(msg_buf)) {
                    msg_buf[expect_index++] = msg_byte;                
    	        }
    		}
    		else {
    		    if(expect_index < expect_start_cnt) {
    		        expect_index = 0;
    		    }
    		}

    		if(NULL == pmsg_expect_ok_end) {
    		    if((expect_index >= expect_start_cnt) && (flagHead == 1)) {
        		    if(expect_index < sizeof(msg_buf)) {
                        msg_buf[expect_index++] = msg_byte;                
        	        }	
        		    
        		    if(msg_byte == 0x0d) {
        		        flagEnd = 1;
        		    }
        		    if((flagEnd == 1) && (msg_byte == 0x0a)) {
                        MsgRet = AT_MSG_RETURN_OK;
        		        break;
        		    }        		    
        		}        		
    		}
    		else {
        		if((expect_index >= expect_start_cnt) && (flagHead== 1)) {
                    if(expect_index < sizeof(msg_buf)) {
                        msg_buf[expect_index++] = msg_byte;                
                    }

        		    if(msg_byte == *(pmsg_expect_ok_end+expect_ok_end_index)) {
                        expect_ok_end_index++;                        
            		}
            		
        		    if(msg_byte == *(pmsg_expect_err_end+expect_err_end_index)) {
                        expect_err_end_index++;                        
            		}
            		
        		    if((expect_ok_end_cnt == expect_ok_end_index) && 
						(expect_ok_end_index != 0)  && (msg_byte == 0x0d)) {
        		        flagEnd = 1;
        		        MsgRet = AT_MSG_RETURN_OK;
        		    }
        		    
        		    if((expect_err_end_cnt == expect_err_end_index) && 
						(expect_err_end_index != 0)  && (msg_byte == 0x0d)) {
        		        flagEnd = 1;        		        
        		        MsgRet = AT_MSG_RETURN_ERR;
        		    }
        		    
        		    if((flagEnd == 1) && (msg_byte == 0x0a)) {
        		        break;
        		    }
        		}
    		}
			
			if(expect_index == expect_start_cnt) {
                flagHead = 1;
    		}
    		remain_size--;
		}
		
        gettimeofday(&end,NULL);
        time_ms_use=(end.tv_sec-start.tv_sec)*1000+(end.tv_usec-start.tv_usec)/1000;//微秒
        if(time_ms_use > repone_ms_time_out)
        {
            MsgRet = AT_MSG_RETURN_TIMEOUT;
            break;
        }
	}

	//printf("ret:%d,msg_buf:%s,expect_index:%d,remain_size:%d\n",MsgRet,msg_buf,expect_index,remain_size);
	if(expect_index < AT_MSG_MAX)
		memcpy(pmsg_buf, msg_buf, expect_index);

    return MsgRet;
}

int MobileOpenModule(void)
{
	Int32   status;	
    char at_msg_buf[AT_MSG_MAX]; 
    
    printf(" Mobile Open Module!\n");
    
	status = mobile_at_init();
	if(OSA_isFail(status))
	{
		printf("net_4g_at_init error!\n");
		return OSA_EFAIL;
	}


    tcflush(g_atCmdHdl, TCIOFLUSH);/* 清空接收和发送缓冲 */

    status = mobile_send_cmd(g_atCmdHdl, "ATE0\r\n");
	if(OSA_isFail(status))
	{
		return OSA_EFAIL;
	}
	
	usleep(50000);
    mobile_get_msg(g_atCmdHdl, at_msg_buf, "OK", NULL, NULL, 300);

	/* message format */
	status = mobile_send_cmd(g_atCmdHdl, "AT+CMGF=1\r\n");
	if(OSA_isFail(status))
	{
		return OSA_EFAIL;
	}   
    usleep(50000);
    mobile_get_msg(g_atCmdHdl, at_msg_buf, "OK", NULL, NULL, 300);
	
    return OSA_SOK;
}


int MobileCloseModule(void)
{	
    printf(" Mobile Close Module!\n");

    if(g_atCmdHdl  > 0)
    {
		tcflush(g_atCmdHdl,TCIFLUSH);
		tcdrain(g_atCmdHdl);
		close(g_atCmdHdl);
    }
    
    g_atCmdHdl = -1;
    return OSA_SOK;
}


void* atReadRes(void* args)
{
    int nb_read;
    Uint32 length = 1024;
    Uint8 buffer[length];
    fd_set fs_read;
    struct timeval tv_timeout; // set polling time; Non-blocking is set to 0;
    while(1)
    {
        FD_ZERO(&fs_read);
        FD_SET(g_atCmdHdl, &fs_read);
        tv_timeout.tv_sec = 0;
        tv_timeout.tv_usec = 50*1000;
        ssize_t fs_sel = select(g_atCmdHdl + 1, &fs_read, NULL, NULL, &tv_timeout);
        switch(fs_sel)
        {
 			case -1:
				printf("select error!!");
				break; //select错误，退出程序
			case 0:
				break; //再次轮询
	        default:
	            if(FD_ISSET(g_atCmdHdl, &fs_read))
                {
                    memset(buffer, 0, sizeof(buffer));
	                ssize_t nb_read = read(g_atCmdHdl, buffer, length);
	                if(nb_read == -1)
                    {
	                    printf("Unable to read from serial port.");
                    }
                    else
                    {
                        printf("%s\n",buffer);
                    }
           	     }
        }
    }
}

int main(int argc, char* argv[])
{
    char cmdBuffer[1024] = {0};
    int ret = 0;
    pthread_t atReadTh;
    int length;

    MobileOpenModule();
    printf("MobileOpenModule done\n ");
    ret = pthread_create(&atReadTh, NULL, atReadRes, NULL);
    if (OSA_isFail(ret))
    {
        printf("create thread:atReadRes failed\n");
        return -1;
    }
    printf("create thread:atReadRes done!\n");
    printf("input 'q' to quit!\n");
    while(1)
    {
        fflush(stdin);
        memset(cmdBuffer, 0, sizeof(cmdBuffer));
        printf("[at_debug_tool]$:");
        fgets(cmdBuffer, sizeof(cmdBuffer), stdin);
        length = strlen(cmdBuffer);
        cmdBuffer[length - 1] = 0;
        if (cmdBuffer[0] == 'q')
        {
            MobileCloseModule();
            return 0;
        }
        else if ((cmdBuffer[0] == 0))
        {
            continue;
        }
        mobile_send_atcmd(cmdBuffer);
    }
    
    return 0;

}




