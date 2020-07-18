#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <termio.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "./CqUsbCam/CqUsbCam.h"
#include "./CqUsbCam/SensorCapbablity.h"
#include "Pintai.h"


#define MAIN_RESOLU_SELECT	 	'a'
#define MAIN_RESOLU_1280_720 			'0'
#define MAIN_RESOLU_1280_960 			'1'
#define MAIN_RESOLU_640_480SKIP 		'2'
#define MAIN_RESOLU_640_480BIN			'3'

#define MAIN_BITDEPTH_SELECT 		'b'
#define MAIN_BITDEPTH_8		 		'0'
#define MAIN_BITDEPTH_16		 	'1'
#define MAIN_BITDEPTH_L8		 	'2'

#define MAIN_PROCTYPE_SELECT 		'c'
#define MAIN_PROCTYPE_N		 		'0'
#define MAIN_PROCTYPE_X		 		'1'
#define MAIN_PROCTYPE_Y		 		'2'
#define MAIN_PROCTYPE_XY		 	'3'

#define MAIN_CHECK_SPEED 		'd'
#define MAIN_TRIGMODE_AUTO 		'e'
#define MAIN_TRIGMODE_FPGA 		'f'
#define MAIN_FPGA_TRIG_FREQ_SELECT	'g'
#define MAIN_EXPO_VALUE_SELECT		'h'
#define MAIN_GAIN_VALUE_SELECT		'i'
#define MAIN_AUTO_GAIN_EXPO_SELECT	'j'
#define MAIN_ROI			'k'
#define MAIN_CAPTURE			'l'
#define MAIN_ROI_BOX			'm'
#define MAIN_ANALOG_GAIN_AUTO_TRIG	'n'
#define MAIN_ANALOG_GAIN_FPGA_TRIG	'o'
#define MAIN_ANALOG_GAIN_1X			'1'
#define MAIN_ANALOG_GAIN_2X			'2'
#define MAIN_ANALOG_GAIN_4X			'4'
#define MAIN_ANALOG_GAIN_8X			'8'
#define MAIN_ROI_X1			'p'
#define MAIN_ROI_X2			'q'
#define MAIN_ROI_X3			'r'
#define MAIN_ROI_X4			's'
#define MAIN_ROI_X5			't'
#define MAIN_ROI_Y1			'u'
#define MAIN_ROI_Y2			'v'
#define MAIN_ROI_Y3			'w'
#define MAIN_ROI_Y4			'x'
#define MAIN_ROI_Y5			'y'
#define MAIN_ROI_WIDTH			'A'
#define MAIN_ROI_HEIGHT			'B'
#define MAIN_SAVE_VEDIO			'D'
#define MAIN_SAVE_ALL			'E'
#define MAIN_WRITE_SENSOR_REG		'F'
#define MAIN_READ_SENSOR_REG		'G'
#define MAIN_WRITE_FPGA_REG		'H'
#define MAIN_READ_FPGA_REG		'I'
#define MAIN_SELECT_SENSOR		'J'
#define MAIN_EXIT_NORMAL		'z'

string sensor = "AR0135";//	or AR0134
unsigned int g_width=1280;
unsigned int g_height=960;

int g_byteBitDepthNo=1;


pthread_mutex_t mutexDisp;
pthread_mutex_t mutexCam;
CCameraCtrl camctrl;
char*imgBuf = NULL;
int show_channel=0;
cq_uint16_t accel_x;
cq_uint16_t accel_y;
cq_uint16_t accel_z;
cq_uint16_t gyro_x;
cq_uint16_t gyro_y;
cq_uint16_t gyro_z;
cq_uint16_t imu_temp;
vector<cv::Mat> imageRGB;

void Disp(void* frameData)
{
	//pthread_mutex_lock(&mutexDisp);
	

	
	CImgFrame* imgframe = (CImgFrame*)frameData;
	int offset = 0;
	int imglen = camctrl.cam[0].height * camctrl.cam[0].width;
	if (imgBuf == NULL)
	{
		imgBuf = new char[imglen];
	}
	int i=0;
	accel_x = accel_y = accel_z = 0;
	gyro_x = gyro_y = gyro_z = 0;
	accel_x = (imgframe->IMUdata[i++] & 0xff) << 8;
	accel_x = accel_x + imgframe->IMUdata[i++];
	accel_y = (imgframe->IMUdata[i++] & 0xff) << 8;
	accel_y = accel_y + imgframe->IMUdata[i++];
	accel_z = (imgframe->IMUdata[i++] & 0xff) << 8;
	accel_z = accel_z + imgframe->IMUdata[i++];
	imu_temp = (imgframe->IMUdata[i++] & 0xff) << 8;
	imu_temp = imu_temp + imgframe->IMUdata[i++];
	gyro_x = (imgframe->IMUdata[i++] & 0xff) << 8;
	gyro_x = gyro_x + imgframe->IMUdata[i++];
	gyro_y = (imgframe->IMUdata[i++] & 0xff) << 8;
	gyro_y = gyro_y + imgframe->IMUdata[i++];
	gyro_z = (imgframe->IMUdata[i++] & 0xff) << 8;
	gyro_z = gyro_z + imgframe->IMUdata[i++];

	memcpy(imgBuf, imgframe->m_imgBuf + imglen * 0, imglen);
	cv::Mat frame(g_height, g_width, CV_8UC1, imgBuf);	
	cv::imshow("disp",frame);
	
	memcpy(imgBuf, imgframe->m_imgBuf + imglen * 1, imglen);
	cv::Mat frame1(g_height, g_width, CV_8UC1, imgBuf);
	cv::imshow("disp1",frame1);
	
	memcpy(imgBuf, imgframe->m_imgBuf + imglen * 2, imglen);
	cv::Mat frame2(g_height, g_width, CV_8UC1, imgBuf);	
	cv::Mat frameRGB,frameOut;
	cv::cvtColor(frame2, frameRGB, CV_BayerBG2BGR);


	//white balance
		cv::split(frameRGB, imageRGB);
		
		double R, G, B;
		B = cv::mean(imageRGB[0])[0];
		G = cv::mean(imageRGB[1])[0];
		R = cv::mean(imageRGB[2])[0];
		
		double KR, KG, KB;
		KB = (R + G + B) / (3 * B);
		KG = (R + G + B) / (3 * G);
		KR = (R + G + B) / (3 * R);

		imageRGB[0] = imageRGB[0] * KB;
		imageRGB[1] = imageRGB[1] * KG;
		imageRGB[2] = imageRGB[2] * KR;

		cv::merge(imageRGB, frameOut);
		cv::imshow("disp2", frameOut);
		cv::waitKey(10);
	
	//pthread_mutex_unlock(&mutexDisp);

}



CCqUsbCam cam0, *pCamInUse;


unsigned short hex2dec(char *hex)

{

	unsigned short  number=0;

	char *p=hex;

	for(p=hex;*p;++p)
	{
		if((hex[p-hex]<='z')&&(hex[p-hex]>='a'))
			hex[p-hex]=hex[p-hex]-32;
		number=number*16+(hex[p-hex]>='A'?hex[p-hex]-'A'+10:hex[p-hex]-'0');
	}

	return number;

}

void checkspeed()
{
	unsigned int speed = 0;
	cam0.GetUsbSpeed(speed);
	if(speed==LIBUSB_SPEED_SUPER)
	{
		printf("USB 3.0 device found on cam0!\n");
		cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_SUPER);
	}
	else if(speed==LIBUSB_SPEED_HIGH)
	{
		printf("USB 2.0 device found on cam0!\n");
		cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_HIGH);
	}
	else
	{
		printf("Device speed unknown on cam0!\n");
	}
}

void timerFunction(int sig)
{

	unsigned long iByteCntPerSec=0;
	unsigned long iFrameCntPerSec=0;

	pthread_mutex_lock(&mutexCam);

	cam0.GetRecvByteCnt(iByteCntPerSec);
	cam0.ClearRecvByteCnt();
	cam0.GetRecvFrameCnt(iFrameCntPerSec);
	cam0.ClearRecvFrameCnt();

	printf("cam0: %ld Fps     %0.4f MBs\n", iFrameCntPerSec, float(iByteCntPerSec)/1024.0/1024.0);
	printf("accel(x,y,z)    gyro(x,y,z)\n");
	printf("%16d|%16d\n",accel_x,gyro_x);
	printf("%16d|%16d\n",accel_y,gyro_y);
	printf("%16d|%16d\n",accel_z,gyro_z);
	//system("gnome-terminal -e  sudo echo  hahah >./text");
	/***************************************************************/
	alarm(1);

	pthread_mutex_unlock(&mutexCam);

}



int main(int argc, char *argv[])
{
	cq_int32_t ret;
	ret =pthread_mutex_init(&mutexDisp, NULL);
	if(ret!=0)
		printf("pthread_mutex_init failed");
	ret =pthread_mutex_init(&mutexCam, NULL);
	if(ret!=0)
		printf("pthread_mutex_init failed");

	cam0.SelectSensor(sensor);

	int usbCnt=CCqUsbCam::OpenUSB();
	printf("%d usb device(s) found!\n", usbCnt);
	if(usbCnt<=0)
	{
		printf("exiting ...\n");
		return -1;
	}
	cam0.ClaimInterface(0);

	checkspeed();

	cam0.InitSensor();

	pCamInUse=&cam0;
	camctrl.init(pCamInUse);
	while(1)
	{
		printf("Please input your choice ...\n");
		printf("\
				\'a\':	Select Camera\n\
				\'b\':	Set Auto Gain, Auto Expo(0,2)\n\
				\'c\':	Set Expo(0~65535)\n\
				\'d\':	Set Gain(0~65535)\n\
				\'e\':	Set Mirror(default:0,X:4,Y:8,XY:12)\n\
				\'f\':	Set Analog Gain(1,2,4,8)\n\
				\n\
				\'l\':	Begin to capture\n\
				\n\
				\'z\':	Exit\n"\
				);
		char ch=getchar();
		getchar();
		printf("Your choice is %c\n", ch);
		switch(ch)
		{
			case 'a':
				{
					printf("select cameara (Dec, 0,1,2)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str,9,stdin);
					unsigned int cameraValue=atoi(str);
					printf("Your input is %d \n", cameraValue);
					for (int i = 0; i < 3; i++)
					{
						camctrl.cam[i].checked = 0;
					}
					camctrl.cam[cameraValue].checked = 1;
					
					break;
				}
			case 'b':
				{
					printf("select auto Gain auto expo (Dec, 0 close,2, open)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str,9,stdin);
					unsigned int cameraValue=atoi(str);
					printf("Your input is %d \n", cameraValue);
					camctrl.setCamFunction(0, cameraValue);
					break;			
				}
			case 'c':
				{
					printf("Please input the expo value (Dec, 0~65536)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str,9,stdin);
					unsigned int expoValue=atoi(str);
					printf("Your input is %d \n", expoValue);
					camctrl.setCamFunction(1, expoValue);
					break;
				}
			case 'd':
				{
					printf("Please input the gain value (Dec, 0~64)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str,9,stdin);
					unsigned int gainValue=atoi(str);
					printf("Your input is %d \n", gainValue);
					camctrl.setCamFunction(2, gainValue);
					break;
				}		
			case 'e':
				{
					printf("Set Mirror(default:0,X:4,Y:8,XY:12)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str,9,stdin);
					unsigned int v=atoi(str);
					printf("Your input is %d \n", v);
					camctrl.setCamFunction(3, v);
					break;
				}	
				case 'f':
				{
					printf("Set Analog Gain(1,2,4,8)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str,9,stdin);
					unsigned int v=atoi(str);
					printf("Your input is %d \n", v);
					camctrl.setCamFunction(4, v);
					break;
				}	
			case MAIN_CAPTURE:
				{
					camctrl.setGenFunction(0, 3);
					cv::namedWindow("disp",CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
					//cam0.StartCap(g_height,  (g_byteBitDepthNo==1? g_width: g_width*2), Disp);
					printf("datalen:%d\n",camctrl.getTotalDataLen());
					cam0.StartCap(camctrl.getTotalDataLen(), 1, Disp);
					
					signal(SIGALRM, timerFunction);
					alarm(1);

					printf("Press any key to stop capturing\n");


					getchar();


					pthread_mutex_lock(&mutexCam);
					alarm(0);
					cam0.StopCap();
					pthread_mutex_unlock(&mutexCam);

					pthread_mutex_lock(&mutexDisp);
					cv::destroyWindow("disp");
					cv::waitKey(1);
					cv::waitKey(1);
					cv::waitKey(1);
					cv::waitKey(1);
					pthread_mutex_unlock(&mutexDisp);



					break;
				}
			
			case MAIN_EXIT_NORMAL:
				cam0.ReleaseInterface();
				CCqUsbCam::CloseUSB();
				printf("Exiting ...\n");
				exit(0);
				break;
			default:
				printf("Bad inut ...\n");
		}

	}



	return 0;

}



