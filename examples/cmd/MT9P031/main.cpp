//#define V4L2
//https://arcoresearchgroup.wordpress.com/2020/06/02/virtual-camera-for-opencv-using-v4l2loopback/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <termio.h>

#ifdef V4L2
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../../../../CqUsbCam_Linux/CqUsbCam/CqUsbCam.h"
#include "../../../../CqUsbCam_Linux/CqUsbCam/SensorCapbablity.h"

#define MAIN_RESOLU_SELECT	 		'a'
#define MAIN_RESOLU_752_480 		'0'
#define MAIN_RESOLU_640_480 		'1'
#define MAIN_RESOLU_640_480SKIP 	'2'
#define MAIN_RESOLU_640_480BIN		'3'
#define MAIN_RESOLU_1280_1024		'4'

#define MAIN_BITDEPTH_SELECT 		'b'
#define MAIN_BITDEPTH_8		 		'0'
#define MAIN_BITDEPTH_16		 	'1'
#define MAIN_BITDEPTH_L8		 	'2'

#define MAIN_PROCTYPE_SELECT 		'c'
#define MAIN_PROCTYPE_N		 		'0'
#define MAIN_PROCTYPE_X		 		'1'
#define MAIN_PROCTYPE_Y		 		'2'
#define MAIN_PROCTYPE_XY		 	'3'


#define MAIN_CHECK_SPEED 			'd'

#define MAIN_TRIGMODE_SELECT		'e'
#define MAIN_TRIGMODE_AUTO 			'0'
#define MAIN_TRIGMODE_FPGA 			'1'
#define MAIN_TRIGMODE_SOFT 			'2'
#define MAIN_TRIGMODE_SIGNAL		'3'

#define MAIN_FPGA_TRIG_FREQ_SELECT	'g'
#define MAIN_EXPO_VALUE_SELECT		'h'
#define MAIN_GAIN_VALUE_SELECT		'i'
#define MAIN_AUTO_GAIN_EXPO_SELECT	'j'
#define MAIN_ROI					'k'
#define MAIN_CAPTURE				'l'
#define MAIN_ROI_BOX				'm'
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
#define MAIN_ROI_WIDTH		'A'
#define MAIN_ROI_HEIGHT		'B'
#define MAIN_SAVE_VEDIO		'D'
#define MAIN_SAVE_ALL		'E'
#define MAIN_WRITE_SENSOR_REG	'F'
#define MAIN_READ_SENSOR_REG		'G'
#define MAIN_WRITE_FPGA_REG		'H'
#define MAIN_READ_FPGA_REG		'I'
#define MAIN_SELECT_SENSOR		'J'
#define MAIN_EXIT_NORMAL		'z'

string sensor = "MT9P031";
unsigned int g_width=2592;
unsigned int g_height=1944;

pthread_mutex_t mutexDisp;
pthread_mutex_t mutexCam;
int output=-1;
void Disp(void* frameData)
{
	pthread_mutex_lock(&mutexDisp);
	CImgFrame* m_frame=(CImgFrame*)frameData;
	printf("trig source:%d",m_frame->m_imgHead[2]);
	cv::Mat frame(g_height, g_width, CV_8UC1, (unsigned char*)m_frame->m_imgBuf);
	#ifdef V4L2
	size_t written = write(output, frame.data, g_height*g_width);
        if (written < 0) {
            printf( "ERROR: could not write to output device!\n");
            close(output);
        }
		#else
		cv::resize(frame,frame,cv::Size(800,600));
		cv::imshow("disp",frame);
		cv::waitKey(10);
		#endif
		
	pthread_mutex_unlock(&mutexDisp);

}



CCqUsbCam cam0,  *pCamInUse;


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
	cam0.InitSensor(); 

	checkspeed();

	pCamInUse=&cam0;

	while(1)
	{
		printf("Please input your choice ...\n");
		printf("\
				\'b\':	Fpga Trig 0 or 1 sensor\n\
				\'c\':	Select proc type\n\
				\'d\':	Check speed\n\
				\'e\':	Select trig mode\n\
				\'g\':	Set fpga trig frequency\n\
				\'h\':	Set expo value\n\
				\'i\':	Set gain value\n\
				\n\
				\'l\':	Start capture\n\
				\n\
				\'F\':	Write sensor\n\
				\'G\':	Read sensor\n\
				\'H\':	Write FPGA\n\
				\'I\':	Read FPGA\n\
				\n\
				\'z\':	Exit\n"\
	 );
		char ch=getchar();
		getchar();
		printf("Your choice is %c\n", ch);
		switch(ch)
		{
#if 0
			case MAIN_RESOLU_SELECT:
				{
					printf("Please input your choice ...\n");
					printf("\
							\'0\':	752x480\n\
							\'1\':	640x480\n\
							"\
					      );
					char ch=getchar();
					getchar();	
					printf("Your choice is %c\n", ch);
					switch(ch)
					{
						case MAIN_RESOLU_752_480:
							pCamInUse->SetResolution(RESOLU_752_480);
							if(pCamInUse==&cam0)
							{
								g_width=752;
								g_height=480;
							}
							break;
						case MAIN_RESOLU_640_480:
							pCamInUse->SetResolution(RESOLU_640_480);
							if(pCamInUse==&cam0)
							{
								g_width=640;
								g_height=480;
							}
							break;

						default:
							printf("Bad inut ...");
					}
					break;
				}
#endif
			case MAIN_PROCTYPE_SELECT:
				{
					printf("Please input your choice ...\n");
					printf("\
							\'0\':	Normal\n\
							\'1\':	X mirror\n\
							\'2\':	Y mirror\n\
							\'3\':	XY mirror\n"\
					      );

					char ch=getchar();
					getchar();
					printf("Your choice is %c", ch);
					switch(ch)
					{
						case MAIN_PROCTYPE_N:
							pCamInUse->SetMirrorType(MIRROR_NORMAL);
							break;
						case MAIN_PROCTYPE_X:
							pCamInUse->SetMirrorType(MIRROR_X);
							break;
						case MAIN_PROCTYPE_Y:
							pCamInUse->SetMirrorType(MIRROR_Y);
							break;
						case MAIN_PROCTYPE_XY:
							pCamInUse->SetMirrorType(MIRROR_XY);
							break;
						default:
							printf("Bad inut ...\n");
					}
					break;
				}
			case MAIN_CHECK_SPEED:
				{
					unsigned int speed = 0;
					pCamInUse->GetUsbSpeed(speed);
					if(speed==LIBUSB_SPEED_SUPER)
					{
						printf("USB 3.0 device found!\n");
						pCamInUse->SendUsbSpeed2Fpga(USB_SPEED_SUPER);
					}
					else if(speed==LIBUSB_SPEED_HIGH)
					{
						printf("USB 2.0 device found!\n");
						pCamInUse->SendUsbSpeed2Fpga(USB_SPEED_HIGH);
					}
					else
					{
						printf("Device speed unknown!\n");
					}
					break;
				}
			case MAIN_TRIGMODE_SELECT:
				{
					printf("Please input your choice ...\n");
					printf("\
							\'0\':	Auto\n\
							\'1\':	FPGA\n\
							\'2\':	Soft\n\
							\'3\':	Signal\n"\
					      );
					char ch=getchar();
					getchar();
					printf("Your choice is %c", ch);
					switch(ch)
					{
						case MAIN_TRIGMODE_AUTO:
							pCamInUse->SetTrigMode(TRIGMODE_AUTO);
							break;
						case MAIN_TRIGMODE_FPGA:
							pCamInUse->SetTrigMode(TRIGMODE_FPGA);
							break;
						case MAIN_TRIGMODE_SOFT:
							pCamInUse->SetTrigMode(TRIGMODE_SOFT);
							break;
						case MAIN_TRIGMODE_SIGNAL:
							pCamInUse->SetTrigMode(TRIGMODE_SIGNAL);
							break;
						default:
							printf("Bad inut ...\n");
					}
					break;
				}
			case MAIN_BITDEPTH_SELECT:
			{
				printf("FPGA trig 0 or 1 sensor?\n");
				char str[10];
				memset(str,0,sizeof(str));
				fgets(str, 9,stdin);
				unsigned int s=atoi(str);
				printf("Your input is %d \n", s);
				if(s==0||s==1)
				{
					pCamInUse->SetBitDepth(s);
				}
				else
				{
					printf("input error\n");
				}
				break;

			}
			case MAIN_FPGA_TRIG_FREQ_SELECT:
				{
					printf("Please input the trig frequency (Dec, 0~45), make sure the device is in FPGA trig mode\n");
					char str[10];
					memset(str,0,sizeof(str));
					//gets(str);
					fgets(str, 9,stdin);
					unsigned int freq=atoi(str);
					printf("Your input is %d \n", freq);
					pCamInUse->SetFpgaTrigFreq(freq);	
					break;				
				}
#if 0
			case MAIN_AUTO_GAIN_EXPO_SELECT:
				{
					printf("Please input your choice ...\n");
					printf("\
							\'0\':	Manual 	gain, Manual 	expo\n\
							\'1\':	Manual 	gain, Auto 	expo\n\
							\'2\':	Auto	gain, Manual 	expo\n\
							\'3\':	Auto 	gain, Auto	expo\n"\
					      );
					char ch=getchar();
					getchar();
					printf("Your choice is %c\n", ch);
					switch(ch)
					{
						case '0':
							pCamInUse->SetAutoGainExpo(false, false);
							break;
						case '1':
							pCamInUse->SetAutoGainExpo(false, true);
							break;
						case '2':
							pCamInUse->SetAutoGainExpo(true, false);
							break;
						case '3':
							pCamInUse->SetAutoGainExpo(true, true);
							break;
						default:
							printf("Bad inut ...\n");					
					}
					break;
				}
#endif
			case MAIN_EXPO_VALUE_SELECT:
				{
					printf("Please input the expo value (Dec, 0~65536)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned int expoValue=atoi(str);
					printf("Your input is %d \n", expoValue);
					pCamInUse->SetExpoValue(expoValue);
					break;
				}
			case MAIN_GAIN_VALUE_SELECT:
				{
					printf("Please input the gain value (Dec, 0~64)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned int gainValue=atoi(str);
					printf("Your input is %d \n", gainValue);
					pCamInUse->SetGainValue(gainValue);
					break;
				}		

			case MAIN_CAPTURE:
				{
					//cv::namedWindow("disp",CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
					cv::namedWindow("disp",0);
					
					
#ifdef V4L2
					output = open("/dev/video6", O_RDWR);
    if(output < 0) {
        printf("ERROR: could not open output device!\n" );
        return -2;
    }

    // configure params for output device
    struct v4l2_format vid_format;
    memset(&vid_format, 0, sizeof(vid_format));
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

    if (ioctl(output, VIDIOC_G_FMT, &vid_format) < 0) {
		printf("ERROR: unable to set video format!\n");
        return -1;
    }

    vid_format.fmt.pix.width = g_width;
    vid_format.fmt.pix.height = g_height;

    // NOTE: change this according to below filters...
    // Chose one from the supported formats on Chrome:
    // - V4L2_PIX_FMT_YUV420,
    // - V4L2_PIX_FMT_Y16,
    // - V4L2_PIX_FMT_Z16,
    // - V4L2_PIX_FMT_INVZ,
    // - V4L2_PIX_FMT_YUYV,
    // - V4L2_PIX_FMT_RGB24,
    // - V4L2_PIX_FMT_MJPEG,
    // - V4L2_PIX_FMT_JPEG
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;

    vid_format.fmt.pix.sizeimage = g_width*g_height;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(output, VIDIOC_S_FMT, &vid_format) < 0) {
		printf("ERROR: unable to set video format!\n");
        //std::cerr << "ERROR: unable to set video format!\n" << strerror(errno);
        return -1;
    }
	#endif
					cam0.StartCap(g_height,  g_width, Disp);
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

			case MAIN_WRITE_SENSOR_REG:
				{
					printf("Please input the reg address (Hex. Just number, no \'0x\' needed))\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned int iDecRegAddr=hex2dec(str);
					printf("Your input is %x \n", iDecRegAddr);
					printf("Please input the reg value (Hex. Just number, no \'0x\' needed))\n");
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned int iDecRegValue=hex2dec(str);
					printf("Your input is %x \n", iDecRegValue);
					pCamInUse->WrSensorReg(iDecRegAddr, iDecRegValue);
					//goto Badinput;
					break;
				}
			case MAIN_READ_SENSOR_REG:
				{
					printf("Please input the reg address (Hex. Just number, no \'0x\' needed))\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned int iDecRegAddr=hex2dec(str);
					printf("Your input is %x \n", iDecRegAddr);
					unsigned int iValue=0;
					pCamInUse->RdSensorReg(iDecRegAddr, iValue);
					printf("reg value is %04x\n", iValue&0xffff);
					break;
				}
			case MAIN_WRITE_FPGA_REG:
				{
					printf("Please input the reg address (int)\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned char iDecRegAddr=atoi(str);//hex2dec(str);
					printf("Your input is %x \n", iDecRegAddr);
					printf("Please input the reg value (int)\n");
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned char iDecRegValue=atoi(str);//hex2dec(str);
					printf("Your input is %x \n", iDecRegValue);
					pCamInUse->WrFpgaReg(iDecRegAddr, iDecRegValue);
					break;
				}
			case MAIN_READ_FPGA_REG:
				{
					printf("Please input the reg address (Hex. Just number, no \'0x\' needed))\n");
					char str[10];
					memset(str,0,sizeof(str));
					fgets(str, 9,stdin);
					unsigned int iDecRegAddr=hex2dec(str);
					printf("Your input is %x \n", iDecRegAddr);
					unsigned int iValue=0;
					pCamInUse->RdFpgaReg(iDecRegAddr, iValue);
					printf("reg value is %02x\n", iValue&0xff);
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



