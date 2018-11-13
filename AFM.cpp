/*
             define from bcm2835.h                       define from Board DVK511
                 3.3V | | 5V               ->                 3.3V | | 5V
    RPI_V2_GPIO_P1_03 | | 5V               ->                  SDA | | 5V 
    RPI_V2_GPIO_P1_05 | | GND              ->                  SCL | | GND
       RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->                  IO7 | | TX
                  GND | | RPI_GPIO_P1_10   ->                  GND | | RX
       RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->                  IO0 | | IO1
    RPI_V2_GPIO_P1_13 | | GND              ->                  IO2 | | GND
       RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->                  IO3 | | IO4
                  VCC | | RPI_GPIO_P1_18   ->                  VCC | | IO5
       RPI_GPIO_P1_19 | | GND              ->                 MOSI | | GND
       RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->                 MISO | | IO6
       RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->                  SCK | | CE0
                  GND | | RPI_GPIO_P1_26   ->                  GND | | CE1

::if your raspberry Pi is version 1 or rev 1 or rev A
RPI_V2_GPIO_P1_03->RPI_GPIO_P1_03
RPI_V2_GPIO_P1_05->RPI_GPIO_P1_05
RPI_V2_GPIO_P1_13->RPI_GPIO_P1_13
::
*/

#include<opencv2/opencv.hpp>
#include<opencv/highgui.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include "bmp.h"
using namespace cv;

//#include <wiringPi.h>



#include <sys/time.h>




//CS      -----   SPICS  
//DIN     -----   MOSI
//DOUT  -----   MISO
//SCLK   -----   SCLK
//DRDY  -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset


#define w 200                     //image
#define h 200

#define  DRDY  RPI_GPIO_P1_11         //P0
#define  RST  RPI_GPIO_P1_12     //P1
#define	SPICS	RPI_GPIO_P1_15	//P3   ads1256  cs
#define	SPICS1	RPI_GPIO_P1_16	//P4   DAC8552 CS

#define CS_1() bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)

#define CS1_1() bcm2835_gpio_write(SPICS1,HIGH)
#define CS1_0()  bcm2835_gpio_write(SPICS1,LOW)


#define DRDY_IS_LOW()	((bcm2835_gpio_lev(DRDY)==0))

#define RST_1() 	bcm2835_gpio_write(RST,HIGH);
#define RST_0() 	bcm2835_gpio_write(RST,LOW);



/* Unsigned integer types  */
#define uint8_t unsigned char
#define uint16_t unsigned short    
#define uint32_t unsigned long     



#define channel_A   0x30
#define channel_B   0x34

//Trackbar parameter definition
int g_switch_value = 1;  //on-off

//typedef enum {FALSE = 0,TRUE = !FALSE} bool;


/* gain channel?*/
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* GAIN   1 */
	ADS1256_GAIN_2			= (1),	/*GAIN   2 */
	ADS1256_GAIN_4			= (2),	/*GAIN   4 */
	ADS1256_GAIN_8			= (3),	/*GAIN   8 */
	ADS1256_GAIN_16			= (4),	/* GAIN  16 */
	ADS1256_GAIN_32			= (5),	/*GAIN    32 */
	ADS1256_GAIN_64			= (6),	/*GAIN    64 */
}ADS1256_GAIN_E;

/* Sampling speed choice*/
/* 
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* GAIN  */
	ADS1256_DRATE_E DataRate;	/* DATA output  speed*/
	int32_t AdcNow[8];			/* ADC  Conversion value */
	uint8_t Channel;			/* The current channel*/
	uint8_t ScanMode;	/*Scanning mode,   0  Single-ended input  8 channel， 1 Differential input  4 channel*/
}ADS1256_VAR_T;



/*Register definition： Table 23. Register Map --- ADS1256 datasheet Page 30*/
enum
{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10, // xxH
};

/* Command definition： TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};


ADS1256_VAR_T g_tADS1256;
static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/*reset the default values  */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};




void WriteBMP(char*img,const char* filename);
void  bsp_DelayUS(uint64_t micros);
void ADS1256_StartScan(uint8_t _ucScanMode);
static void ADS1256_Send8Bit(uint8_t _data);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);
static void ADS1256_DelayDATA(void);
static uint8_t ADS1256_Recive8Bit(void);
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
static uint8_t ADS1256_ReadReg(uint8_t _RegID);
static void ADS1256_WriteCmd(uint8_t _cmd);
uint8_t ADS1256_ReadChipID(void);
static void ADS1256_SetChannal(uint8_t _ch);
static void ADS1256_SetDiffChannal(uint8_t _ch);
static void ADS1256_WaitDRDY(void);
static int32_t ADS1256_ReadData(void);

int32_t ADS1256_GetAdc(uint8_t _ch);
void ADS1256_ISR(void);
uint8_t ADS1256_Scan(void);
void on_change(int position);


/***************************************************/
void Write_DAC8552(uint8_t channel, uint16_t Data);
uint16_t Voltage_Convert(float Vref, float voltage);


/******************************************
* Functions:save image           *
******************************************/
bool clSaveImage(const char* path, uint8_t image[], int32_t scan_min,int32_t scan_max)
{
	FILE *pFile;
	unsigned short fileType;
	ClBitMapFileHeader bmpFileHeader;
	ClBitMapInfoHeader bmpInfoHeader;
	int step;
	int offset;
	unsigned char pixVal = '\0';
	int i, j;
	ClRgbQuad* quad;

	pFile = fopen(path, "wb");
	if (!pFile)
	{
		return false;
	}

	fileType = 0x4D42;
	fwrite(&fileType, sizeof(unsigned short), 1, pFile);

	
	
	step = w;
	offset = step % 4;
	if (offset != 4)
	{
		step += 4 - offset;
	}

	bmpFileHeader.bfSize = 54 + 256 * 4 + h*step;
	bmpFileHeader.bfReserved1 = 0;
	bmpFileHeader.bfReserved2 = 0;
	bmpFileHeader.bfOffBits = 54 + 256 * 4;
	fwrite(&bmpFileHeader, sizeof(ClBitMapFileHeader), 1, pFile);

	bmpInfoHeader.biSize = 40;
	bmpInfoHeader.biWidth = w;
	bmpInfoHeader.biHeight = h;
	bmpInfoHeader.biPlanes = 1;
	bmpInfoHeader.biBitCount = 8;
	bmpInfoHeader.biCompression = 0;
	bmpInfoHeader.biSizeImage = 4000;
	bmpInfoHeader.biXPelsPerMeter = 4000;
	bmpInfoHeader.biYPelsPerMeter = 10;
	bmpInfoHeader.biClrUsed = 256;
	bmpInfoHeader.biClrImportant = 1000;
	fwrite(&bmpInfoHeader, sizeof(ClBitMapInfoHeader), 1, pFile);

	quad = (ClRgbQuad*)malloc(sizeof(ClRgbQuad) * 256);
	for (i = 0; i<128; i++)
	{
		quad[i].rgbBlue = 0;
		quad[i].rgbGreen = i;
		quad[i].rgbRed = 2 * i;
		quad[i].rgbReserved = 0;
	}
	for (i = 128; i<256; i++)
	{
		quad[i].rgbBlue = 2 * (i - 128) + 1;;
		quad[i].rgbGreen = i;
		quad[i].rgbRed = 255;
		quad[i].rgbReserved = 0;
	}
	fwrite(quad, sizeof(ClRgbQuad), 256, pFile);
	free(quad);

	for (i = h - 1; i>-1; i--)
	{
		for (j = 0; j<w; j++)
		{
			pixVal = image[i*w+j];
			fwrite(&pixVal, sizeof(unsigned char), 1, pFile);
		}
                if(offset!=0)
                {
                	for(j=0;j<4-offset;j++)
                        {
                            pixVal=0;
                            fwrite(&pixVal, sizeof(unsigned char), 1, pFile);
                        }
                }

	}
	fclose(pFile);

	return true;
}

/******************************************
 * Functions:show switch change           *
 ******************************************/
void on_change(int position)
{
	if(position == 1)
		printf("On\n");
      else
           printf("Off\n");
}


/***************************************************
name:Writebmp
****************************************************/
void WriteBMP(char*img,const char* filename)
{
    int l=(w*3+3)/4*4;
    int bmi[]= {l*h+54,0,54,40,w,h,1|3*8<<16,0,l*h,0,0,100,0};
    FILE *fp = fopen(filename,"wb");
    fprintf(fp,"BM");
    fwrite(&bmi,52,1,fp);
    fwrite(img,1,l*h,fp);
    fclose(fp);
}



void  bsp_DelayUS(uint64_t micros)
{
    bcm2835_delayMicroseconds(micros);
}

/*
*********************************************************************************************************
*	name: bsp_InitADS1256
*	function: Configuration of the STM32 GPIO and SPI interface，The connection ADS1256
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/


void bsp_InitADS1256(void)
{
#ifdef SOFT_SPI
	CS_1();
	SCK_0();
	DI_0();
#endif

//ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS);	/* 配置ADC参数： 增益1:1, 数据输出速率 1KHz */
}




/*
*********************************************************************************************************
*	name: ADS1256_StartScan
*	function: Configuration DRDY PIN for external interrupt is triggered
*	parameter: _ucDiffMode : 0  Single-ended input  8 channel， 1 Differential input  4 channe
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_StartScan(uint8_t _ucScanMode)
{
	g_tADS1256.ScanMode = _ucScanMode;
	/* 开始扫描前, 清零结果缓冲区 */
	{
		uint8_t i;

		g_tADS1256.Channel = 0;

		for (i = 0; i < 8; i++)
		{
			g_tADS1256.AdcNow[i] = 0;
		}
	}

}

/*
*********************************************************************************************************
*	name: ADS1256_Send8Bit
*	function: SPI bus to send 8 bit data
*	parameter: _data:  data
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data)
{

	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}

/*
*********************************************************************************************************
*	name: ADS1256_CfgADC
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain:gain 1-64
*                      _drate:  data  rate
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* Storage ads1256 register configuration parameters */

		/*Status register define
			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

			Bit 3 ORDER: Data Output Bit Order
				0 = Most Significant Bit First (default)
				1 = Least Significant Bit First
			Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
			byte first. The ORDER bit only controls the bit order of the output data within the byte.

			Bit 2 ACAL : Auto-Calibration
				0 = Auto-Calibration Disabled (default)
				1 = Auto-Calibration Enabled
			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
			values.

			Bit 1 BUFEN: Analog Input Buffer Enable
				0 = Buffer Disabled (default)
				1 = Buffer Enabled

			Bit 0 DRDY :  Data Ready (Read Only)
				This bit duplicates the state of the DRDY pin.

			ACAL=1  enable  calibration
		*/
		//buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer
        buf[0] = (0 << 3) | (1 << 2) | (0 << 1);  // The internal buffer is prohibited

        //ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;	

		/*	ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 μ A
				10 = Sensor Detect Current = 2 μ A
				11 = Sensor Detect Current = 10μ A
				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
				000 = 1 (default)
				001 = 2
				010 = 4
				011 = 8
				100 = 16
				101 = 32
				110 = 64
				111 = 64
		*/
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	

		CS_0();	/* SPI片选 = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* Write command register, send the register address */
		ADS1256_Send8Bit(0x03);			/* Register number 4,Initialize the number  -1*/

		ADS1256_Send8Bit(buf[0]);	/* Set the status register */
		ADS1256_Send8Bit(buf[1]);	/* Set the input channel parameters */
		ADS1256_Send8Bit(buf[2]);	/* Set the ADCON control register,gain */
		ADS1256_Send8Bit(buf[3]);	/* Set the output rate */

		CS_1();	/* SPI  cs = 1 */
	}

	bsp_DelayUS(50);
}


/*
*********************************************************************************************************
*	name: ADS1256_DelayDATA
*	function: delay
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(10);	/* The minimum time delay 6.5us */
}




/*
*********************************************************************************************************
*	name: ADS1256_Recive8Bit
*	function: SPI bus receive function
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteReg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/*Write command register */
	ADS1256_Send8Bit(0x00);		/*Write the register number */

	ADS1256_Send8Bit(_RegValue);	/*send register value */
	CS_1();	/* SPI   cs = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadReg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* Write command register */
	ADS1256_Send8Bit(0x00);	/* Write the register number */

	ADS1256_DelayDATA();	/*delay time */

	read = ADS1256_Recive8Bit();	/* Read the register values */
	CS_1();	/* SPI   cs  = 1 */

	return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteCmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPI   cs = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI  cs  = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadChipID
*	function: Read the chip ID
*	parameter: _cmd : NULL
*	The return value: four high status register
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/*
*********************************************************************************************************
*	name: ADS1256_SetChannal
*	function: Configuration channel number
*	parameter:  _ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}

/*
*********************************************************************************************************
*	name: ADS1256_SetDiffChannal
*	function: The configuration difference channel
*	parameter:  _ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* DiffChannal  AIN0， AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/*DiffChannal   AIN2， AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/*DiffChannal    AIN4， AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/*DiffChannal   AIN6， AIN7 */
	}
}

/*
*********************************************************************************************************
*	name: ADS1256_WaitDRDY
*	function: delay time  wait for automatic calibration
*	parameter:  NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 400000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 400000)
	{
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		
	}
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadData
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
    static uint8_t buf[3];

	CS_0();	/* SPI   cs = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* read ADC command  */

	ADS1256_DelayDATA();	/*delay time  */

	/*Read the sample results 24bit*/
    buf[0] = ADS1256_Recive8Bit();
    buf[1] = ADS1256_Recive8Bit();
    buf[2] = ADS1256_Recive8Bit();

    read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
    read |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
    read |= buf[2];

	CS_1();	/* SPI片选 = 1 */

	/* Extend a signed number*/
    if (read & 0x800000)
    {
	    read |= 0xFF000000;
    }

	return (int32_t)read;
}


/*
*********************************************************************************************************
*	name: ADS1256_GetAdc
*	function: read ADC value
*	parameter:  channel number 0--7
*	The return value:  ADC vaule (signed number)
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch)
{
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	iTemp = g_tADS1256.AdcNow[_ch];

	return iTemp;
}

/*
*********************************************************************************************************
*	name: ADS1256_ISR
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	if (g_tADS1256.ScanMode == 0)	/*  0  Single-ended input  8 channel， 1 Differential input  4 channe */
	{

		ADS1256_SetChannal(g_tADS1256.Channel);	/*Switch channel mode */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		/*if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();	
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	
		}

		if (++g_tADS1256.Channel >= 8)
		{
			g_tADS1256.Channel = 0;
		}*/
		g_tADS1256.AdcNow[0] = ADS1256_ReadData(); //zijixiede

	}
	else	/*DiffChannal*/
	{
		
		ADS1256_SetDiffChannal(g_tADS1256.Channel);	/* change DiffChannal */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[3] = ADS1256_ReadData();	
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	
		}

		if (++g_tADS1256.Channel >= 4)
		{
			g_tADS1256.Channel = 0;
		}
	}
}

/*
*********************************************************************************************************
*	name: ADS1256_Scan
*	function: 
*	parameter:NULL
*	The return value:  1
*********************************************************************************************************
*/
uint8_t ADS1256_Scan(void)
{
	if (DRDY_IS_LOW())
	{
		ADS1256_ISR();
		return 1;
	}

	return 0;
}
/*
*********************************************************************************************************
*	name: Write_DAC8552
*	function:  DAC send data 
*	parameter: channel : output channel number 
*			   data : output DAC value 
*	The return value:  NULL
*********************************************************************************************************
*/
void Write_DAC8552(uint8_t channel, uint16_t Data)
{
    uint8_t i;

	 CS1_1() ;
	 CS1_0() ;
      bcm2835_spi_transfer(channel);
      bcm2835_spi_transfer((Data>>8));
      bcm2835_spi_transfer((Data&0xff));  
      CS1_1() ;
}
/*
*********************************************************************************************************
*	name: Voltage_Convert
*	function:  Voltage value conversion function
*	parameter: Vref : The reference voltage 3.3V or 5V
*			   voltage : output DAC value 
*	The return value:  NULL
*********************************************************************************************************
*/
uint16_t Voltage_Convert(float Vref, float voltage)
{
	uint16_t _D_;
	_D_ = (uint16_t)(65536 * voltage / Vref);
    
	return _D_;
}

/*
*********************************************************************************************************
*	name: main
*	function:  
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/

int  main(int argc, char* argv[])
{
    int32_t adc;
    int32_t volt[h][w];
    uint8_t i;
    int32_t iTemp;
    float voltmax=3.3;
    uint8_t count=1;
    uchar image_tmp;
    int range=32;

    uint16_t scanx,scany;
    uint8_t image[w*h*3];
    uint8_t image1[h*w];
    float DAx=0.0;
    float DAy=0.0;
    float scanstep=voltmax/w;
    float scanstart=0.0;
    uint16_t scanrangex=w;
    uint16_t scanrangey=h;
    int32_t scanmin=5000000;
    int32_t scanmax=0;
    int32_t scanminold=5000000;
    int32_t scanmaxold=0;
    int32_t scanminlevel;      //set a minlevel for the data,if data<minlevel,data=minlevel
    int32_t scanmaxlevel;      //set a maxlevel for the data,if data>maxlevel,data=maxlevel
    int32_t temp;
    bool scan10lineflag=0;
    float cer=0.08;      //nonlinear of ceramic
    //float contr=0.25;
    //float contr1=0.80;
    int contr=25;
    char name[100];
    int cer1=85;
    int delay=30;
    int inc_x=100;
    int inc_y=100;
	bool save_flag = false;
	char* fileName;

    CvSize imgSize;
    imgSize.width = w;
    imgSize.height = h;
    IplImage*img=cvCreateImage(imgSize,IPL_DEPTH_8U,3);
    cvNamedWindow("AFM",0);
    cvNamedWindow("setting",1);

    cvCreateTrackbar("off","setting",&g_switch_value,1,on_change);
    cvCreateTrackbar("incx","setting",&inc_x,200,NULL);
    cvCreateTrackbar("incy","setting",&inc_y,200,NULL);
    //cvCreateTrackbar("inc","setting",&inc,200,NULL);
    cvCreateTrackbar("contrast","setting",&contr,80,NULL);
    cvCreateTrackbar("range","setting",&range,35,NULL);
    cvCreateTrackbar("delay","setting",&delay,1000,NULL);
    cvCreateTrackbar("ceramic","setting",&cer1,200,NULL);


    for(int y=0;y<img->height;y++)
    {
        uchar*ptr=(uchar*)(img->imageData+y*img->widthStep);
        for(int x=0;x<img->width;x++)
        {
            ptr[3*x]=155;//.....R.
            ptr[3*x+1]=155;//.....G.
            ptr[3*x+2]=155;//.....B.
        }
    }
		
    if (!bcm2835_init())
        return 1;
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // The default
    bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
    bcm2835_gpio_write(SPICS, HIGH);
    bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);

    bcm2835_gpio_write(SPICS1, HIGH);
    bcm2835_gpio_fsel(SPICS1, BCM2835_GPIO_FSEL_OUTP);//


    ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_15000SPS);
    ADS1256_StartScan(0);
    /*while(1)
    {
        printf("ceramic coefficient(0~0.10):");
        scanf("%f",&cer);

        if(cer>=0 && cer<=cer1)
        {
            break;

        }
        else{
           printf("error!\n");
        }

    }*/
    //cer=0.08;

    /*while(1)
    {
        printf("contrast factor(0~0.80):");
        scanf("%f",&contr);
        if(contr<0 || contr>contr1)
        {
            printf("error!\n");
        }
        else{
            break;
        }

    }*/

    /*while(1)
    {
        printf("scan range(0~20000):");
        scanf("%d",&range);
        if(range<0 || range>20000)
        {
            printf("error!\n");
        }
        else{
            break;
        }

    }*/
    //range=4800;
    //printf("delaytime for every line(1~20):");
    //scanf("%d",&delay);
    //delay=1;
    voltmax=range*10.0*5.00/350;
    scanstep=voltmax/w;

    //printf("inclination for x(0~1):");       //adjust the inclination of x
    //scanf("%f",&inc_x);
    //printf("inclination for y(0~1):");
    //scanf("%f",&inc_y);

    Write_DAC8552(0x30, Voltage_Convert(5.0,0.0));    	//Write channel A buffer (0x30)
    Write_DAC8552(0x34, Voltage_Convert(5.0,0.0));    	//Write channel B buffer (0x34)

    while(1)
    {
		if(!cvGetWindowHandle("AFM"))    //if the "x" in the image window is clicked, terminate the program
		{
			break;
		}
		if(!cvGetWindowHandle("setting"))
		{
			break;
		}
        if(g_switch_value)
        {

            voltmax=range*10.0*5.00/350;
            scanstep=voltmax/w;
            cer=(float)cer1/1000;

            DAx=0.0;
            Write_DAC8552(0x30, Voltage_Convert(5.0,DAx));    	//Write channel A buffer (0x30)

            DAy=0.0;
            Write_DAC8552(0x34, Voltage_Convert(5.0,DAy));    	//Write channel B buffer (0x34)
                        //while((ADS1256_Scan() == 0));

            if(scan10lineflag==0)   //scan 10 line at the beginning
            {
                for(scany=0;scany<20;scany++)
                {
                    for(scanx=0;scanx<scanrangex;scanx++)
                    {
                        while((ADS1256_Scan() == 0));
                        adc=ADS1256_GetAdc(0);
                        volt[scany][scanx]=(adc * 100) / 167;
                        //volt[scany][scanx]=(ADS1256_GetAdc(0) * 100) / 167;      //AD0 input
                        DAx=DAx+scanstep;
                        DAx=DAx-DAx*scanstep*cer;         //correct nonlinear

                        Write_DAC8552(0x30, Voltage_Convert(5.0,DAx));    	//Write channel A buffer (0x30)
						if(scanx>=10 && scany>=10 && scanx<=190)
						{
                        if(volt[scany][scanx]>scanmax)
                            scanmax=volt[scany][scanx];
                        if(volt[scany][scanx]<scanmin)
                            scanmin=volt[scany][scanx];
						}

                    }

                    DAx=0.0;                                             //scanx return to 0.0
                    Write_DAC8552(0x30, Voltage_Convert(5.0,DAx));    	//Write channel A buffer (0x30)
					cvWaitKey(delay);

                    DAy=DAy+scanstep;             //scany step
                    DAy=DAy-DAy*scanstep*cer;         //correct nonlinear
                    Write_DAC8552(0x34, Voltage_Convert(5.0,DAy));    	//Write channel B buffer (0x34)
			  
                }
                scan10lineflag=1;
                DAx=0.0;                                             //scanx return to 0.0
                Write_DAC8552(0x30, Voltage_Convert(5.0,DAx));    	//Write channel A buffer (0x30)

                DAy=0.0;             //scany return 0
                Write_DAC8552(0x34, Voltage_Convert(5.0,DAy));    	//Write channel B buffer (0x34)

            }

            printf("scanmin =%f, scanmax =%f\n",(float)scanmin/1000000,(float)scanmax/1000000);
            //scanminlevel=scanmin+contr*(scanmax-scanmin);
            //scanmaxlevel=scanmax;
            scanminold=scanmin;
            scanmaxold=scanmax;

            scanmin=5000000;
            scanmax=0;

            for(scany=0;scany<scanrangey;scany++)
            {
                uchar*ptr=(uchar*)(img->imageData+ scany*img->widthStep);
                if(g_switch_value==0)
                    break;
                scanminlevel=scanminold+contr*(scanmaxold-scanminold)/100;
                scanmaxlevel=scanmaxold;

				if(!cvGetWindowHandle("AFM"))
				{
	    			break;
				}
				if(!cvGetWindowHandle("setting"))
				{
	    			break;
				}
                for(scanx=0;scanx<scanrangex;scanx++)
                {
                    while((ADS1256_Scan() == 0));
                    adc=ADS1256_GetAdc(0);
                    volt[scany][scanx]=(adc * 100) / 167;    //read from AD0

                    DAx=DAx+scanstep;   //x move one step
                    DAx=DAx-DAx*scanstep*cer;         //correct nonlinear
                    Write_DAC8552(0x30, Voltage_Convert(5.0,DAx));    	//Write channel A buffer (0x30)

                    volt[scany][scanx]=volt[scany][scanx]+(inc_x-100)*(float)((200-scanx)*50);      //adjust the inclination of x
                    volt[scany][scanx]=volt[scany][scanx]+(inc_y-100)*(float)((200-scany)*50);      //adjust the inclination of y
					if(scanx>=10 && scanx<=190 && scany>=10 && scany<=190){
						if(volt[scany][scanx]>scanmax)
							scanmax=volt[scany][scanx];
						if(volt[scany][scanx]<scanmin)
							scanmin=volt[scany][scanx];
					}
                    if(scanmax==scanmin)
                    {
                        image_tmp=255;
                    }
                    else{
                        if(volt[scany][scanx]>scanmaxlevel)
                        {
                            image_tmp=255;
                        }else if (volt[scany][scanx]<scanminlevel){
                            image_tmp=0;
                        }else {
                            image_tmp=255*(volt[scany][scanx]-scanminlevel)/(scanmaxlevel-scanminlevel);
									//image_tmp=65;
                        }
                                //image_tmp=255*(volt[scany][scanx]-scanmin)/(scanmax-scanmin);
                                // image[i]=255*(volt[scany][scanx]-scanmin)/5000000;
                    }

					image1[scany*w+scanx] = image_tmp;
                    if(image_tmp>=128)
                    {
                        ptr[3*scanx]=2*(image_tmp-128)+1;
                        ptr[3*scanx+1]=image_tmp;
                        ptr[3*scanx+2]=255;
                                
                    }
                    else
                    {
                        ptr[3*scanx]=0;
                        ptr[3*scanx+1]=image_tmp;
                        ptr[3*scanx+2]=2*image_tmp;
                    }
                }
                DAx=0.0;                                             //x return to 0.0
                Write_DAC8552(0x30, Voltage_Convert(5.0,DAx));    	//Write channel A buffer (0x30)

                DAy=DAy+scanstep;             //y move one step
                DAy=DAy-DAy*scanstep*cer;         //correct nonlinear
                Write_DAC8552(0x34, Voltage_Convert(5.0,DAy));    	//Write channel B buffer (0x34)

                cvShowImage("AFM",img);    //show image for every line
                cvWaitKey(delay);   //wait at least 1ms to show image

            }
            DAx=0.0;                                             //scanx return to 0.0
            Write_DAC8552(0x30, Voltage_Convert(5.0,DAx));    	//Write channel A buffer (0x30)

            DAy=0.0;                                             //scany return 0
            Write_DAC8552(0x34, Voltage_Convert(5.0,DAy));    	//Write channel B buffer (0x34)
			if(!cvGetWindowHandle("AFM"))
			{
				break;
			}
			if(!cvGetWindowHandle("setting"))
			{
				break;
			}
			sprintf(name,"/home/pi/Desktop/AFM/%d.bmp",count);
                        fileName=name;
			save_flag = clSaveImage(fileName, image1,scanmin,scanmax);//new imagesave 2018/11/13


            
            //cvSaveImage("saveimage",img,0);
            //Mat imgnew;
            //imgnew=cvarrToMat(img);
            //imwrite(name,imgnew);// 20180622
            count++;
            if(count>=21)
            {
                count=1;
            }
            if(cvWaitKey(1)==27)
                break;

        }
        else{
            cvShowImage("AFM",img);
            if(cvWaitKey(1)==27)
                break;
        }
    }
    bcm2835_spi_end();
    bcm2835_close();
	
    return 0;

}
