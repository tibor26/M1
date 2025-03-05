#include "stm32g0xx_hal.h"
#include "TLC6983.h"
#include "constant.h"
//=======================================================
uint32_t thisRxSPIBuffer[16];
uint32_t TxSPIBuffer[32]; // = { 0xff }; // 256 bytes  // power save mode
uint32_t RxSPIBuffer[32]; // = { 0xff }; // 256 bytes
uint32_t SRAMbuffer0[0x480];

uint8_t pixelBuffer[DisplayHeight][DisplayChipWidth];

uint8_t thisBitposition;
uint8_t *dataPtr;
uint32_t VsyncBuffer[4];
uint8_t VSyncBufferLength;
volatile uint8_t VSyncBufferSent = 0;
uint16_t SRAMbuffer0Length;
volatile uint8_t SRAMbuffer0Complete = 0;
uint8_t TLC6983Enabled = 0;
static volatile uint8_t brightness_update = 0;

uint8_t newBitPosition = 0;
uint8_t* newDataPtr;

extern SPI_HandleTypeDef hspi2;
extern IWDG_HandleTypeDef hiwdg;
//==========================================================
void InitialiseTLC6983(void)
{
    uint16_t i;
    
    HAL_GPIO_WritePin(SPI2_MOSI_GPIO_Port, SPI2_MOSI_Pin, GPIO_PIN_RESET);
     	
    TxSPIBuffer[0] = 0xFF;
    TxSPIBuffer[1] = 0xFF;
    TxSPIBuffer[2] = 0xFF;
    TxSPIBuffer[3] = 0xFF;
    TxSPIBuffer[4] = 0xFF;
    TxSPIBuffer[5] = 0xFF;
    
    writeChipIndex2();          // First write is only required because device is not in IDLE state yet
    for(i=0; i<50; i++)
    {
      CLR_WDT();	        // Clear watch-dog   
    }
  
    writeChipIndex2();          //Write chip index.
    for(i=0; i<50; i++)
    {
      CLR_WDT();	// Clear watch-dog   
    } 
     
    write_new_Register(Write_FC0, newFC0);
    for(i=0; i<50; i++)
    {
      CLR_WDT();	// Clear watch-dog   
    } 
	
    write_new_Register(Write_FC1, newFC1);
    for(i=0; i<50; i++)
    {
      CLR_WDT();	// Clear watch-dog   
    } 

    write_new_Register(Write_FC2, newFC2);
    for(i=0; i<50; i++)
    {
      CLR_WDT();	// Clear watch-dog   
    } 
	
    write_new_Register(Write_FC3, newFC3);
    for(i=0; i<50; i++)
    {
      CLR_WDT();	// Clear watch-dog   
    } 
    write_new_Register(Write_FC4, newFC4);
   
    saveVSyncFrame(VsyncBuffer);
    
    TLC6983Enabled = 1;
}

void writeStringtoScreen(void)
{
	// row decoder
	// set N columns 0-7 A columns 8-15

	uint32_t* SRamPtr;

	newDataPtr = (uint8_t *)SRAMbuffer0; // ENGR:CT
	SRamPtr = SRAMbuffer0; 
	
    
	uint32_t* vsyncPtr = VsyncBuffer; //ff3fbcea
	*SRamPtr++ =   *vsyncPtr; // copy Vsync command buffer.
	*SRamPtr++ =  0xffffffff; // terminate Vsync command.
	newDataPtr = (uint8_t *)SRamPtr; // ENGR:CT
    
	//int length;
	//uint64_t RAMpattern = 0x00000000FFFF; //   1001 0010 0100 1001 0010  // 924924924924
	//int pixelcounter = 0;
	
	uint8_t * pixelPtr = (uint8_t *)pixelBuffer; // ENGR:CT cast
	for (uint8_t j = 0; j < 8; j++)	// rows   lines
	{	   	    
		for (uint8_t k = 0; k < 16; k++) {
			newBitPosition = 3;
			uint8_t thisByte = 0xdF; // MSB first then Low   // start bit
			*newDataPtr = thisByte;
			
			shift16intothisBuffer(WriteSRAM); // 4 bytes for end of frame

            //  Columns Pixels along the row
            uint8_t Pixel[3];
            if (k > TLC6983_SWITCH_COUNT)
            {
                Pixel[0] = *pixelPtr++;
                Pixel[1] = *pixelPtr++;
                Pixel[2] = *pixelPtr++;
            }
            else
            {
                Pixel[2] = *pixelPtr++;
                Pixel[1] = *pixelPtr++;
                Pixel[0] = *pixelPtr++;
            }
            for (uint8_t i = 0; i < 3; i++)
            {
                if (Pixel[i])
                {
                    shift16intothisBuffer(0xFFFF);
                }
                else
                {
                    shift16intothisBuffer(0x0000);
                }
            }
    
			newAdd2EndBits();
    
			for (uint8_t i = 0; i < 4; i++)
				*newDataPtr++ = 0xff; // end of frame signal
		}
	}
        SRAMbuffer0Length = newDataPtr - (uint8_t*) SRAMbuffer0;
        SRAMbuffer0Complete = 2; // must send it twice

}


void shift16intoBuffer(uint16_t data16) // with checkbit
{
    uint8_t blankData = 0xff;
    uint8_t startBitMask = blankData << (8 - thisBitposition);
    uint8_t startBitData = *dataPtr & startBitMask;
    uint8_t thisShiftedData = (data16 >> (8 + thisBitposition)) & 0xff;
    *dataPtr++ = thisShiftedData | startBitData;
    *dataPtr++ = (data16 >>  thisBitposition) & 0xff;
    uint8_t bottomMask = 0xff >> (8 - thisBitposition);
    *dataPtr = ((data16 & bottomMask)  << (8 - thisBitposition)) & 0xff;
    uint8_t checkbit = (~data16) & 1;
    thisBitposition++;
    thisBitposition &= 7;
    *dataPtr |= checkbit << (8 - thisBitposition);
    *dataPtr |= 0xff >> thisBitposition;
}


void shift16intothisBuffer(uint16_t data16) // with checkbit
{    
    uint8_t blankData = 0xff;
    uint8_t startBitMask = blankData << (8 - newBitPosition);
    uint8_t startBitData = *newDataPtr & startBitMask;
    uint8_t thisShiftedData = (data16 >> (8 + newBitPosition)) & 0xff;
    *newDataPtr++ = thisShiftedData |  startBitData;
    *newDataPtr++ = (data16 >>  newBitPosition) & 0xff;
    uint8_t bottomMask = 0xff >> (8 - newBitPosition);
    *newDataPtr = ((data16 & bottomMask)  << (8 - newBitPosition)) & 0xff;
    uint8_t checkbit = (~data16) & 1;
    newBitPosition++;
    newBitPosition &= 7;
    *newDataPtr |= checkbit << (8 - newBitPosition);
    *newDataPtr |= 0xff >> newBitPosition;   
}


void addEndBits2(void)
{
	
    // if thisbitposition < 7 then there is already 2 end bits included.
    //otherwise we will need to add another byte 
	
    //uint8_t bottomMask = 0xff >> thisBitposition;
    *dataPtr |= 0xff >> thisBitposition;
	
    if (thisBitposition > 6)
    {
        dataPtr++;
        *dataPtr = 0xff;
    }
    thisBitposition += 2;
    thisBitposition &= 7;
    dataPtr++; // so the count includes the end bits	
}


void newAdd2EndBits(void)
{
    // add 2 end bits
    *newDataPtr |= 0xff >> newBitPosition;
	
    if (newBitPosition > 6)
    {
        newDataPtr++;
        *newDataPtr = 0xff;
    }
    newBitPosition += 2;
    newBitPosition &= 7;
    newDataPtr++; // so the count includes the end bits	     
} 


void saveVSyncFrame(uint32_t* buffer)
{
    uint16_t commandID = VSync;
    
    uint32_t*ptr =	buffer;
    for (uint8_t i = 0; i < 3; i++)
        *ptr++ = 0xffffffff;      // end of frame signal
	
    newDataPtr = (uint8_t *)VsyncBuffer; // should be ff3fbcea
    newBitPosition = 2;
    uint8_t thisByte = 0xBF; // MSB first then Low   // start bit
    *newDataPtr = thisByte;
    shift16intothisBuffer(commandID); // 4 bytes for end of frame
    newAdd2EndBits();
    VSyncBufferLength = newDataPtr - (uint8_t*)VsyncBuffer + 5;
}


void write_new_Register(uint16_t commandID,uint64_t Ldata)
{

	uint32_t*ptr =	TxSPIBuffer;
	for (uint8_t i = 0; i < 8; i++)
		*ptr++ = 0xffffffff;                                            //set TxSPIBuffer

	dataPtr = (uint8_t *)TxSPIBuffer;                                       // ENGR:CT
	// start bit has low
	uint8_t thisByte = 0xBF;                                                // MSB first then Low
	thisBitposition = 2;
	*dataPtr = thisByte;
                    
	shift16intoBuffer(commandID);
		
	uint16_t first16bits = (long long)Ldata >> 32 & 0xFFFF;                            
	shift16intoBuffer(first16bits);
	uint16_t second16bits = (long long)Ldata >> 16 & 0xFFFF;
	shift16intoBuffer(second16bits);
	uint16_t third16bits = (long long)Ldata  & 0xFFFF;
	shift16intoBuffer(third16bits);
	addEndBits2();
	*dataPtr++ = 0xff; // to be sure the last bits are high
	*dataPtr++ = 0xff; // to be sure the last bits are high
		
    int length = (dataPtr - (uint8_t*)TxSPIBuffer + 1) & 0xfe;
    while (hspi2.hdmatx->Instance->CNDTR)
    {
        CLR_WDT();	// Clear watch-dog
    }
    // reload DMA manually
    CLEAR_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
    CLEAR_BIT(SPI2->CR2, SPI_CR2_TXDMAEN);
    hspi2.hdmatx->Instance->CNDTR = length + 1;
    hspi2.hdmatx->Instance->CMAR = (uint32_t)TxSPIBuffer;
    SET_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
    SET_BIT(SPI2->CR2, SPI_CR2_TXDMAEN);
}


void writeChipIndex2(void)
{
	dataPtr = (uint8_t *)TxSPIBuffer;                                       // ENGR:CT
	// start bit has low
	uint8_t thisByte = 0xBF;                                                // MSB first then Low
	thisBitposition = 2;
	*dataPtr = thisByte;
	uint16_t commandID = WriteChipIndex;
	shift16intoBuffer(commandID);
	
	addEndBits2();
	
    int length = dataPtr - (uint8_t*)TxSPIBuffer;
    while (hspi2.hdmatx->Instance->CNDTR)
    {
        CLR_WDT();	// Clear watch-dog
    }
    if (hspi2.hdmatx->Instance->CCR & DMA_CCR_EN)  // DMA already set up, just reload
    {
        CLEAR_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN);
        CLEAR_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
        hspi2.hdmatx->Instance->CNDTR = length + 1;
        hspi2.hdmatx->Instance->CMAR = (uint32_t)TxSPIBuffer;
        SET_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
        SET_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN);
    }
    else  // first transfer, init DMA using HAL
    {
        HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)TxSPIBuffer, length + 1);
    }
}


void fillPowerTest(uint16_t data)
{
	uint32_t* SRamPtr;
	
	newDataPtr = (uint8_t *)SRAMbuffer0; // ENGR:CT
	SRamPtr = SRAMbuffer0;
	
	uint32_t* vsyncPtr = VsyncBuffer; //ff3fbcea
	*SRamPtr++ = *vsyncPtr; // copy Vsync command buffer.
	*SRamPtr++ = 0xffffffff; // terminate Vsync command.
	newDataPtr = (uint8_t *)SRamPtr; // ENGR:CT

	for (uint8_t j = 0; j < 8; j++)	// rows   lines
	{
		for (uint8_t k = 0; k < 16; k++)
		{
			newBitPosition = 3;
			uint8_t thisByte = 0xdF; // MSB first then Low   // start bit
			*newDataPtr = thisByte;

			shift16intothisBuffer(WriteSRAM); // 4 bytes for end of frame
			
			shift16intothisBuffer(data);
			shift16intothisBuffer(data);
			shift16intothisBuffer(data);

			newAdd2EndBits();

			for (uint8_t i = 0; i < 4; i++)
				*newDataPtr++ = 0xff; // end of frame signal
		}
	}
	SRAMbuffer0Length = newDataPtr - (uint8_t*)SRAMbuffer0;
	SRAMbuffer0Complete = 2; // must send it twice
}

/*
 * Turn on only used leds
 * mainly exclude top and bottom row
 * 4 columns on left, 3 on right
 * add 5 leds on top row used by "hello"
 **/
void fillOnlyUsedLeds(void)
{
    for (uint8_t i = 0; i < DisplayHeight; i++)
    {
        for (uint8_t j = 0; j < DisplayChipWidth; j++)
        {
            if (1 <= i && i <= 6 && 4 <= j && j <= 32)
            {
                pixelBuffer[i][j] = 1;
            }
            else
            {
                pixelBuffer[i][j] = 0;
            }
        }
    }
    // leds on top row used by "hello" display
    pixelBuffer[0][6]  = 1;
    pixelBuffer[0][18] = 1;
    pixelBuffer[0][19] = 1;
    pixelBuffer[0][22] = 1;
    pixelBuffer[0][23] = 1;
    // other unused leds
    pixelBuffer[2][5] = 0;
    pixelBuffer[5][5] = 0;
    writeStringtoScreen();
}

void dotmatrix_set_brightness(uint32_t brightness)
{
    if (brightness)
    {
        newFC3 = (0xFFFF000000FFULL & newFC3) | (TLC6983_GLOBAL_BRIGHTNESS << 32) | (brightness << 24) | (brightness << 16) | (brightness << 8);
    }
    else
    {
        newFC3 = (0xFFF8000000FFULL & newFC3);
    }
    brightness_update = 1;
}

void dotmatrix_brighness_update(void)
{
    if (brightness_update)
    {
        brightness_update = 0;
        write_new_Register(Write_FC3, newFC3);
    }
}


const uint64_t newFC0	=
					  (long long)(0x01 & 0x03) << 46	// MOD_Size 1		  2 bits   Module size is 1 unit				
					| (long long)(0x01 & 0x03) << 44	// reserved			  2 bits
					| (long long)(0x00 & 0x07) << 41	// Grp_Delay_B		  3 bits
					| (long long)(0x00 & 0x07) << 38	// Grp_Delay_G		  3 bits
					| (long long)(0x00 & 0x07) << 35	// Grp_Delay_R		  3 bits
					| (long long)(0x00 & 0x01) << 34	// reserved			  1 bits
					| (long long)(0x00 & 0x03) << 32	// reserved		adj	  2 bits
			
					| (long long)(0x0F & 0x0F) << 28	// Freq Mult		  4 bits   GCLK multiplier set to 16x			
					| (long long)(0x00 & 0x01) << 27	// Freq Mode Low	  1 bits   Frequency GCLK multiplier mode set to 40-80MHz
					| (long long)(0x00 & 0x07) << 24	// reserved			  3 bits   
					| (long long)(0x02 & 0x07) << 21	// SubP_num			  3 bits   sub period is set to 32
					| (long long)(0x07 & 0x1F) << 16	// Scan_num			  5 bits   scan 8 lines is 0x07     
			
					| (long long)(0x00 & 0x01) << 15	// Lodrm_En			  1 bit   disable led open removal function    
					| (long long)(0x00 & 0x03) << 13	// Psp_Mod			  2 bits  power saving plus mode set to disable
					| (long long)(0x01 & 0x01) << 12	// Psp_En			  1 bits  enable power saving mode
					| (long long)(0x00 & 0x07) << 9		// reserved			  3 bits  
					| (long long)(0x01 & 0x01) << 8		// PDC_En			  1 bits  enable predischarge
					| (long long)(0x00 & 0x07) << 5 	// reserved			  3 bits
					| (long long)(0x00 & 0x1F); // Chip_Num			  5 bits  device 0



const uint64_t newFC1 =	
					  (long long)(0x00 & 0x01) << 47    // reserved			  1 bits   
					| (long long)(0x00 & 0x3F) << 41    // Blk_Adj			  6 bits  Blank Field Adjustment  set to 2 as per spreadsheet Displays in operation sheet TLC calculations
					| (long long)(0x00 & 0x0F) << 37    // Line_Swt			  4 bits  Scan Line Switch Time	set to 6 = 180   // was 0x0a
					| (long long)(0x00 & 0x0F) << 33    // Lg_Enh_B			  4 bits  Low GreyScale Enhancement Blue
					| (long long)(0x00 & 0x0F) << 29    // Lg_Enh_G			  4 bits  Low GreyScale Enhancement Green
					| (long long)(0x00 & 0x0F) << 25    // Lg_Enh_R			  4 bits  Low GreyScale Enhancement Red
					| (long long)(0x00 & 0x1F) << 20    // Lg_Step_B		  5 bits  Adjust Smooth Brightness in Greyscale
					| (long long)(0x00 & 0x1F) << 15    // Lg_Step_G		  5 bits  Adjust Smooth Brightness in Greyscale
					| (long long)(0x00 & 0x1F) << 10    // Lg_Step_R		  5 bits  Adjust Smooth Brightness in Greyscale
					| (long long)(0x3ff & 0x3FF); // Segment Length	 10 bits  Calculation on Excel  0x214  reversed is 00 1010 0001 0x0A1

const uint64_t newFC2 =
					  (long long)(0x00 & 0x1F) << 43    // reserved			5 bits   
					| (long long)(0x00 & 0x01) << 42    // Subp_Max			1 bits  
					| (long long)(0x01 & 0x01) << 41	// ChBImmunity		1 bits 
					| (long long)(0x01 & 0x01) << 40	// ChGImmunity		1 bits 
					| (long long)(0x01 & 0x01) << 39	// ChRImmunity		1 bits 
					| (long long)(0x00 & 0x07) << 36    // reserved			3 bits	
					| (long long)(0x00 & 0x0F) << 32    // Lg_Color_B		4 bits	
					| (long long)(0x00 & 0x0F) << 28    // Lg_Color_G		4 bits	
					| (long long)(0x00 & 0x0F) << 24    // Lg_Color_R		4 bits	
					| (long long)(0x00 & 0x0F) << 20    // Decouple_B		4 bits	
					| (long long)(0x00 & 0x0F) << 16    // Decouple_G		4 bits	
					| (long long)(0x00 & 0x0F) << 12    // Decouple_R		4 bits	
					| (long long)(0x0F & 0x0F) << 8     // V_PDC_B			4 bits	set to 1V  should try 2.1V 0x0F
					| (long long)(0x0F & 0x0F) << 4     // V_PDC_G			4 bits	
					| (long long)(0x0F & 0x0F); // V_PDC_R			4 bits	
				
uint64_t newFC3 =
					  (long long)(0x00 & 0x07) << 45    // LSDVTH_B			  3 bits   
					| (long long)(0x00 & 0x07) << 42    // LSDVTH_G			  3 bits 
					| (long long)(0x00 & 0x07) << 39    // LSDVTH_R			  3 bits 
					| (long long)(0x00 & 0x0F) << 35    // LSD_RM			  4 bits  short removal level 8
					| (long long)(0x00 & 0x07) << 32    // Gbl_Brightness	  3 bits  middle Global Brightness, start at 0 then it's set with TLC6983_GLOBAL_BRIGHTNESS
					| (long long)(0x00 & 0xFF) << 24    // BlueBrightness	  8 bits  almost Maximum  // start at zero then it's increased
					| (long long)(0x00 & 0xFF) << 16    // GrenBrightness	  8 bits  almost Maximum  // to max on startup message fade in
					| (long long)(0x00 & 0xFF) << 8     // RedBrightness	  8 bits  almost Maximum
					| (long long)(0x01 & 0x01) << 7     // ReadBackEnable	  1 bits 
					| (long long)(0x00 & 0x01) << 6     // reserved			  1 bits	
					| (long long)(0x0F & 0x03) << 4     // BlueOpenThresh	  2 bits	
					| (long long)(0x0F & 0x03) << 2		// GreenOpenThresh	  2 bits	
					| (long long)(0x0F & 0x03);			// RedOpenThresh	  2 bits	
												    
const uint64_t newFC4 =
					  (long long)(0x00 & 0x07) << 45    // reserved							 3 bits   
					| (long long)(0x00 & 0x01) << 44    // EnableDecopulingEnhancement		 1 bits 
					| (long long)(0x00 & 0x0F) << 40    // Decouple3						 4 bits  level 9 decoupling
					| (long long)(0x00 & 0x01) << 39	// Decouple2 ON OFF Channels		 1 bits 
					| (long long)(0x00 & 0x0F) << 35	// FirstLineDim						 4 bits  
					| (long long)(0x00 & 0x01) << 34    // Corse Blue BrightCompensation	 1 bits 
					| (long long)(0x00 & 0x01) << 33    // Corse GreenBrightCompensation	 1 bits 
					| (long long)(0x00 & 0x01) << 32    // Corse Red  BrightCompensation	 1 bits 
					| (long long)(0x00 & 0x0F) << 28    // reserved							 4 bits 
					| (long long)(0x00 & 0x03) << 26    // Blue  SlewRate	ON				 2 bits 
					| (long long)(0x00 & 0x03) << 24    // Green SlewRate	ON				 2 bits 
					| (long long)(0x00 & 0x03) << 22    // Red   SlewRate	ON				 2 bits 
					| (long long)(0x00 & 0x01) << 21    // Blue  SlewRate	OFF				 1 bits 
					| (long long)(0x00 & 0x01) << 20    // Green SlewRate	OFF				 1 bits 
					| (long long)(0x00 & 0x01) << 19    // Red   SlewRate	OFF				 1 bits 
					| (long long)(0x00 & 0x01) << 18    // Enable Blue  Fine Compensation	 1 bits 
					| (long long)(0x00 & 0x01) << 17    // Enable Green Fine Compensation	 1 bits 
					| (long long)(0x00 & 0x01) << 16    // Enable Red   Fine Compensation	 1 bits 
					| (long long)(0x00 & 0x01) << 15    // reserved							 1 bits 
					| (long long)(0x00 & 0x01) << 14    // Scan_Reverse						 1 bits	  // non functional in 1 chip solution
					| (long long)(0x00 & 0x3FF) << 4    // reserved							10 bits	
					| (long long)(0x00 & 0x01) << 3     // Imax 10 or 20mA					 1 bits	
					| (long long)(0x00 & 0x03) << 1     // reserved							 2 bits	
					| (long long)(0x01 & 0x01);			// Enable last SOUT Function		 1 bits	




