#pragma once
#define TLC6983
#define DisplayHeight 8
#define DisplayWidth 36
#define DisplayChipWidth 48
#define TLC6983_GLOBAL_BRIGHTNESS  0x03ULL
#define TLC6983_SWITCH_COUNT       7

#define Write_FC0  0xAA00  
#define Write_FC1  0xAA01  
#define Write_FC2  0xAA02  
#define Write_FC3  0xAA03  
#define Write_FC4  0xAA04  
#define Write_FC10 0xAA0A  
#define Write_FC11 0xAA0B
#define Read_FC0   0xAA60	
#define Read_FC1   0xAA61  
#define Read_FC2   0xAA62
#define Read_FC3   0xAA63
#define Read_FC4   0xAA64
#define Read_FC10  0xAA6A
#define Read_FC11  0xAA6B
#define Read_FC12  0xAA6C
#define Read_FC13  0xAA6D
#define Read_FC14  0xAA6E
#define Read_FC15  0xAA6F
#define WriteChipIndex	0xAA10
#define Read_ChipIndex	0xAA70
#define VSync			0xAAF0
#define SoftReset		0xAA80
#define WriteSRAM		0xAA30

void InitialiseTLC6983(void);
void writeChipIndex2(void);
void shift16intoBuffer(uint16_t data16);
void write_new_Register(uint16_t commandID, const uint64_t Ldata);
void addEndBits2(void);

void shift16intothisBuffer(uint16_t data16);
void saveVSyncFrame(uint32_t* buffer);
void newAdd2EndBits(void);

void InitialiseVSyncReadRegisters(void);
void fillPowerTest(uint16_t data);
void fillOnlyUsedLeds(void);

void dotmatrix_set_brightness(uint32_t brightness);
void dotmatrix_brighness_update(void);

extern uint32_t TxSPIBuffer[];
extern uint32_t RxSPIBuffer[];
extern uint32_t VsyncBuffer[4];
extern uint8_t VSyncBufferLength;
extern volatile uint8_t VSyncBufferSent;
extern uint16_t SRAMbuffer0Length;
extern volatile uint8_t SRAMbuffer0Complete;
extern uint8_t TLC6983Enabled;
extern uint32_t SRAMbuffer0[0x480];
//extern uint32_t SRAMbuffer1[0x400];

extern const uint64_t newFC0;
extern const uint64_t newFC1;
extern const uint64_t newFC2;
extern uint64_t newFC3;
extern const uint64_t newFC4;
extern uint32_t thisRxSPIBuffer[16]; 
extern uint8_t pixelBuffer[DisplayHeight][DisplayChipWidth];




