#include <stdint.h>
//#include "constant.h"
#include "TLC6983.h"
//#define DisplayHeight 12
//#define DisplayWidth 42
//#define DisplayChipWidth 48

void showString(char *thisString, uint8_t autoCenter, uint8_t fixedFontWidth);
extern void writeStringtoScreen(void);

//char A_5x7[8] = { 6, 9, 9, 16, 9, 9, 9 ,0};
//char N_8x8[8] = { 0x41, 0x61, 0x51, 0x49, 0x45, 0x43, 0x41,0 };
//char A_8x8[8] = { 8, 0x14, 0x22, 0x41, 0x7F, 0x41, 0x41 ,0};
//char M_8x8[8] = { 0x63, 0x55, 0x49, 0x49, 0x49, 0x49, 0x49,0 };

const char Ascii_installed[0x80-0x20] = // Starts at 0x20, ends at 0x7f
{
    38, // space 0x20
    0, // ! 0x21
    0, // " 0x22
    0, // # 0x23
    0, // $ 0x24
    0, // % 0x25
    0, // & 0x26
    0, // ' 0x27
    0, // ( 0x28
    0, // ) 0x29
    0, // * 0x2A
    0, // + 0x2B
    0, // , 0x2C
    0, // - 0x2D
    39, // . 0x2E
    0, // / 0x2F
    1, // 0 0x30
    2, // 1 0x31
    3, // 2 0x32
    4, // 3 0x33
    5, // 4 0x34
    6, // 5 0x35
    7, // 6 0x36
    8, // 7 0x37
    9, // 8 0x38
    10, // 9 0x39
    37, // : 0x3A
    0, // ; 0x3B
    0, // < 0x3C
    0, // = 0x3D
    0, // > 0x3E
    0, // ? 0x3F
    0, // @ 0x40
    11, // A 0x41
    12, // B 0x42
    13, // C 0x43
    14, // D 0x44
    15, // E 0x45
    16, // F 0x46
    17, // G 0x47
    18, // H 0x48
    19, // I 0x49
    20, // J 0x4A
    0, // K 0x4B
    21, // L 0x4C
    22, // M 0x4D
    23, // N 0x4E
    24, // O 0x4F
    25, // P 0x50
    0, // Q 0x51
    26, // R 0x52
    27, // S 0x53
    28, // T 0x54
    29, // U 0x55
    0, // V 0x56
    30, // W 0x57
    0, // X 0x58
    31, // Y 0x59
    0, // Z 0x5A
    0, // [ 0x5B
    0, // \ 0x5C
    0, // ] 0x5D
    0, // ^ 0x5E
    0, // _ 0x5F
    0, // ` 0x60
    0, // a 0x61
    0, // b 0x62
    0, // c 0x63
    0, // d 0x64
    32, // e 0x65
    0, // f 0x66
    0, // g 0x67
    33, // h 0x68
    0, // i 0x69
    0, // j 0x6A
    0, // k 0x6B
    34, // l 0x6C
    0, // m 0x6D
    0, // n 0x6E
    35, // o 0x6F
    0, // p 0x70
    0, // q 0x71
    0, // r 0x72
    0, // s 0x73
    0, // t 0x74
    0, // u 0x75
    0, // v 0x76
    0, // w 0x77
    0, // x 0x78
    0, // y 0x79
    36, // z 0x7A
    0, // { 0x7B
    0, // | 0x7C
    0, // } 0x7D
    0, // ~ 0x7E
    38, // del 0x7F
};

const char _0_5x6[10] = { 6, 6, 0x1C, 0x22, 0x22, 0x22, 0x22, 0x1C }; // 1
const char _1_5x6[10] = { 6, 4, 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04 }; // 2
const char _2_5x6[10] = { 6, 6, 0x1C, 0x22, 0x02, 0x1C, 0x20, 0x3E }; // 3
const char _3_5x6[10] = { 6, 6, 0x1C, 0x22, 0x0C, 0x02, 0x22, 0x1C }; // 4
const char _4_5x6[10] = { 6, 6, 0x04, 0x0C, 0x14, 0x3E, 0x04, 0x04 }; // 5
const char _5_5x6[10] = { 6, 6, 0x3E, 0x20, 0x3C, 0x02, 0x22, 0x1C }; // 6
const char _6_5x6[10] = { 6, 6, 0x1C, 0x20, 0x3C, 0x22, 0x22, 0x1C }; // 7
const char _7_5x6[10] = { 6, 6, 0x3E, 0x04, 0x08, 0x08, 0x08, 0x08 }; // 8
const char _8_5x6[10] = { 6, 6, 0x1C, 0x22, 0x1C, 0x22, 0x22, 0x1C }; // 9
const char _9_5x6[10] = { 6, 6, 0x1C, 0x22, 0x22, 0x1E, 0x02, 0x1C }; // 10
const char A_5x6[10] = { 6, 6, 0x1C, 0x22, 0x22, 0x3E, 0x22, 0x22 }; // 11
const char B_5x6[10] = { 6, 6, 0x3C, 0x22, 0x3C, 0x22, 0x22, 0x3C }; // 12
const char C_5x6[10] = { 6, 6, 0x1C, 0x22, 0x20, 0x20, 0x22, 0x1C }; // 13
const char D_5x6[10] = { 6, 6, 0x3C, 0x22, 0x22, 0x22, 0x22, 0x3C }; // 14
const char E_5x6[10] = { 6, 6, 0x3E, 0x20, 0x3C, 0x20, 0x20, 0x3E }; // 15
const char F_5x6[10] = { 6, 6, 0x3E, 0x20, 0x3C, 0x20, 0x20, 0x20 }; // 16
const char G_5x6[10] = { 6, 6, 0x1C, 0x22, 0x20, 0x26, 0x22, 0x1C }; // 17
const char H_5x6[10] = { 6, 6, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x22 }; // 18
const char I_5x6[10] = { 6, 4, 0x0E, 0x04, 0x04, 0x04, 0x04, 0x0E }; // 19
const char J_5x6[10] = { 6, 6, 0x02, 0x02, 0x02, 0x22, 0x22, 0x1C }; // 20
const char L_5x6[10] = { 6, 6, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3E }; // 21
const char M_5x6[10] = { 6, 6, 0x22, 0x36, 0x2A, 0x22, 0x22, 0x22 }; // 22
const char N_5x6[10] = { 6, 6, 0x22, 0x32, 0x2A, 0x26, 0x22, 0x22 }; // 23
const char O_5x6[10] = { 6, 6, 0x1C, 0x22, 0x22, 0x22, 0x22, 0x1C }; // 24
const char P_5x6[10] = { 6, 6, 0x3C, 0x22, 0x22, 0x3C, 0x20, 0x20 }; // 25
const char R_5x6[10] = { 6, 6, 0x3C, 0x22, 0x22, 0x3C, 0x24, 0x22 }; // 26
const char S_5x6[10] = { 6, 6, 0x1C, 0x20, 0x1C, 0x02, 0x22, 0x1C }; // 27
const char T_5x6[10] = { 6, 6, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08 }; // 28
const char U_5x6[10] = { 6, 6, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C }; // 29
const char W_5x6[10] = { 6, 6, 0x22, 0x22, 0x22, 0x2A, 0x36, 0x22 }; // 30
const char Y_5x6[10] = { 6, 6, 0x22, 0x22, 0x14, 0x08, 0x08, 0x08 }; // 31
const char e_5x7[10] = { 7, 6, 0x00, 0x00, 0x1C, 0x22, 0x3E, 0x20, 0x1C }; // 32
const char h_5x7[10] = { 7, 6, 0x20, 0x20, 0x2C, 0x32, 0x22, 0x22, 0x22 }; // 33
const char l_5x7[10] = { 7, 4, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E }; // 34
const char o_5x7[10] = { 7, 6, 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C }; // 35
const char z_5x6[10] = { 6, 6, 0x00, 0x3E, 0x04, 0x08, 0x10, 0x3E }; // 36
const char Colon_5x6[10] = { 6, 2, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00 }; // 37
const char Space_5x6[10] = { 6, 6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 38
const char DecimalPlace_5x6[10] = { 6, 2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 }; // 39

const char * const tableOfAsciiChars[39] = {
    _0_5x6, // 1
    _1_5x6, // 2
    _2_5x6, // 3
    _3_5x6, // 4
    _4_5x6, // 5
    _5_5x6, // 6
    _6_5x6, // 7
    _7_5x6, // 8
    _8_5x6, // 9
    _9_5x6, // 10
    A_5x6, // 11
    B_5x6, // 12
    C_5x6, // 13
    D_5x6, // 14
    E_5x6, // 15
    F_5x6, // 16
    G_5x6, // 17
    H_5x6, // 18
    I_5x6, // 19
    J_5x6, // 20
    L_5x6, // 21
    M_5x6, // 22
    N_5x6, // 23
    O_5x6, // 24
    P_5x6, // 25
    R_5x6, // 26
    S_5x6, // 27
    T_5x6, // 28
    U_5x6, // 29
    W_5x6, // 30
    Y_5x6, // 31
    e_5x7, // 32
    h_5x7, // 33
    l_5x7, // 34
    o_5x7, // 35
    z_5x6, // 36
    Colon_5x6, // 37
    Space_5x6, // 38
    DecimalPlace_5x6, // 39
};


void showString(char *thisString, uint8_t autoCenter, uint8_t fixedFontWidth) // with auto center string
{
    for (int i = 0; i < DisplayHeight; i++)        // clear pixel buffer   12x48 16bit  = 1152bytes  Screen buffer 0xe08;3.5k
    {
        for (int j = 0; j < DisplayChipWidth; j++)
        {
            pixelBuffer[i][j] = 0;
        }
    }

    char* ptr = thisString;
    // check length
    int length = 0;
    while ((*ptr++) && (length++ < 20)) ;  // 20 characters Maximum

    //calc width of string to center text on screen of 42 pixels
    ptr = thisString;
    uint8_t *isInstalledPtr;
    int fontWidth = 0;
    int fontHeight;
    int thisChar;
    int thisFontHeight = 0;
    int MaxFontHeight = 0;
    int fullWidth = 0;
    for (int i = 0; i < length; i++)        // length of string in characters
    {
        thisChar = *ptr++;
        isInstalledPtr = (uint8_t *)Ascii_installed; // ENGR:CT
        isInstalledPtr += thisChar - 0x20;
        int thisOffset = *isInstalledPtr;
        if (thisOffset)
        {
            uint8_t *CharFontLocn = (uint8_t *)tableOfAsciiChars[thisOffset - 1]; // ENGR:CT cast, the first installed char in position 1 is the zeroth element in the tableOfAsciiChars
            thisFontHeight = *CharFontLocn++;
            fontWidth = *CharFontLocn++;
            if ((fixedFontWidth) && (thisChar != 0x7E)&& (thisChar > 0x30)&& (thisChar != ':'))
                fontWidth = fixedFontWidth;
            fullWidth += fontWidth; // width of string
        }
        if (thisFontHeight > MaxFontHeight)
            MaxFontHeight = thisFontHeight;
        if (MaxFontHeight > DisplayHeight)
            MaxFontHeight = DisplayHeight;
    }

    ptr = thisString;
    int PixelPosition = 0;
    if (autoCenter)
        if (fullWidth <= DisplayWidth)
            PixelPosition = (DisplayWidth - fullWidth) / 2; // (width of display 42 - width of string) /2  is left start column.
        // center in display height
        // this display is DisplayHeight(12) rows high

    PixelPosition += ((DisplayHeight - MaxFontHeight) / 2) * DisplayChipWidth; // 48 pixels wide on the display chip

    for (int k = 0; k < length; k++) // move along string
    {

        thisChar = *ptr++;
        //check if it is installed in the font array
        isInstalledPtr = (uint8_t *)Ascii_installed; // ENGR:CT
        isInstalledPtr += thisChar - 0x20;
        int thisOffset = *isInstalledPtr;
        //int characterNumber = 0;
        //int lineNumber = 0;

        if (thisOffset)  // if there is no character font for this particular character it is ignored and not printed.
        {
            uint8_t *CharFontLocn = (uint8_t *)tableOfAsciiChars[thisOffset - 1]; // ENGR:CT
            fontHeight = *CharFontLocn++;
            fontWidth = *CharFontLocn++;
            int centerOffsetChar = 0;
            if ((fixedFontWidth) && (thisChar < 0x80)&& (thisChar > 0x30)&& (thisChar != ':'))
            {
                if (fixedFontWidth != fontWidth)
                    centerOffsetChar = (fixedFontWidth - fontWidth) / 2;
                fontWidth = fixedFontWidth;

            }

            for (int i = 0; i < fontHeight; i++)          //work down character height
            {
                int thisPixelPattern = *CharFontLocn++;
                for (int j = fontWidth - 1; j >= 0; j--)  // shift pixel data into line
                {
                    pixelBuffer[i][PixelPosition + fontWidth - j - centerOffsetChar] = thisPixelPattern & (1 << j);
                }
            }
            PixelPosition += fontWidth;
        }
    }
    writeStringtoScreen();
}
