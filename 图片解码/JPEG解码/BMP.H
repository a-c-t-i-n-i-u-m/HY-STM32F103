typedef struct tagBITMAPINFOHEADER 
{
    DWORD biSize ;
    LONG  biWidth ;
    LONG  biHeight ;
    WORD  biPlanes ;
    WORD  biBitCount ;
    DWORD biCompression ;
    DWORD biSizeImage ;
    LONG  biXPelsPerMeter ;
    LONG  biYPelsPerMeter ;
    DWORD biClrUsed ;
    DWORD biClrImportant ;
}BITMAPINFOHEADER ;

typedef __packed struct tagBITMAPFILEHEADER 
{
    short bfType ;
    DWORD bfSize ;
    WORD  bfReserved1 ;
    WORD  bfReserved2 ;
    DWORD bfOffBits ;
}BITMAPFILEHEADER ;

/* constants for the biCompression field */
#define BI_RGB       0L
#define BI_RLE8      1L
#define BI_RLE4      2L
#define BI_BITFIELDS 3L 


typedef struct tagRGBQUAD 
{
    BYTE rgbBlue ;
    BYTE rgbGreen ;
    BYTE rgbRed ;
    BYTE rgbReserved ;
}RGBQUAD ;