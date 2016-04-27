/*
	Musical note table for audio generation with TIMER0 in CTC 
	mode, toggle output on match witha 16MHz Crystal.

	Daniel Jose Viana, 2015

*/


#include <avr/pgmspace.h>



//	    name note   freq(Hz) 

#define _C0    1    //  16,35  
#define _C0h   2    //  17,32  
#define _D0    3    //  18,35  
#define _D0h   4    //  19,45  
#define _E0    5    //  20,60  
#define _F0    6    //  21,83  
#define _F0h   7    //  23,12  
#define _G0    8    //  24,50  
#define _G0h   9    //  25,96  
#define _A0   10    //  27,50  
#define _A0h  11    //  29,14  
#define _B0   12    //  30,87  
#define _C1   13    //  32,70  
#define _C1h  14    //  34,65  
#define _D1   15    //  36,71  
#define _D1h  16    //  38,89  
#define _E1   17    //  41,20  
#define _F1   18    //  43,65  
#define _F1h  19    //  46,25  
#define _G1   20    //  49,00  
#define _G1h  21    //  51,91  
#define _A1   22    //  55,00  
#define _A1h  23    //  58,27  
#define _B1   24    //  61,74  
#define _C2   25    //  65,41  
#define _C2h  26    //  69,30  
#define _D2   27    //  73,42  
#define _D2h  28    //  77,78  
#define _E2   29    //  82,41  
#define _F2   30    //  87,31  
#define _F2h  31    //  92,50  
#define _G2   32    //  98,00  
#define _G2h  33    // 103,83  
#define _A2   34    // 110,00  
#define _A2h  35    // 116,54  
#define _B2   36    // 123,47  
#define _C3   37    // 130,81  
#define _C3h  38    // 138,59  
#define _D3   39    // 146,83  
#define _D3h  40    // 155,56  
#define _E3   41    // 164,81  
#define _F3   42    // 174,61  
#define _F3h  43    // 185,00  
#define _G3   44    // 196,00  
#define _G3h  45    // 207,65  
#define _A3   46    // 220,00  
#define _A3h  47    // 233,08  
#define _B3   48    // 246,94  
#define _C4   49    // 261,63  
#define _C4h  50    // 277,18  
#define _D4   51    // 293,66  
#define _D4h  52    // 311,13  
#define _E4   53    // 329,63  
#define _F4   54    // 349,23  
#define _F4h  55    // 369,99  
#define _G4   56    // 392,00  
#define _G4h  57    // 415,30  
#define _A4   58    // 440,00  
#define _A4h  59    // 466,16  
#define _B4   60    // 493,88  
#define _C5   61    // 523,25  
#define _C5h  62    // 554,37  
#define _D5   63    // 587,33  
#define _D5h  64    // 622,25  
#define _E5   65    // 659,25  
#define _F5   66    // 698,46  
#define _F5h  67    // 739,99  
#define _G5   68    // 783,99  
#define _G5h  69    // 830,61  
#define _A5   70    // 880,00  
#define _A5h  71    // 932,33  
#define _B5   72    // 987,77  
#define _C6   73    //1046,50  
#define _C6h  74    //1108,73  
#define _D6   75    //1174,66  
#define _D6h  76    //1244,51  
#define _E6   77    //1318,51  
#define _F6   78    //1396,91  
#define _F6h  79    //1479,98  
#define _G6   80    //1567,98  
#define _G6h  81    //1661,22  
#define _A6   82    //1760,00  
#define _A6h  83    //1864,66  
#define _B6   84    //1975,53  
#define _C7   85    //2093,00  
#define _C7h  86    //2217,46  
#define _D7   87    //2349,32  
#define _D7h  88    //2489,02  
#define _E7   89    //2637,02  
#define _F7   90    //2793,83  
#define _F7h  91    //2959,96  
#define _G7   92    //3135,96  
#define _G7h  93    //3322,44  
#define _A7   94    //3520,00  
#define _A7h  95    //3729,31  
#define _B7   96    //3951,07  
#define _C8   97    //4186,01  
#define _C8h  98    //4434,92  
#define _D8   99    //4698,63  
#define _D8h  100   //4978,03  
#define _E8   101   //5274,04  
#define _F8   102   //5587,65  
#define _F8h  103   //5919,91  
#define _G8   104   //6271,93  
#define _G8h  105   //6644,88  
#define _A8   106   //7040,00  
#define _A8h  107   //7458,62  
#define _B8   108   //7902,13  


const uint8_t note_table[] PROGMEM = { // OCR0 Value  
	   0,  // note  name  freq(Hz)  Presc - first value is Dummy
	   0,  //  1    C0     16,35    1024 
	   0,  //  2    C#0    17,32    1024 
	   0,  //  3    D0     18,35    1024 
	   0,  //  4    D#0    19,45    1024 
	   0,  //  5    E0     20,60    1024 
	   0,  //  6    F0     21,83    1024 
	   0,  //  7    F#0    23,12    1024 
	   0,  //  8    G0     24,50    1024 
	   0,  //  9    G#0    25,96    1024 
	   0,  // 10    A0     27,50    1024 
	   0,  // 11    A#0    29,14    1024 
	 252,  // 12    B0     30,87    1024 
	 237,  // 13    C1     32,70    1024 
	 224,  // 14    C#1    34,65    1024 
	 211,  // 15    D1     36,71    1024 
	 199,  // 16    D#1    38,89    1024 
	 188,  // 17    E1     41,20    1024 
	 177,  // 18    F1     43,65    1024 
	 167,  // 19    F#1    46,25    1024 
	 158,  // 20    G1     49,00    1024 
	 149,  // 21    G#1    51,91    1024 
	 141,  // 22    A1     55,00    1024 
	 133,  // 23    A#1    58,27    1024 
	 125,  // 24    B1     61,74    1024 
	 118,  // 25    C2     65,41    1024 
	 111,  // 26    C#2    69,30    1024 
	 105,  // 27    D2     73,42    1024 
	  99,  // 28    D#2    77,78    1024 
	  93,  // 29    E2     82,41    1024 
	  88,  // 30    F2     87,31    1024 
	  83,  // 31    F#2    92,50    1024 
	  78,  // 32    G2     98,00    1024 
	  74,  // 33    G#2   103,83    1024 
	  70,  // 34    A2    110,00    1024 
	  66,  // 35    A#2   116,54    1024 
	 252,  // 36    B2    123,47     256 
	 237,  // 37    C3    130,81     256 
	 224,  // 38    C#3   138,59     256 
	 211,  // 39    D3    146,83     256 
	 199,  // 40    D#3   155,56     256 
	 188,  // 41    E3    164,81     256 
	 177,  // 42    F3    174,61     256 
	 167,  // 43    F#3   185,00     256 
	 158,  // 44    G3    196,00     256 
	 149,  // 45    G#3   207,65     256 
	 141,  // 46    A3    220,00     256 
	 133,  // 47    A#3   233,08     256 
	 125,  // 48    B3    246,94     256 
	 118,  // 49    C4    261,63     256 
	 111,  // 50    C#4   277,18     256 
	 105,  // 51    D4    293,66     256 
	  99,  // 52    D#4   311,13     256 
	  93,  // 53    E4    329,63     256 
	  88,  // 54    F4    349,23     256 
	  83,  // 55    F#4   369,99     256 
	  78,  // 56    G4    392,00     256 
	  74,  // 57    G#4   415,30     256 
	  70,  // 58    A4    440,00     256 
	  66,  // 59    A#4   466,16     256 
	 252,  // 60    B4    493,88      64 
	 237,  // 61    C5    523,25      64 
	 224,  // 62    C#5   554,37      64 
	 211,  // 63    D5    587,33      64 
	 199,  // 64    D#5   622,25      64 
	 188,  // 65    E5    659,25      64 
	 177,  // 66    F5    698,46      64 
	 167,  // 67    F#5   739,99      64 
	 158,  // 68    G5    783,99      64 
	 149,  // 69    G#5   830,61      64 
	 141,  // 70    A5    880,00      64 
	 133,  // 71    A#5   932,33      64 
	 125,  // 72    B5    987,77      64 
	 118,  // 73    C6   1046,50      64 
	 111,  // 74    C#6  1108,73      64 
	 105,  // 75    D6   1174,66      64 
	  99,  // 76    D#6  1244,51      64 
	  93,  // 77    E6   1318,51      64 
	  88,  // 78    F6   1396,91      64 
	  83,  // 79    F#6  1479,98      64 
	  78,  // 80    G6   1567,98      64 
	  74,  // 81    G#6  1661,22      64 
	  70,  // 82    A6   1760,00      64 
	  66,  // 83    A#6  1864,66      64 
	  62,  // 84    B6   1975,53      64 
	  58,  // 85    C7   2093,00      64 
	  55,  // 86    C#7  2217,46      64 
	  52,  // 87    D7   2349,32      64 
	  49,  // 88    D#7  2489,02      64 
	  46,  // 89    E7   2637,02      64 
	  43,  // 90    F7   2793,83      64 
	  41,  // 91    F#7  2959,96      64 
	  38,  // 92    G7   3135,96      64 
	  36,  // 93    G#7  3322,44      64 
	  34,  // 94    A7   3520,00      64 
	  32,  // 95    A#7  3729,31      64 
	  30,  // 96    B7   3951,07      64 
	 237,  // 97    C8   4186,01       8 
	 224,  // 98    C#8  4434,92       8 
	 211,  // 99    D8   4698,63       8 
	 199,  // 100   D#8  4978,03       8 
	 188,  // 101   E8   5274,04       8 
	 177,  // 102   F8   5587,65       8 
	 167,  // 103   F#8  5919,91       8 
	 158,  // 104   G8   6271,93       8 
	 149,  // 105   G#8  6644,88       8 
	 141,  // 106   A8   7040,00       8 
	 133,  // 107   A#8  7458,62       8 
	 125   // 108   B8   7902,13       8 
};

