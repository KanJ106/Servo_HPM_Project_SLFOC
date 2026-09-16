#include "segger_rtt_interface.h"

#define DEBUG_LOG 1

#if (DEBUG_LOG == 1)
	#define DEBUG_PRINT(...) SEGGER_RTT_printf(__VA_ARGS__)
#else
	#define DEBUG_PRINT(...) ((void)0)
#endif

/*
*   rtt 重写了printf 用SEGGER_RTT_Write实现
*
*/


void RTT_Init(void)
{
    //SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Init\r\n\r\n");
}

void RTT_Test(void)
{
    SEGGER_RTT_WriteString(0, "###### Testing SEGGER_printf() ######\r\n");

    DEBUG_PRINT(0, "printf Test: %%c,         'S' : %c.\r\n", 'S');
    DEBUG_PRINT(0, "printf Test: %%5c,        'E' : %5c.\r\n", 'E');
    DEBUG_PRINT(0, "printf Test: %%-5c,       'G' : %-5c.\r\n", 'G');
    DEBUG_PRINT(0, "printf Test: %%5.3c,      'G' : %-5c.\r\n", 'G');
    DEBUG_PRINT(0, "printf Test: %%.3c,       'E' : %-5c.\r\n", 'E');
    DEBUG_PRINT(0, "printf Test: %%c,         'R' : %c.\r\n", 'R');

    DEBUG_PRINT(0, "printf Test: %%s,      \"RTT\" : %s.\r\n", "RTT");
    DEBUG_PRINT(0, "printf Test: %%s, \"RTT\\r\\nRocks.\" : %s.\r\n", "RTT\r\nRocks.");

    DEBUG_PRINT(0, "printf Test: %%u,       12345 : %u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%+u,      12345 : %+u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%.3u,     12345 : %.3u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%.6u,     12345 : %.6u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%6.3u,    12345 : %6.3u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%8.6u,    12345 : %8.6u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%08u,     12345 : %08u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%08.6u,   12345 : %08.6u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%0u,      12345 : %0u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%-.6u,    12345 : %-.6u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%-6.3u,   12345 : %-6.3u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%-8.6u,   12345 : %-8.6u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%-08u,    12345 : %-08u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%-08.6u,  12345 : %-08.6u.\r\n", 12345);
    DEBUG_PRINT(0, "printf Test: %%-0u,     12345 : %-0u.\r\n", 12345);

    DEBUG_PRINT(0, "printf Test: %%u,      -12345 : %u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%+u,     -12345 : %+u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%.3u,    -12345 : %.3u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%.6u,    -12345 : %.6u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%6.3u,   -12345 : %6.3u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%8.6u,   -12345 : %8.6u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%08u,    -12345 : %08u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%08.6u,  -12345 : %08.6u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%0u,     -12345 : %0u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-.6u,   -12345 : %-.6u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-6.3u,  -12345 : %-6.3u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-8.6u,  -12345 : %-8.6u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-08u,   -12345 : %-08u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-08.6u, -12345 : %-08.6u.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-0u,    -12345 : %-0u.\r\n", -12345);

    DEBUG_PRINT(0, "printf Test: %%d,      -12345 : %d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%+d,     -12345 : %+d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%.3d,    -12345 : %.3d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%.6d,    -12345 : %.6d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%6.3d,   -12345 : %6.3d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%8.6d,   -12345 : %8.6d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%08d,    -12345 : %08d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%08.6d,  -12345 : %08.6d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%0d,     -12345 : %0d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-.6d,   -12345 : %-.6d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-6.3d,  -12345 : %-6.3d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-8.6d,  -12345 : %-8.6d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-08d,   -12345 : %-08d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-08.6d, -12345 : %-08.6d.\r\n", -12345);
    DEBUG_PRINT(0, "printf Test: %%-0d,    -12345 : %-0d.\r\n", -12345);

    DEBUG_PRINT(0, "printf Test: %%x,      0x1234ABC : %x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%+x,     0x1234ABC : %+x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%.3x,    0x1234ABC : %.3x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%.6x,    0x1234ABC : %.6x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%6.3x,   0x1234ABC : %6.3x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%8.6x,   0x1234ABC : %8.6x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%08x,    0x1234ABC : %08x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%08.6x,  0x1234ABC : %08.6x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%0x,     0x1234ABC : %0x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%-.6x,   0x1234ABC : %-.6x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%-6.3x,  0x1234ABC : %-6.3x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%-8.6x,  0x1234ABC : %-8.6x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%-08x,   0x1234ABC : %-08x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%-08.6x, 0x1234ABC : %-08.6x.\r\n", 0x1234ABC);
    DEBUG_PRINT(0, "printf Test: %%-0x,    0x1234ABC : %-0x.\r\n", 0x1234ABC);

    SEGGER_RTT_WriteString(0, "###### SEGGER_printf() Tests done. ######\r\n");


}