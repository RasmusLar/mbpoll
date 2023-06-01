/* Copyright (c) 2015-2023 Pascal JEAN, All rights reserved.
 *
 * mbpoll is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * mbpoll is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mbpoll.  If not, see <http://www.gnu.org/licenses/>.
 */
#define _GNU_SOURCE

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <getopt.h>
#include <inttypes.h>
#include <modbus.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
#include <sys/time.h>

#include <unistd.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif
#include "custom-rts.h"
#include "mbpoll-config.h"
#include "serial.h"
#include "version-git.h"

/* constants ================================================================ */
#define AUTHORS "Pascal JEAN"
#define WEBSITE "https://github.com/epsilonrt/mbpoll"

/* conditionals ============================================================= */
#if defined(__GNUC__) && __SIZEOF_FLOAT__ != 4 && !defined(__STDC_IEC_559__)
#error it seems that this platform does not conform to the IEEE-754 standard !
#define MBPOLL_FLOAT_DISABLE
#endif

/* macros =================================================================== */
#define BASENAME(f) (f)

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
#include <libgen.h>
#undef BASENAME
#define BASENAME(f) basename(f)
#endif

#ifndef NDEBUG
#define PDEBUG(fmt, ...) \
    printfInternal("%s:%d: %s(): " fmt, BASENAME(__FILE__), __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define PDEBUG(...)     \
    if (ctx.bIsVerbose) \
    printfInternal(__VA_ARGS__)
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* types ==================================================================== */
typedef enum {
    eModeRtu,
    eModeTcp,
    eModeUnknown = -1,
} eModes;

typedef enum {
    eFuncCoil          = 0,
    eFuncDiscreteInput = 1,
    eFuncInputReg      = 3,
    eFuncHoldingReg    = 4,
    eFuncUnknown       = -1,
} eFunctions;

typedef enum {
    eFormatDec,
    eFormatInt16,
    eFormatHex,
    eFormatString,
    eFormatInt,
    eFormatFloat,
    eFormatBin,
    eFormatUnknown = -1,
} eFormats;

/* macros =================================================================== */
#define SIZEOF_ILIST(list) (sizeof(list) / sizeof(int))
/*
 * Le pointeur sur les données est de type void *, les macros ci-dessous
 * permettent de caster l'accès aux données en fonction de leur format
 */
#define DUINT8(p, i)  ((uint8_t*)(p))[i]
#define DUINT16(p, i) ((uint16_t*)(p))[i]
#define DINT32(p, i)  ((int32_t*)(p))[i]
#define DFLOAT(p, i)  ((float*)(p))[i]

/* constants ================================================================ */
static char const* sModeList[]   = {"RTU", "TCP"};
static int const iModeList[]     = {eModeRtu, eModeTcp};
static int const iParityList[]   = {SERIAL_PARITY_EVEN, SERIAL_PARITY_ODD, SERIAL_PARITY_NONE};
static int const iDatabitsList[] = {SERIAL_DATABIT_8, SERIAL_DATABIT_7};
static int const iStopbitsList[] = {SERIAL_STOPBIT_ONE, SERIAL_STOPBIT_TWO};

static char const* sFunctionList[]
    = {"discrete output (coil)", "discrete input", "input register", "output (holding) register"};
static int const iFunctionList[] = {eFuncCoil, eFuncDiscreteInput, eFuncInputReg, eFuncHoldingReg};

static char const sModeStr[]         = "mode";
static char const sSlaveAddrStr[]    = "slave address";
static char const sRtuParityStr[]    = "rtu parity";
static char const sRtuStopbitsStr[]  = "rtu stop bits";
static char const sRtuDatabitsStr[]  = "rtu data bits";
static char const sRtuBaudrateStr[]  = "rtu baudrate";
static char const sTcpPortStr[]      = "tcp port";
static char const sTimeoutStr[]      = "timeout";
static char const sPollRateStr[]     = "poll rate";
static char const sFunctionStr[]     = "function";
static char const sFormatStr[]       = "format";
static char const sNumOfValuesStr[]  = "number of values";
static char const sStartRefStr[]     = "start reference";
static char const sDataStr[]         = "data";
static char const sUnknownStr[]      = "unknown";
static char const sIntStr[]          = "32-bit integer";
static char const sFloatStr[]        = "32-bit float";
static char const sWordStr[]         = "16-bit register";
static char const sLittleEndianStr[] = "(little endian)";
static char const sBigEndianStr[]    = "(big endian)";
static char* progname;

#ifdef MBPOLL_GPIO_RTS
static char const sRtsPinStr[] = "RTS pin";
#endif

/* structures =============================================================== */
typedef struct xChipIoContext xChipIoContext;

typedef struct xMbPollContext {
    eModes eMode;
    eFunctions eFunction;
    eFormats eFormat;
    int* piSlaveAddr;
    int iSlaveCount;
    int* piStartRef;
    int iStartCount;
    int iCount;
    char* sTcpPort;
    char* sDevice;
    xSerialIos xRtu;
    int iRtuBaudrate;
    eSerialDataBits eRtuDatabits;
    eSerialStopBits eRtuStopbits;
    eSerialParity eRtuParity;
    int iRtuMode;
    bool bIsDefaultMode;
    int iPduOffset;
    bool bWriteSingleAsMany;
    bool bIsChipIo;
    bool bIsBigEndian;
    modbus_t* xBus;
    modbus_t* forwardBus;
    char const* forwardDevice;
    void* pvData;
    int iTxCount;
    int iRxCount;
    int iErrorCount;

    xChipIoContext* xChip; // TODO: séparer la partie chipio

    // Paramètres
    double dTimeout;
    int iPollRate;
    bool bIsVerbose;
    bool bIsReportSlaveID;
    bool bIsQuiet;
    bool bIsPolling;
#ifdef MBPOLL_GPIO_RTS
    int iRtsPin;
#endif

} xMbPollContext;

/* private variables ======================================================== */

static xMbPollContext ctx = {
    .eMode = DEFAULT_MODE,
    .eFunction = DEFAULT_FUNCTION,
    .eFormat = eFormatDec,
    .piSlaveAddr = NULL,
    .iSlaveCount = -1,
    .piStartRef = NULL,
    .iStartCount = -1,
    .iCount = DEFAULT_NUMOFVALUES,
    .sTcpPort = DEFAULT_TCP_PORT,
    .sDevice = NULL,
    .xRtu =
        {
            .baud = DEFAULT_RTU_BAUDRATE,
            .dbits = DEFAULT_RTU_DATABITS,
            .sbits = DEFAULT_RTU_STOPBITS,
            .parity = DEFAULT_RTU_PARITY,
            .flow = SERIAL_FLOW_NONE,
        },
    .iRtuMode = MODBUS_RTU_RTS_NONE,
    .bIsDefaultMode = true,
    .iPduOffset = 1,
    .bWriteSingleAsMany = false,
    .bIsChipIo = false,
    .bIsBigEndian = false,
#ifdef MBPOLL_GPIO_RTS
    .iRtsPin = -1,
#endif

    // Variables de travail
    .xBus = NULL,
    .forwardBus = NULL,
    .forwardDevice = NULL,
    .pvData = NULL,
    // Paramètres
    .dTimeout = DEFAULT_TIMEOUT,
    .iPollRate = DEFAULT_POLLRATE,
    .bIsVerbose = false,
    .bIsReportSlaveID = false,
    .bIsQuiet = false,
    .bIsPolling = true,
};

#ifdef USE_CHIPIO
// -----------------------------------------------------------------------------
#include <chipio/serial.h>
#include <sysio/rpi.h>

/* private variables ======================================================== */
// Paramètres
static int iChipIoSlaveAddr = DEFAULT_CHIPIO_SLAVEADDR;
static int iChipIoIrqPin    = DEFAULT_CHIPIO_IRQPIN;
// static bool  bIsChipIo = false;

// Variables de travail
static xChipIo* xChip;
static xChipIoSerial* xChipSerial;

/* constants ================================================================ */
static char const sChipIoSlaveAddrStr[] = "chipio slave address";
static char const sChipIoIrqPinStr[]    = "chipio irq pin";
// option -i et -n supplémentaires pour chipio
#endif /* USE_CHIPIO == 0 */

/* private functions ======================================================== */
int printfInternal(char const* format, ...);
int putcharInternal(int c);
void vAllocate(xMbPollContext* ctx);
void vPrintReadValues(int iAddr, int iCount, xMbPollContext* ctx);
void vPrintConfig(xMbPollContext const* ctx);
void vPrintCommunicationSetup(xMbPollContext const* ctx);
void vReportSlaveID(xMbPollContext const* ctx);
void vHello(void);
void vVersion(void);
void vWarranty(void);
void vUsage(FILE* stream, int exit_msg);
void vFailureExit(bool bHelp, char const* format, ...);
#define vSyntaxErrorExit(fmt, ...) vFailureExit(true, fmt, ##__VA_ARGS__)
#define vIoErrorExit(fmt, ...)     vFailureExit(false, fmt, ##__VA_ARGS__)
void vCheckEnum(char const* sName, int iElmt, int const* iList, int iSize);
void vCheckIntRange(char const* sName, int i, int min, int max);
void vCheckDoubleRange(char const* sName, double d, double min, double max);
int iGetInt(char const* sName, char const* sNum, int iBase);
int* iGetIntList(char const* sName, char const* sList, int* iLen);
void vPrintIntList(int* iList, int iLen);
double dGetDouble(char const* sName, char const* sNum);
int iGetEnum(char const* sName, char* sElmt, char const** psStrList, int const* iList, int iSize);
char const* sEnumToStr(int iElmt, int const* iList, char const** psStrList, int iSize);
char const* sFunctionToStr(eFunctions eFunction);
char const* sModeToStr(eModes eMode);
void vSigIntHandler(int sig);
float fSwapFloat(float f);
int32_t lSwapLong(int32_t l);
void mb_delay(unsigned long d);

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
// Portage des fonctions Microsoft
// -----------------------------------------------------------------------------
static char* _strlwr(char* str) {
    char* p = str;

    while (*p) {
        *p = tolower(*p);
        p++;
    }
    return str;
}
#else
// Portage des fonctions POSIX ou GNU

// -----------------------------------------------------------------------------
// posix
#define strcasecmp _stricmp

// -----------------------------------------------------------------------------
// posix
static char* basename(char* path) {
    static char fname[_MAX_FNAME];
    _splitpath(path, NULL, NULL, fname, NULL);

    return fname;
}

// -----------------------------------------------------------------------------
// posix
static char* strcasestr(char const* haystack, char const* needle) {
    size_t nlen = strlen(needle);
    size_t hlen = strlen(haystack) - nlen + 1;
    size_t i;

    for (i = 0; i < hlen; i++) {
        int j;
        for (j = 0; j < nlen; j++) {
            unsigned char c1 = haystack[i + j];
            unsigned char c2 = needle[j];
            if (toupper(c1) != toupper(c2)) {
                goto next;
            }
        }
        return (char*)haystack + i;
next:;
    }
    return NULL;
}

// -----------------------------------------------------------------------------
// posix
static char* index(char const* s, int c) {
    while ((s) && (*s)) {
        if (c == *s) {
            return (char*)s;
        }
        s++;
    }
    return NULL;
}

#endif

/* main ===================================================================== */

static char device1[] = "192.168.10.4";
static char device2[] = "192.168.10.11";

int main(int argc, char** argv) {
    int iRet = 0;
    progname = argv[0];
    // Connection au bus

    ctx.sDevice = device1;
    ctx.xBus    = modbus_new_tcp_pi(ctx.sDevice, DEFAULT_TCP_PORT);
    if (modbus_connect(ctx.xBus) == -1) {
        modbus_free(ctx.xBus);
        vIoErrorExit("Connection failed to driver: %s", modbus_strerror(errno));
    }

    /*
     * évites que l'esclave prenne l'impulsion de 40µs créée par le driver à
     * l'ouverture du port comme un bit de start.
     */
    mb_delay(20);
    // Réglage du timeout de réponse
    uint32_t sec, usec;
    sec  = (uint32_t)ctx.dTimeout;
    usec = (uint32_t)((ctx.dTimeout - sec) * 1E6);
    modbus_set_response_timeout(ctx.xBus, sec, usec);

    // vSigIntHandler() intercepte le CTRL+C
    signal(SIGINT, vSigIntHandler);
    ctx.forwardDevice = device2;
    ctx.forwardBus    = modbus_new_tcp_pi(ctx.forwardDevice, DEFAULT_TCP_PORT);

    if (modbus_connect(ctx.forwardBus) == -1) {
        modbus_free(ctx.xBus);
        modbus_free(ctx.forwardBus);
        vIoErrorExit("Connection failed to UR: %s", modbus_strerror(errno));
    }

    modbus_set_response_timeout(ctx.xBus, sec, usec);
    modbus_set_response_timeout(ctx.forwardBus, sec, usec);
    modbus_set_slave(ctx.xBus, 0);
    modbus_set_slave(ctx.forwardBus, 0);

    ctx.pvData = calloc(1, 6 * 4);
    vPrintConfig(&ctx);
    do {
        // libmodbus utilise les adresses PDU !

        ctx.iTxCount++;

        // Ecriture ------------------------------------------------------------
        iRet = modbus_read_registers(ctx.forwardBus, 128, 4, ctx.pvData);
        iRet = modbus_write_registers(ctx.xBus, 4, iRet, ctx.pvData);

        if (iRet == 4) {
            ctx.iRxCount++;
            printfInternal("Written %d references.\n", ctx.iCount);
        } else {
            ctx.iErrorCount++;
            fprintf(
                stderr,
                "Write %s failed: %s\n",
                sFunctionToStr(ctx.eFunction),
                modbus_strerror(errno));
        }

        ctx.iTxCount++;

        printfInternal("-- Polling slave %d, forwarding to %s...", 0, ctx.forwardDevice);
        if (ctx.bIsPolling) {
            printfInternal(" Ctrl-C to stop)\n");
        } else {
            putcharInternal('\n');
        }
        // iRet = modbus_read_input_registers(
        //     ctx.xBus, iStartReg + iNbReg, iNbReg, ctx.pvData);
        // iRet = modbus_write_registers(ctx.forwardBus, 129 + iNbReg, iRet, ctx.pvData);
        iRet = modbus_read_registers(ctx.xBus, 4, 6, ctx.pvData);
        iRet = modbus_write_registers(ctx.forwardBus, 135, iRet, ctx.pvData);

        if (iRet == 6) {
            ctx.iRxCount++;
            // vPrintReadValues(4, ctx.iCount, &ctx);
        } else {
            ctx.iErrorCount++;
            fprintf(
                stderr,
                "Read %s failed: %s\n",
                sFunctionToStr(ctx.eFunction),
                modbus_strerror(errno));
        }
    } while (ctx.bIsPolling);

    vSigIntHandler(SIGTERM);
    return 0;
}

/* private functions ======================================================== */

// -----------------------------------------------------------------------------
void vPrintReadValues(int iAddr, int iCount, xMbPollContext* ctx) {
    int i;
    for (i = 0; i < iCount; i++) {
        printfInternal("[%d]: \t", iAddr);

        switch (ctx->eFormat) {
            case eFormatBin:
                printfInternal("%c", (DUINT8(ctx->pvData, i) != FALSE) ? '1' : '0');
                iAddr++;
                break;

            case eFormatDec: {
                uint16_t v = DUINT16(ctx->pvData, i);
                if (v & 0x8000) {
                    printfInternal("%u (%d)", v, (int)(int16_t)v);
                } else {
                    printfInternal("%u", v);
                }
                iAddr++;
            } break;

            case eFormatInt16:
                printfInternal("%d", (int)(int16_t)(DUINT16(ctx->pvData, i)));
                iAddr++;
                break;

            case eFormatHex:
                printfInternal("0x%04X", DUINT16(ctx->pvData, i));
                iAddr++;
                break;

            case eFormatString:
                printfInternal(
                    "%c%c",
                    (char)((int)(DUINT16(ctx->pvData, i) / 256)),
                    (char)(DUINT16(ctx->pvData, i) % 256));
                iAddr++;
                break;

            case eFormatInt:
                printfInternal("%d", lSwapLong(DINT32(ctx->pvData, i)));
                iAddr += 2;
                break;

            case eFormatFloat:
                printfInternal("%g", fSwapFloat(DFLOAT(ctx->pvData, i)));
                iAddr += 2;
                break;

            default: // Impossible normalement
                break;
        }
        putcharInternal('\n');
    }
}

// -----------------------------------------------------------------------------
void vReportSlaveID(xMbPollContext const* ctx) {
    uint8_t ucReport[256];

    modbus_set_slave(ctx->xBus, ctx->piSlaveAddr[0]);
    // Affichage de la configuration
    printfInternal("Protocol configuration: Modbus %s\n", sModeList[ctx->eMode]);
    printfInternal("Slave configuration...: address = %d, report slave id\n", ctx->piSlaveAddr[0]);

    vPrintCommunicationSetup(ctx);

    int iRet = modbus_report_slave_id(ctx->xBus, 256, ucReport);

    if (iRet < 0) {
        fprintf(stderr, "Report slave ID failed(%d): %s\n", iRet, modbus_strerror(errno));
    } else {
        if (iRet > 1) {
            int iLen = iRet - 2;

            printfInternal(
                "Length: %d\n"
                "Id    : 0x%02X\n"
                "Status: %s\n",
                iRet,
                ucReport[0],
                (ucReport[1]) ? "On" : "Off");

            if (iLen > 0) {
                int i;
                printfInternal("Data  : ");
                for (i = 2; i < (iLen + 2); i++) {
                    if (isprint(ucReport[i])) {
                        putcharInternal(ucReport[i]);
                    } else {
                        printfInternal("\\%02X", ucReport[i]);
                    }
                }
                putcharInternal('\n');
            }
        } else {
            fprintf(stderr, "no data available\n");
        }
    }
}

// -----------------------------------------------------------------------------
void vPrintCommunicationSetup(xMbPollContext const* ctx) {
    printf("Forwarding address....: address = %s", ctx->forwardDevice);
    printf("\n                        start reference = %d, count = %d\n", 4, 6);
    printf(
        "Communication.........: %s, port %s, t/o %.2f s, poll rate %d ms\n",
        ctx->sDevice,
        ctx->sTcpPort,
        ctx->dTimeout,
        ctx->iPollRate);
}

// -----------------------------------------------------------------------------
void vPrintConfig(xMbPollContext const* ctx) {
    // Affichage de la configuration
    printf("Protocol configuration: Modbus TCP\n");

    vPrintCommunicationSetup(ctx);
    printf("Data type.............: ");
    switch (ctx->eFunction) {
        case eFuncDiscreteInput: printf("discrete input\n"); break;

        case eFuncCoil: printf("discrete output (coil)\n"); break;

        case eFuncInputReg:
            if (ctx->eFormat == eFormatInt) {
                printf("%s %s", sIntStr, ctx->bIsBigEndian ? sBigEndianStr : sLittleEndianStr);
            } else if (ctx->eFormat == eFormatFloat) {
                printf("%s %s", sFloatStr, ctx->bIsBigEndian ? sBigEndianStr : sLittleEndianStr);
            } else {
                printf("%s", sWordStr);
            }
            printf(", input register table\n");
            break;

        case eFuncHoldingReg:
            if (ctx->eFormat == eFormatInt) {
                printf("%s %s", sIntStr, ctx->bIsBigEndian ? sBigEndianStr : sLittleEndianStr);
            } else if (ctx->eFormat == eFormatFloat) {
                printf("%s %s", sFloatStr, ctx->bIsBigEndian ? sBigEndianStr : sLittleEndianStr);
            } else {
                printf("%s", sWordStr);
            }
            printf(", output (holding) register table\n");
            break;

        default: // Impossible, la valeur a été vérifiée, évite un warning de gcc
            break;
    }
    putcharInternal('\n');
}

int printfInternal(char const* format, ...) {
    // va_list va;

    // va_start (va, format);
    // return vprintf(format, va);
    return 0;
}

int putcharInternal(int c) {
    // return putchar(c);
    return 0;
}

// -----------------------------------------------------------------------------
// Allocation de la mémoire pour les données à écrire ou à lire
void vAllocate(xMbPollContext* ctx) {
    size_t ulDataSize = ctx->iCount;
    switch (ctx->eFunction) {
        case eFuncCoil:
        case eFuncDiscreteInput:
            // 1 bit est stocké dans un octet
            break;

        case eFuncInputReg:
        case eFuncHoldingReg:
            if ((ctx->eFormat == eFormatInt) || (ctx->eFormat == eFormatFloat)) {
                // Registres 32-bits
                ulDataSize *= 4;
            } else {
                // Registres 16-bits
                ulDataSize *= 2;
            }
            break;

        default: // Impossible, la valeur a été vérifiée, évite un warning de gcc
            break;
    }
    ctx->pvData = calloc(1, ulDataSize);
    assert(ctx->pvData);
}

// -----------------------------------------------------------------------------
void vSigIntHandler(int sig) {
    if ((ctx.bIsPolling)) {
        printf(
            "--- %s poll statistics ---\n"
            "%d frames transmitted, %d received, %d errors, %.1f%% frame loss\n",
            ctx.sDevice,
            ctx.iTxCount,
            ctx.iRxCount,
            ctx.iErrorCount,
            (double)(ctx.iTxCount - ctx.iRxCount) * 100.0 / (double)ctx.iTxCount);
    }

    free(ctx.pvData);
    free(ctx.piSlaveAddr);
    modbus_close(ctx.xBus);
    modbus_free(ctx.xBus);
#ifdef USE_CHIPIO
    // -----------------------------------------------------------------------------
    vChipIoSerialDelete(xChipSerial);
    iChipIoClose(xChip);
// -----------------------------------------------------------------------------
#endif /* USE_CHIPIO defined */
    if (sig == SIGINT) {
        printf("\nEverything was closed neatly.\nHave a nice day!\n");
    } else {
        putchar('\n');
    }
    fflush(stdout);
    exit(ctx.iErrorCount == 0 ? EXIT_SUCCESS : EXIT_FAILURE);
}

// -----------------------------------------------------------------------------
void vFailureExit(bool bHelp, char const* format, ...) {
    va_list va;

    va_start(va, format);
    fprintf(stderr, "%s: ", progname);
    vfprintf(stderr, format, va);
    if (bHelp) {
        fprintf(stderr, " ! Try -h for help.\n");
    } else {
        fprintf(stderr, ".\n");
    }
    va_end(va);
    fflush(stderr);
    free(ctx.pvData);
    free(ctx.piSlaveAddr);
    exit(EXIT_FAILURE);
}

// -----------------------------------------------------------------------------
void vVersion(void) {
    printf("%s\n", VERSION_SHORT);
    exit(EXIT_SUCCESS);
}

// -----------------------------------------------------------------------------
void vWarranty(void) {
    printf(
        "Copyright (c) 2015-2023 %s, All rights reserved.\n\n"

        " mbpoll is free software: you can redistribute it and/or modify\n"
        " it under the terms of the GNU General Public License as published by\n"
        " the Free Software Foundation, either version 3 of the License, or\n"
        " (at your option) any later version.\n\n"

        " mbpoll is distributed in the hope that it will be useful,\n"
        " but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
        " MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
        " GNU General Public License for more details.\n\n"

        " You should have received a copy of the GNU General Public License\n"
        " along with mbpoll. If not, see <http://www.gnu.org/licenses/>.\n",
        AUTHORS);
    exit(EXIT_SUCCESS);
}

// -----------------------------------------------------------------------------
void vCheckEnum(char const* sName, int iElmt, int const* iList, int iSize) {
    int i;
    for (i = 0; i < iSize; i++) {
        if (iElmt == iList[i]) {
            return;
        }
    }
    vSyntaxErrorExit("Illegal %s: %d", sName, iElmt);
}

// -----------------------------------------------------------------------------
void vCheckIntRange(char const* sName, int i, int min, int max) {
    if ((i < min) || (i > max)) {
        vSyntaxErrorExit("%s out of range (%d)", sName, i);
    }
}

// -----------------------------------------------------------------------------
void vCheckDoubleRange(char const* sName, double d, double min, double max) {
    if ((d < min) || (d > max)) {
        vSyntaxErrorExit("%s out of range (%g)", sName, d);
    }
}

// -----------------------------------------------------------------------------
int iGetEnum(char const* sName, char* sElmt, char const** psStrList, int const* iList, int iSize) {
    int i;
    for (i = 0; i < iSize; i++) {
        if (strcasecmp(sElmt, psStrList[i]) == 0) {
            PDEBUG("Set %s=%s\n", sName, _strlwr(sElmt));
            return iList[i];
        }
    }
    vSyntaxErrorExit("Illegal %s: %s", sName, sElmt);
    return -1;
}

// -----------------------------------------------------------------------------
char const* sEnumToStr(int iElmt, int const* iList, char const** psStrList, int iSize) {
    int i;

    for (i = 0; i < iSize;) {
        if (iElmt == iList[i]) {
            return psStrList[i];
        }
        i++;
    }
    return sUnknownStr;
}

// -----------------------------------------------------------------------------
char const* sModeToStr(eModes eMode) {
    return sEnumToStr(eMode, iModeList, sModeList, SIZEOF_ILIST(iModeList));
}

// -----------------------------------------------------------------------------
char const* sFunctionToStr(eFunctions eFunction) {
    return sEnumToStr(eFunction, iFunctionList, sFunctionList, SIZEOF_ILIST(iFunctionList));
}

// -----------------------------------------------------------------------------
void vPrintIntList(int* iList, int iLen) {
    int i;
    putchar('[');
    for (i = 0; i < iLen; i++) {
        printf("%d", iList[i]);
        if (i != (iLen - 1)) {
            putchar(',');
        } else {
            putchar(']');
        }
    }
}

// -----------------------------------------------------------------------------
int* iGetIntList(char const* name, char const* sList, int* iLen) {
    // 12,3,5:9,45

    int* iList = NULL;
    int i, iFirst = 0, iCount = 0;
    bool bIsLast  = false;
    char const* p = sList;
    char* endptr;

    PDEBUG("iGetIntList(%s)\n", sList);

    // Comptage et vérification de la liste des entiers
    while (*p) {
        i = strtol(p, &endptr, 0);
        if (endptr == p) {
            vSyntaxErrorExit("Illegal %s value: %s", name, p);
        }
        p = endptr;
        PDEBUG("Integer found: %d\n", i);

        if (*p == ':') {
            // i est le premier d'un plage first:last
            if (bIsLast) {
                // il ne peut pas y avoir 2 * ':' de suite !
                vSyntaxErrorExit("Illegal %s delimiter: '%c'", name, *p);
            }
            PDEBUG("Is First\n");
            iFirst  = i;
            bIsLast = true;
        } else if ((*p == ',') || (*p == 0)) {
            if (bIsLast) {
                int iRange, iLast;

                // i est dernier d'une plage first:last
                iLast  = MAX(iFirst, i);
                iFirst = MIN(iFirst, i);
                iRange = iLast - iFirst + 1;
                PDEBUG("Is Last, add %d items\n", iRange);
                iCount += iRange;
                bIsLast = false;
            } else {
                iCount++;
            }
        } else {
            vSyntaxErrorExit("Illegal %s delimiter: '%c'", name, *p);
        }

        if (*p) {
            p++; // On passe le délimiteur
        }
        PDEBUG("iCount=%d\n", iCount);
    }

    if (iCount > 0) {
        int iIndex = 0;

        // Allocation
        iList = calloc(iCount, sizeof(int));

        // Affectation
        p = sList;
        while (*p) {
            i = strtol(p, &endptr, 0);
            p = endptr;

            if (*p == ':') {
                // i est le premier d'un plage first:last
                iFirst  = i;
                bIsLast = true;
            } else if ((*p == ',') || (*p == 0)) {
                if (bIsLast) {
                    // i est dernier d'une plage first:last
                    int iLast = MAX(iFirst, i);
                    iFirst    = MIN(iFirst, i);

                    for (i = iFirst; i <= iLast; i++) {
                        iList[iIndex++] = i;
                    }
                    bIsLast = false;
                } else {
                    iList[iIndex++] = i;
                }
            }

            if (*p) {
                p++; // On passe le délimiteur
            }
        }
#ifdef DEBUG
        if (ctx.bIsVerbose) {
            vPrintIntList(iList, iCount);
            putchar('\n');
        }
#endif
    }
    *iLen = iCount;
    return iList;
}

// -----------------------------------------------------------------------------
int iGetInt(char const* name, char const* num, int base) {
    char* endptr;

    int i = strtol(num, &endptr, base);
    if (endptr == num) {
        vSyntaxErrorExit("Illegal %s value: %s", name, num);
    }

    PDEBUG("Set %s=%d\n", name, i);
    return i;
}

// -----------------------------------------------------------------------------
double dGetDouble(char const* name, char const* num) {
    char* endptr;

    double d = strtod(num, &endptr);
    if (endptr == num) {
        vSyntaxErrorExit("Illegal %s value: %s", name, num);
    }

    PDEBUG("Set %s=%g\n", name, d);
    return d;
}

// -----------------------------------------------------------------------------
float fSwapFloat(float f) {
    float ret = f;

    if (ctx.bIsBigEndian) {
        uint16_t* in  = (uint16_t*)&f;
        uint16_t* out = (uint16_t*)&ret;
        out[0]        = in[1];
        out[1]        = in[0];
    }
    return ret;
}

// -----------------------------------------------------------------------------
int32_t lSwapLong(int32_t l) {
    int32_t ret = l;

    if (ctx.bIsBigEndian) {
        uint16_t* in  = (uint16_t*)&l;
        uint16_t* out = (uint16_t*)&ret;
        out[0]        = in[1];
        out[1]        = in[0];
    }
    return ret;
}

// -----------------------------------------------------------------------------
void mb_delay(unsigned long d) {
    if (d) {
#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
        if (d == -1) {
            sleep(-1);
        } else {
            struct timespec dt;

            dt.tv_nsec = (d % 1000UL) * 1000000UL;
            dt.tv_sec  = d / 1000UL;
            nanosleep(&dt, NULL);
        }
#else
        Sleep(d);
#endif
    }
}

/* ========================================================================== */
