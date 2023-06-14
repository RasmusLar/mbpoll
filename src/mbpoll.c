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

#include <pthread.h>
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

/* constants ================================================================ */
static char const sIntStr[]          = "32-bit integer";
static char const sLittleEndianStr[] = "(little endian)";
static char const sBigEndianStr[]    = "(big endian)";
static char* progname;

#ifdef MBPOLL_GPIO_RTS
static char const sRtsPinStr[] = "RTS pin";
#endif

/* structures =============================================================== */
typedef struct xThreadContext {
    modbus_t* readBus;
    modbus_t* writeBus;
    int readAddress;
    int writeAddress;
    int numberOfRegisters;
    uint16_t* values;
    pthread_mutex_t* readLock;
    pthread_mutex_t* writeLock;
    pthread_mutex_t* printLock;
} xThreadContext;

typedef struct xMbPollContext {
    uint16_t* piSlaveAddr;
    uint16_t iCount;
    char const* sTcpPort;
    char const* sDevice;
    bool bIsBigEndian;
    modbus_t* xBus;
    modbus_t* forwardBus;
    char const* forwardDevice;
    uint16_t pvData1[12];
    uint16_t pvData2[8];
    uint64_t iTxCount;
    uint64_t iRxCount;
    uint64_t iErrorCount;

    // Paramètres
    double dTimeout;
    int iPollRate;
    bool bIsVerbose;
    bool bIsPolling;
    bool threadLoop;
    uint8_t threadInstances;
#ifdef MBPOLL_GPIO_RTS
    int iRtsPin;
#endif

} xMbPollContext;

/* private variables ======================================================== */

static xMbPollContext ctx = {
    .iCount       = DEFAULT_NUMOFVALUES,
    .sTcpPort     = DEFAULT_TCP_PORT,
    .sDevice      = NULL,
    .bIsBigEndian = false,
#ifdef MBPOLL_GPIO_RTS
    .iRtsPin = -1,
#endif

    // Variables de travail
    .xBus          = NULL,
    .forwardBus    = NULL,
    .forwardDevice = NULL,
    // Paramètres
    .dTimeout        = DEFAULT_TIMEOUT,
    .iPollRate       = DEFAULT_POLLRATE,
    .bIsVerbose      = false,
    .bIsPolling      = true,
    .threadLoop      = true,
    .threadInstances = 0,
};
static xThreadContext threadContext;
static xThreadContext mainContext;

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
void vPrintConfig(xMbPollContext const* ctx);
void vPrintCommunicationSetup(xMbPollContext const* ctx);
void vFailureExit(bool bHelp, char const* format, ...);
#define vSyntaxErrorExit(fmt, ...) vFailureExit(true, fmt, ##__VA_ARGS__)
#define vIoErrorExit(fmt, ...)     vFailureExit(false, fmt, ##__VA_ARGS__)
void vPrintIntList(int* iList, int iLen);
void vSigIntHandler(int sig);
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

#define DEVICE_1 "localhost"; //"192.168.0.11";
#define DEVICE_2 "192.168.10.4";

// static struct timespec yieldTime = {
//     .tv_nsec = 0x1,
//     .tv_sec  = 0,
// };

// void yield() { nanosleep(&yieldTime, NULL); }

void takeLock(pthread_mutex_t* lock) { pthread_mutex_lock(lock); }

void releaseLock(pthread_mutex_t* lock) { pthread_mutex_unlock(lock); }

void checkForErrors(
    int const* iRet, int const* supposedRet, char const* writeOrRead, pthread_mutex_t* lock) {
    takeLock(lock);
    if (*iRet == *supposedRet) {
        printfInternal("%s %d references.\n", writeOrRead, *iRet);
    } else {
        ctx.iErrorCount++;
        fprintf(
            stderr,
            "%s %d values failed: %s, returned %d\n",
            writeOrRead,
            *supposedRet,
            modbus_strerror(errno),
            *iRet);
    }
    releaseLock(lock);
}

void* forwardThread(void* vThreadContext) {
    xThreadContext* threadContext = vThreadContext;
    ctx.threadInstances++;
    int result = 0;
    do {
        takeLock(threadContext->readLock);
        ctx.iRxCount++;
        result = modbus_read_registers(
            threadContext->readBus,
            threadContext->readAddress,
            threadContext->numberOfRegisters,
            threadContext->values);
        releaseLock(threadContext->readLock);
        checkForErrors(
            &result, &(threadContext->numberOfRegisters), "Read", threadContext->printLock);
        if (result == threadContext->numberOfRegisters && ctx.threadLoop) {
            takeLock(threadContext->writeLock);
            ctx.iTxCount++;
            // if (threadContext->numberOfRegisters == 1) {
            //     result = modbus_write_register(
            //         threadContext->bus, threadContext->address, threadContext->values);
            // }
            result = modbus_write_registers(
                threadContext->writeBus,
                threadContext->writeAddress,
                threadContext->numberOfRegisters,
                threadContext->values);
            releaseLock(threadContext->writeLock);
            checkForErrors(
                &result, &(threadContext->numberOfRegisters), "Write", threadContext->printLock);
        }
        if (result != threadContext->numberOfRegisters) {
            // Auto balancing when errors, take all locks to stop all threads, sleep and restart.
            takeLock(threadContext->readLock);
            takeLock(threadContext->writeLock);
            takeLock(threadContext->printLock);
            usleep(0x400);
            releaseLock(threadContext->readLock);
            releaseLock(threadContext->writeLock);
            releaseLock(threadContext->printLock);
        }
    } while (ctx.threadLoop);
    // fprintf(stderr, "Last usleep: %lu, last yield: %ld\n", microSleep, yieldTime.tv_nsec);
    ctx.threadInstances--;
    return NULL;
}

int main(int argc, char** argv) {
    progname = argv[0];
    // Connection au bus

    ctx.sDevice = DEVICE_1;
    ctx.xBus    = modbus_new_tcp_pi(ctx.sDevice, ctx.sTcpPort);
    if (modbus_connect(ctx.xBus) == -1) {
        modbus_free(ctx.xBus);
        vIoErrorExit("Connection failed to UR at '%s': '%s'", ctx.sDevice, modbus_strerror(errno));
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

    // vSigIntHandler() intercepte le CTRL+C
    signal(SIGINT, vSigIntHandler);
    ctx.forwardDevice = DEVICE_2;
    ctx.forwardBus    = modbus_new_tcp_pi(ctx.forwardDevice, ctx.sTcpPort);

    if (modbus_connect(ctx.forwardBus) == -1) {
        modbus_free(ctx.xBus);
        modbus_free(ctx.forwardBus);
        vIoErrorExit(
            "Connection failed to SEW at '%s': '%s'", ctx.forwardDevice, modbus_strerror(errno));
    }

    modbus_set_response_timeout(ctx.xBus, sec, usec);
    modbus_set_response_timeout(ctx.forwardBus, sec, usec);
    modbus_set_slave(ctx.xBus, 0);
    modbus_set_slave(ctx.forwardBus, 0);

    vPrintConfig(&ctx);
    pthread_t threadId1;
    pthread_t threadId2;
    pthread_mutex_t xBusLock       = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t forwardBusLock = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t printLock      = PTHREAD_MUTEX_INITIALIZER;

    threadContext.readBus           = ctx.xBus;
    threadContext.readLock          = &xBusLock;
    threadContext.readAddress       = 192;
    threadContext.writeBus          = ctx.forwardBus;
    threadContext.writeLock         = &forwardBusLock;
    threadContext.writeAddress      = 4;
    threadContext.numberOfRegisters = 6;
    threadContext.values            = ctx.pvData1;
    threadContext.printLock         = &printLock;

    mainContext.readBus           = ctx.forwardBus;
    mainContext.readLock          = &forwardBusLock;
    mainContext.readAddress       = 4;
    mainContext.writeBus          = ctx.xBus;
    mainContext.writeLock         = &xBusLock;
    mainContext.writeAddress      = 200;
    mainContext.numberOfRegisters = 4;
    mainContext.values            = ctx.pvData2;
    mainContext.printLock         = &printLock;

    if (ctx.bIsPolling) {
        printfInternal(" Ctrl-C to stop)\n");
    } else {
        putcharInternal('\n');
    }
    pthread_create(&threadId1, NULL, forwardThread, &threadContext);
    pthread_create(&threadId2, NULL, forwardThread, &mainContext);
    while (ctx.threadLoop) {
        // ~6.5 ms
        usleep(0x10000);
    }
    // pthread_join(threadId1, NULL);
    // pthread_join(threadId2, NULL);
    vSigIntHandler(SIGTERM);
    return 0;
}

/* private functions ======================================================== */

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
    printf("%s %s", sIntStr, ctx->bIsBigEndian ? sBigEndianStr : sLittleEndianStr);
    printf(", output (holding) register table\n");

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

void waitForThread(uint8_t previousThread) {
    for (size_t i = 0; i < 50 && ctx.threadInstances != 0 && previousThread == ctx.threadInstances;
         i++) {
        usleep(100);
    }
}

// -----------------------------------------------------------------------------
void vSigIntHandler(int sig) {
    ctx.threadLoop = false;
    waitForThread(ctx.threadInstances);
    releaseLock(threadContext.printLock);
    releaseLock(threadContext.readLock);
    releaseLock(threadContext.writeLock);
    waitForThread(ctx.threadInstances);

    if (ctx.threadInstances != 0) {
        fprintf(stderr, "Threads not closed properly, still %d running\n", ctx.threadInstances);
    }
    if ((ctx.bIsPolling)) {
        printf(
            "--- %s poll statistics ---\n"
            "%lu frames written, %lu read, %lu errors, %.1Lf%% frame loss\n",
            ctx.sDevice,
            ctx.iTxCount,
            ctx.iRxCount,
            ctx.iErrorCount,
            (long double)(ctx.iErrorCount) * 100.0 / (long double)ctx.iRxCount);
    }

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
    free(ctx.piSlaveAddr);
    exit(EXIT_FAILURE);
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
