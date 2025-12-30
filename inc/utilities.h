#ifndef __UTILITIES_H__
#define __UTILITIES_H__
#include <signal.h>

#define MAXSTRLEN           64
#define MAXNUMSAMPLES       0x200
#define BYTESPERLED         0x03
#define BYTESPERSAMPLE      (BYTESPERLED*2)

typedef enum {
    PING = 0,
    PONG = 1,
} pingpongIndex_t;

typedef enum {
    NOERROR=SIGINT+1,//SIGINT Ctrl C is reserved.
    READMAX30102_FIFO_ERROR,
    DONE,
} TerminateError;

void terminate(int); 
#endif
