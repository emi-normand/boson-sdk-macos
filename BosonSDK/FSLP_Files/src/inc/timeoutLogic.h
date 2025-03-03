#ifndef TIMEOUT_LOGIC_H
#define TIMEOUT_LOGIC_H

#ifdef _WIN32
    // Windows-specific includes
    #include <Windows.h>
#else
    // Unix/macOS includes
    #include <sys/time.h>
    #include <unistd.h>
#endif

// Cross-platform timeout structure
typedef struct {
#ifdef _WIN32
    LARGE_INTEGER startTime;
    LARGE_INTEGER frequency;
#else
    struct timeval startTime;
#endif
    unsigned int timeoutMs;
} TimeoutLogic;

// Function prototypes
void initTimeout(TimeoutLogic* timeout, unsigned int timeoutMs);
int hasTimedOut(TimeoutLogic* timeout);
void sleepMs(unsigned int ms);

#endif // TIMEOUT_LOGIC_H 