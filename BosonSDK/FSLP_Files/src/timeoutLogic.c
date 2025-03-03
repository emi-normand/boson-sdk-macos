#include "inc/timeoutLogic.h"

void initTimeout(TimeoutLogic* timeout, unsigned int timeoutMs) {
    timeout->timeoutMs = timeoutMs;
    
#ifdef _WIN32
    // Windows implementation
    QueryPerformanceFrequency(&timeout->frequency);
    QueryPerformanceCounter(&timeout->startTime);
#else
    // Unix/macOS implementation
    gettimeofday(&timeout->startTime, NULL);
#endif
}

int hasTimedOut(TimeoutLogic* timeout) {
#ifdef _WIN32
    // Windows implementation
    LARGE_INTEGER currentTime;
    QueryPerformanceCounter(&currentTime);
    
    LONGLONG elapsedTime = currentTime.QuadPart - timeout->startTime.QuadPart;
    LONGLONG elapsedMs = (elapsedTime * 1000) / timeout->frequency.QuadPart;
    
    return elapsedMs >= timeout->timeoutMs;
#else
    // Unix/macOS implementation
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    
    long elapsedSec = currentTime.tv_sec - timeout->startTime.tv_sec;
    long elapsedUsec = currentTime.tv_usec - timeout->startTime.tv_usec;
    
    // Convert to milliseconds
    long elapsedMs = (elapsedSec * 1000) + (elapsedUsec / 1000);
    
    return elapsedMs >= timeout->timeoutMs;
#endif
}

void sleepMs(unsigned int ms) {
#ifdef _WIN32
    // Windows implementation
    Sleep(ms);
#else
    // Unix/macOS implementation
    usleep(ms * 1000);
#endif
}