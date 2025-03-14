#ifndef DISPLAY_CHECK_H
#define DISPLAY_CHECK_H

#ifdef _WIN32
#include <Windows.h>
#define HAS_DISPLAY (GetSystemMetrics(SM_CMONITORS) > 0)
#elif defined(__linux__)
#include <cstdlib>
#define HAS_DISPLAY (std::getenv("DISPLAY") != nullptr)
#else
#define HAS_DISPLAY false // Default to no display for unknown platforms
#endif

#endif // DISPLAY_CHECK_H