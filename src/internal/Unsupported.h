#pragma once

#include <MeMessage.h>

// Karma's own solver is replaced by Bullet. Callers relying on its internals learn so once, on stderr.
#define ME_UNSUPPORTED(name)                                    \
    do                                                          \
    {                                                           \
        static bool warned = false;                             \
        if (!warned)                                            \
        {                                                       \
            warned = true;                                      \
            MeWarning(0, "%s is not supported", name);          \
        }                                                       \
    } while (0)
