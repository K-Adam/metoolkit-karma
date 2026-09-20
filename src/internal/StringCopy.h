#pragma once

#include <cstdlib>
#include <cstring>

// Every owner of these copies releases them with std::free, so they must come from std::malloc.
inline char* duplicateString(const char* s)
{
    if (!s)
        return nullptr;
    size_t n = std::strlen(s) + 1;
    char* copy = static_cast<char*>(std::malloc(n));
    std::memcpy(copy, s, n);
    return copy;
}
