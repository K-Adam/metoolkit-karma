#pragma once

#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>

// Karma files always use '.' for decimals, so number text must not follow the host's C locale.

// strtod semantics: leading space and a '+' are skipped, and *end == s means nothing was read. Unlike strtol,
// an out of range value or a '-' on an unsigned type is not read either.
template <class T>
T parseNumber(const char* s, char** end)
{
    const char* p = s + std::strspn(s, " \t\r\n");
    if (*p == '+' && p[1] != '-')
        ++p;
    T v{};
    const char* e;
#ifdef __ANDROID__
    // The NDK's libc++ has no floating point from_chars. Bionic's strtod ignores the locale anyway.
    if constexpr (std::is_floating_point_v<T>)
    {
        char* d;
        v = (T)std::strtod(p, &d);
        e = d != p ? d : nullptr;
    }
    else
#endif
    {
        auto r = std::from_chars(p, p + std::strlen(p), v);
        e = r.ec == std::errc() ? r.ptr : nullptr;
    }
    bool ok = e != nullptr;
    // from_chars accepts "nan" and "inf", which no Karma file holds and which would poison the simulation.
    if constexpr (std::is_floating_point_v<T>)
        ok = ok && std::isfinite(v);
    *end = const_cast<char*>(ok ? e : s);
    return ok ? v : T{};
}

// The shortest text that reads back to the same value.
template <class T>
std::string formatReal(T v)
{
    char buf[32];
    return std::string(buf, std::to_chars(buf, buf + sizeof(buf), v).ptr);
}
