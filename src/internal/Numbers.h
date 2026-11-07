#pragma once

#include <charconv>
#include <cmath>
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
    auto r = std::from_chars(p, p + std::strlen(p), v);
    bool ok = r.ec == std::errc();
    // from_chars accepts "nan" and "inf", which no Karma file holds and which would poison the simulation.
    if constexpr (std::is_floating_point_v<T>)
        ok = ok && std::isfinite(v);
    *end = const_cast<char*>(ok ? r.ptr : s);
    return ok ? v : T{};
}

// The shortest text that reads back to the same value.
template <class T>
std::string formatReal(T v)
{
    char buf[32];
    return std::string(buf, std::to_chars(buf, buf + sizeof(buf), v).ptr);
}
