#pragma once

#include <charconv>
#include <cstring>
#include <string>

// Karma files always use '.' for decimals, so number text must not follow the host's C locale.

// strtod semantics: leading space and a '+' are skipped, and *end == s means nothing was read.
template <class T>
T parseReal(const char* s, char** end)
{
    const char* p = s + std::strspn(s, " \t\r\n");
    if (*p == '+' && p[1] != '-')
        ++p;
    T v{};
    auto r = std::from_chars(p, p + std::strlen(p), v);
    *end = const_cast<char*>(r.ec == std::errc() ? r.ptr : s);
    return v;
}

// The shortest text that reads back to the same value.
template <class T>
std::string formatReal(T v)
{
    char buf[32];
    return std::string(buf, std::to_chars(buf, buf + sizeof(buf), v).ptr);
}
