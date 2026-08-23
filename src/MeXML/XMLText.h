#pragma once

#include <cstring>
#include <string>

namespace mexml {

struct Entity
{
    char c;
    const char* name;
};

inline const Entity kEntities[] = {{'&', "&amp;"}, {'<', "&lt;"}, {'>', "&gt;"}, {'"', "&quot;"}, {'\'', "&apos;"}};

inline std::string decode(const std::string& s)
{
    std::string out;
    for (size_t i = 0; i < s.size(); ++i)
    {
        const Entity* match = nullptr;
        if (s[i] == '&')
        {
            for (const Entity& e : kEntities)
                if (s.compare(i, std::strlen(e.name), e.name) == 0)
                    match = &e;
        }
        if (match)
        {
            out += match->c;
            i += std::strlen(match->name) - 1;
        }
        else
            out += s[i];
    }
    return out;
}

inline std::string encode(const char* s)
{
    std::string out;
    for (; *s; ++s)
    {
        const Entity* match = nullptr;
        for (const Entity& e : kEntities)
            if (*s == e.c)
                match = &e;
        out += match ? match->name : std::string(1, *s);
    }
    return out;
}

inline std::string trim(const std::string& s)
{
    const char* ws = " \t\r\n";
    size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos)
        return std::string();
    return s.substr(b, s.find_last_not_of(ws) - b + 1);
}

} // namespace mexml
