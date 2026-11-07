#include <cstdlib>
#include <cstring>

#include <MeCommandLine.h>

#include "Numbers.h"

namespace {

void eatAt(MeCommandLineOptions* options, int pos, int count)
{
    int eat[] = {pos, pos + 1};
    MeCommandLineOptionsEat(options, eat, count);
}

// The value that follows arg, or null if arg or its value is absent.
const char* value(MeCommandLineOptions* options, const char* arg, MeBool eat)
{
    int pos = MeCommandLineOptionsGetPos(options, arg);
    if (pos < 0 || pos + 1 >= options->m_argc)
        return nullptr;
    const char* v = options->p_argv[pos + 1];
    if (eat)
        eatAt(options, pos, 2);
    return v;
}

} // namespace

extern "C" {

MeCommandLineOptions* MEAPI MeCommandLineOptionsCreate(const int argc, const char** argv)
{
    // A private copy of the pointer array, so eating options leaves the caller's argv alone.
    auto* options = static_cast<MeCommandLineOptions*>(MeMemoryAPI.create(sizeof(MeCommandLineOptions)));
    options->p_argv = static_cast<const char**>(MeMemoryAPI.create(sizeof(char*) * (argc + 1)));
    for (int i = 0; i < argc; ++i)
        options->p_argv[i] = argv[i];
    options->p_argv[argc] = nullptr;
    options->m_argc = argc;
    return options;
}

void MEAPI MeCommandLineOptionsDestroy(MeCommandLineOptions* options)
{
    if (!options)
        return;
    MeMemoryAPI.destroy(options->p_argv);
    MeMemoryAPI.destroy(options);
}

void MEAPI MeCommandLineOptionsEat(MeCommandLineOptions* options, const int* eat, const int num_eat)
{
    int kept = 0;
    for (int i = 0; i < options->m_argc; ++i)
    {
        bool eaten = false;
        for (int j = 0; j < num_eat && !eaten; ++j)
            eaten = eat[j] == i;
        if (!eaten)
            options->p_argv[kept++] = options->p_argv[i];
    }
    options->m_argc = kept;
    options->p_argv[kept] = nullptr;
}

int MEAPI MeCommandLineOptionsGetPos(MeCommandLineOptions* options, const char* arg)
{
    for (int i = 0; i < options->m_argc; ++i)
        if (std::strcmp(options->p_argv[i], arg) == 0)
            return i;
    return -1;
}

MeBool MEAPI MeCommandLineOptionsCheckFor(MeCommandLineOptions* options, const char* arg, const MeBool eat)
{
    int pos = MeCommandLineOptionsGetPos(options, arg);
    if (pos < 0)
        return MEFALSE;
    if (eat)
        eatAt(options, pos, 1);
    return 1;
}

MeBool MEAPI MeCommandLineOptionsCheckForList(MeCommandLineOptions* options, const char* arglist[], const MeBool eat)
{
    MeBool found = MEFALSE;
    for (int i = 0; arglist[i]; ++i)
        if (MeCommandLineOptionsCheckFor(options, arglist[i], eat))
            found = 1;
    return found;
}

int MEAPI MeCommandLineOptionsGetNumeric(MeCommandLineOptions* options, const char* arg, const MeBool eat)
{
    const char* v = value(options, arg, eat);
    return v ? std::atoi(v) : 0;
}

double MEAPI MeCommandLineOptionsGetFloat(MeCommandLineOptions* options, const char* arg, const MeBool eat)
{
    const char* v = value(options, arg, eat);
    char* end;
    return v ? parseNumber<double>(v, &end) : 0.0;
}

char* MEAPI MeCommandLineOptionsGetString(MeCommandLineOptions* options, const char* arg, const MeBool eat)
{
    return const_cast<char*>(value(options, arg, eat));
}

} // extern "C"
