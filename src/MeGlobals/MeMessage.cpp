#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <MeMessage.h>

extern "C" {

int MeInfoLevel = 0;
int MeLogLevel = 0;
int MeWarningLevel = 0;
int MeFatalErrorLevel = 0;
int MeDebugLevel = 0;

void MEAPI MeShowStdout(const int, const char* const string)
{
    std::fputs(string, stdout);
    std::fputc('\n', stdout);
}

void MEAPI MeShowStderr(const int, const char* const string)
{
    std::fputs(string, stderr);
    std::fputc('\n', stderr);
}

MeShow MeInfoShow = MeShowStdout;
MeShow MeLogShow = MeShowStdout;
MeShow MeWarningShow = MeShowStderr;
MeShow MeFatalErrorShow = MeShowStderr;
MeShow MeDebugShow = MeShowStderr;

} // extern "C"

namespace {

void show(MeShow out, int limit, const char* prefix, int level, const char* format, va_list ap)
{
    if (level > limit || !out)
        return;
    char buffer[MeMAXMESSAGE];
    int n = std::snprintf(buffer, sizeof(buffer), "%s", prefix);
    std::vsnprintf(buffer + n, sizeof(buffer) - n, format, ap);
    out(level, buffer);
}

} // namespace

extern "C" {

void MEAPI MeHandlerInfo(const int level, const char* const format, va_list ap)
{
    show(MeInfoShow, MeInfoLevel, "", level, format, ap);
}

void MEAPI MeHandlerLog(const int level, const char* const format, va_list ap)
{
    show(MeLogShow, MeLogLevel, "", level, format, ap);
}

void MEAPI MeHandlerWarning(const int level, const char* const format, va_list ap)
{
    show(MeWarningShow, MeWarningLevel, "Warning: ", level, format, ap);
}

void MEAPI MeHandlerFatalError(const int level, const char* const format, va_list ap)
{
    show(MeFatalErrorShow, MeFatalErrorLevel, "Fatal error: ", level, format, ap);
    std::abort();
}

void MEAPI MeHandlerDebug(const int level, const char* const format, va_list ap)
{
    show(MeDebugShow, MeDebugLevel, "Debug: ", level, format, ap);
}

MeHandler MeInfoHandler = MeHandlerInfo;
MeHandler MeLogHandler = MeHandlerLog;
MeHandler MeWarningHandler = MeHandlerWarning;
MeHandler MeFatalErrorHandler = MeHandlerFatalError;
MeHandler MeDebugHandler = MeHandlerDebug;

#define ME_SWAP(Type, var, n) \
    Type old = var;           \
    var = n;                  \
    return old

MeShow MEAPI MeSetInfoShow(MeShow n) { ME_SWAP(MeShow, MeInfoShow, n); }
MeShow MEAPI MeSetLogShow(MeShow n) { ME_SWAP(MeShow, MeLogShow, n); }
MeShow MEAPI MeSetWarningShow(MeShow n) { ME_SWAP(MeShow, MeWarningShow, n); }
MeShow MEAPI MeSetFatalErrorShow(MeShow n) { ME_SWAP(MeShow, MeFatalErrorShow, n); }
MeShow MEAPI MeSetDebugShow(MeShow n) { ME_SWAP(MeShow, MeDebugShow, n); }
MeHandler MEAPI MeSetInfoHandler(MeHandler n) { ME_SWAP(MeHandler, MeInfoHandler, n); }
MeHandler MEAPI MeSetLogHandler(MeHandler n) { ME_SWAP(MeHandler, MeLogHandler, n); }
MeHandler MEAPI MeSetWarningHandler(MeHandler n) { ME_SWAP(MeHandler, MeWarningHandler, n); }
MeHandler MEAPI MeSetFatalErrorHandler(MeHandler n) { ME_SWAP(MeHandler, MeFatalErrorHandler, n); }
MeHandler MEAPI MeSetDebugHandler(MeHandler n) { ME_SWAP(MeHandler, MeDebugHandler, n); }

#define ME_DISPATCH(handler)            \
    va_list ap;                         \
    va_start(ap, format);               \
    if (handler)                        \
        handler(level, format, ap);     \
    va_end(ap)

void MEAPI MeInfo(const int level, const char* const format, ...) { ME_DISPATCH(MeInfoHandler); }
void MEAPI MeLog(const int level, const char* const format, ...) { ME_DISPATCH(MeLogHandler); }
void MEAPI MeWarning(const int level, const char* const format, ...) { ME_DISPATCH(MeWarningHandler); }
void MEAPI MeDebug(const int level, const char* const format, ...) { ME_DISPATCH(MeDebugHandler); }

void MEAPI MeFatalError(const int level, const char* const format, ...)
{
    ME_DISPATCH(MeFatalErrorHandler);
    // A replacement handler may return, but callers rely on this call not returning.
    std::abort();
}

char* MEAPI MeShortenPath(char* path)
{
    char* slash = std::strrchr(path, '/');
    char* backslash = std::strrchr(path, '\\');
    char* last = slash > backslash ? slash : backslash;
    return last ? last + 1 : path;
}

} // extern "C"
