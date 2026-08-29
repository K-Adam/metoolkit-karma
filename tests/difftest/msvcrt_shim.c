/* The 2002 libs import msvcrt.dll stdio, which UCRT inlines or no longer exports.
   Old msvcrt _iob was an array of 32-byte FILEs. */
#include <stdio.h>
#include <stdarg.h>
static char fake_iob[3 * 32];
char *_imp___iob = fake_iob;
static FILE *map(FILE *f)
{
    char *p = (char *)f;
    return (p >= fake_iob && p < fake_iob + sizeof fake_iob) ? __acrt_iob_func((unsigned)(p - fake_iob) / 32) : f;
}
static int __cdecl s_fputs(const char *s, FILE *f) { return fprintf(map(f), "%s", s); }
static size_t __cdecl s_fwrite(const void *b, size_t s, size_t n, FILE *f) { return _fwrite_nolock(b, s, n, map(f)); }
static int __cdecl s_fprintf(FILE *f, const char *fmt, ...) { int r; va_list a; va_start(a, fmt); r = vfprintf(map(f), fmt, a); va_end(a); return r; }
static int __cdecl s_printf(const char *fmt, ...) { int r; va_list a; va_start(a, fmt); r = vprintf(fmt, a); va_end(a); return r; }
static int __cdecl s_sprintf(char *d, const char *fmt, ...) { int r; va_list a; va_start(a, fmt); r = vsprintf(d, fmt, a); va_end(a); return r; }
static int __cdecl s_snprintf(char *d, size_t n, const char *fmt, ...) { int r; va_list a; va_start(a, fmt); r = _vsnprintf(d, n, fmt, a); va_end(a); return r; }
static int __cdecl s_vsnprintf(char *d, size_t n, const char *fmt, va_list a) { return _vsnprintf(d, n, fmt, a); }
static int __cdecl s_sscanf(const char *s, const char *fmt, ...) { int r; va_list a; va_start(a, fmt); r = vsscanf(s, fmt, a); va_end(a); return r; }
void *_imp__fputs = s_fputs, *_imp__fwrite = s_fwrite, *_imp__fprintf = s_fprintf, *_imp__printf = s_printf,
     *_imp__sprintf = s_sprintf, *_imp___snprintf = s_snprintf, *_imp___vsnprintf = s_vsnprintf, *_imp__sscanf = s_sscanf;

/* The reference libs report messages in modal dialogs, which block unattended runs. */
#include <MeMessage.h>
static int __cdecl quiet_messages(void)
{
    MeSetFatalErrorShow(MeShowStderr);
    MeSetWarningShow(MeShowStderr);
    MeSetInfoShow(MeShowStderr);
    MeSetLogShow(MeShowStderr);
    MeSetDebugShow(MeShowStderr);
    return 0;
}
#pragma section(".CRT$XIU", read)
__declspec(allocate(".CRT$XIU")) int(__cdecl *quiet_messages_init)(void) = quiet_messages;
