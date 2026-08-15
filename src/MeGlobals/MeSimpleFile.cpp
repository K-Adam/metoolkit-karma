#include <fcntl.h>
#include <string>
#include <sys/stat.h>
#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#define O_BINARY 0
#define _open open
#define _lseek lseek
#define _read read
#define _write write
#define _close close
#endif

#include <MeMemory.h>
#include <MeSimpleFile.h>

namespace {

// Demos are run from their own directory with shared data next to them in Resources.
const char* const searchPaths[] = {"../Resources/", "../../Resources/", "../../../Resources/"};

} // namespace

extern "C" {

int MEAPI MeOpenRaw(const char* filename, MeOpenMode_enum mode)
{
    switch (mode)
    {
    case kMeOpenModeRDONLY: return _open(filename, O_RDONLY);
    case kMeOpenModeWRONLY: return _open(filename, O_WRONLY | O_CREAT | O_TRUNC, S_IREAD | S_IWRITE);
    case kMeOpenModeRDWR: return _open(filename, O_RDWR);
    case kMeOpenModeRDBINARY: return _open(filename, O_RDONLY | O_BINARY);
    }
    return -1;
}

int MEAPI MeOpen(const char* filename, MeOpenMode_enum mode)
{
    return MeOpenRaw(filename, mode);
}

int MEAPI MeOpenWithSearch(const char* filename, MeOpenMode_enum mode)
{
    int f = MeOpen(filename, mode);
    for (int i = 0; f < 0 && MeGetDefaultFileLocation(i); ++i)
        f = MeOpen((std::string(MeGetDefaultFileLocation(i)) + filename).c_str(), mode);
    return f;
}

int MEAPI MeLseek(int file, int offset, MeSeekOrigin_enum origin)
{
    static const int whence[] = {SEEK_SET, SEEK_CUR, SEEK_END};
    if (origin < kMeSeekSET || origin > kMeSeekEND)
        return -1;
    return static_cast<int>(_lseek(file, offset, whence[origin]));
}

int MEAPI MeRead(int file, void* buf, int count)
{
    return _read(file, buf, count);
}

int MEAPI MeWrite(int file, void* buf, int count)
{
    return _write(file, buf, count);
}

int MEAPI MeClose(int file)
{
    return _close(file);
}

const char* MEAPI MeGetDefaultFileLocation(int i)
{
    return i >= 0 && i < int(sizeof(searchPaths) / sizeof(*searchPaths)) ? searchPaths[i] : nullptr;
}

char* MEAPI MeLoadWholeFileHandle(const int f, int* size)
{
    int length = MeLseek(f, 0, kMeSeekEND);
    if (length < 0 || MeLseek(f, 0, kMeSeekSET) < 0)
        return nullptr;
    // One spare byte so text files come back NUL terminated.
    char* data = static_cast<char*>(MeMemoryAPI.create(length + 1));
    if (!data)
        return nullptr;
    // Text mode reads can return fewer bytes than the file length.
    int n = MeRead(f, data, length);
    if (n < 0)
    {
        MeMemoryAPI.destroy(data);
        return nullptr;
    }
    data[n] = '\0';
    if (size)
        *size = n;
    return data;
}

void MEAPI MeSaveWholeFileHandle(const int f, const char* data, const int size)
{
    MeWrite(f, const_cast<char*>(data), size);
}

} // extern "C"
