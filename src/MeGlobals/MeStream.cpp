#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <MeSimpleFile.h>
#include <MeStream.h>

#include "StringCopy.h"

namespace {

const char* modeString(MeOpenMode_enum mode)
{
    switch (mode)
    {
    case kMeOpenModeWRONLY: return "w";
    case kMeOpenModeRDWR: return "r+";
    case kMeOpenModeRDBINARY: return "rb";
    default: return "r";
    }
}

void reserve(MeStream s, unsigned int needed)
{
    if (needed <= s->bufSize)
        return;
    unsigned int size = s->bGrowFast ? (s->bufSize ? s->bufSize : 64) : needed;
    while (size < needed)
        size *= 2;
    s->buffer = static_cast<char*>(std::realloc(s->buffer, size));
    s->bufSize = size;
}

} // namespace

extern "C" {

MeStream MEAPI MeStreamOpen(const char* filename, MeOpenMode_enum mode)
{
    std::FILE* file = std::fopen(filename, modeString(mode));
    if (!file)
        return nullptr;
    MeStream s = static_cast<MeStream>(std::calloc(1, sizeof(_MeStream)));
    s->filename = duplicateString(filename);
    s->handle = file;
    return s;
}

MeStream MEAPI MeStreamOpenWithSearch(const char* filename, MeOpenMode_enum mode)
{
    MeStream s = MeStreamOpen(filename, mode);
    for (int i = 0; !s && MeGetDefaultFileLocation(i); ++i)
        s = MeStreamOpen((std::string(MeGetDefaultFileLocation(i)) + filename).c_str(), mode);
    return s;
}

MeStream MEAPI MeStreamOpenAsMemBuffer(unsigned int initialSize)
{
    MeStream s = static_cast<MeStream>(std::calloc(1, sizeof(_MeStream)));
    s->bUseMemblock = 1;
    s->bGrowFast = 1;
    s->buffer = static_cast<char*>(std::malloc(initialSize ? initialSize : 1));
    s->bufSize = initialSize;
    return s;
}

MeStream MEAPI MeStreamCreateFromMemBuffer(char* buffer, unsigned int bufLength, unsigned int bufSize)
{
    // The stream copies the caller's block so that growing and closing never touch memory it does not own.
    MeStream s = MeStreamOpenAsMemBuffer(bufSize > bufLength ? bufSize : bufLength);
    std::memcpy(s->buffer, buffer, bufLength);
    s->bufLength = bufLength;
    return s;
}

void MEAPI MeStreamMemBufferFreeSlackSpace(MeStream stream)
{
    if (!stream->bUseMemblock || stream->bufSize == stream->bufLength)
        return;
    stream->buffer = static_cast<char*>(std::realloc(stream->buffer, stream->bufLength ? stream->bufLength : 1));
    stream->bufSize = stream->bufLength;
}

void MEAPI MeStreamMemBufferUseConservativeGrowth(MeStream stream, MeBool bConsGrowth)
{
    stream->bGrowFast = !bConsGrowth;
}

void MEAPI MeStreamClose(MeStream stream)
{
    if (!stream)
        return;
    if (stream->handle)
        std::fclose(static_cast<std::FILE*>(stream->handle));
    std::free(stream->buffer);
    std::free(const_cast<char*>(stream->filename));
    std::free(stream);
}

size_t MEAPI MeStreamRead(void* buffer, size_t size, size_t count, MeStream stream)
{
    if (!stream->bUseMemblock)
        return std::fread(buffer, size, count, static_cast<std::FILE*>(stream->handle));
    if (size == 0)
        return 0;
    size_t items = (stream->bufLength - stream->curIndex) / size;
    if (items > count)
        items = count;
    std::memcpy(buffer, stream->buffer + stream->curIndex, items * size);
    stream->curIndex += static_cast<unsigned int>(items * size);
    return items;
}

char* MEAPI MeStreamReadLine(char* string, int n, MeStream stream)
{
    if (!stream->bUseMemblock)
        return std::fgets(string, n, static_cast<std::FILE*>(stream->handle));
    if (n <= 0 || stream->curIndex >= stream->bufLength)
        return nullptr;
    int i = 0;
    while (i < n - 1 && stream->curIndex < stream->bufLength)
    {
        char c = stream->buffer[stream->curIndex++];
        string[i++] = c;
        if (c == '\n')
            break;
    }
    string[i] = '\0';
    return string;
}

size_t MEAPI MeStreamWrite(void* buffer, size_t size, size_t count, MeStream stream)
{
    if (!stream->bUseMemblock)
        return std::fwrite(buffer, size, count, static_cast<std::FILE*>(stream->handle));
    unsigned int bytes = static_cast<unsigned int>(size * count);
    reserve(stream, stream->curIndex + bytes);
    std::memcpy(stream->buffer + stream->curIndex, buffer, bytes);
    stream->curIndex += bytes;
    if (stream->curIndex > stream->bufLength)
        stream->bufLength = stream->curIndex;
    return count;
}

void MEAPI MeStreamRewind(MeStream stream)
{
    if (stream->bUseMemblock)
        stream->curIndex = 0;
    else
        std::rewind(static_cast<std::FILE*>(stream->handle));
}

} // extern "C"
