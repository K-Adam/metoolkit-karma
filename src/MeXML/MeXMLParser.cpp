#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <MeXMLParser.h>

#include "XMLText.h"

namespace {

struct InputImpl
{
    MeXMLInput pub;
    int depth;        // elements opened and not yet closed
    bool selfClosing; // the last start tag was <name/>
};

InputImpl* impl(MeXMLInput* in)
{
    return reinterpret_cast<InputImpl*>(in);
}

enum class Token
{
    Start,
    End,
    Eof
};

int get(MeXMLInput* in)
{
    if (in->top > 0)
        return in->stack[--in->top];
    if (in->bufptr >= in->bufmax)
    {
        in->bufmax = static_cast<int>(MeStreamRead(in->buffer, 1, sizeof(in->buffer), in->stream));
        in->bufptr = 0;
        if (in->bufmax <= 0)
        {
            in->eof = 1;
            return EOF;
        }
    }
    char c = in->buffer[in->bufptr++];
    if (c == '\n')
    {
        ++in->line;
        in->posn = 0;
    }
    else
        ++in->posn;
    return static_cast<unsigned char>(c);
}

MeXMLError fail(MeXMLInput* in, MeXMLError code, const char* what)
{
    std::snprintf(in->error, sizeof(in->error), "line %d: %s", in->line + 1, what);
    return code;
}

// Reads the text before the next markup and the markup itself, skipping comments,
// declarations and processing instructions.
Token nextTag(MeXMLInput* in, std::string& text, std::string& tag)
{
    text.clear();
    for (;;)
    {
        int c;
        while ((c = get(in)) != EOF && c != '<')
            text += static_cast<char>(c);
        if (c == EOF)
            return Token::Eof;
        tag.clear();
        char quote = 0;
        while ((c = get(in)) != EOF)
        {
            if (c == '>' && !quote && (tag.compare(0, 3, "!--") != 0 || (tag.size() >= 5 && tag.compare(tag.size() - 2, 2, "--") == 0)))
                break;
            if ((c == '"' || c == '\'') && tag.compare(0, 3, "!--") != 0)
            {
                if (!quote)
                    quote = static_cast<char>(c);
                else if (quote == c)
                    quote = 0;
            }
            tag += static_cast<char>(c);
        }
        if (c == EOF)
            return Token::Eof;
        if (tag.empty() || tag[0] == '?' || tag[0] == '!')
            continue;
        InputImpl* x = impl(in);
        if (tag[0] == '/')
        {
            tag = mexml::trim(tag.substr(1));
            --x->depth;
            return Token::End;
        }
        x->selfClosing = tag.back() == '/';
        if (x->selfClosing)
            tag.pop_back();
        else
            ++x->depth;
        return Token::Start;
    }
}

// Reads the character data of the element whose start tag was just read, and its end tag.
MeXMLError readPCDATA(MeXMLInput* in, std::string& out)
{
    out.clear();
    if (impl(in)->selfClosing)
    {
        impl(in)->selfClosing = false;
        return MeXMLErrorNone;
    }
    std::string tag;
    switch (nextTag(in, out, tag))
    {
    case Token::End:
        out = mexml::trim(mexml::decode(out));
        return MeXMLErrorNone;
    case Token::Start:
        return fail(in, MeXMLErrorMalformed, "element found where character data was expected");
    default:
        return fail(in, MeXMLErrorEOF, "unexpected end of file");
    }
}

MeXMLHandler* findHandler(MeXMLHandler handlers[], const char* name)
{
    for (MeXMLHandler* h = handlers; h->type != MeXMLActionEnd; ++h)
        if (h->name && std::strcmp(h->name, name) == 0)
            return h;
    return nullptr;
}

// Processes the children of parent, or the top level of the document when parent is null.
MeXMLError processContent(MeXMLInput* in, MeXMLElement* parent, MeXMLHandler handlers[], void* data, void* userdata)
{
    InputImpl* x = impl(in);
    int level = parent ? parent->level : 0;
    // A self-closing or already processed element has no content left to read.
    if (x->depth < level)
        return MeXMLErrorNone;
    for (MeXMLHandler* h = handlers; h->type != MeXMLActionEnd; ++h)
        h->called = 0;

    std::string text, tag;
    for (;;)
    {
        int base = x->depth;
        Token t = nextTag(in, text, tag);
        if (t == Token::Eof)
            return parent ? fail(in, MeXMLErrorEOF, "unexpected end of file") : MeXMLErrorNone;
        if (t == Token::End)
        {
            if (!parent || tag != parent->name)
                return fail(in, MeXMLErrorMalformed, "mismatched end tag");
            return MeXMLErrorNone;
        }

        MeXMLElement child{};
        std::strncpy(child.name, tag.c_str(), sizeof(child.name) - 1);
        child.fi = in;
        child.level = level + 1;
        size_t space = std::strcspn(child.name, " \t\r\n");
        if (child.name[space])
        {
            child.name[space] = '\0';
            child.attr = child.name + space + 1;
        }

        MeXMLError err = MeXMLErrorNone;
        if (MeXMLHandler* h = findHandler(handlers, child.name))
        {
            h->called = 1;
            if (h->type == MeXMLActionCallback)
                err = reinterpret_cast<MeXMLCallback>(h->fn)(&child, userdata);
            else
            {
                err = reinterpret_cast<MeXMLParseFn>(h->fn)(in, h, data);
                if (err == MeXMLErrorNone && h->cb)
                    err = h->cb(&child, data, userdata);
            }
        }
        if (err != MeXMLErrorNone)
            return err;

        // Skip whatever the handler left unread, including unhandled elements.
        while (x->depth > base)
        {
            if (nextTag(in, text, tag) == Token::Eof)
                return fail(in, MeXMLErrorEOF, "unexpected end of file");
        }
    }
}

template <class T>
T convert(const char* s, char** end);

template <>
int convert<int>(const char* s, char** end)
{
    return static_cast<int>(std::strtol(s, end, 10));
}

template <>
unsigned int convert<unsigned int>(const char* s, char** end)
{
    return static_cast<unsigned int>(std::strtoul(s, end, 10));
}

template <>
float convert<float>(const char* s, char** end)
{
    return std::strtof(s, end);
}

template <>
double convert<double>(const char* s, char** end)
{
    return std::strtod(s, end);
}

template <class T>
MeXMLError parseArray(MeXMLInput* in, const MeXMLHandler* h, void* data, unsigned int count)
{
    std::string text;
    if (MeXMLError err = readPCDATA(in, text))
        return err;
    T* out = reinterpret_cast<T*>(static_cast<char*>(data) + h->offset);
    char* cursor = &text[0];
    for (unsigned int i = 0; i < count; ++i)
    {
        char* rest;
        if (!MeXMLParseComma(cursor, &rest))
            return fail(in, MeXMLErrorInvalidValue, "too few values");
        char* end;
        out[i] = convert<T>(cursor, &end);
        while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
            ++end;
        if (end == cursor || *end)
            return fail(in, MeXMLErrorInvalidValue, "invalid number");
        cursor = rest;
    }
    return MeXMLErrorNone;
}

} // namespace

extern "C" {

int MEAPI MeXMLParseComma(char* in, char** out)
{
    while (*in == ' ' || *in == '\t' || *in == '\r' || *in == '\n')
        ++in;
    if (!*in)
    {
        *out = in;
        return 0;
    }
    char* comma = std::strchr(in, ',');
    if (comma)
    {
        *comma = '\0';
        *out = comma + 1;
    }
    else
        *out = in + std::strlen(in);
    return 1;
}

MeXMLError MEAPI MeXMLParseUInt(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<unsigned int>(in, h, data, 1);
}

MeXMLError MEAPI MeXMLParseUIntArray(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<unsigned int>(in, h, data, h->max);
}

MeXMLError MEAPI MeXMLParseInt(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<int>(in, h, data, 1);
}

MeXMLError MEAPI MeXMLParseIntArray(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<int>(in, h, data, h->max);
}

MeXMLError MEAPI MeXMLParseFloat(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<float>(in, h, data, 1);
}

MeXMLError MEAPI MeXMLParseFloatArray(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<float>(in, h, data, h->max);
}

MeXMLError MEAPI MeXMLParseMeReal(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<MeReal>(in, h, data, 1);
}

MeXMLError MEAPI MeXMLParseMeRealArray(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<MeReal>(in, h, data, h->max);
}

MeXMLError MEAPI MeXMLParseDouble(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<double>(in, h, data, 1);
}

MeXMLError MEAPI MeXMLParseDoubleArray(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    return parseArray<double>(in, h, data, h->max);
}

// ME_XML_STRING_HANDLER puts the buffer length in max.
MeXMLError MEAPI MeXMLParseString(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    std::string text;
    if (MeXMLError err = readPCDATA(in, text))
        return err;
    if (h->max == 0)
        return MeXMLErrorNone;
    char* out = static_cast<char*>(data) + h->offset;
    std::strncpy(out, text.c_str(), h->max);
    out[h->max - 1] = '\0';
    return MeXMLErrorNone;
}

// ME_XML_STRING_ARRAY_HANDLER puts the length of each string in max and their number in maxstr.
MeXMLError MEAPI MeXMLParseStringArray(MeXMLInput* in, const MeXMLHandler* h, void* data)
{
    std::string text;
    if (MeXMLError err = readPCDATA(in, text))
        return err;
    if (h->max == 0)
        return MeXMLErrorNone;
    char* out = static_cast<char*>(data) + h->offset;
    char* cursor = &text[0];
    for (unsigned int i = 0; i < h->maxstr; ++i)
    {
        char* rest;
        if (!MeXMLParseComma(cursor, &rest))
            return fail(in, MeXMLErrorInvalidValue, "too few values");
        std::strncpy(out + i * h->max, cursor, h->max);
        out[i * h->max + h->max - 1] = '\0';
        cursor = rest;
    }
    return MeXMLErrorNone;
}

MeXMLInput* MEAPI MeXMLInputCreate(MeStream file)
{
    InputImpl* x = new InputImpl{};
    x->pub.stream = file;
    return &x->pub;
}

void MEAPI MeXMLInputDestroy(MeXMLInput* input)
{
    delete impl(input);
}

void MEAPI MeXMLInputSetUserData(MeXMLInput* input, void* userData)
{
    input->userdata = userData;
}

void* MEAPI MeXMLInputGetUserData(const MeXMLInput* input)
{
    return input->userdata;
}

MeXMLError MEAPI MeXMLInputProcess(MeXMLInput* input, MeXMLHandler handlers[], void* userdata)
{
    return processContent(input, nullptr, handlers, userdata, userdata);
}

const char* MEAPI MeXMLInputGetErrorString(const MeXMLInput* input)
{
    return input->error;
}

void MEAPI MeXMLInputSetErrorString(MeXMLInput* input, char* error, ...)
{
    va_list ap;
    va_start(ap, error);
    std::vsnprintf(input->error, sizeof(input->error), error, ap);
    va_end(ap);
}

MeXMLError MEAPI MeXMLElementProcess(MeXMLElement* elem, MeXMLHandler handlers[], void* data, void* userdata)
{
    return processContent(elem->fi, elem, handlers, data, userdata);
}

MeXMLInput* MEAPI MeXMLElementGetInput(MeXMLElement* elem)
{
    return elem->fi;
}

MeBool MEAPI MeXMLHandlerWasCalled(MeXMLHandler handlers[], char* name)
{
    MeXMLHandler* h = findHandler(handlers, name);
    return h && h->called;
}

void MEAPI MeXMLElementHandlerCreate(MeXMLHandler* handler, char* name, void* fn)
{
    *handler = MeXMLHandler{};
    handler->name = static_cast<char*>(std::malloc(std::strlen(name) + 1));
    std::strcpy(handler->name, name);
    handler->type = MeXMLActionCallback;
    handler->fn = fn;
}

void MEAPI MeXMLElementHandlerDestroy(MeXMLHandler* handler)
{
    std::free(handler->name);
    handler->name = nullptr;
}

} // extern "C"
