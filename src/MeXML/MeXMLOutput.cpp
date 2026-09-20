#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <MeXMLOutput.h>

#include "StringCopy.h"
#include "XMLText.h"

namespace {

void write(MeXMLOutput* op, const std::string& s)
{
    MeStreamWrite(const_cast<char*>(s.data()), 1, s.size(), op->stream);
}

void newline(MeXMLOutput* op, int depth)
{
    write(op, "\n" + std::string(depth, '\t'));
}

std::string format(const char* fmt, va_list ap)
{
    va_list copy;
    va_copy(copy, ap);
    int n = std::vsnprintf(nullptr, 0, fmt, copy);
    va_end(copy);
    std::string s(n > 0 ? n : 0, '\0');
    std::vsnprintf(&s[0], s.size() + 1, fmt, ap);
    return s;
}

void closeElement(MeXMLOutput* op)
{
    tagNode* node = op->tagHead;
    --op->depth;
    if (node->contents == kElement)
        newline(op, op->depth);
    write(op, std::string("</") + node->tag + ">");
    op->tagHead = node->next;
    std::free(node->tag);
    std::free(node);
}

} // namespace

extern "C" {

MeXMLOutput* MEAPI MeXMLOutputCreate(MeStream stream)
{
    MeXMLOutput* op = static_cast<MeXMLOutput*>(std::calloc(1, sizeof(MeXMLOutput)));
    op->stream = stream;
    return op;
}

void MEAPI MeXMLOutputDestroy(MeXMLOutput* op)
{
    if (!op)
        return;
    while (op->tagHead)
        closeElement(op);
    write(op, "\n");
    std::free(op);
}

// The variable arguments are attribute name and value strings, terminated by NULL.
// An attribute whose value is NULL is left out.
// The returned ID is the element's depth, and 0 stands for the document itself.
int MEAPI MeXMLWriteElement(MeXMLOutput* op, int parent, char* tag, ...)
{
    while (op->depth > parent && op->tagHead)
        closeElement(op);
    if (op->tagHead)
        op->tagHead->contents = kElement;
    newline(op, op->depth);

    std::string start = std::string("<") + tag;
    va_list ap;
    va_start(ap, tag);
    while (const char* name = va_arg(ap, const char*))
    {
        const char* value = va_arg(ap, const char*);
        if (value)
            start += std::string(" ") + name + "=\"" + mexml::encode(value) + "\"";
    }
    va_end(ap);
    write(op, start + ">");

    tagNode* node = static_cast<tagNode*>(std::malloc(sizeof(tagNode)));
    node->tag = duplicateString(tag);
    node->contents = kNothing;
    node->next = op->tagHead;
    op->tagHead = node;
    return ++op->depth;
}

void MEAPI MeXMLWritePCDATA(MeXMLOutput* op, char* format_, ...)
{
    va_list ap;
    va_start(ap, format_);
    write(op, mexml::encode(format(format_, ap).c_str()));
    va_end(ap);
    if (op->tagHead)
        op->tagHead->contents = kPCDATA;
}

void MEAPI MeXMLWriteComment(MeXMLOutput* op, char* c, ...)
{
    if (op->tagHead)
        op->tagHead->contents = kElement;
    newline(op, op->depth);
    va_list ap;
    va_start(ap, c);
    write(op, "<!-- " + format(c, ap) + " -->");
    va_end(ap);
}

} // extern "C"
