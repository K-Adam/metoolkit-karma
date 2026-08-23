#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <MeXMLTree.h>

#include "XMLText.h"

namespace {

char* duplicate(const std::string& s)
{
    char* copy = static_cast<char*>(std::malloc(s.size() + 1));
    std::memcpy(copy, s.c_str(), s.size() + 1);
    return copy;
}

void traverse(PElement* e, PElement* parent, PElementCB cb, MeBool rootFirst, void* userdata)
{
    if (rootFirst)
        cb(e, parent, userdata);
    for (PElementNode* n = e->childHead; n;)
    {
        // The callback may free the node, as PElementDestroyChildren does.
        PElementNode* next = n->next;
        traverse(n->current, e, cb, rootFirst, userdata);
        n = next;
    }
    if (!rootFirst)
        cb(e, parent, userdata);
}

void push(PElementIt* it, PElement* e)
{
    PElementNode* n = static_cast<PElementNode*>(std::malloc(sizeof(PElementNode)));
    n->current = e;
    n->next = it->stackHead;
    it->stackHead = n;
}

} // namespace

extern "C" {

PElement* MEAPI PElementCreate(int type, char*, void* data, CDataFreeFunc freeFunc, char* attrs)
{
    PElement* e = static_cast<PElement*>(std::calloc(1, sizeof(PElement)));
    e->type = type;
    e->cdata = data;
    e->freeFunc = freeFunc;
    if (attrs)
        PElementParseAttributes(e, attrs);
    return e;
}

void MEAPI PElementDestroyChildren(PElement* e, PElement*, void*)
{
    while (PElementNode* n = e->childHead)
    {
        e->childHead = n->next;
        PElementDestroy(n->current);
        std::free(n);
    }
}

void MEAPI PElementDestroy(PElement* e)
{
    if (!e)
        return;
    PElementDestroyChildren(e, nullptr, nullptr);
    while (AttributeNode* n = e->attrHead)
    {
        e->attrHead = n->next;
        std::free(n->current->attr);
        std::free(n->current->value);
        std::free(n->current);
        std::free(n);
    }
    if (e->freeFunc && e->cdata)
        e->freeFunc(e->cdata);
    std::free(e);
}

void MEAPI PElementAddAttribute(PElement* e, char* attr, char* val)
{
    Attribute* a = static_cast<Attribute*>(std::malloc(sizeof(Attribute)));
    a->attr = duplicate(attr);
    a->value = duplicate(val);
    AttributeNode** tail = &e->attrHead;
    while (*tail)
        tail = &(*tail)->next;
    *tail = static_cast<AttributeNode*>(std::malloc(sizeof(AttributeNode)));
    (*tail)->current = a;
    (*tail)->next = nullptr;
}

void MEAPI PElementParseAttributes(PElement* e, char* attrs)
{
    const std::string s = attrs;
    const char* ws = " \t\r\n";
    size_t i = 0;
    for (;;)
    {
        size_t nameStart = s.find_first_not_of(ws, i);
        size_t eq = s.find('=', nameStart);
        if (nameStart == std::string::npos || eq == std::string::npos)
            return;
        size_t open = s.find_first_of("\"'", eq);
        if (open == std::string::npos)
            return;
        size_t close = s.find(s[open], open + 1);
        if (close == std::string::npos)
            return;
        std::string name = mexml::trim(s.substr(nameStart, eq - nameStart));
        std::string value = mexml::decode(s.substr(open + 1, close - open - 1));
        PElementAddAttribute(e, &name[0], &value[0]);
        i = close + 1;
    }
}

void MEAPI PElementInsert(PElement* e, PElement* parent)
{
    PElementNode** tail = &parent->childHead;
    while (*tail)
        tail = &(*tail)->next;
    *tail = static_cast<PElementNode*>(std::malloc(sizeof(PElementNode)));
    (*tail)->current = e;
    (*tail)->next = nullptr;
}

PElement* MEAPI PElementFind(PElement* root, PElementCompareCB cb, void* k1, void* k2)
{
    if (PElement* found = cb(root, k1, k2))
        return found;
    for (PElementNode* n = root->childHead; n; n = n->next)
        if (PElement* found = PElementFind(n->current, cb, k1, k2))
            return found;
    return nullptr;
}

char* MEAPI PElementGetAttributeValue(PElement* elem, char* attr)
{
    for (AttributeNode* n = elem->attrHead; n; n = n->next)
        if (std::strcmp(n->current->attr, attr) == 0)
            return n->current->value;
    return nullptr;
}

PElement* MEAPI PElementCompareAttributes(PElement* e, void* attr, void* attrVal)
{
    char* value = PElementGetAttributeValue(e, static_cast<char*>(attr));
    return value && std::strcmp(value, static_cast<char*>(attrVal)) == 0 ? e : nullptr;
}

void MEAPI PElementTraverseAll(PElement* root, PElementCB cb, MeBool rootFirst, void* userdata)
{
    traverse(root, nullptr, cb, rootFirst, userdata);
}

void MEAPI PElementInitIterator(PElement* root, PElementIt* it)
{
    it->stackHead = nullptr;
    push(it, root);
}

PElement* MEAPI PElementGetNext(PElementIt* it)
{
    PElementNode* top = it->stackHead;
    if (!top)
        return nullptr;
    it->stackHead = top->next;
    PElement* e = top->current;
    std::free(top);
    // Pushing children in reverse keeps the walk in document order.
    std::vector<PElement*> children;
    for (PElementNode* n = e->childHead; n; n = n->next)
        children.push_back(n->current);
    for (auto c = children.rbegin(); c != children.rend(); ++c)
        push(it, *c);
    return e;
}

void MEAPI PElementIteratorDestroy(PElementIt* it)
{
    while (PElementNode* n = it->stackHead)
    {
        it->stackHead = n->next;
        std::free(n);
    }
}

PElement* MEAPI PElementLookup(PElement* root, char* attr, char* val)
{
    return PElementFind(root, PElementCompareAttributes, attr, val);
}

} // extern "C"
