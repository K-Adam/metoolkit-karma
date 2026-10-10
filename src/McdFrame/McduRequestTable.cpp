#include <vector>

#include <McdModel.h>
#include <McduRequestTable.h>

// One global table of requests indexed by the request IDs of both models, which a framework's hello callback
// uses to give each new pair its request. Entries are copies, so pairs can point at them.

namespace {

int idCount = 0;
std::vector<McdRequest> table;

McdRequest* entry(int id1, int id2)
{
    if (id1 < 0 || id2 < 0 || id1 >= idCount || id2 >= idCount)
        return nullptr;
    return &table[(size_t)id1 * idCount + id2];
}

McdRequest* pairEntry(McdModelPair* p)
{
    return entry(McdModelGetRequestID(p->model1), McdModelGetRequestID(p->model2));
}

} // namespace

extern "C" {

MeBool MEAPI McduRequestTableInit(int idMaxCount, McdRequest* defaultRequest)
{
    // Keeps idMaxCount squared within a 32-bit size_t.
    if (idMaxCount <= 0 || idMaxCount > 65535)
        return 0;
    idCount = idMaxCount;
    table.assign((size_t)idMaxCount * idMaxCount, defaultRequest ? *defaultRequest : McdRequest{});
    return 1;
}

void MEAPI McduRequestTableTerm()
{
    idCount = 0;
    table = {};
}

// Interactions are symmetric, so both orders get the request.
void MEAPI McduRequestTableSetRequest(int requestId1, int requestId2, McdRequest* request)
{
    McdRequest* a = entry(requestId1, requestId2);
    if (!a || !request)
        return;
    *a = *request;
    *entry(requestId2, requestId1) = *request;
}

McdRequest* MEAPI McduRequestTableGetRequestPtr(int id1, int id2)
{
    return entry(id1, id2);
}

void MEAPI McduRequestTableGetRequest(McdModelPair* p, McdRequest* request)
{
    if (McdRequest* r = pairEntry(p))
        *request = *r;
}

void MEAPI McduHelloRequestTableCB(McdModelPair* p)
{
    if (McdRequest* r = pairEntry(p))
        McdModelPairSetRequestPtr(p, r);
}

} // extern "C"
