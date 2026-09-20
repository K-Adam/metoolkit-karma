#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <MeAssetDB.h>
#include <MdtDefaults.h>
#include <MeMath.h>

#include "StringCopy.h"

namespace {

const int kHashSize = 64;

void assign(char*& field, const char* value)
{
    char* copy = duplicateString(value);
    std::free(field);
    field = copy;
}

bool same(const char* a, const char* b)
{
    return a && b && std::strcmp(a, b) == 0;
}

void copyMatrix(MeMatrix4 to, const MeMatrix4Ptr from)
{
    if (from)
        MeMatrix4Copy(to, from);
    else
        MeMatrix4TMMakeIdentity(to);
}

// Lists are circular with a sentinel whose current is null, which also ends iteration.
template <class Node>
Node* sentinel()
{
    Node* nil = static_cast<Node*>(std::calloc(1, sizeof(Node)));
    nil->prev = nil->next = nil;
    return nil;
}

template <class Node, class T>
void append(Node* nil, T* item)
{
    Node* n = static_cast<Node*>(std::malloc(sizeof(Node)));
    n->current = item;
    n->next = nil;
    n->prev = nil->prev;
    nil->prev->next = n;
    nil->prev = n;
}

template <class Node, class T>
void unlink(Node* nil, T* item)
{
    for (Node* n = nil->next; n != nil; n = n->next)
    {
        if (n->current == item)
        {
            n->prev->next = n->next;
            n->next->prev = n->prev;
            std::free(n);
            return;
        }
    }
}

template <class Node>
std::vector<decltype(Node::current)> items(const Node* nil)
{
    std::vector<decltype(Node::current)> out;
    for (Node* n = nil->next; n != nil; n = n->next)
        out.push_back(n->current);
    return out;
}

template <class It>
auto advance(It* it)
{
    auto item = it->node->current;
    if (item)
        it->node = it->node->next;
    return item;
}

template <class Node, class T>
void sortedByName(const Node* nil, T** out)
{
    auto all = items(nil);
    std::sort(all.begin(), all.end(), [](T* a, T* b) { return std::strcmp(a->id, b->id) < 0; });
    std::copy(all.begin(), all.end(), out);
}

template <class T>
struct Members;

template <>
struct Members<MeFGeometry>
{
    static constexpr auto nil = &MeFAsset::nilGeometry;
    static constexpr auto hash = &MeFAsset::nameToGeometry;
    static constexpr auto count = &MeFAsset::geomCount;
};

template <>
struct Members<MeFModel>
{
    static constexpr auto nil = &MeFAsset::nilModel;
    static constexpr auto hash = &MeFAsset::nameToModel;
    static constexpr auto count = &MeFAsset::modelCount;
};

template <>
struct Members<MeFAssetPart>
{
    static constexpr auto nil = &MeFAsset::nilPart;
    static constexpr auto hash = &MeFAsset::nameToPart;
    static constexpr auto count = &MeFAsset::partCount;
};

template <>
struct Members<MeFJoint>
{
    static constexpr auto nil = &MeFAsset::nilJoint;
    static constexpr auto hash = &MeFAsset::nameToJoint;
    static constexpr auto count = &MeFAsset::jointCount;
};

// A shared name maps to one item. When that item leaves the table another holder of the name takes the key,
// so the bucket never keeps a key whose owner is gone.
template <class T>
void unhash(T* item)
{
    MeHash* names = item->asset->*Members<T>::hash;
    if (MeHashLookup(item->id, names) != item)
        return;
    MeHashDelete(item->id, names);
    for (T* other : items(item->asset->*Members<T>::nil))
        if (other != item && same(other->id, item->id))
        {
            MeHashInsert(other->id, other, names);
            return;
        }
}

template <class T>
void insertItem(MeFAsset* asset, T* item)
{
    append(asset->*Members<T>::nil, item);
    item->asset = asset;
    MeHashInsert(item->id, item, item->asset->*Members<T>::hash);
    ++(asset->*Members<T>::count);
}

template <class T>
void removeItem(T* item)
{
    MeFAsset* asset = item->asset;
    if (!asset)
        return;
    unhash(item);
    unlink(asset->*Members<T>::nil, item);
    --(asset->*Members<T>::count);
    item->asset = nullptr;
}

template <class T>
void renameItem(T* item, const char* name)
{
    if (item->asset)
        unhash(item);
    assign(item->id, name);
    if (item->asset)
        MeHashInsert(item->id, item, item->asset->*Members<T>::hash);
}

char* makeUnique(const MeHash* a, const MeHash* b, const char* name, char* buffer, int bufLength)
{
    auto used = [&](const char* n) { return (a && MeHashLookup(n, a)) || (b && MeHashLookup(n, b)); };
    std::string candidate = name;
    for (int i = 1; used(candidate.c_str()); ++i)
        candidate = name + std::to_string(i);
    std::snprintf(buffer, bufLength, "%s", candidate.c_str());
    return buffer;
}

// Renames the items of source whose names dest already uses, and points the references in source at the new names.
template <class T>
void renameClashes(const MeFAsset* dest, MeFAsset* source, void(MEAPI* resolve)(MeFAsset*, const char*, const char*))
{
    MeHash* taken = dest->*Members<T>::hash;
    char buffer[256];
    for (T* item : items(source->*Members<T>::nil))
        if (MeHashLookup(item->id, taken))
        {
            std::string old = item->id;
            renameItem(item, makeUnique(taken, source->*Members<T>::hash, item->id, buffer, sizeof(buffer)));
            if (resolve)
                resolve(source, old.c_str(), item->id);
        }
}

template <class Node, class T>
void moveAll(const Node* nil, MeFAsset* dest, void(MEAPI* insert)(MeFAsset*, T*))
{
    for (T* item : items(nil))
        insert(dest, item);
}

MeHash* table(const MeFAsset* a, MeHash* MeFAsset::*member)
{
    return a ? a->*member : nullptr;
}

// Disabled collisions are a maxParts x maxParts bit matrix indexed by part index.
bool bit(const MeFAsset* a, int i, int j)
{
    unsigned int k = static_cast<unsigned int>(i * a->maxParts + j);
    return (a->disabledColArray[k / 32] >> (k % 32)) & 1;
}

void setBit(MeFAsset* a, int i, int j, bool on)
{
    unsigned int k = static_cast<unsigned int>(i * a->maxParts + j);
    if (on)
        a->disabledColArray[k / 32] |= 1u << (k % 32);
    else
        a->disabledColArray[k / 32] &= ~(1u << (k % 32));
}

size_t matrixWords(int n)
{
    return (static_cast<size_t>(n) * n + 31) / 32;
}

void growMatrix(MeFAsset* a, int index)
{
    if (index < a->maxParts)
        return;
    int n = std::max({index + 1, a->maxParts * 2, 8});
    MeU32* old = a->disabledColArray;
    int oldN = a->maxParts;
    a->disabledColArray = static_cast<MeU32*>(std::calloc(matrixWords(n), sizeof(MeU32)));
    a->maxParts = n;
    for (int i = 0; i < oldN; ++i)
        for (int j = 0; j < oldN; ++j)
        {
            unsigned int k = static_cast<unsigned int>(i * oldN + j);
            if ((old[k / 32] >> (k % 32)) & 1)
                setBit(a, i, j, true);
        }
    std::free(old);
}

bool samePairAsset(const MeFAssetPart* p1, const MeFAssetPart* p2)
{
    return p1->asset && p1->asset == p2->asset && p1->index >= 0 && p2->index >= 0;
}

struct Bounds
{
    MeVector3 min, max, centre;
};

Bounds bounds(const MyMesh* mesh)
{
    Bounds b{};
    for (int i = 0; i < 3; ++i)
    {
        b.min[i] = mesh->numVerts ? mesh->verts[0][i] : 0;
        b.max[i] = b.min[i];
    }
    for (int v = 0; v < mesh->numVerts; ++v)
        for (int i = 0; i < 3; ++i)
        {
            b.min[i] = std::min(b.min[i], mesh->verts[v][i]);
            b.max[i] = std::max(b.max[i], mesh->verts[v][i]);
        }
    for (int i = 0; i < 3; ++i)
        b.centre[i] = (b.min[i] + b.max[i]) / 2;
    return b;
}

// Places the primitive at the mesh centre, expressed in the frame given by relTM.
MeFPrimitive* primitiveAt(const char* name, MeFPrimitiveType type, const MeVector3 centre, const MeMatrix4Ptr relTM)
{
    MeFPrimitive* p = MeFPrimitiveCreate(name, type);
    copyMatrix(p->tm, relTM);
    MeVector3 t;
    for (int j = 0; j < 3; ++j)
        t[j] = p->tm[3][j] + centre[0] * p->tm[0][j] + centre[1] * p->tm[1][j] + centre[2] * p->tm[2][j];
    for (int j = 0; j < 3; ++j)
        p->tm[3][j] = t[j];
    return p;
}

void destroyAllJoints(MeFAsset* asset)
{
    for (MeFJoint* j : items(asset->nilJoint))
        MeFJointDestroy(j);
}

enum class Kind
{
    Real,
    Int,
    Bool
};

void* propertyField(const MeFJoint* cj, MeFJointProperty p, Kind& kind)
{
    MeFJoint* j = const_cast<MeFJoint*>(cj);
    kind = Kind::Real;
    if (p >= kMeFJointPropertyStop1 && p <= kMeFJointPropertyStop3)
        return &j->stop[p - kMeFJointPropertyStop1];
    if (p >= kMeFJointPropertyStiffness1 && p <= kMeFJointPropertyStiffness6)
        return &j->stiff[p - kMeFJointPropertyStiffness1];
    if (p >= kMeFJointPropertyDamping1 && p <= kMeFJointPropertyDamping6)
        return &j->damp[p - kMeFJointPropertyDamping1];
    if (p >= kMeFJointPropertyStrength1 && p <= kMeFJointPropertyStrength6)
        return &j->strength[p - kMeFJointPropertyStrength1];
    if (p >= kMeFJointPropertyDesiredVelocity1 && p <= kMeFJointPropertyDesiredVelocity2)
        return &j->desVel[p - kMeFJointPropertyDesiredVelocity1];
    if (p == kMeFJointPropertySpecialFloat1)
        return &j->special_f[0];
    kind = Kind::Int;
    if (p >= kMeFJointPropertySpecialInt1 && p <= kMeFJointPropertySpecialInt2)
        return &j->special_i[p - kMeFJointPropertySpecialInt1];
    kind = Kind::Bool;
    if (p == kMeFJointPropertyLimited1)
        return &j->bLimited[0];
    if (p == kMeFJointPropertyMotorized1)
        return &j->bMotorized[0];
    if (p == kMeFJointPropertySpecialBool1)
        return &j->special_b[0];
    return nullptr;
}

template <class T>
bool getProperty(const MeFJoint* j, MeFJointProperty p, Kind want, T* out)
{
    Kind kind;
    void* field = propertyField(j, p, kind);
    if (!field || kind != want)
        return false;
    *out = want == Kind::Real ? static_cast<T>(*static_cast<MeReal*>(field)) : static_cast<T>(*static_cast<int*>(field));
    return true;
}

template <class T>
void setProperty(MeFJoint* j, MeFJointProperty p, Kind want, T value)
{
    Kind kind;
    void* field = propertyField(j, p, kind);
    if (!field || kind != want)
        return;
    if (want == Kind::Real)
        *static_cast<MeReal*>(field) = static_cast<MeReal>(value);
    else
        *static_cast<int*>(field) = static_cast<int>(value);
}

} // namespace

extern "C" {

/* asset database */

MeAssetDB* MEAPI MeAssetDBCreate()
{
    MeAssetDB* db = static_cast<MeAssetDB*>(std::calloc(1, sizeof(MeAssetDB)));
    db->nilAsset = sentinel<MeFAssetNode>();
    return db;
}

void MEAPI MeAssetDBDeleteContents(MeAssetDB* const db)
{
    for (MeFAsset* a : items(db->nilAsset))
        MeFAssetDestroy(a);
}

void MEAPI MeAssetDBDestroy(MeAssetDB* const db)
{
    if (!db)
        return;
    MeAssetDBDeleteContents(db);
    std::free(db->nilAsset);
    std::free(db);
}

void MEAPI MeAssetDBInsertCopy(MeAssetDB* const to, const MeAssetDB* const from)
{
    for (MeFAsset* a : items(from->nilAsset))
        MeAssetDBInsertAsset(to, MeFAssetCreateCopy(a, 1));
}

MeAssetDB* MEAPI MeAssetDBCreateCopy(const MeAssetDB* const db)
{
    MeAssetDB* copy = MeAssetDBCreate();
    MeAssetDBInsertCopy(copy, db);
    return copy;
}

MeBool MEAPI MeAssetDBIsEmpty(const MeAssetDB* const db)
{
    return db->assetCount == 0;
}

int MEAPI MeAssetDBGetAssetCount(const MeAssetDB* const db)
{
    return static_cast<int>(db->assetCount);
}

void MEAPI MeAssetDBInitAssetIterator(const MeAssetDB* const db, MeFAssetIt* const it)
{
    it->node = db->nilAsset->next;
}

MeFAsset* MEAPI MeAssetDBGetAsset(MeFAssetIt* const it)
{
    return advance(it);
}

MeFAsset* MEAPI MeAssetDBLookupAsset(const MeAssetDB* const db, int id)
{
    for (MeFAsset* a : items(db->nilAsset))
        if (a->id == id)
            return a;
    return nullptr;
}

MeFAsset* MEAPI MeAssetDBLookupAssetByName(const MeAssetDB* const db, const char* const name)
{
    for (MeFAsset* a : items(db->nilAsset))
        if (same(a->name, name))
            return a;
    return nullptr;
}

void MEAPI MeAssetDBInsertAsset(MeAssetDB* const db, MeFAsset* const asset)
{
    MeAssetDBRemoveAsset(asset);
    append(db->nilAsset, asset);
    asset->db = db;
    ++db->assetCount;
}

void MEAPI MeAssetDBRemoveAsset(MeFAsset* const asset)
{
    if (!asset->db)
        return;
    unlink(asset->db->nilAsset, asset);
    --asset->db->assetCount;
    asset->db = nullptr;
}

/* asset */

MeFAsset* MEAPI MeFAssetCreate(const char* const name, int id)
{
    MeFAsset* a = static_cast<MeFAsset*>(std::calloc(1, sizeof(MeFAsset)));
    a->name = duplicateString(name);
    a->id = id;
    a->nilGeometry = sentinel<MeFGeometryNode>();
    a->nilModel = sentinel<MeFModelNode>();
    a->nilPart = sentinel<MeFAssetPartNode>();
    a->nilJoint = sentinel<MeFJointNode>();
    a->nameToGeometry = MeHashCreate(kHashSize);
    a->nameToModel = MeHashCreate(kHashSize);
    a->nameToPart = MeHashCreate(kHashSize);
    a->nameToJoint = MeHashCreate(kHashSize);
    a->disabledColIndexPool = MeIDPoolCreate();
    a->graphicScale = 1;
    a->massScale = 1;
    a->lengthScale = 1;
    return a;
}

MeFAsset* MEAPI MeFAssetCreateCopy(const MeFAsset* const asset, MeBool recurse)
{
    MeFAsset* a = MeFAssetCreate(asset->name, asset->id);
    a->graphicHint = duplicateString(asset->graphicHint);
    a->refPart = duplicateString(asset->refPart);
    a->graphicScale = asset->graphicScale;
    a->massScale = asset->massScale;
    a->lengthScale = asset->lengthScale;
    if (!recurse)
        return a;
    for (MeFGeometry* g : items(asset->nilGeometry))
        MeFAssetInsertGeometry(a, MeFGeometryCreateCopy(g, 1));
    for (MeFModel* m : items(asset->nilModel))
        MeFAssetInsertModel(a, MeFModelCreateCopy(m, 1));
    for (MeFJoint* j : items(asset->nilJoint))
        MeFAssetInsertJoint(a, MeFJointCreateCopy(j, 1));
    auto parts = items(asset->nilPart);
    for (MeFAssetPart* p : parts)
        MeFAssetInsertPart(a, MeFAssetPartCreateCopy(p, 1));
    for (MeFAssetPart* p : parts)
        for (MeFAssetPart* q : parts)
            if (!MeFAssetPartIsCollisionEnabled(p, q))
                MeFAssetPartEnableCollision(MeFAssetLookupPart(a, p->id), MeFAssetLookupPart(a, q->id), 0);
    return a;
}

void MEAPI MeFAssetDestroy(MeFAsset* const asset)
{
    if (!asset)
        return;
    MeAssetDBRemoveAsset(asset);
    MeFAssetRemoveAllGeometry(asset);
    MeFAssetRemoveAllModels(asset);
    MeFAssetRemoveAllParts(asset);
    destroyAllJoints(asset);
    MeHashDestroy(asset->nameToGeometry);
    MeHashDestroy(asset->nameToModel);
    MeHashDestroy(asset->nameToPart);
    MeHashDestroy(asset->nameToJoint);
    MeIDPoolDestroy(asset->disabledColIndexPool);
    std::free(asset->disabledColArray);
    std::free(asset->nilGeometry);
    std::free(asset->nilModel);
    std::free(asset->nilPart);
    std::free(asset->nilJoint);
    std::free(asset->name);
    std::free(asset->graphicHint);
    std::free(asset->refPart);
    std::free(asset);
}

// Copies everything in source into dest, renaming whatever clashes with names already in dest.
void MEAPI MeFAssetCombine(MeFAsset* const dest, MeFAsset* const source)
{
    MeFAsset* tmp = MeFAssetCreateCopy(source, 1);
    renameClashes<MeFGeometry>(dest, tmp, MeFAssetResolveGeometryReferences);
    renameClashes<MeFModel>(dest, tmp, MeFAssetResolveModelReferences);
    renameClashes<MeFAssetPart>(dest, tmp, MeFAssetResolvePartReferences);
    renameClashes<MeFJoint>(dest, tmp, nullptr);

    std::vector<std::pair<std::string, std::string>> disabled;
    auto parts = items(tmp->nilPart);
    for (MeFAssetPart* p : parts)
        for (MeFAssetPart* q : parts)
            if (!MeFAssetPartIsCollisionEnabled(p, q))
                disabled.emplace_back(p->id, q->id);

    moveAll(tmp->nilGeometry, dest, MeFAssetInsertGeometry);
    moveAll(tmp->nilModel, dest, MeFAssetInsertModel);
    moveAll(tmp->nilPart, dest, MeFAssetInsertPart);
    moveAll(tmp->nilJoint, dest, MeFAssetInsertJoint);
    for (auto& d : disabled)
        MeFAssetPartEnableCollision(MeFAssetLookupPart(dest, &d.first[0]), MeFAssetLookupPart(dest, &d.second[0]), 0);
    MeFAssetDestroy(tmp);
}

char* MEAPI MeFAssetGetName(const MeFAsset* const asset)
{
    return asset->name;
}

int MEAPI MeFAssetGetID(const MeFAsset* const asset)
{
    return asset->id;
}

char* MEAPI MeFAssetGetReferencePart(const MeFAsset* const asset)
{
    return asset->refPart;
}

char* MEAPI MeFAssetGetGraphicHint(const MeFAsset* const asset)
{
    return asset->graphicHint;
}

MeReal MEAPI MeFAssetGetGraphicScale(const MeFAsset* const asset)
{
    return asset->graphicScale;
}

int MEAPI MeFAssetGetGeometryCount(const MeFAsset* const asset)
{
    return asset->geomCount;
}

int MEAPI MeFAssetGetModelCount(const MeFAsset* const asset)
{
    return asset->modelCount;
}

int MEAPI MeFAssetGetJointCount(const MeFAsset* const asset)
{
    return asset->jointCount;
}

int MEAPI MeFAssetGetPartCount(const MeFAsset* const asset)
{
    return asset->partCount;
}

void MEAPI MeFAssetInitGeometryIterator(const MeFAsset* const asset, MeFGeometryIt* const it)
{
    it->node = asset->nilGeometry->next;
}

MeFGeometry* MEAPI MeFAssetGetGeometry(MeFGeometryIt* const it)
{
    return advance(it);
}

void MEAPI MeFAssetInitModelIterator(const MeFAsset* const asset, MeFModelIt* const it)
{
    it->node = asset->nilModel->next;
}

MeFModel* MEAPI MeFAssetGetModel(MeFModelIt* const it)
{
    return advance(it);
}

void MEAPI MeFAssetInitJointIterator(const MeFAsset* const asset, MeFJointIt* const it)
{
    it->node = asset->nilJoint->next;
}

MeFJoint* MEAPI MeFAssetGetJoint(MeFJointIt* const it)
{
    return advance(it);
}

void MEAPI MeFAssetInitPartIterator(const MeFAsset* const asset, MeFAssetPartIt* const it)
{
    it->node = asset->nilPart->next;
}

MeFAssetPart* MEAPI MeFAssetGetPart(MeFAssetPartIt* const it)
{
    return advance(it);
}

MeFGeometry* MEAPI MeFAssetLookupGeometry(const MeFAsset* const asset, char* const name)
{
    return asset && name ? static_cast<MeFGeometry*>(MeHashLookup(name, asset->nameToGeometry)) : nullptr;
}

MeFModel* MEAPI MeFAssetLookupModel(const MeFAsset* const asset, char* const name)
{
    return asset && name ? static_cast<MeFModel*>(MeHashLookup(name, asset->nameToModel)) : nullptr;
}

MeFJoint* MEAPI MeFAssetLookupJoint(const MeFAsset* const asset, char* const name)
{
    return asset && name ? static_cast<MeFJoint*>(MeHashLookup(name, asset->nameToJoint)) : nullptr;
}

MeFAssetPart* MEAPI MeFAssetLookupPart(const MeFAsset* const asset, char* const name)
{
    return asset && name ? static_cast<MeFAssetPart*>(MeHashLookup(name, asset->nameToPart)) : nullptr;
}

void MEAPI MeFAssetGetGeometrySortedByName(const MeFAsset* const asset, MeFGeometry** geomArray)
{
    sortedByName(asset->nilGeometry, geomArray);
}

void MEAPI MeFAssetGetModelsSortedByName(const MeFAsset* const asset, MeFModel** modelArray)
{
    sortedByName(asset->nilModel, modelArray);
}

void MEAPI MeFAssetGetPartsSortedByName(const MeFAsset* const asset, MeFAssetPart** partArray)
{
    sortedByName(asset->nilPart, partArray);
}

void MEAPI MeFAssetGetJointsSortedByName(const MeFAsset* const asset, MeFJoint** jointArray)
{
    sortedByName(asset->nilJoint, jointArray);
}

char* MEAPI MeFAssetMakeGeometryNameUnique(const MeFAsset* const asset, const MeFAsset* const asset2, char* name, char* buffer, int bufLength)
{
    return makeUnique(table(asset, &MeFAsset::nameToGeometry), table(asset2, &MeFAsset::nameToGeometry), name, buffer, bufLength);
}

char* MEAPI MeFAssetMakeModelNameUnique(const MeFAsset* const asset, const MeFAsset* const asset2, char* name, char* buffer, int bufLength)
{
    return makeUnique(table(asset, &MeFAsset::nameToModel), table(asset2, &MeFAsset::nameToModel), name, buffer, bufLength);
}

char* MEAPI MeFAssetMakePartNameUnique(const MeFAsset* const asset, const MeFAsset* const asset2, char* name, char* buffer, int bufLength)
{
    return makeUnique(table(asset, &MeFAsset::nameToPart), table(asset2, &MeFAsset::nameToPart), name, buffer, bufLength);
}

char* MEAPI MeFAssetMakeJointNameUnique(const MeFAsset* const asset, const MeFAsset* const asset2, char* name, char* buffer, int bufLength)
{
    return makeUnique(table(asset, &MeFAsset::nameToJoint), table(asset2, &MeFAsset::nameToJoint), name, buffer, bufLength);
}

MeBool MEAPI MeFAssetIsEmpty(const MeFAsset* const asset)
{
    return !asset->geomCount && !asset->modelCount && !asset->jointCount && !asset->partCount;
}

MeReal MEAPI MeFAssetGetMassScale(const MeFAsset* const asset)
{
    return asset->massScale;
}

MeReal MEAPI MeFAssetGetLengthScale(const MeFAsset* const asset)
{
    return asset->lengthScale;
}

void MEAPI MeFAssetRename(MeFAsset* const asset, const char* const name)
{
    assign(asset->name, name);
}

void MEAPI MeFAssetSetID(MeFAsset* const asset, int id)
{
    asset->id = id;
}

void MEAPI MeFAssetSetReferencePart(MeFAsset* const asset, const char* const part)
{
    assign(asset->refPart, part);
}

void MEAPI MeFAssetSetGraphicHint(MeFAsset* const asset, const char* const hint)
{
    assign(asset->graphicHint, hint);
}

void MEAPI MeFAssetSetGraphicScale(MeFAsset* const asset, MeReal scale)
{
    asset->graphicScale = scale;
}

void MEAPI MeFAssetInsertGeometry(MeFAsset* const asset, MeFGeometry* const geometry)
{
    MeFAssetRemoveGeometry(geometry);
    insertItem(asset, geometry);
}

void MEAPI MeFAssetRemoveGeometry(MeFGeometry* const geometry)
{
    removeItem(geometry);
}

void MEAPI MeFAssetInsertModel(MeFAsset* const asset, MeFModel* const model)
{
    MeFAssetRemoveModel(model);
    insertItem(asset, model);
}

void MEAPI MeFAssetRemoveModel(MeFModel* const model)
{
    removeItem(model);
}

void MEAPI MeFAssetInsertJoint(MeFAsset* const asset, MeFJoint* const joint)
{
    MeFAssetRemoveJoint(joint);
    insertItem(asset, joint);
}

void MEAPI MeFAssetRemoveJoint(MeFJoint* const joint)
{
    removeItem(joint);
}

void MEAPI MeFAssetInsertPart(MeFAsset* const asset, MeFAssetPart* const part)
{
    MeFAssetRemovePart(part);
    insertItem(asset, part);
    part->index = MeIDPoolRequestID(asset->disabledColIndexPool);
    growMatrix(asset, part->index);
}

void MEAPI MeFAssetRemovePart(MeFAssetPart* const part)
{
    MeFAsset* asset = part->asset;
    if (!asset)
        return;
    MeFAssetPartEnableAllCollisions(part);
    MeIDPoolReturnID(asset->disabledColIndexPool, part->index);
    removeItem(part);
    part->index = -1;
}

void MEAPI MeFAssetRemoveAllGeometry(MeFAsset* const asset)
{
    for (MeFGeometry* g : items(asset->nilGeometry))
        MeFGeometryDestroy(g);
}

void MEAPI MeFAssetRemoveAllModels(MeFAsset* const asset)
{
    for (MeFModel* m : items(asset->nilModel))
        MeFModelDestroy(m);
}

void MEAPI MeFAssetRemoveAllParts(MeFAsset* const asset)
{
    for (MeFAssetPart* p : items(asset->nilPart))
        MeFAssetPartDestroy(p);
}

void MEAPI MeFAssetScale(MeFAsset* const asset, MeReal scale)
{
    for (MeFGeometry* g : items(asset->nilGeometry))
        MeFGeometryScale(g, scale);
    for (MeFModel* m : items(asset->nilModel))
        MeFModelScale(m, scale);
    for (MeFAssetPart* p : items(asset->nilPart))
        MeFAssetPartScale(p, scale);
    for (MeFJoint* j : items(asset->nilJoint))
        MeFJointScale(j, scale);
}

void MEAPI MeFAssetResolveGeometryReferences(MeFAsset* const asset, const char* oldName, const char* newName)
{
    for (MeFModel* m : items(asset->nilModel))
        if (same(m->geometry, oldName))
            assign(m->geometry, newName);
}

void MEAPI MeFAssetResolveModelReferences(MeFAsset* const asset, const char* oldName, const char* newName)
{
    for (MeFAssetPart* p : items(asset->nilPart))
        if (same(p->model, oldName))
            assign(p->model, newName);
}

void MEAPI MeFAssetResolvePartReferences(MeFAsset* const asset, const char* oldName, const char* newName)
{
    for (MeFAssetPart* p : items(asset->nilPart))
        if (same(p->parent, oldName))
            assign(p->parent, newName);
    for (MeFJoint* j : items(asset->nilJoint))
        for (char*& part : j->part)
            if (same(part, oldName))
                assign(part, newName);
    if (same(asset->refPart, oldName))
        assign(asset->refPart, newName);
}

void MEAPI MeFAssetSetMassScale(MeFAsset* const asset, MeReal scale)
{
    asset->massScale = scale;
}

void MEAPI MeFAssetSetLengthScale(MeFAsset* const asset, MeReal scale)
{
    asset->lengthScale = scale;
}

/* geometry */

MeFGeometry* MEAPI MeFGeometryCreate(const char* const name)
{
    MeFGeometry* g = static_cast<MeFGeometry*>(std::calloc(1, sizeof(MeFGeometry)));
    g->id = duplicateString(name);
    g->nilPrimitive = sentinel<MeFPrimitiveNode>();
    g->graphicScale = 1;
    return g;
}

MeFGeometry* MEAPI MeFGeometryCreateCopy(const MeFGeometry* const geometry, MeBool recurse)
{
    MeFGeometry* g = MeFGeometryCreate(geometry->id);
    g->graphicHint = duplicateString(geometry->graphicHint);
    g->graphicScale = geometry->graphicScale;
    std::memcpy(g->graphicOffset, geometry->graphicOffset, sizeof(MeVector3));
    if (recurse)
        for (MeFPrimitive* p : items(geometry->nilPrimitive))
            MeFGeometryInsertPrimitive(g, MeFPrimitiveCreateCopy(p, 1));
    return g;
}

void MEAPI MeFGeometryDestroy(MeFGeometry* const geometry)
{
    if (!geometry)
        return;
    MeFAssetRemoveGeometry(geometry);
    for (MeFPrimitive* p : items(geometry->nilPrimitive))
        MeFPrimitiveDestroy(p);
    std::free(geometry->nilPrimitive);
    std::free(geometry->id);
    std::free(geometry->graphicHint);
    std::free(geometry);
}

char* MEAPI MeFGeometryGetName(const MeFGeometry* const geometry)
{
    return geometry->id;
}

char* MEAPI MeFGeometryGetGraphicHint(const MeFGeometry* const geometry)
{
    return geometry->graphicHint;
}

MeReal MEAPI MeFGeometryGetGraphicScale(const MeFGeometry* const geometry)
{
    return geometry->graphicScale;
}

void MEAPI MeFGeometryGetGraphicOffset(const MeFGeometry* const geometry, MeVector3 v)
{
    std::memcpy(v, geometry->graphicOffset, sizeof(MeVector3));
}

void MEAPI MeFGeometryInitPrimitiveIterator(const MeFGeometry* const geometry, MeFPrimitiveIt* const it)
{
    it->node = geometry->nilPrimitive->next;
}

MeFPrimitive* MEAPI MeFGeometryGetPrimitive(MeFPrimitiveIt* const it)
{
    return advance(it);
}

MeFPrimitive* MEAPI MeFGeometryLookupPrimitive(const MeFGeometry* const geometry, char* const name)
{
    for (MeFPrimitive* p : items(geometry->nilPrimitive))
        if (same(p->id, name))
            return p;
    return nullptr;
}

int MEAPI MeFGeometryGetPrimitiveCount(const MeFGeometry* const geometry)
{
    return geometry->primCount;
}

void MEAPI MeFGeometryRename(MeFGeometry* const geometry, const char* const name)
{
    renameItem(geometry, name);
}

void MEAPI MeFGeometrySetGraphicHint(MeFGeometry* const geometry, const char* const hint)
{
    assign(geometry->graphicHint, hint);
}

void MEAPI MeFGeometrySetGraphicScale(MeFGeometry* const geometry, MeReal scale)
{
    geometry->graphicScale = scale;
}

void MEAPI MeFGeometrySetGraphicOffset(MeFGeometry* const geometry, MeReal dx, MeReal dy, MeReal dz)
{
    geometry->graphicOffset[0] = dx;
    geometry->graphicOffset[1] = dy;
    geometry->graphicOffset[2] = dz;
}

void MEAPI MeFGeometryInsertPrimitive(MeFGeometry* const geometry, MeFPrimitive* const p)
{
    MeFGeometryRemovePrimitive(p);
    append(geometry->nilPrimitive, p);
    p->geometry = geometry;
    ++geometry->primCount;
}

void MEAPI MeFGeometryRemovePrimitive(MeFPrimitive* const p)
{
    if (!p->geometry)
        return;
    unlink(p->geometry->nilPrimitive, p);
    --p->geometry->primCount;
    p->geometry = nullptr;
}

void MEAPI MeFGeometryScaleNoGraphic(MeFGeometry* const geometry, MeReal scale)
{
    for (MeFPrimitive* p : items(geometry->nilPrimitive))
        MeFPrimitiveScale(p, scale);
}

void MEAPI MeFGeometryScale(MeFGeometry* const geometry, MeReal scale)
{
    MeFGeometryScaleNoGraphic(geometry, scale);
    geometry->graphicScale *= scale;
    MeVector3Scale(geometry->graphicOffset, scale);
}

/* primitive */

MeFPrimitive* MEAPI MeFPrimitiveCreate(const char* const name, MeFPrimitiveType type)
{
    MeFPrimitive* p = static_cast<MeFPrimitive*>(std::calloc(1, sizeof(MeFPrimitive)));
    p->id = duplicateString(name);
    p->type = type;
    MeMatrix4TMMakeIdentity(p->tm);
    return p;
}

MeFPrimitive* MEAPI MeFSphereCreateFromMesh(const char* const name, const MyMesh* const mesh, const MeMatrix4Ptr relTM)
{
    Bounds b = bounds(mesh);
    MeReal r2 = 0;
    for (int v = 0; v < mesh->numVerts; ++v)
    {
        MeReal d2 = 0;
        for (int i = 0; i < 3; ++i)
            d2 += (mesh->verts[v][i] - b.centre[i]) * (mesh->verts[v][i] - b.centre[i]);
        r2 = std::max(r2, d2);
    }
    MeFPrimitive* p = primitiveAt(name, kMeFPrimitiveTypeSphere, b.centre, relTM);
    p->dims[0] = std::sqrt(r2);
    return p;
}

MeFPrimitive* MEAPI MeFBoxCreateFromMesh(const char* const name, const MyMesh* const mesh, const MeMatrix4Ptr relTM)
{
    Bounds b = bounds(mesh);
    MeFPrimitive* p = primitiveAt(name, kMeFPrimitiveTypeBox, b.centre, relTM);
    for (int i = 0; i < 3; ++i)
        p->dims[i] = b.max[i] - b.min[i];
    return p;
}

// Cylinders run along z, as McdCylinder does.
MeFPrimitive* MEAPI MeFCylinderCreateFromMesh(const char* const name, const MyMesh* const mesh, const MeMatrix4Ptr relTM)
{
    Bounds b = bounds(mesh);
    MeReal r2 = 0;
    for (int v = 0; v < mesh->numVerts; ++v)
    {
        MeReal dx = mesh->verts[v][0] - b.centre[0];
        MeReal dy = mesh->verts[v][1] - b.centre[1];
        r2 = std::max(r2, dx * dx + dy * dy);
    }
    MeFPrimitive* p = primitiveAt(name, kMeFPrimitiveTypeCylinder, b.centre, relTM);
    p->dims[0] = std::sqrt(r2);
    p->dims[1] = b.max[2] - b.min[2];
    return p;
}

MeFPrimitive* MEAPI MeFConvexCreateFromMesh(const char* const name, const MyMesh* const mesh, const MeMatrix4Ptr relTM)
{
    MeFPrimitive* p = MeFPrimitiveCreate(name, kMeFPrimitiveTypeConvex);
    copyMatrix(p->tm, relTM);
    MeFPrimitiveSetVertexArray(p, mesh->verts, mesh->numVerts);
    return p;
}

MeFPrimitive* MEAPI MeFPrimitiveCreateCopy(const MeFPrimitive* const p, MeBool)
{
    MeFPrimitive* c = MeFPrimitiveCreate(p->id, p->type);
    std::memcpy(c->dims, p->dims, sizeof(MeVector3));
    std::memcpy(c->tm, p->tm, sizeof(MeMatrix4));
    MeFPrimitiveSetVertexArray(c, p->vertices, p->nVertices);
    return c;
}

void MEAPI MeFPrimitiveDestroy(MeFPrimitive* const p)
{
    if (!p)
        return;
    MeFGeometryRemovePrimitive(p);
    std::free(p->vertices);
    std::free(p->id);
    std::free(p);
}

MeFPrimitiveType MEAPI MeFPrimitiveGetType(const MeFPrimitive* const p)
{
    return p->type;
}

char* MEAPI MeFPrimitiveGetName(const MeFPrimitive* const p)
{
    return p->id;
}

MeReal MEAPI MeFPrimitiveGetRadius(const MeFPrimitive* const p)
{
    return p->dims[0];
}

void MEAPI MeFPrimitiveGetDimensions(const MeFPrimitive* const p, MeVector3 dims)
{
    std::memcpy(dims, p->dims, sizeof(MeVector3));
}

MeReal MEAPI MeFPrimitiveGetHeight(const MeFPrimitive* const p)
{
    return p->dims[1];
}

int MEAPI MeFPrimitiveGetVertexCount(const MeFPrimitive* const p)
{
    return p->nVertices;
}

MeVector3* MEAPI MeFPrimitiveGetVertexArray(MeFPrimitive* const p)
{
    return p->vertices;
}

void MEAPI MeFPrimitiveInitVertexIterator(MeFPrimitive* const p, MeFVertexIt* const it)
{
    it->prim = p;
    it->currentVertex = 0;
}

MeReal* MEAPI MeFPrimitiveGetVertex(MeFVertexIt* const it)
{
    if (it->currentVertex >= it->prim->nVertices)
        return nullptr;
    return it->prim->vertices[it->currentVertex++];
}

MeMatrix4Ptr MEAPI MeFPrimitiveGetTransformPtr(MeFPrimitive* const p)
{
    return p->tm;
}

void MEAPI MeFPrimitiveRename(MeFPrimitive* const p, const char* const name)
{
    assign(p->id, name);
}

void MEAPI MeFPrimitiveSetRadius(MeFPrimitive* const sphere, MeReal r)
{
    sphere->dims[0] = r;
}

void MEAPI MeFPrimitiveSetDimensions(MeFPrimitive* const box, MeReal dx, MeReal dy, MeReal dz)
{
    box->dims[0] = dx;
    box->dims[1] = dy;
    box->dims[2] = dz;
}

void MEAPI MeFPrimitiveSetHeight(MeFPrimitive* const p, MeReal height)
{
    p->dims[1] = height;
}

void MEAPI MeFPrimitiveAddVertex(MeFPrimitive* const p, MeVector3 vertex)
{
    if (p->nVertices == p->maxVertices)
    {
        p->maxVertices = p->maxVertices ? p->maxVertices * 2 : 16;
        p->vertices = static_cast<MeVector3*>(std::realloc(p->vertices, p->maxVertices * sizeof(MeVector3)));
    }
    std::memcpy(p->vertices[p->nVertices++], vertex, sizeof(MeVector3));
}

void MEAPI MeFPrimitiveSetVertexArray(MeFPrimitive* const p, MeVector3* vertices, int nVertices)
{
    std::free(p->vertices);
    p->vertices = nullptr;
    p->nVertices = p->maxVertices = 0;
    for (int i = 0; i < nVertices; ++i)
        MeFPrimitiveAddVertex(p, vertices[i]);
}

void MEAPI MeFPrimitiveSetTransform(MeFPrimitive* const p, const MeMatrix4Ptr tm)
{
    copyMatrix(p->tm, tm);
}

void MEAPI MeFPrimitiveScale(MeFPrimitive* const p, MeReal scale)
{
    MeVector3Scale(p->dims, scale);
    for (int i = 0; i < p->nVertices; ++i)
        MeVector3Scale(p->vertices[i], scale);
    MeVector3Scale(p->tm[3], scale);
}

/* model */

MeFModel* MEAPI MeFModelCreate(const char* const name, MeFModelType type)
{
    MeFModel* m = static_cast<MeFModel*>(std::calloc(1, sizeof(MeFModel)));
    m->id = duplicateString(name);
    m->type = type;
    m->mass = MDTBODY_DEFAULT_MASS;
    m->density = 1;
    m->inertia[0] = m->inertia[3] = m->inertia[5] = MDTBODY_DEFAULT_INERTIA;
    m->linearDamp = MDTBODY_DEFAULT_LINEAR_DAMPING;
    m->angularDamp = MDTBODY_DEFAULT_ANGULAR_DAMPING;
    m->fastSpin[0] = MDTBODY_DEFAULT_FAST_SPIN_X;
    m->fastSpin[1] = MDTBODY_DEFAULT_FAST_SPIN_Y;
    m->fastSpin[2] = MDTBODY_DEFAULT_FAST_SPIN_Z;
    return m;
}

void MEAPI MeFModelCopy(MeFModel* dst, const MeFModel* const src, MeBool setGeometry)
{
    dst->type = src->type;
    if (setGeometry)
        assign(dst->geometry, src->geometry);
    std::memcpy(dst->mass_offset, src->mass_offset, sizeof(MeVector3));
    dst->mass = src->mass;
    dst->density = src->density;
    std::memcpy(dst->inertia, src->inertia, sizeof(dst->inertia));
    dst->linearDamp = src->linearDamp;
    dst->angularDamp = src->angularDamp;
    std::memcpy(dst->fastSpin, src->fastSpin, sizeof(MeVector3));
    dst->useFastSpin = src->useFastSpin;
}

MeFModel* MEAPI MeFModelCreateCopy(const MeFModel* const model, MeBool)
{
    MeFModel* m = MeFModelCreate(model->id, model->type);
    MeFModelCopy(m, model, 1);
    return m;
}

void MEAPI MeFModelDestroy(MeFModel* const model)
{
    if (!model)
        return;
    MeFAssetRemoveModel(model);
    std::free(model->id);
    std::free(model->geometry);
    std::free(model);
}

char* MEAPI MeFModelGetName(const MeFModel* const model)
{
    return model->id;
}

MeFModelType MEAPI MeFModelGetType(const MeFModel* const model)
{
    return model->type;
}

char* MEAPI MeFModelGetGeometryName(const MeFModel* const model)
{
    return model->geometry;
}

MeFGeometry* MEAPI MeFModelGetGeometry(const MeFModel* const model)
{
    return MeFAssetLookupGeometry(model->asset, model->geometry);
}

void MEAPI MeFModelGetMassOffset(const MeFModel* const model, MeVector3 v)
{
    std::memcpy(v, model->mass_offset, sizeof(MeVector3));
}

MeReal MEAPI MeFModelGetMass(const MeFModel* const model)
{
    return model->mass;
}

MeReal MEAPI MeFModelGetDensity(const MeFModel* const model)
{
    return model->density;
}

// inertia holds the upper triangle row by row: xx, xy, xz, yy, yz, zz.
void MEAPI MeFModelGetInertiaTensor(const MeFModel* const model, const MeMatrix3Ptr I)
{
    const MeReal* t = model->inertia;
    I[0][0] = t[0];
    I[0][1] = I[1][0] = t[1];
    I[0][2] = I[2][0] = t[2];
    I[1][1] = t[3];
    I[1][2] = I[2][1] = t[4];
    I[2][2] = t[5];
}

MeReal MEAPI MeFModelGetLinearVelocityDamping(const MeFModel* const model)
{
    return model->linearDamp;
}

MeReal MEAPI MeFModelGetAngularVelocityDamping(const MeFModel* const model)
{
    return model->angularDamp;
}

void MEAPI MeFModelGetFastSpinAxis(const MeFModel* const model, MeVector3 axis)
{
    std::memcpy(axis, model->fastSpin, sizeof(MeVector3));
}

MeBool MEAPI MeFModelIsFastSpinAxisEnabled(const MeFModel* const model)
{
    return model->useFastSpin;
}

void MEAPI MeFModelRename(MeFModel* const model, const char* const name)
{
    renameItem(model, name);
}

void MEAPI MeFModelSetType(MeFModel* const model, MeFModelType type)
{
    model->type = type;
}

void MEAPI MeFModelSetGeometry(MeFModel* const model, const MeFGeometry* const fg)
{
    assign(model->geometry, fg ? fg->id : nullptr);
}

void MEAPI MeFModelSetGeometryByName(MeFModel* const model, const char* const name)
{
    assign(model->geometry, name);
}

void MEAPI MeFModelSetMassOffset(MeFModel* const model, MeVector3 v)
{
    std::memcpy(model->mass_offset, v, sizeof(MeVector3));
}

void MEAPI MeFModelSetMass(MeFModel* const model, MeReal mass)
{
    model->mass = mass;
}

void MEAPI MeFModelSetDensity(MeFModel* const model, MeReal density)
{
    model->density = density;
}

void MEAPI MeFModelSetInertiaTensor(MeFModel* const model, const MeMatrix3Ptr I)
{
    MeReal* t = model->inertia;
    t[0] = I[0][0];
    t[1] = I[0][1];
    t[2] = I[0][2];
    t[3] = I[1][1];
    t[4] = I[1][2];
    t[5] = I[2][2];
}

void MEAPI MeFModelSetLinearVelocityDamping(MeFModel* const model, MeReal d)
{
    model->linearDamp = d;
}

void MEAPI MeFModelSetAngularVelocityDamping(MeFModel* const model, MeReal d)
{
    model->angularDamp = d;
}

void MEAPI MeFModelSetFastSpinAxis(MeFModel* const model, MeReal x, MeReal y, MeReal z)
{
    model->fastSpin[0] = x;
    model->fastSpin[1] = y;
    model->fastSpin[2] = z;
}

void MEAPI MeFModelEnableFastSpinAxis(MeFModel* const model, MeBool b)
{
    model->useFastSpin = b;
}

// Lengths scale and mass is kept, so inertia scales with the square.
void MEAPI MeFModelScale(MeFModel* const model, MeReal scale)
{
    MeVector3Scale(model->mass_offset, scale);
    for (MeReal& i : model->inertia)
        i *= scale * scale;
}

/* asset part */

MeFAssetPart* MEAPI MeFAssetPartCreate(const char* const name, MeFModel* const model, const MeMatrix4Ptr relTM)
{
    MeFAssetPart* p = static_cast<MeFAssetPart*>(std::calloc(1, sizeof(MeFAssetPart)));
    p->id = duplicateString(name);
    p->model = duplicateString(model ? model->id : nullptr);
    copyMatrix(p->tm, relTM);
    p->graphicScale = 1;
    p->index = -1;
    return p;
}

MeFAssetPart* MEAPI MeFAssetPartCreateCopy(const MeFAssetPart* const part, MeBool)
{
    MeFAssetPart* p = MeFAssetPartCreate(part->id, nullptr, const_cast<MeMatrix4Ptr>(part->tm));
    p->model = duplicateString(part->model);
    p->graphicHint = duplicateString(part->graphicHint);
    p->parent = duplicateString(part->parent);
    p->graphicScale = part->graphicScale;
    std::memcpy(p->graphicOffset, part->graphicOffset, sizeof(MeVector3));
    return p;
}

void MEAPI MeFAssetPartDestroy(MeFAssetPart* const part)
{
    if (!part)
        return;
    MeFAssetRemovePart(part);
    std::free(part->id);
    std::free(part->model);
    std::free(part->graphicHint);
    std::free(part->parent);
    std::free(part);
}

char* MEAPI MeFAssetPartGetName(const MeFAssetPart* const part)
{
    return part->id;
}

char* MEAPI MeFAssetPartGetModelName(const MeFAssetPart* const part)
{
    return part->model;
}

MeFModel* MEAPI MeFAssetPartGetModel(const MeFAssetPart* const part)
{
    return MeFAssetLookupModel(part->asset, part->model);
}

MeFGeometry* MEAPI MeFAssetPartGetGeometry(const MeFAssetPart* const part)
{
    MeFModel* m = MeFAssetPartGetModel(part);
    return m ? MeFModelGetGeometry(m) : nullptr;
}

MeMatrix4Ptr MEAPI MeFAssetPartGetTransformPtr(MeFAssetPart* const part)
{
    return part->tm;
}

void MEAPI MeFAssetPartGetPosition(const MeFAssetPart* const part, MeVector3 pos)
{
    std::memcpy(pos, part->tm[3], sizeof(MeVector3));
}

char* MEAPI MeFAssetPartGetGraphicHint(const MeFAssetPart* const part)
{
    return part->graphicHint;
}

MeReal MEAPI MeFAssetPartGetGraphicScale(const MeFAssetPart* const part)
{
    return part->graphicScale;
}

void MEAPI MeFAssetPartGetGraphicOffset(const MeFAssetPart* const part, MeVector3 v)
{
    std::memcpy(v, part->graphicOffset, sizeof(MeVector3));
}

char* MEAPI MeFAssetPartGetParentPartName(const MeFAssetPart* const part)
{
    return part->parent;
}

MeFAssetPart* MEAPI MeFAssetPartGetParentPart(const MeFAssetPart* const part)
{
    return MeFAssetLookupPart(part->asset, part->parent);
}

MeBool MEAPI MeFAssetPartIsCollisionEnabled(const MeFAssetPart* const p1, MeFAssetPart* const p2)
{
    return !samePairAsset(p1, p2) || !bit(p1->asset, p1->index, p2->index);
}

int MEAPI MeFAssetPartGetDisabledCollisionIndex(const MeFAssetPart* const part)
{
    return part->index;
}

void MEAPI MeFAssetPartRename(MeFAssetPart* const part, const char* const name)
{
    renameItem(part, name);
}

void MEAPI MeFAssetPartSetGraphicHint(MeFAssetPart* const part, const char* const hint)
{
    assign(part->graphicHint, hint);
}

void MEAPI MeFAssetPartSetGraphicScale(MeFAssetPart* const part, MeReal scale)
{
    part->graphicScale = scale;
}

void MEAPI MeFAssetPartSetGraphicOffset(MeFAssetPart* const part, MeReal dx, MeReal dy, MeReal dz)
{
    part->graphicOffset[0] = dx;
    part->graphicOffset[1] = dy;
    part->graphicOffset[2] = dz;
}

void MEAPI MeFAssetPartScale(MeFAssetPart* const part, MeReal scale)
{
    MeVector3Scale(part->tm[3], scale);
    part->graphicScale *= scale;
    MeVector3Scale(part->graphicOffset, scale);
}

void MEAPI MeFAssetPartSetTransform(MeFAssetPart* const part, const MeMatrix4Ptr tm)
{
    copyMatrix(part->tm, tm);
}

void MEAPI MeFAssetPartSetPosition(MeFAssetPart* const part, MeReal x, MeReal y, MeReal z)
{
    part->tm[3][0] = x;
    part->tm[3][1] = y;
    part->tm[3][2] = z;
}

void MEAPI MeFAssetPartSetParentPart(MeFAssetPart* const part, const MeFAssetPart* const parent)
{
    assign(part->parent, parent ? parent->id : nullptr);
}

void MEAPI MeFAssetPartSetParentPartByName(MeFAssetPart* const part, const char* const name)
{
    assign(part->parent, name);
}

void MEAPI MeFAssetPartSetModel(MeFAssetPart* const part, const MeFModel* const model)
{
    assign(part->model, model ? model->id : nullptr);
}

void MEAPI MeFAssetPartSetModelByName(MeFAssetPart* const part, const char* const name)
{
    assign(part->model, name);
}

void MEAPI MeFAssetPartEnableCollision(const MeFAssetPart* const p1, const MeFAssetPart* const p2, MeBool enable)
{
    if (!p1 || !p2 || !samePairAsset(p1, p2))
        return;
    setBit(p1->asset, p1->index, p2->index, !enable);
    setBit(p1->asset, p2->index, p1->index, !enable);
}

void MEAPI MeFAssetPartEnableAllCollisions(const MeFAssetPart* const part)
{
    MeFAsset* a = part->asset;
    if (!a || part->index < 0)
        return;
    for (int i = 0; i < a->maxParts; ++i)
    {
        setBit(a, part->index, i, false);
        setBit(a, i, part->index, false);
    }
}

/* joint */

void MEAPI MeFJointSetGenericDefaults(MeFJoint* const j)
{
    for (int i = 0; i < 2; ++i)
    {
        MeVector3Set(j->pos[i], 0, 0, 0);
        MeVector3Set(j->pax[i], 1, 0, 0);
        MeVector3Set(j->oax[i], 0, 1, 0);
    }
}

void MEAPI MeFJointSetPerTypeDefaults(MeFJoint* const j)
{
    std::memset(j->stop, 0, sizeof(j->stop));
    std::memset(j->stiff, 0, sizeof(j->stiff));
    std::memset(j->damp, 0, sizeof(j->damp));
    j->bLimited[0] = j->bMotorized[0] = 0;
    std::memset(j->strength, 0, sizeof(j->strength));
    std::memset(j->desVel, 0, sizeof(j->desVel));
    j->special_f[0] = 0;
    j->special_i[0] = j->special_i[1] = 0;
    j->special_b[0] = 0;
    if (j->type == kMeFJointTypeSkeletal)
    {
        j->stop[0] = MDTSKELETAL_DEFAULT_PRI_ANGLE;
        j->stop[1] = MDTSKELETAL_DEFAULT_SEC_ANGLE;
        j->stop[2] = MDTSKELETAL_DEFAULT_TWIST_ANGLE;
    }
}

MeFJoint* MEAPI MeFJointCreate(const char* const name, MeFJointType type)
{
    MeFJoint* j = static_cast<MeFJoint*>(std::calloc(1, sizeof(MeFJoint)));
    j->id = duplicateString(name);
    j->type = type;
    MeFJointSetGenericDefaults(j);
    MeFJointSetPerTypeDefaults(j);
    return j;
}

void MEAPI MeFJointCopy(MeFJoint* const dst, const MeFJoint* const src, MeBool copyAxes)
{
    dst->type = src->type;
    if (copyAxes)
    {
        std::memcpy(dst->pos, src->pos, sizeof(dst->pos));
        std::memcpy(dst->pax, src->pax, sizeof(dst->pax));
        std::memcpy(dst->oax, src->oax, sizeof(dst->oax));
    }
    std::memcpy(dst->stop, src->stop, sizeof(dst->stop));
    std::memcpy(dst->stiff, src->stiff, sizeof(dst->stiff));
    std::memcpy(dst->damp, src->damp, sizeof(dst->damp));
    dst->bLimited[0] = src->bLimited[0];
    dst->bMotorized[0] = src->bMotorized[0];
    std::memcpy(dst->strength, src->strength, sizeof(dst->strength));
    std::memcpy(dst->desVel, src->desVel, sizeof(dst->desVel));
    dst->special_f[0] = src->special_f[0];
    std::memcpy(dst->special_i, src->special_i, sizeof(dst->special_i));
    dst->special_b[0] = src->special_b[0];
}

MeFJoint* MEAPI MeFJointCreateCopy(const MeFJoint* const joint, MeBool)
{
    MeFJoint* j = MeFJointCreate(joint->id, joint->type);
    MeFJointCopy(j, joint, 1);
    j->part[0] = duplicateString(joint->part[0]);
    j->part[1] = duplicateString(joint->part[1]);
    return j;
}

void MEAPI MeFJointDestroy(MeFJoint* const joint)
{
    if (!joint)
        return;
    MeFAssetRemoveJoint(joint);
    std::free(joint->id);
    std::free(joint->part[0]);
    std::free(joint->part[1]);
    std::free(joint);
}

char* MEAPI MeFJointGetName(const MeFJoint* const joint)
{
    return joint->id;
}

MeFJointType MEAPI MeFJointGetType(const MeFJoint* const joint)
{
    return joint->type;
}

char* MEAPI MeFJointGetPartName(const MeFJoint* const joint, int index)
{
    return joint->part[index];
}

MeFAssetPart* MEAPI MeFJointGetPart(const MeFJoint* const joint, int index)
{
    return MeFAssetLookupPart(joint->asset, joint->part[index]);
}

void MEAPI MeFJointGetPosition(const MeFJoint* const joint, int index, MeVector3 pos)
{
    std::memcpy(pos, joint->pos[index], sizeof(MeVector3));
}

void MEAPI MeFJointGetPrimaryAxis(const MeFJoint* const joint, int index, MeVector3 axis)
{
    std::memcpy(axis, joint->pax[index], sizeof(MeVector3));
}

void MEAPI MeFJointGetOrthogonalAxis(const MeFJoint* const joint, int index, MeVector3 axis)
{
    std::memcpy(axis, joint->oax[index], sizeof(MeVector3));
}

void MEAPI MeFJointSetType(MeFJoint* const fj, MeFJointType type)
{
    fj->type = type;
    MeFJointSetPerTypeDefaults(fj);
}

void MEAPI MeFJointRename(MeFJoint* const joint, const char* const name)
{
    renameItem(joint, name);
}

void MEAPI MeFJointSetPosition(MeFJoint* const joint, int index, MeReal x, MeReal y, MeReal z)
{
    MeVector3Set(joint->pos[index], x, y, z);
}

void MEAPI MeFJointSetPrimaryAxis(MeFJoint* const joint, int index, MeReal x, MeReal y, MeReal z)
{
    MeVector3Set(joint->pax[index], x, y, z);
}

void MEAPI MeFJointSetOrthogonalAxis(MeFJoint* const joint, int index, MeReal x, MeReal y, MeReal z)
{
    MeVector3Set(joint->oax[index], x, y, z);
}

void MEAPI MeFJointSetPart(MeFJoint* const j, const MeFAssetPart* const part, int index)
{
    assign(j->part[index], part ? part->id : nullptr);
}

void MEAPI MeFJointSetPartByName(MeFJoint* const j, const char* const part, int index)
{
    assign(j->part[index], part);
}

// Positions are lengths, and so are the limits of a prismatic joint.
void MEAPI MeFJointScale(MeFJoint* const joint, MeReal scale)
{
    MeVector3Scale(joint->pos[0], scale);
    MeVector3Scale(joint->pos[1], scale);
    if (joint->type == kMeFJointTypePrismatic)
        for (MeReal& s : joint->stop)
            s *= scale;
}

MeBool MEAPI MeFJointGetProperty1i(const MeFJoint* const joint, MeFJointProperty p, int* i)
{
    return getProperty(joint, p, Kind::Int, i);
}

MeBool MEAPI MeFJointGetProperty1ui(const MeFJoint* const joint, MeFJointProperty p, unsigned int* u)
{
    return getProperty(joint, p, Kind::Int, u);
}

MeBool MEAPI MeFJointGetProperty1b(const MeFJoint* const joint, MeFJointProperty p, MeBool* b)
{
    return getProperty(joint, p, Kind::Bool, b);
}

MeBool MEAPI MeFJointGetProperty1f(const MeFJoint* const joint, MeFJointProperty p, MeReal* x)
{
    return getProperty(joint, p, Kind::Real, x);
}

void MEAPI MeFJointSetProperty1i(MeFJoint* const joint, MeFJointProperty p, int i)
{
    setProperty(joint, p, Kind::Int, i);
}

void MEAPI MeFJointSetProperty1ui(MeFJoint* const joint, MeFJointProperty p, unsigned int i)
{
    setProperty(joint, p, Kind::Int, i);
}

void MEAPI MeFJointSetProperty1b(MeFJoint* const joint, MeFJointProperty p, MeBool b)
{
    setProperty(joint, p, Kind::Bool, b);
}

void MEAPI MeFJointSetProperty1f(MeFJoint* const joint, MeFJointProperty p, MeReal x)
{
    setProperty(joint, p, Kind::Real, x);
}

} // extern "C"
