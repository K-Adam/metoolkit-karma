#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <new>
#include <string>
#include <vector>

#include <MeASELoad.h>
#include <MeMessage.h>

#include "Numbers.h"

namespace {

// Material indices in the file are bounded by this, so malformed counts cannot request huge allocations.
const int kMaxMaterials = 4096;
// Bounds the recursion on nested blocks.
const int kMaxDepth = 64;

struct Part
{
    std::string name;
    std::vector<MeASEFace> faces;
    std::vector<std::array<MeReal, 3>> verts;
    std::vector<MeASEUV> uvs;
    std::vector<std::array<int, 3>> tfaces;
    int materialRef = -1;
};

std::vector<std::string> tokenize(const std::string& text)
{
    std::vector<std::string> tokens;
    size_t i = 0, n = text.size();
    while (i < n)
    {
        unsigned char c = text[i];
        if (std::isspace(c))
            ++i;
        else if (c == '{' || c == '}')
            tokens.emplace_back(1, text[i++]);
        else if (c == '"')
        {
            size_t close = text.find('"', i + 1);
            if (close == std::string::npos)
                close = n;
            tokens.push_back(text.substr(i + 1, close - i - 1));
            i = close + 1;
        }
        else
        {
            size_t start = i;
            while (i < n && !std::isspace((unsigned char)text[i]) && text[i] != '{' && text[i] != '}')
                ++i;
            tokens.push_back(text.substr(start, i - start));
        }
    }
    return tokens;
}

const auto ignore = [](const std::string&) { return false; };

struct Parser
{
    std::vector<std::string> tok;
    size_t i = 0;
    int depth = 0;
    // Every entry the file declares takes from this, so the allocations stay proportional to the file size.
    size_t budget = 0;
    bool ok = true;

    bool more() const { return ok && i < tok.size(); }
    bool atKey() const { return more() && tok[i][0] == '*'; }
    bool at(const char* s) const { return more() && tok[i] == s; }

    bool fail()
    {
        ok = false;
        return false;
    }

    // Skips the arguments of the key just read, including any nested blocks.
    void skipArgs()
    {
        while (more() && !atKey() && !at("}"))
        {
            if (at("{"))
                block(ignore);
            else
                ++i;
        }
    }

    // Parses { ... }, handing each key to onKey. Keys it does not take are skipped.
    void block(const std::function<bool(const std::string&)>& onKey)
    {
        if (!at("{") || depth == kMaxDepth)
        {
            fail();
            return;
        }
        ++i;
        ++depth;
        while (more() && !at("}"))
        {
            if (atKey())
            {
                std::string key = tok[i++];
                if (!onKey(key))
                    skipArgs();
            }
            else if (at("{"))
                block(ignore);
            else
                ++i;
        }
        --depth;
        if (!at("}"))
        {
            fail();
            return;
        }
        ++i;
    }

    bool real(MeReal& v)
    {
        if (!more())
            return fail();
        const char* s = tok[i].c_str();
        char* end;
        double d = parseNumber<double>(s, &end);
        if (end == s || *end)
            return fail();
        ++i;
        v = (MeReal)d;
        return true;
    }

    // Face indices come as "0:", so one trailing colon is accepted.
    bool integer(int& v, int lo, int hi)
    {
        if (!more())
            return fail();
        const char* s = tok[i].c_str();
        char* end;
        long l = std::strtol(s, &end, 10);
        if (*end == ':')
            ++end;
        if (end == s || *end || l < lo || l >= hi)
            return fail();
        ++i;
        v = (int)l;
        return true;
    }

    bool reals(MeReal* v, int n)
    {
        for (int k = 0; k < n; ++k)
            if (!real(v[k]))
                return false;
        return true;
    }

    bool spend(size_t n)
    {
        if (n > budget)
            return fail();
        budget -= n;
        return true;
    }

    bool count(int& v) { return integer(v, 0, 1 << 30) && spend((size_t)v); }
};

void colour(Parser& p, float out[4])
{
    MeReal c[3];
    if (p.reals(c, 3))
        for (int k = 0; k < 3; ++k)
            out[k] = (float)c[k];
    out[3] = 1;
}

bool subMaterialKey(Parser& p, const std::string& key, MeASESubMaterial& m)
{
    if (key == "*MATERIAL_AMBIENT")
        colour(p, m.ambient);
    else if (key == "*MATERIAL_DIFFUSE")
        colour(p, m.diffuse);
    else if (key == "*MATERIAL_SPECULAR")
        colour(p, m.specular);
    else if (key == "*MAP_DIFFUSE")
        p.block([&](const std::string& k) {
            if (k != "*BITMAP" || !p.more())
                return false;
            std::strncpy(m.texFilename, p.tok[p.i++].c_str(), sizeof(m.texFilename) - 1);
            m.type = MeASEMaterialFlagTexture;
            return true;
        });
    else
        return false;
    return true;
}

MeASESubMaterial newSubMaterial()
{
    MeASESubMaterial m{};
    m.type = MeASEMaterialFlagColor;
    return m;
}

// A material without sub-materials becomes its own single sub-material, so faces always index subMaterials.
void parseMaterial(Parser& p, std::vector<MeASESubMaterial>& subs)
{
    MeASESubMaterial own = newSubMaterial();
    p.block([&](const std::string& key) {
        if (key == "*SUBMATERIAL")
        {
            int j;
            if (!p.integer(j, 0, kMaxMaterials))
                return true;
            if ((int)subs.size() <= j)
            {
                if (!p.spend(j + 1 - subs.size()))
                    return true;
                subs.resize(j + 1, newSubMaterial());
            }
            p.block([&](const std::string& k) { return subMaterialKey(p, k, subs[j]); });
            return true;
        }
        return subMaterialKey(p, key, own);
    });
    if (subs.empty())
        subs.push_back(own);
}

void parseMaterialList(Parser& p, std::vector<std::vector<MeASESubMaterial>>& materials)
{
    p.block([&](const std::string& key) {
        if (key != "*MATERIAL")
            return false;
        int index;
        if (!p.integer(index, 0, kMaxMaterials))
            return true;
        if ((int)materials.size() <= index)
        {
            if (!p.spend(index + 1 - materials.size()))
                return true;
            materials.resize(index + 1);
        }
        materials[index].clear();
        parseMaterial(p, materials[index]);
        return true;
    });
}

void parseFace(Parser& p, Part& part)
{
    int f;
    if (!p.integer(f, 0, (int)part.faces.size()))
        return;
    MeASEFace& face = part.faces[f];
    const char* labels[3] = {"A:", "B:", "C:"};
    for (int k = 0; k < 3; ++k)
    {
        if (!p.at(labels[k]))
        {
            p.fail();
            return;
        }
        ++p.i;
        if (!p.integer(face.vertexId[k], 0, (int)part.verts.size()))
            return;
    }
    p.skipArgs();
    // The material ID follows its face in the list.
    if (p.at("*MESH_SMOOTHING"))
    {
        ++p.i;
        p.skipArgs();
    }
    if (p.at("*MESH_MTLID"))
    {
        ++p.i;
        p.integer(face.subMaterialId, 0, 1 << 30);
    }
}

void parseMesh(Parser& p, Part& part)
{
    int lastFace = -1, corner = 3;
    p.block([&](const std::string& key) {
        if (key == "*MESH_NUMVERTEX")
        {
            int n;
            if (p.count(n))
                part.verts.assign(n, {0, 0, 0});
        }
        else if (key == "*MESH_NUMFACES")
        {
            int n;
            if (p.count(n))
                part.faces.assign(n, MeASEFace{});
        }
        else if (key == "*MESH_NUMTVERTEX")
        {
            int n;
            if (p.count(n))
                part.uvs.assign(n, MeASEUV{});
        }
        else if (key == "*MESH_NUMTVFACES")
        {
            int n;
            if (p.count(n))
                part.tfaces.assign(n, {0, 0, 0});
        }
        else if (key == "*MESH_VERTEX_LIST")
            p.block([&](const std::string& k) {
                int v;
                if (k == "*MESH_VERTEX" && p.integer(v, 0, (int)part.verts.size()))
                    p.reals(part.verts[v].data(), 3);
                return k == "*MESH_VERTEX";
            });
        else if (key == "*MESH_FACE_LIST")
            p.block([&](const std::string& k) {
                if (k == "*MESH_FACE")
                    parseFace(p, part);
                return k == "*MESH_FACE";
            });
        else if (key == "*MESH_TVERTLIST")
            p.block([&](const std::string& k) {
                int v;
                if (k == "*MESH_TVERT" && p.integer(v, 0, (int)part.uvs.size()))
                    (void)(p.real(part.uvs[v].u) && p.real(part.uvs[v].v));
                return k == "*MESH_TVERT";
            });
        else if (key == "*MESH_TFACELIST")
            p.block([&](const std::string& k) {
                int f;
                if (k == "*MESH_TFACE" && p.integer(f, 0, (int)part.tfaces.size()))
                    for (int c = 0; c < 3; ++c)
                        p.integer(part.tfaces[f][c], 0, (int)part.uvs.size());
                return k == "*MESH_TFACE";
            });
        else if (key == "*MESH_NORMALS")
            p.block([&](const std::string& k) {
                if (k == "*MESH_FACENORMAL" && p.integer(lastFace, 0, (int)part.faces.size()))
                {
                    p.reals(part.faces[lastFace].normal, 3);
                    corner = 0;
                }
                else if (k == "*MESH_VERTEXNORMAL" && lastFace >= 0 && lastFace < (int)part.faces.size() && corner < 3)
                {
                    int v;
                    if (p.integer(v, 0, 1 << 30))
                        p.reals(part.faces[lastFace].vNormal[corner++], 3);
                }
                else
                    return false;
                return true;
            });
        else
            return false;
        return true;
    });
}

void parseGeomObject(Parser& p, Part& part)
{
    p.block([&](const std::string& key) {
        if (key == "*NODE_NAME" && p.more() && !p.atKey())
            part.name = p.tok[p.i++];
        else if (key == "*MESH")
            parseMesh(p, part);
        else if (key == "*MATERIAL_REF")
        {
            p.integer(part.materialRef, 0, kMaxMaterials);
        }
        else
            return false;
        return true;
    });
}

// The public structs are released with std::free, so they come from std::calloc.
template <class T>
T* allocate(size_t n)
{
    if (n == 0)
        return nullptr;
    T* out = static_cast<T*>(std::calloc(n, sizeof(T)));
    if (!out)
        throw std::bad_alloc();
    return out;
}

template <class T>
T* copyArray(const T* data, size_t n)
{
    T* out = allocate<T>(n);
    if (n)
        std::memcpy(out, data, n * sizeof(T));
    return out;
}

// Linked into the list before it is filled, so a failed allocation leaves nothing unowned.
void makeObject(MeASEObject** link, const Part& part, MeASEMaterialStore* store, const MeReal scale[3])
{
    MeASEObject* o = *link = allocate<MeASEObject>(1);
    o->matStore = store;
    o->isLoaded = 1;
    std::strncpy(o->name, part.name.c_str(), sizeof(o->name) - 1);
    o->numVerts = (int)part.verts.size();
    o->numFaces = (int)part.faces.size();
    o->numUvs = (int)part.uvs.size();
    o->verts = allocate<MeVector3>(part.verts.size());
    for (int v = 0; v < o->numVerts; ++v)
        for (int k = 0; k < 3; ++k)
            o->verts[v][k] = part.verts[v][k] * scale[k];
    o->faces = copyArray(part.faces.data(), part.faces.size());
    o->uvs = copyArray(part.uvs.data(), part.uvs.size());
}

// Runs once the material list is known. Texture coordinates are copied into the faces, so merging parts later
// needs no UV index offset.
bool finishPart(Part& part, const MeASEMaterialStore& store)
{
    if (!part.tfaces.empty() && part.tfaces.size() != part.faces.size())
        return false;
    if (part.materialRef >= store.numMaterials)
        return false;
    for (size_t f = 0; f < part.faces.size(); ++f)
    {
        MeASEFace& face = part.faces[f];
        // Faces declared by MESH_NUMFACES but never listed still need valid vertices.
        for (int id : face.vertexId)
            if (id >= (int)part.verts.size())
                return false;
        if (!part.tfaces.empty())
            for (int c = 0; c < 3; ++c)
            {
                // A later MESH_NUMTVERTEX may have shrunk the list after the face was read.
                if (part.tfaces[f][c] >= (int)part.uvs.size())
                    return false;
                face.map[c] = part.uvs[part.tfaces[f][c]];
            }
        face.materialId = std::max(part.materialRef, 0);
        // 3ds Max wraps sub-material IDs around the number of sub-materials.
        if (part.materialRef >= 0)
            face.subMaterialId %= store.materials[face.materialId].numSubs;
        else
            face.subMaterialId = 0;
    }
    return true;
}

void appendPart(Part& to, const Part& from)
{
    int vOffset = (int)to.verts.size();
    to.verts.insert(to.verts.end(), from.verts.begin(), from.verts.end());
    to.uvs.insert(to.uvs.end(), from.uvs.begin(), from.uvs.end());
    for (MeASEFace f : from.faces)
    {
        for (int& id : f.vertexId)
            id += vOffset;
        to.faces.push_back(f);
    }
}

void destroyStore(MeASEMaterialStore* store)
{
    if (!store)
        return;
    for (int m = 0; m < store->numMaterials; ++m)
        std::free(store->materials[m].subMaterials);
    std::free(store->materials);
    std::free(store);
}

// Leaves head null when the parts turn out malformed.
void fill(MeASEMaterialStore* store, std::vector<std::vector<MeASESubMaterial>>& materials, std::vector<Part>& parts,
          const MeReal scale[3], MeBool asParts, MeASEObject** head)
{
    for (size_t m = 0; m < materials.size(); ++m)
    {
        // A material listed by index but never defined still gets one plain sub-material.
        if (materials[m].empty())
            materials[m].push_back(newSubMaterial());
        store->materials[m].numSubs = (int)materials[m].size();
        store->materials[m].subMaterials = copyArray(materials[m].data(), materials[m].size());
    }
    for (Part& part : parts)
        if (!finishPart(part, *store))
            return;
    if (!asParts)
    {
        for (size_t k = 1; k < parts.size(); ++k)
            appendPart(parts[0], parts[k]);
        parts.resize(1);
    }
    MeASEObject** link = head;
    for (const Part& part : parts)
    {
        makeObject(link, part, store, scale);
        link = &(*link)->nextObject;
    }
}

// Allocation failures, however large the file, end in a clean null instead of crossing the C boundary.
MeASEObject* load(MeStream fp, const MeReal scale[3], MeBool asParts)
{
    MeASEMaterialStore* store = nullptr;
    MeASEObject* head = nullptr;
    try
    {
        std::string text;
        char buffer[4096];
        size_t n;
        while ((n = MeStreamRead(buffer, 1, sizeof(buffer), fp)) > 0)
            text.append(buffer, n);

        Parser p;
        p.tok = tokenize(text);
        p.budget = p.tok.size();
        std::vector<std::vector<MeASESubMaterial>> materials;
        std::vector<Part> parts;
        while (p.more())
        {
            if (!p.atKey())
            {
                p.fail();
                break;
            }
            std::string key = p.tok[p.i++];
            if (key == "*MATERIAL_LIST")
                parseMaterialList(p, materials);
            else if (key == "*GEOMOBJECT")
            {
                parts.emplace_back();
                parseGeomObject(p, parts.back());
            }
            else
                p.skipArgs();
        }
        if (!p.ok || parts.empty())
            return nullptr;
        // Faces of a file without materials point at material 0, so it must exist.
        if (materials.empty())
            materials.emplace_back();

        store = allocate<MeASEMaterialStore>(1);
        store->materials = allocate<MeASEMaterial>(materials.size() + 1);
        store->numMaterials = (int)materials.size();
        fill(store, materials, parts, scale, asParts, &head);
    }
    catch (const std::bad_alloc&)
    {
        if (head)
            MeASEObjectDestroy(head);
        else
            destroyStore(store);
        return nullptr;
    }
    if (!head)
        destroyStore(store);
    return head;
}

} // namespace

extern "C" {

MeASEObject* MEAPI MeASEObjectLoadPartsFromStream(MeStream fp, MeReal xScale, MeReal yScale, MeReal zScale,
                                                  MeBool asParts)
{
    if (!fp)
        return nullptr;
    const MeReal scale[3] = {xScale, yScale, zScale};
    return load(fp, scale, asParts);
}

MeASEObject* MEAPI MeASEObjectLoadParts(char* filename, MeReal xScale, MeReal yScale, MeReal zScale, MeBool asParts)
{
    MeStream s = MeStreamOpenWithSearch(filename, kMeOpenModeRDBINARY);
    if (!s)
    {
        MeWarning(0, "MeASEObjectLoad: cannot open %s", filename);
        return nullptr;
    }
    MeASEObject* o = MeASEObjectLoadPartsFromStream(s, xScale, yScale, zScale, asParts);
    MeStreamClose(s);
    if (!o)
        MeWarning(0, "MeASEObjectLoad: %s is not a valid ASE file", filename);
    return o;
}

MeASEObject* MEAPI MeASEObjectLoad(char* filename, MeReal xScale, MeReal yScale, MeReal zScale)
{
    return MeASEObjectLoadParts(filename, xScale, yScale, zScale, 0);
}

// All parts of one file share the material store, which the first part owns.
void MEAPI MeASEObjectDestroy(MeASEObject* object)
{
    if (!object)
        return;
    destroyStore(object->matStore);
    while (object)
    {
        MeASEObject* next = object->nextObject;
        std::free(object->verts);
        std::free(object->faces);
        std::free(object->uvs);
        std::free(object);
        object = next;
    }
}

} // extern "C"
