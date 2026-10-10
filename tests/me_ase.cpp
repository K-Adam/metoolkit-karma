#include <cstdio>
#include <cstring>
#include <string>

#include <MeASELoad.h>
#include <MeAssetDB.h>

#include "check.h"

namespace {

const char* kMaterials = R"(*3DSMAX_ASCIIEXPORT 200
*SCENE { *SCENE_FILENAME "two.max" }
*MATERIAL_LIST {
  *MATERIAL_COUNT 1
  *MATERIAL 0 {
    *MATERIAL_NAME "Multi"
    *NUMSUBMTLS 2
    *SUBMATERIAL 0 { *MATERIAL_DIFFUSE 1.0 0.0 0.0 }
    *SUBMATERIAL 1 {
      *MATERIAL_AMBIENT 0.1 0.2 0.3
      *MAP_DIFFUSE { *MAP_NAME "Map" *BITMAP "C:\maps\wood.bmp" }
    }
  }
}
)";

// A tetrahedron, with texture coordinates, normals and a material.
const char* kTetra = R"(*GEOMOBJECT {
  *NODE_NAME "Tetra"
  *NODE_TM { *NODE_NAME "Tetra" *TM_ROW0 1.0 0.0 0.0 }
  *MESH {
    *TIMEVALUE 0
    *MESH_NUMVERTEX 4
    *MESH_NUMFACES 4
    *MESH_VERTEX_LIST {
      *MESH_VERTEX 0 0.0 0.0 0.0
      *MESH_VERTEX 1 1.0 0.0 0.0
      *MESH_VERTEX 2 0.0 1.0 0.0
      *MESH_VERTEX 3 0.0 0.0 1.0
    }
    *MESH_FACE_LIST {
      *MESH_FACE 0: A: 0 B: 2 C: 1 AB: 1 BC: 1 CA: 0 *MESH_SMOOTHING 1 *MESH_MTLID 0
      *MESH_FACE 1: A: 0 B: 1 C: 3 AB: 1 BC: 1 CA: 0 *MESH_SMOOTHING *MESH_MTLID 1
      *MESH_FACE 2: A: 0 B: 3 C: 2 AB: 1 BC: 1 CA: 0 *MESH_SMOOTHING 1,2 *MESH_MTLID 2
      *MESH_FACE 3: A: 1 B: 2 C: 3 AB: 1 BC: 1 CA: 0 *MESH_MTLID 3
    }
    *MESH_NUMTVERTEX 2
    *MESH_TVERTLIST {
      *MESH_TVERT 0 0.0 0.0 0.0
      *MESH_TVERT 1 0.25 0.75 0.0
    }
    *MESH_NUMTVFACES 4
    *MESH_TFACELIST {
      *MESH_TFACE 0 0 1 0
      *MESH_TFACE 1 0 0 0
      *MESH_TFACE 2 0 0 0
      *MESH_TFACE 3 1 1 1
    }
    *MESH_NORMALS {
      *MESH_FACENORMAL 0 0.0 0.0 -1.0
        *MESH_VERTEXNORMAL 0 0.0 0.0 -1.0
        *MESH_VERTEXNORMAL 2 0.0 0.5 -1.0
        *MESH_VERTEXNORMAL 1 0.0 0.0 -1.0
    }
  }
  *MATERIAL_REF 0
}
)";

const char* kQuad = R"(*GEOMOBJECT {
  *NODE_NAME "Quad"
  *MESH {
    *MESH_NUMVERTEX 3
    *MESH_NUMFACES 1
    *MESH_VERTEX_LIST {
      *MESH_VERTEX 0 5.0 0.0 0.0
      *MESH_VERTEX 1 6.0 0.0 0.0
      *MESH_VERTEX 2 5.0 1.0 0.0
    }
    *MESH_FACE_LIST { *MESH_FACE 0: A: 0 B: 1 C: 2 }
  }
}
)";

MeASEObject* load(const std::string& text, MeBool asParts)
{
    unsigned n = (unsigned)text.size();
    MeStream s = MeStreamCreateFromMemBuffer(const_cast<char*>(text.data()), n, n);
    MeASEObject* o = MeASEObjectLoadPartsFromStream(s, 2, 2, 2, asParts);
    MeStreamClose(s);
    return o;
}

// Replaces the first occurrence of what in text.
std::string with(std::string text, const char* what, const char* by)
{
    size_t at = text.find(what);
    CHECK(at != std::string::npos);
    return text.replace(at, std::strlen(what), by);
}

} // namespace

int main()
{
    const std::string file = std::string(kMaterials) + kTetra + kQuad;

    MeASEObject* parts = load(file, 1);
    CHECK(parts && parts->isLoaded);
    CHECK(std::strcmp(parts->name, "Tetra") == 0);
    CHECK(parts->numVerts == 4 && parts->numFaces == 4 && parts->numUvs == 2);
    CHECK_NEAR(parts->verts[3][2], 2, 0);
    CHECK(parts->faces[0].vertexId[1] == 2);
    CHECK_NEAR(parts->faces[0].map[1].u, 0.25, 1e-6);
    CHECK_NEAR(parts->faces[3].map[2].v, 0.75, 1e-6);
    CHECK_NEAR(parts->faces[0].normal[2], -1, 0);
    CHECK_NEAR(parts->faces[0].vNormal[1][1], 0.5, 0);
    // Sub-material IDs wrap around the two sub-materials.
    const int subs[4] = {0, 1, 0, 1};
    for (int f = 0; f < 4; ++f)
        CHECK(parts->faces[f].materialId == 0 && parts->faces[f].subMaterialId == subs[f]);
    MeASEMaterialStore* store = parts->matStore;
    CHECK(store->numMaterials == 1 && store->materials[0].numSubs == 2);
    MeASESubMaterial& wood = store->materials[0].subMaterials[1];
    CHECK(wood.type == MeASEMaterialFlagTexture && std::strcmp(wood.texFilename, "C:\\maps\\wood.bmp") == 0);
    CHECK_NEAR(wood.ambient[2], 0.3, 1e-6);
    CHECK(store->materials[0].subMaterials[0].type == MeASEMaterialFlagColor);

    MeASEObject* quad = parts->nextObject;
    CHECK(quad && quad->matStore == store && !quad->nextObject);
    CHECK(quad->numFaces == 1 && quad->numUvs == 0 && quad->faces[0].vertexId[2] == 2);
    CHECK_NEAR(quad->verts[0][0], 10, 0);
    MeASEObjectDestroy(parts);

    // Merged, the second object's vertex indices follow the first object's vertices.
    MeASEObject* merged = load(file, 0);
    CHECK(merged && !merged->nextObject && merged->numVerts == 7 && merged->numFaces == 5);
    CHECK(merged->faces[4].vertexId[0] == 4 && merged->faces[4].vertexId[2] == 6);
    MeASEObjectDestroy(merged);
    MeASEObjectDestroy(nullptr);

    // Malformed input fails cleanly instead of reading out of bounds.
    const char* bad[][2] = {
        {"*MESH_VERTEX 3 0.0", "*MESH_VERTEX 4 0.0"},
        {"A: 1 B: 2 C: 3", "A: 1 B: 2 C: 9"},
        {"A: 1 B: 2 C: 3", "A: 1 B: 2 C: -1"},
        {"*MESH_TFACE 3 1 1 1", "*MESH_TFACE 3 1 1 2"},
        {"*MESH_NUMTVFACES 4", "*MESH_NUMTVFACES 3"},
        {"*MATERIAL_REF 0", "*MATERIAL_REF 1"},
        {"*MESH_NUMVERTEX 4", "*MESH_NUMVERTEX 999999999"},
        {"*MESH_VERTEX 1 1.0", "*MESH_VERTEX 1 x"},
        {"*MESH_NUMFACES 4", "*MESH_NUMFACES 5"},
        {"*MESH_NORMALS {", "*MESH_NUMTVERTEX 1 *MESH_NORMALS {"},
        {"*SUBMATERIAL 1 {", "*SUBMATERIAL 4000 {"},
        {"*MESH_NUMVERTEX 3", "*MESH_NUMVERTEX 3000"},
    };
    for (auto& b : bad)
        CHECK(load(with(file, b[0], b[1]), 1) == nullptr);
    CHECK(load(file.substr(0, file.size() - 3), 1) == nullptr);
    CHECK(load(std::string(100000, '{'), 1) == nullptr);
    CHECK(load(std::string(kMaterials), 1) == nullptr);
    CHECK(load("", 1) == nullptr);
    CHECK(MeASEObjectLoad(const_cast<char*>("no_such_file.ase"), 1, 1, 1) == nullptr);

    // Each object becomes a convex primitive of the geometry.
    const char* path = "me_ase_test.ase";
    std::FILE* f = std::fopen(path, "wb");
    CHECK(f);
    std::fwrite(file.data(), 1, file.size(), f);
    std::fclose(f);
    MeASEObject* fromFile = MeASEObjectLoad(const_cast<char*>(path), 1, 1, 1);
    CHECK(fromFile && fromFile->numVerts == 7);
    MeASEObjectDestroy(fromFile);
    MeFGeometry* g = MeFGeometryCreateFromASE("ase", const_cast<char*>(path), 1, 1, 3);
    CHECK(g && std::strcmp(MeFGeometryGetName(g), "ase") == 0 && MeFGeometryGetPrimitiveCount(g) == 2);
    MeFPrimitive* tetra = MeFGeometryLookupPrimitive(g, const_cast<char*>("Tetra"));
    CHECK(tetra && MeFPrimitiveGetType(tetra) == kMeFPrimitiveTypeConvex && MeFPrimitiveGetVertexCount(tetra) == 4);
    CHECK_NEAR(MeFPrimitiveGetVertexArray(tetra)[3][2], 3, 0);
    MeFGeometryDestroy(g);
    CHECK(MeFGeometryCreateFromASE("none", const_cast<char*>("no_such_file.ase"), 1, 1, 1) == nullptr);
    std::remove(path);
    return 0;
}
