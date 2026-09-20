#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <MeAssetDBXMLIO.h>

namespace {

template <class Enum>
struct Name
{
    Enum value;
    const char* name;
};

const Name<MeFPrimitiveType> kPrimitiveTypes[] = {
    {kMeFPrimitiveTypeSphere, "sphere"}, {kMeFPrimitiveTypeBox, "box"},     {kMeFPrimitiveTypeCylinder, "cylinder"},
    {kMeFPrimitiveTypeSphyl, "sphyl"},   {kMeFPrimitiveTypePlane, "plane"}, {kMeFPrimitiveTypeConvex, "convex"}};

const Name<MeFModelType> kModelTypes[] = {{kMeFModelTypeDynamicsAndGeometry, "dynamics_and_geometry"},
                                          {kMeFModelTypeDynamicsOnly, "dynamics_only"},
                                          {kMeFModelTypeGeometryOnly, "geometry_only"}};

const Name<MeFJointType> kJointTypes[] = {
    {kMeFJointTypeCarwheel, "carwheel"},    {kMeFJointTypeHinge, "hinge"},         {kMeFJointTypeBallAndSocket, "ballandsocket"},
    {kMeFJointTypeConeLimit, "conelimit"}, {kMeFJointTypeUniversal, "universal"}, {kMeFJointTypeRpro, "rpro"},
    {kMeFJointTypePrismatic, "prismatic"},  {kMeFJointTypeSkeletal, "skeletal"},   {kMeFJointTypeAngular3, "angular3"},
    {kMeFJointTypeSpring6, "spring6"}};

template <class Enum, size_t N>
Enum lookup(const Name<Enum> (&names)[N], const char* name, Enum fallback)
{
    for (const auto& n : names)
        if (name && std::strcmp(n.name, name) == 0)
            return n.value;
    return fallback;
}

template <class Enum, size_t N>
const char* nameOf(const Name<Enum> (&names)[N], Enum value)
{
    for (const auto& n : names)
        if (n.value == value)
            return n.name;
    return "unknown";
}

constexpr unsigned bit(MeFJointType t)
{
    return 1u << t;
}

struct JointField
{
    unsigned types;
    const char* tag;
    size_t offset;
    bool real;
};

// The rpro strength tags are not attested anywhere and follow the spring6 naming. Universal and ball and socket
// joints have no per type properties.
const unsigned kLimited = bit(kMeFJointTypeHinge) | bit(kMeFJointTypePrismatic);
const unsigned kSkeletal = bit(kMeFJointTypeSkeletal);
const unsigned kCarwheel = bit(kMeFJointTypeCarwheel);
const unsigned kConeLimit = bit(kMeFJointTypeConeLimit);
const unsigned kAngular3 = bit(kMeFJointTypeAngular3);
const unsigned kSpring6 = bit(kMeFJointTypeSpring6);
const unsigned kRpro = bit(kMeFJointTypeRpro);
const JointField kJointFields[] = {
    {kLimited, "HIGH_LIMIT", offsetof(MeFJoint, stop[0]), true},
    {kLimited, "LOW_LIMIT", offsetof(MeFJoint, stop[1]), true},
    {kLimited, "HIGH_STIFFNESS", offsetof(MeFJoint, stiff[0]), true},
    {kLimited, "LOW_STIFFNESS", offsetof(MeFJoint, stiff[1]), true},
    {kLimited, "LIMITED", offsetof(MeFJoint, bLimited[0]), false},
    {kLimited, "MOTORIZED", offsetof(MeFJoint, bMotorized[0]), false},
    {kLimited, "DES_VEL", offsetof(MeFJoint, desVel[0]), true},
    {kLimited, "MAX_FORCE", offsetof(MeFJoint, strength[0]), true},
    {kSkeletal, "CONE_TYPE", offsetof(MeFJoint, special_i[1]), false},
    {kSkeletal, "CONE_HALF_ANGLE_X", offsetof(MeFJoint, stop[0]), true},
    {kSkeletal, "CONE_HALF_ANGLE_Y", offsetof(MeFJoint, stop[1]), true},
    {kSkeletal, "CONE_STIFFNESS", offsetof(MeFJoint, stiff[0]), true},
    {kSkeletal, "CONE_DAMPING", offsetof(MeFJoint, damp[0]), true},
    {kSkeletal, "TWIST_TYPE", offsetof(MeFJoint, special_i[0]), false},
    {kSkeletal, "TWIST_HALF_ANGLE", offsetof(MeFJoint, stop[2]), true},
    {kSkeletal, "TWIST_STIFFNESS", offsetof(MeFJoint, stiff[1]), true},
    {kSkeletal, "TWIST_DAMPING", offsetof(MeFJoint, damp[1]), true},
    {kCarwheel, "HINGE_DES_VEL", offsetof(MeFJoint, desVel[0]), true},
    {kCarwheel, "HINGE_MAX_FORCE", offsetof(MeFJoint, strength[0]), true},
    {kCarwheel, "STEERING_DES_VEL", offsetof(MeFJoint, desVel[1]), true},
    {kCarwheel, "STEERING_MAX_FORCE", offsetof(MeFJoint, strength[1]), true},
    {kCarwheel, "STEERING_LOCK", offsetof(MeFJoint, special_b[0]), false},
    {kCarwheel, "SUSP_HIGH_LIMIT", offsetof(MeFJoint, stop[0]), true},
    {kCarwheel, "SUSP_LOW_LIMIT", offsetof(MeFJoint, stop[1]), true},
    {kCarwheel, "SUSP_REF", offsetof(MeFJoint, special_f[0]), true},
    {kCarwheel, "SUSP_STIFFNESS", offsetof(MeFJoint, stiff[0]), true},
    {kCarwheel, "SUSP_DAMPING", offsetof(MeFJoint, damp[0]), true},
    {kCarwheel, "SUSP_SOFTNESS", offsetof(MeFJoint, stiff[2]), true},
    {kConeLimit, "HALF_ANGLE", offsetof(MeFJoint, stop[0]), true},
    {kConeLimit | kAngular3, "STIFFNESS", offsetof(MeFJoint, stiff[0]), true},
    {kConeLimit | kAngular3, "DAMPING", offsetof(MeFJoint, damp[0]), true},
    {kAngular3, "ROTATION_ENABLED", offsetof(MeFJoint, special_b[0]), false},
    {kSpring6, "LINEAR_STIFF_X", offsetof(MeFJoint, stiff[0]), true},
    {kSpring6, "LINEAR_STIFF_Y", offsetof(MeFJoint, stiff[1]), true},
    {kSpring6, "LINEAR_STIFF_Z", offsetof(MeFJoint, stiff[2]), true},
    {kSpring6, "ANGULAR_STIFF_X", offsetof(MeFJoint, stiff[3]), true},
    {kSpring6, "ANGULAR_STIFF_Y", offsetof(MeFJoint, stiff[4]), true},
    {kSpring6, "ANGULAR_STIFF_Z", offsetof(MeFJoint, stiff[5]), true},
    {kSpring6, "LINEAR_DAMP_X", offsetof(MeFJoint, damp[0]), true},
    {kSpring6, "LINEAR_DAMP_Y", offsetof(MeFJoint, damp[1]), true},
    {kSpring6, "LINEAR_DAMP_Z", offsetof(MeFJoint, damp[2]), true},
    {kSpring6, "ANGULAR_DAMP_X", offsetof(MeFJoint, damp[3]), true},
    {kSpring6, "ANGULAR_DAMP_Y", offsetof(MeFJoint, damp[4]), true},
    {kSpring6, "ANGULAR_DAMP_Z", offsetof(MeFJoint, damp[5]), true},
    {kRpro, "LINEAR_STRENGTH_X", offsetof(MeFJoint, strength[0]), true},
    {kRpro, "LINEAR_STRENGTH_Y", offsetof(MeFJoint, strength[1]), true},
    {kRpro, "LINEAR_STRENGTH_Z", offsetof(MeFJoint, strength[2]), true},
    {kRpro, "ANGULAR_STRENGTH_X", offsetof(MeFJoint, strength[3]), true},
    {kRpro, "ANGULAR_STRENGTH_Y", offsetof(MeFJoint, strength[4]), true},
    {kRpro, "ANGULAR_STRENGTH_Z", offsetof(MeFJoint, strength[5]), true},
};

struct VectorField
{
    const char* tag;
    size_t offset;
};

const VectorField kJointVectors[] = {
    {"POS1", offsetof(MeFJoint, pos[0])},             {"POS2", offsetof(MeFJoint, pos[1])},
    {"PRIMARY_AXIS1", offsetof(MeFJoint, pax[0])},    {"PRIMARY_AXIS2", offsetof(MeFJoint, pax[1])},
    {"ORTHOGONAL_AXIS1", offsetof(MeFJoint, oax[0])}, {"ORTHOGONAL_AXIS2", offsetof(MeFJoint, oax[1])},
};

MeXMLHandler parseHandler(const char* tag, MeXMLParseFn fn, size_t offset, unsigned int count)
{
    MeXMLHandler h{};
    h.name = const_cast<char*>(tag);
    h.type = MeXMLActionParse;
    h.fn = reinterpret_cast<void*>(fn);
    h.offset = static_cast<unsigned int>(offset);
    h.max = count;
    return h;
}

MeXMLHandler reals(const char* tag, size_t offset, unsigned int count)
{
    return parseHandler(tag, MeXMLParseMeRealArray, offset, count);
}

MeXMLHandler real(const char* tag, size_t offset)
{
    return parseHandler(tag, MeXMLParseMeReal, offset, 0);
}

MeXMLHandler integer(const char* tag, size_t offset)
{
    return parseHandler(tag, MeXMLParseInt, offset, 0);
}

MeXMLHandler element(const char* tag, MeXMLCallback fn)
{
    MeXMLHandler h{};
    h.name = const_cast<char*>(tag);
    h.type = MeXMLActionCallback;
    h.fn = reinterpret_cast<void*>(fn);
    return h;
}

MeXMLHandler end()
{
    MeXMLHandler h{};
    h.type = MeXMLActionEnd;
    return h;
}

class Attributes
{
public:
    explicit Attributes(const MeXMLElement* elem) : e_(PElementCreate(0, nullptr, nullptr, nullptr, elem->attr)) {}
    ~Attributes() { PElementDestroy(e_); }
    Attributes(const Attributes&) = delete;
    Attributes& operator=(const Attributes&) = delete;
    const char* operator[](const char* name) const { return PElementGetAttributeValue(e_, const_cast<char*>(name)); }

private:
    PElement* e_;
};

MeXMLError process(const MeXMLElement* elem, std::vector<MeXMLHandler> handlers, void* data)
{
    handlers.push_back(end());
    return MeXMLElementProcess(const_cast<MeXMLElement*>(elem), handlers.data(), data, data);
}

// Every object is keyed by its id, so an element without one cannot be loaded.
bool hasId(const MeXMLElement* elem, const Attributes& attrs)
{
    if (attrs["id"])
        return true;
    MeXMLInputSetErrorString(elem->fi, const_cast<char*>("line %d: <%s> has no id"), elem->fi->line + 1, elem->name);
    return false;
}

struct ReadContext
{
    MeAssetDBXMLInput* input;
    bool firstOnly;
    MeFAsset* first;
};

struct AssetContext
{
    MeFAsset* asset;
    std::vector<std::pair<std::string, std::string>> disabled;
};

MeXMLError MEAPI readVertex(const MeXMLElement* elem, void* prim)
{
    MeVector3 v;
    MeXMLHandler h = reals("VERTEX", 0, 3);
    MeXMLError err = MeXMLParseMeRealArray(elem->fi, &h, v);
    if (err == MeXMLErrorNone)
        MeFPrimitiveAddVertex(static_cast<MeFPrimitive*>(prim), v);
    return err;
}

MeXMLError MEAPI readPrimitive(const MeXMLElement* elem, void* geometry)
{
    Attributes attrs(elem);
    if (!hasId(elem, attrs))
        return MeXMLErrorSemantic;
    MeFPrimitive* p = MeFPrimitiveCreate(attrs["id"], lookup(kPrimitiveTypes, attrs["type"], kMeFPrimitiveTypeUnknown));
    MeFGeometryInsertPrimitive(static_cast<MeFGeometry*>(geometry), p);
    return process(elem,
                   {real("RADIUS", offsetof(MeFPrimitive, dims[0])), real("HEIGHT", offsetof(MeFPrimitive, dims[1])),
                    reals("DIMS", offsetof(MeFPrimitive, dims), 3), reals("TM", offsetof(MeFPrimitive, tm), 16),
                    element("VERTEX", readVertex)},
                   p);
}

template <class T>
void readGraphic(const Attributes& attrs, T* item, void(MEAPI* setHint)(T*, const char*), void(MEAPI* setScale)(T*, MeReal))
{
    if (const char* g = attrs["graphic"])
        setHint(item, g);
    if (const char* s = attrs["scale"])
        setScale(item, static_cast<MeReal>(std::strtod(s, nullptr)));
}

MeXMLError MEAPI readGeometry(const MeXMLElement* elem, void* parent)
{
    Attributes attrs(elem);
    if (!hasId(elem, attrs))
        return MeXMLErrorSemantic;
    MeFGeometry* g = MeFGeometryCreate(attrs["id"]);
    readGraphic(attrs, g, MeFGeometrySetGraphicHint, MeFGeometrySetGraphicScale);
    MeFAssetInsertGeometry(static_cast<AssetContext*>(parent)->asset, g);
    return process(elem, {element("PRIMITIVE", readPrimitive)}, g);
}

MeXMLError MEAPI readDynamics(const MeXMLElement* elem, void* model)
{
    return process(elem,
                   {real("MASS", offsetof(MeFModel, mass)), real("DENSITY", offsetof(MeFModel, density)),
                    reals("MASS_OFFSET", offsetof(MeFModel, mass_offset), 3), reals("INERTIA", offsetof(MeFModel, inertia), 6),
                    real("LIN_DAMP", offsetof(MeFModel, linearDamp)), real("ANG_DAMP", offsetof(MeFModel, angularDamp)),
                    reals("FAST_SPIN", offsetof(MeFModel, fastSpin), 3), integer("USE_FAST_SPIN", offsetof(MeFModel, useFastSpin))},
                   model);
}

MeXMLError MEAPI readModel(const MeXMLElement* elem, void* parent)
{
    Attributes attrs(elem);
    if (!hasId(elem, attrs))
        return MeXMLErrorSemantic;
    MeFModel* m = MeFModelCreate(attrs["id"], lookup(kModelTypes, attrs["type"], kMeFModelTypeDynamicsAndGeometry));
    MeFModelSetGeometryByName(m, attrs["geometry"]);
    MeFAssetInsertModel(static_cast<AssetContext*>(parent)->asset, m);
    return process(elem, {element("DYNAMICS", readDynamics)}, m);
}

MeXMLError MEAPI readPart(const MeXMLElement* elem, void* parent)
{
    Attributes attrs(elem);
    if (!hasId(elem, attrs))
        return MeXMLErrorSemantic;
    MeFAssetPart* p = MeFAssetPartCreate(attrs["id"], nullptr, nullptr);
    MeFAssetPartSetModelByName(p, attrs["model"]);
    MeFAssetPartSetParentPartByName(p, attrs["parent"]);
    readGraphic(attrs, p, MeFAssetPartSetGraphicHint, MeFAssetPartSetGraphicScale);
    MeFAssetInsertPart(static_cast<AssetContext*>(parent)->asset, p);
    return process(elem, {reals("TM", offsetof(MeFAssetPart, tm), 16)}, p);
}

MeXMLError MEAPI readJoint(const MeXMLElement* elem, void* parent)
{
    Attributes attrs(elem);
    if (!hasId(elem, attrs))
        return MeXMLErrorSemantic;
    MeFJointType type = lookup(kJointTypes, attrs["type"], kMeFJointTypeUnknown);
    MeFJoint* j = MeFJointCreate(attrs["id"], type);
    MeFJointSetPartByName(j, attrs["part1"], 0);
    MeFJointSetPartByName(j, attrs["part2"], 1);
    MeFAssetInsertJoint(static_cast<AssetContext*>(parent)->asset, j);
    std::vector<MeXMLHandler> handlers;
    for (const JointField& f : kJointFields)
        if (f.types & bit(type))
            handlers.push_back(f.real ? real(f.tag, f.offset) : integer(f.tag, f.offset));
    for (const VectorField& f : kJointVectors)
        handlers.push_back(reals(f.tag, f.offset, 3));
    return process(elem, handlers, j);
}

MeXMLError MEAPI readNoCollision(const MeXMLElement* elem, void* parent)
{
    Attributes attrs(elem);
    if (attrs["part1"] && attrs["part2"])
        static_cast<AssetContext*>(parent)->disabled.emplace_back(attrs["part1"], attrs["part2"]);
    return MeXMLErrorNone;
}

MeXMLError MEAPI readAsset(const MeXMLElement* elem, void* parent)
{
    ReadContext* ctx = static_cast<ReadContext*>(parent);
    if (ctx->firstOnly && ctx->first)
        return MeXMLErrorNone;
    Attributes attrs(elem);
    if (!hasId(elem, attrs))
        return MeXMLErrorSemantic;
    MeIDPool* pool = ctx->input->IDPool;
    AssetContext actx{MeFAssetCreate(attrs["id"], pool ? MeIDPoolRequestID(pool) : 0), {}};
    readGraphic(attrs, actx.asset, MeFAssetSetGraphicHint, MeFAssetSetGraphicScale);
    MeXMLError err = process(elem,
                             {element("GEOMETRY", readGeometry), element("MODEL", readModel), element("PART", readPart),
                              element("JOINT", readJoint), element("NO_COLLISION", readNoCollision)},
                             &actx);
    if (err != MeXMLErrorNone)
    {
        if (pool)
            MeIDPoolReturnID(pool, actx.asset->id);
        MeFAssetDestroy(actx.asset);
        return err;
    }
    // Collision pairs are applied last because they may precede the parts they name.
    for (auto& d : actx.disabled)
        MeFAssetPartEnableCollision(MeFAssetLookupPart(actx.asset, &d.first[0]), MeFAssetLookupPart(actx.asset, &d.second[0]), 0);
    MeAssetDBInsertAsset(ctx->input->db, actx.asset);
    if (!ctx->first)
        ctx->first = actx.asset;
    return MeXMLErrorNone;
}

MeXMLError MEAPI readKarma(const MeXMLElement* elem, void* parent)
{
    return process(elem, {element("ASSET", readAsset)}, parent);
}

MeFAsset* read(MeAssetDBXMLInput* input, MeStream stream, bool firstOnly, bool& ok)
{
    ReadContext ctx{input, firstOnly, nullptr};
    MeXMLInput* in = MeXMLInputCreate(stream);
    MeXMLHandler handlers[] = {element("KARMA", readKarma), end()};
    ok = MeXMLInputProcess(in, handlers, &ctx) == MeXMLErrorNone;
    if (!ok)
        std::fprintf(stderr, "MeAssetDBXMLIO: %s\n", MeXMLInputGetErrorString(in));
    MeXMLInputDestroy(in);
    return ctx.first;
}

std::string numbers(const MeReal* v, int n)
{
    std::string s;
    char buf[32];
    for (int i = 0; i < n; ++i)
    {
        std::snprintf(buf, sizeof(buf), "%.*g", std::numeric_limits<MeReal>::max_digits10, static_cast<double>(v[i]));
        s += (i ? "," : "") + std::string(buf);
    }
    return s;
}

void leaf(MeXMLOutput* op, int parent, const char* tag, const std::string& text)
{
    MeXMLWriteElement(op, parent, const_cast<char*>(tag), nullptr);
    MeXMLWritePCDATA(op, const_cast<char*>("%s"), text.c_str());
}

void leaf(MeXMLOutput* op, int parent, const char* tag, const MeReal* v, int n)
{
    leaf(op, parent, tag, numbers(v, n));
}

void leaf(MeXMLOutput* op, int parent, const char* tag, int value)
{
    leaf(op, parent, tag, std::to_string(value));
}

char* str(const char* s)
{
    return const_cast<char*>(s);
}

void writeAsset(MeXMLOutput* op, int root, MeFAsset* a)
{
    std::string scale = numbers(&a->graphicScale, 1);
    int ae = MeXMLWriteElement(op, root, str("ASSET"), "id", a->name, "graphic", a->graphicHint, "scale", scale.c_str(), nullptr);

    MeFGeometryIt git;
    MeFAssetInitGeometryIterator(a, &git);
    while (MeFGeometry* g = MeFAssetGetGeometry(&git))
    {
        std::string gscale = numbers(&g->graphicScale, 1);
        int ge = MeXMLWriteElement(op, ae, str("GEOMETRY"), "id", g->id, "graphic", g->graphicHint, "scale",
                                   g->graphicScale != 1 ? gscale.c_str() : nullptr, nullptr);
        MeFPrimitiveIt pit;
        MeFGeometryInitPrimitiveIterator(g, &pit);
        while (MeFPrimitive* p = MeFGeometryGetPrimitive(&pit))
        {
            int pe = MeXMLWriteElement(op, ge, str("PRIMITIVE"), "id", p->id, "type", nameOf(kPrimitiveTypes, p->type), nullptr);
            switch (p->type)
            {
            case kMeFPrimitiveTypeSphere:
                leaf(op, pe, "RADIUS", p->dims, 1);
                break;
            case kMeFPrimitiveTypeCylinder:
            case kMeFPrimitiveTypeSphyl:
                leaf(op, pe, "RADIUS", p->dims, 1);
                leaf(op, pe, "HEIGHT", p->dims + 1, 1);
                break;
            case kMeFPrimitiveTypeBox:
                leaf(op, pe, "DIMS", p->dims, 3);
                break;
            default:
                break;
            }
            for (int i = 0; i < p->nVertices; ++i)
                leaf(op, pe, "VERTEX", p->vertices[i], 3);
            leaf(op, pe, "TM", &p->tm[0][0], 16);
        }
    }

    MeFModelIt mit;
    MeFAssetInitModelIterator(a, &mit);
    while (MeFModel* m = MeFAssetGetModel(&mit))
    {
        const char* type = nameOf(kModelTypes, m->type);
        int me = MeXMLWriteElement(op, ae, str("MODEL"), "id", m->id, "type", type, "geometry", m->geometry, nullptr);
        if (m->type == kMeFModelTypeGeometryOnly)
            continue;
        int de = MeXMLWriteElement(op, me, str("DYNAMICS"), nullptr);
        leaf(op, de, "MASS", &m->mass, 1);
        leaf(op, de, "DENSITY", &m->density, 1);
        leaf(op, de, "MASS_OFFSET", m->mass_offset, 3);
        leaf(op, de, "INERTIA", m->inertia, 6);
        leaf(op, de, "LIN_DAMP", &m->linearDamp, 1);
        leaf(op, de, "ANG_DAMP", &m->angularDamp, 1);
        leaf(op, de, "FAST_SPIN", m->fastSpin, 3);
        leaf(op, de, "USE_FAST_SPIN", m->useFastSpin);
    }

    std::vector<MeFAssetPart*> parts;
    MeFAssetPartIt pit;
    MeFAssetInitPartIterator(a, &pit);
    while (MeFAssetPart* p = MeFAssetGetPart(&pit))
    {
        parts.push_back(p);
        std::string pscale = numbers(&p->graphicScale, 1);
        int pe = MeXMLWriteElement(op, ae, str("PART"), "id", p->id, "model", p->model, "parent", p->parent, "graphic", p->graphicHint,
                                   "scale", p->graphicScale != 1 ? pscale.c_str() : nullptr, nullptr);
        leaf(op, pe, "TM", &p->tm[0][0], 16);
    }
    for (size_t i = 0; i < parts.size(); ++i)
        for (size_t k = i + 1; k < parts.size(); ++k)
            if (!MeFAssetPartIsCollisionEnabled(parts[i], parts[k]))
                MeXMLWriteElement(op, ae, str("NO_COLLISION"), "part1", parts[i]->id, "part2", parts[k]->id, nullptr);

    MeFJointIt jit;
    MeFAssetInitJointIterator(a, &jit);
    while (MeFJoint* j = MeFAssetGetJoint(&jit))
    {
        int je = MeXMLWriteElement(op, ae, str("JOINT"), "id", j->id, "part1", j->part[0], "part2", j->part[1], "type", nameOf(kJointTypes, j->type), nullptr);
        const char* base = reinterpret_cast<const char*>(j);
        for (const JointField& f : kJointFields)
        {
            if (!(f.types & bit(j->type)))
                continue;
            if (f.real)
                leaf(op, je, f.tag, reinterpret_cast<const MeReal*>(base + f.offset), 1);
            else
                leaf(op, je, f.tag, *reinterpret_cast<const int*>(base + f.offset));
        }
        for (const VectorField& f : kJointVectors)
            leaf(op, je, f.tag, reinterpret_cast<const MeReal*>(base + f.offset), 3);
    }
}

} // namespace

extern "C" {

const char* MEAPI GetLatestKaFileVersionString()
{
    return "1.0";
}

MeAssetDBXMLInput* MEAPI MeAssetDBXMLInputCreate(MeAssetDB* db, MeIDPool* IDPool)
{
    MeAssetDBXMLInput* input = static_cast<MeAssetDBXMLInput*>(std::malloc(sizeof(MeAssetDBXMLInput)));
    input->db = db;
    input->IDPool = IDPool;
    return input;
}

void MEAPI MeAssetDBXMLInputDestroy(MeAssetDBXMLInput* input)
{
    std::free(input);
}

MeBool MEAPI MeAssetDBXMLInputRead(MeAssetDBXMLInput* input, MeStream stream)
{
    bool ok;
    read(input, stream, false, ok);
    return ok;
}

MeFAsset* MEAPI MeAssetDBXMLInputReadFirst(MeAssetDBXMLInput* input, MeStream stream)
{
    bool ok;
    return read(input, stream, true, ok);
}

MeAssetDBXMLOutput* MEAPI MeAssetDBXMLOutputCreate(MeAssetDB* db)
{
    MeAssetDBXMLOutput* output = static_cast<MeAssetDBXMLOutput*>(std::calloc(1, sizeof(MeAssetDBXMLOutput)));
    output->db = db;
    return output;
}

void MEAPI MeAssetDBXMLOutputDestroy(MeAssetDBXMLOutput* output)
{
    if (!output)
        return;
    std::free(output->fileHeader);
    std::free(output);
}

void MEAPI MeAssetDBXMLOutputWrite(MeAssetDBXMLOutput* output, MeStream stream)
{
    static const char declaration[] = "<?xml version=\"1.0\"?>\n";
    MeStreamWrite(const_cast<char*>(declaration), 1, sizeof(declaration) - 1, stream);
    MeXMLOutput* op = MeXMLOutputCreate(stream);
    if (output->fileHeader)
        MeXMLWriteComment(op, str("%s"), output->fileHeader);
    int root = MeXMLWriteElement(op, 0, str("KARMA"), "ka_file_version", GetLatestKaFileVersionString(), nullptr);
    MeFAssetIt it;
    MeAssetDBInitAssetIterator(output->db, &it);
    while (MeFAsset* a = MeAssetDBGetAsset(&it))
        writeAsset(op, root, a);
    MeXMLOutputDestroy(op);
}

void MEAPI MeAssetDBXMLOutputSetFileHeader(MeAssetDBXMLOutput* output, char* header, ...)
{
    va_list ap, copy;
    va_start(ap, header);
    va_copy(copy, ap);
    int n = std::vsnprintf(nullptr, 0, header, copy);
    va_end(copy);
    std::free(output->fileHeader);
    output->fileHeader = static_cast<char*>(std::malloc(n + 1));
    std::vsnprintf(output->fileHeader, n + 1, header, ap);
    va_end(ap);
}

} // extern "C"
