#include "MdtImpl.h"

namespace {

void setOption(MdtContactParamsID p, int option, bool on)
{
    p->options = on ? (p->options | option) : (p->options & ~option);
}

} // namespace

extern "C" {

void MEAPI MdtContactParamsReset(const MdtContactParamsID p)
{
    *p = MdtBclContactParams{};
    p->type = MdtContactTypeFrictionZero;
    p->model1 = p->model2 = MdtFrictionModelBox;
}

MdtContactType MEAPI MdtContactParamsGetType(const MdtContactParamsID p) { return p->type; }
MdtFrictionModel MEAPI MdtContactParamsGetPrimaryFrictionModel(const MdtContactParamsID p) { return p->model1; }
MdtFrictionModel MEAPI MdtContactParamsGetSecondaryFrictionModel(const MdtContactParamsID p) { return p->model2; }
MeReal MEAPI MdtContactParamsGetRestitution(const MdtContactParamsID p) { return p->restitution; }
MeReal MEAPI MdtContactParamsGetRestitutionThreshold(const MdtContactParamsID p) { return p->velThreshold; }
MeReal MEAPI MdtContactParamsGetSoftness(const MdtContactParamsID p) { return p->softness; }
MeReal MEAPI MdtContactParamsGetMaxAdhesiveForce(const MdtContactParamsID p) { return p->max_adhesive_force; }
MeReal MEAPI MdtContactParamsGetPrimaryFriction(const MdtContactParamsID p) { return p->friction1; }
MeReal MEAPI MdtContactParamsGetPrimaryFrictionCoeffecient(const MdtContactParamsID p) { return p->frictioncoeff1; }
MeReal MEAPI MdtContactParamsGetPrimarySlip(const MdtContactParamsID p) { return p->slip1; }
MeReal MEAPI MdtContactParamsGetPrimarySlide(const MdtContactParamsID p) { return p->slide1; }
MeReal MEAPI MdtContactParamsGetSecondaryFriction(const MdtContactParamsID p) { return p->friction2; }
MeReal MEAPI MdtContactParamsGetSecondaryFrictionCoeffecient(const MdtContactParamsID p) { return p->frictioncoeff2; }
MeReal MEAPI MdtContactParamsGetSecondarySlip(const MdtContactParamsID p) { return p->slip2; }
MeReal MEAPI MdtContactParamsGetSecondarySlide(const MdtContactParamsID p) { return p->slide2; }

void MEAPI MdtContactParamsSetType(const MdtContactParamsID p, MdtContactType t) { p->type = t; }
void MEAPI MdtContactParamsSetPrimaryFrictionModel(const MdtContactParamsID p, MdtFrictionModel m) { p->model1 = m; }
void MEAPI MdtContactParamsSetSecondaryFrictionModel(const MdtContactParamsID p, MdtFrictionModel m) { p->model2 = m; }

void MEAPI MdtContactParamsSetFrictionModel(const MdtContactParamsID p, MdtFrictionModel m)
{
    p->model1 = p->model2 = m;
}

void MEAPI MdtContactParamsSetRestitution(const MdtContactParamsID p, MeReal r)
{
    p->restitution = r;
    setOption(p, MdtBclContactOptionBounce, r > 0);
}

void MEAPI MdtContactParamsSetRestitutionThreshold(const MdtContactParamsID p, MeReal v) { p->velThreshold = v; }

void MEAPI MdtContactParamsSetSoftness(const MdtContactParamsID p, MeReal s)
{
    p->softness = s;
    setOption(p, MdtBclContactOptionSoft, s > 0);
}

void MEAPI MdtContactParamsSetMaxAdhesiveForce(const MdtContactParamsID p, MeReal s)
{
    p->max_adhesive_force = s;
    setOption(p, MdtBclContactOptionAdhesive, s > 0);
}

void MEAPI MdtContactParamsSetPrimaryFriction(const MdtContactParamsID p, MeReal f) { p->friction1 = f; }
void MEAPI MdtContactParamsSetPrimaryFrictionCoeffecient(const MdtContactParamsID p, MeReal f) { p->frictioncoeff1 = f; }
void MEAPI MdtContactParamsSetSecondaryFriction(const MdtContactParamsID p, MeReal f) { p->friction2 = f; }
void MEAPI MdtContactParamsSetSecondaryFrictionCoeffecient(const MdtContactParamsID p, MeReal f) { p->frictioncoeff2 = f; }

void MEAPI MdtContactParamsSetFriction(const MdtContactParamsID p, MeReal f)
{
    p->friction1 = p->friction2 = f;
}

void MEAPI MdtContactParamsSetFrictionCoeffecient(const MdtContactParamsID p, MeReal f)
{
    p->frictioncoeff1 = p->frictioncoeff2 = f;
}

} // extern "C"
