#include "qp_optimize.h"

Matrix ObjectiveBuilder::Qvel;
Matrix ObjectiveBuilder::Qacc;
Matrix ObjectiveBuilder::Qjerk;
Matrix ObjectiveBuilder::Qsnap;
Matrix ObjectiveBuilder::B;

constexpr double ConstraintBuilder::derivativeBeginning[8][8];
constexpr double ConstraintBuilder::derivativeEnd[8][8];
