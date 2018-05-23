#ifndef PATHREPLAN_SVMCVXWRAPPER_2_4_H
#define PATHREPLAN_SVMCVXWRAPPER_2_4_H

#include "hyperplane.h"
#include "vectoreuc.h"

hyperplane _2pt4obspt_svm(const vectoreuc& pt1, const vectoreuc& pt2,
                          const vectoreuc& obs1, const vectoreuc& obs2,
                          const vectoreuc& obs3, const vectoreuc& obs4);



#endif
