#ifndef PATHREPLAN_SVMCVXWRAPPER_16_4_H
#define PATHREPLAN_SVMCVXWRAPPER_16_4_H

#include "hyperplane.h"
#include "vectoreuc.h"


hyperplane _16pt4obspt_svm(const vectoreuc& pt1, const vectoreuc& pt2,
                          const vectoreuc& pt3, const vectoreuc& pt4,
                          const vectoreuc& pt5, const vectoreuc& pt6,
                          const vectoreuc& pt7, const vectoreuc& pt8,
                          const vectoreuc& pt9, const vectoreuc& pt10,
                          const vectoreuc& pt11, const vectoreuc& pt12,
                          const vectoreuc& pt13, const vectoreuc& pt14,
                          const vectoreuc& pt15, const vectoreuc& pt16,
                          const vectoreuc& obs1, const vectoreuc& obs2,
                          const vectoreuc& obs3, const vectoreuc& obs4);

#endif
