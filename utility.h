#ifndef PATHREPLAN_UTILITY_H
#define PATHREPLAN_UTILITY_H


double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

#endif
