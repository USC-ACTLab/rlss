#ifndef RLSS_COLLISIONSHAPES_COLLISION_SHAPE_HPP
#define RLSS_COLLISIONSHAPES_COLLISION_SHAPE_HPP

#include<rlss/internal/Util.hpp>

namespace rlss {
template<typename T, unsigned int DIM>
class CollisionShape {
public:
    using VectorDIM = rlss::internal::VectorDIM<T, DIM>;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<T, DIM>;
    using AlignedBox = rlss::internal::AlignedBox<T, DIM>;

    virtual ~CollisionShape() = 0;

    // get the points on the convex hull of the collision shape
    // at given center of mass
    virtual StdVectorVectorDIM convexHullPoints(const VectorDIM& com) const = 0;

    // get the boinding box of the collision shape at center of mass
    virtual AlignedBox boundingBox(const VectorDIM& com) const = 0;
};
}

#endif // RLSS_COLLISIONSHAPES_COLLISION_SHAPE_HPP