#ifndef RLSS_COLLISIONSHAPES_ALIGNED_BOX_HPP
#define RLSS_COLLISIONSHAPES_ALIGNED_BOX_HPP

#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <rlss/internal/Util.hpp>

namespace rlss {

template<typename T, unsigned int DIM>
class AlignedBoxCollisionShape : public CollisionShape<T, DIM> {
public:
    using Base = CollisionShape<T, DIM>;
    using VectorDIM = typename Base::VectorDIM;
    using StdVectorVectorDIM = typename Base::StdVectorVectorDIM;
    using AlignedBox = rlss::internal::AlignedBox<T, DIM>;

    AlignedBoxCollisionShape(const AlignedBox& abox) : m_collision_box(abox) {

    }


    StdVectorVectorDIM convexHullPoints(const VectorDIM& com) const override {
        AlignedBox box(
            m_collision_box.min() + com,
            m_collision_box.max() + com
        );
        return rlss::internal::cornerPoints<T, DIM>(box);
    }

    AlignedBox boundingBox(const VectorDIM& com) const override {
        return AlignedBox(
            m_collision_box.min() + com,
            m_collision_box.max() + com
        );
    }

private:

    // collision box of the robot when CoM is at 0.
    AlignedBox m_collision_box;
};
}

#endif