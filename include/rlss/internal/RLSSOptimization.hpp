#ifndef RLSS_INTERNAL_RLSS_OPTIMIZATION_HPP
#define RLSS_INTERNAL_RLSS_OPTIMIZATION_HPP

#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <rlss/internal/Util.hpp>
#include <rlss/internal/SVM.hpp>
#include <rlss/internal/MathematicaWriter.hpp>

namespace rlss {
namespace internal {

template<typename T, unsigned int DIM>
std::vector<Hyperplane<T, DIM>> robot_safety_hyperplanes(
        const VectorDIM<T, DIM>& robot_position,
        const std::vector<AlignedBox<T, DIM>>&
        other_robot_collision_shape_bounding_boxes,
        std::shared_ptr<CollisionShape<T, DIM>> colshape) {

    using Hyperplane = internal::Hyperplane<T, DIM>;
    using AlignedBox = internal::AlignedBox<T, DIM>;
    using StdVectorVectorDIM = internal::StdVectorVectorDIM<T, DIM>;

    std::vector<Hyperplane> hyperplanes;

    AlignedBox robot_box
            = colshape->boundingBox(robot_position);

    StdVectorVectorDIM robot_points
            = rlss::internal::cornerPoints<T, DIM>(robot_box);

    for(const auto& oth_collision_shape_bbox:
            other_robot_collision_shape_bounding_boxes) {
        StdVectorVectorDIM oth_points
                = rlss::internal::cornerPoints<T, DIM>(oth_collision_shape_bbox);


        Hyperplane svm_hp = rlss::internal::svm<T, DIM>(robot_points, oth_points);


        Hyperplane svm_shifted = rlss::internal::shiftHyperplane<T, DIM>(
                robot_position,
                robot_box,
                svm_hp
        );

        hyperplanes.push_back(svm_shifted);
    }

    return hyperplanes;
}

template<typename T, unsigned int DIM>
void generate_optimization_problem(
    splx::PiecewiseCurveQPGenerator<T, DIM>& qpgen,
    std::shared_ptr<CollisionShape<T, DIM>> colshape,
    const AlignedBox<T, DIM>& wss,
    unsigned int contupto,
    const std::vector<std::pair<unsigned int, T>>& lambdas,
    const std::vector<T>& thetas,
    T obstacle_check_distance,
    const StdVectorVectorDIM<T, DIM>& segments,
    const std::vector<T>& durations,
    const std::vector<AlignedBox<T, DIM>>& oth_rbt_col_shape_bboxes,
    const OccupancyGrid<T, DIM>& occupancy_grid,
    const StdVectorVectorDIM<T, DIM>& current_robot_state,
    MathematicaWriter<T, DIM>& mathematica,
    const std::unordered_map<std::string, std::pair<bool, T>>& soft_parameters
            = std::unordered_map<std::string, std::pair<bool, T>>()
) {
    using VectorDIM = internal::VectorDIM<T, DIM>;
    using AlignedBox = internal::AlignedBox<T, DIM>;
    using Hyperplane = internal::Hyperplane<T, DIM>;
    using StdVectorVectorDIM = internal::StdVectorVectorDIM<T, DIM>;

    assert(segments.size() == qpgen.numPieces() + 1);
    assert(durations.size() == segments.size() - 1);
    assert(current_robot_state.size() > contupto);


    for(const auto& bbox: oth_rbt_col_shape_bboxes) {
        debug_message(
        "other robot collision shape bounding box is [min: ",
        bbox.min().transpose(),
        ", max: ",
        bbox.max().transpose(),
        "]"
        );
        mathematica.otherRobotCollisionBox(bbox);
    }

    qpgen.resetProblem();
    qpgen.setPieceMaxParameters(durations);

    mathematica.discretePath(segments);

    // workspace constraint
    AlignedBox ws = rlss::internal::bufferAlignedBox<T, DIM>(
            VectorDIM::Zero(),
            colshape->boundingBox(VectorDIM::Zero()),
            wss
    );
    qpgen.addBoundingBoxConstraint(ws);

    debug_message(
        "buffered workspace is [min: ",
        ws.min().transpose(),
        ", max: ",
        ws.max().transpose(),
        "]"
    );


    mathematica.selfCollisionBox(
            colshape->boundingBox(current_robot_state[0]));

    // robot to robot avoidance constraints for the first piece
    std::vector<Hyperplane> robot_to_robot_hps
            = robot_safety_hyperplanes<T, DIM>(
                    current_robot_state[0],
                    oth_rbt_col_shape_bboxes,
                    colshape
            );

//    robot_to_robot_hps = internal::pruneHyperplanes<T, DIM>(
//            robot_to_robot_hps, ws);

    for(const auto& hp: robot_to_robot_hps) {
        bool r2r_hyperplane_constraints_soft_enabled
            = soft_parameters.find("robot_to_robot_hyperplane_constraints")
                    != soft_parameters.end()
              && soft_parameters.at("robot_to_robot_hyperplane_constraints").first;
        T r2r_hyperplane_constraints_soft_weight =
                r2r_hyperplane_constraints_soft_enabled
                ? soft_parameters.at("robot_to_robot_hyperplane_constraints").second
                : 0;
        qpgen.addHyperplaneConstraintForPiece(
                0,
                hp,
                r2r_hyperplane_constraints_soft_enabled,
                r2r_hyperplane_constraints_soft_weight
        );
        if(hp.signedDistance(current_robot_state[0]) > 0) {
            debug_message(
                "distance of current point to a first piece hyperplane: ",
                hp.signedDistance(current_robot_state[0])
            );
        }
    }


    assert(robot_to_robot_hps.size() == oth_rbt_col_shape_bboxes.size());

    for(const auto& hp: robot_to_robot_hps) {
        mathematica.robotCollisionAvoidanceHyperplane(hp);
        debug_message(
            "robot to robot collision avoidance hyperplane [n: ",
            hp.normal().transpose(),
            ", d: ",
            hp.offset(),
            "]"
        );
    }


    // robot to obstacle avoidance constraints for all pieces
    for (
        std::size_t p_idx = 0;
        p_idx < qpgen.numPieces();
        p_idx++
    ) {
        AlignedBox from_box
                = colshape->boundingBox(segments[p_idx]);
        AlignedBox to_box
                = colshape->boundingBox(segments[p_idx+1]);
        to_box.extend(from_box);

        StdVectorVectorDIM segments_corners
                = rlss::internal::cornerPoints<T, DIM>(to_box);

        std::vector<Hyperplane> piece_obstacle_hyperplanes;

        for(
            auto it = occupancy_grid.begin(to_box, obstacle_check_distance);
            it != occupancy_grid.end(to_box, obstacle_check_distance);
            ++it
        ) {

            AlignedBox grid_box = *it;

            StdVectorVectorDIM grid_box_corners
                    = rlss::internal::cornerPoints<T, DIM>(grid_box);

            Hyperplane shp = rlss::internal::svm<T, DIM>
                    (
                            segments_corners,
                            grid_box_corners
                    );


            shp = rlss::internal::shiftHyperplane<T, DIM>(
                    VectorDIM::Zero(),
                    colshape->boundingBox(VectorDIM::Zero()),
                    shp
            );


            mathematica.obstacleCollisionBox(grid_box);

            piece_obstacle_hyperplanes.push_back(shp);
        }

        debug_message("before prune num hyperplanes: "
                , piece_obstacle_hyperplanes.size());
//        piece_obstacle_hyperplanes
//            = internal::pruneHyperplanes<T, DIM>(
//                                piece_obstacle_hyperplanes,
//                                ws
//         );
        debug_message("after prune num hyperplanes: "
                , piece_obstacle_hyperplanes.size());

        for(const auto& shp: piece_obstacle_hyperplanes) {
            bool r2o_hyperplane_constraints_soft_enabled
                = soft_parameters.find(
                        "robot_to_obstacle_hyperplane_constraints"
                  )
                  != soft_parameters.end()
                  && soft_parameters.at("robot_to_obstacle_hyperplane_constraints")
                                        .first;

            T r2o_hyperplane_constraints_soft_weight =
                r2o_hyperplane_constraints_soft_enabled
                ? soft_parameters.at("robot_to_obstacle_hyperplane_constraints")
                            .second
                : 0;
            qpgen.addHyperplaneConstraintForPiece(
                    p_idx,
                    shp,
                    r2o_hyperplane_constraints_soft_enabled,
                    r2o_hyperplane_constraints_soft_weight
            );
        }
    }


    // continuity constraints
    for(
        std::size_t p_idx = 0;
        p_idx < qpgen.numPieces() - 1;
        p_idx++
    ) {
        for(unsigned int k = 0; k <= contupto; k++) {
            debug_message(
                "adding continuity constraint between piece ",
                p_idx,
                " and ",
                p_idx+1,
                " for degree ",
                k
            );
            bool continuity_constraints_soft_enabled
                = soft_parameters.find(
                        "continuity_constraints"
                )
                  != soft_parameters.end()
                  && soft_parameters.at("continuity_constraints")
                          .first;

            T continuity_constraints_soft_weight =
                continuity_constraints_soft_enabled
                ? soft_parameters.at("continuity_constraints")
                        .second
                : 0;
            qpgen.addContinuityConstraint(
                p_idx,
                k,
                continuity_constraints_soft_enabled,
                continuity_constraints_soft_weight
            );
        }
    }

    // initial point constraints
    for(unsigned int k = 0; k <= contupto; k++) {
        debug_message(
            "adding initial point constraint for degree ",
            k,
            ". It should be ",
            current_robot_state[k].transpose()
        );
        bool initial_point_constraints_soft_enabled
                = soft_parameters.find(
                        "initial_point_constraints"
                )
                  != soft_parameters.end()
                  && soft_parameters.at("initial_point_constraints")
                          .first;

        T initial_point_constraints_soft_weight =
                initial_point_constraints_soft_enabled
                ? soft_parameters.at("initial_point_constraints")
                        .second
                : 0;
        qpgen.addEvalConstraint(
                0,
                k,
                current_robot_state[k],
                initial_point_constraints_soft_enabled,
                initial_point_constraints_soft_weight
        );
    }



    // energy cost
    for(const auto& [d, l]: lambdas) {
        debug_message("adding integrated squared derivative cost for",
            "degree ", d, " with lambda ", l
        );
        qpgen.addIntegratedSquaredDerivativeCost(d, l);
    }



    // eval cost
    T duration_sum_before = 0;
    for(
        std::size_t p_idx = 0;
        p_idx < qpgen.numPieces();
        duration_sum_before += durations[p_idx], p_idx++
    ) {
        qpgen.addEvalCost(
                std::min(
                    duration_sum_before + durations[p_idx],
                    qpgen.maxParameter()
                ),
                0,
                segments[p_idx+1],
                thetas[p_idx]
        );
    }


}

} // namespace internal
} // namespace rlss

#endif // RLSS_INTERNAL_RLSS_OPTIMIZATION_HPP