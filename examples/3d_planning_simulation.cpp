#include <splx/curve/PiecewiseCurve.hpp>
#include <boost/filesystem/operations.hpp>
#include <ios>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/RLSS.hpp>
#include <iostream>
#include <cxxopts.hpp>
#include <json.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <splx/curve/Bezier.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <memory>


namespace fs = boost::filesystem;
using RLSS = rlss::RLSS<double, 3U>;
using OccupancyGrid = rlss::OccupancyGrid<double, 3U>;
using AlignedBoxCollisionShape = rlss::AlignedBoxCollisionShape<double, 3U>;
using CollisionShape = rlss::CollisionShape<double, 3U>;
using StdVectorVectorDIM = OccupancyGrid::StdVectorVectorDIM;
using VectorDIM = OccupancyGrid::VectorDIM;
using PiecewiseCurveQPGenerator
= splx::PiecewiseCurveQPGenerator<double, 3U>;
using PiecewiseCurve = splx::PiecewiseCurve<double, 3U>;
using Bezier = splx::Bezier<double, 3U>;
using AlignedBox = OccupancyGrid::AlignedBox;

bool allRobotsReachedFinalStates(
        const std::vector<RLSS>& planners,
        const std::vector<StdVectorVectorDIM>& states
) {
    constexpr double required_distance = 0.1;
    for(std::size_t i = 0; i < planners.size(); i++) {
        if((planners[i].goalPosition() - states[i][0]).squaredNorm()
                > required_distance * required_distance) {
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[]) {


    cxxopts::Options options("RLSS 3D", "");
    options.add_options()
            (
                "c,config",
                "Config file path",
                cxxopts::value<std::string>()
                    ->default_value("../examples/3d_config.json")
            )
            (
                "h, help",
                "help"
            )
    ;

    auto parsed_options = options.parse(argc, argv);

    if(parsed_options.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    std::fstream json_fs(
        parsed_options["config"].as<std::string>(), std::ios_base::in);
    nlohmann::json config_json = nlohmann::json::parse(json_fs);
    json_fs.close();

    std::string base_path = config_json["base_path"];
    std::string obstacle_directory = config_json["obstacle_directory"];
    std::string case_description_directory
        = config_json["case_description_directory"];
    double replanning_period = config_json["replanning_period"];
    std::vector<double> occupancy_grid_step_size
        = config_json["occupancy_grid_step_size"];


    OccupancyGrid::Coordinate step_size(
        occupancy_grid_step_size[0],
        occupancy_grid_step_size[1],
        occupancy_grid_step_size[2]
    );
    OccupancyGrid occupancy_grid(step_size);

    for(auto& p: fs::directory_iterator(base_path + obstacle_directory)) {
        std::fstream obstacle_file(p.path().string(), std::ios_base::in);
        StdVectorVectorDIM obstacle_pts;
        VectorDIM vec;
        while(obstacle_file >> vec(0)) {
            obstacle_file >> vec(1) >> vec(2);
            obstacle_pts.push_back(vec);
        }
        occupancy_grid.addObstacle(obstacle_pts);
    }


    std::vector<RLSS> planners;
    for(auto& p:
        fs::directory_iterator(base_path + case_description_directory)
    ) {
        std::fstream robot_description(p.path().string(), std::ios_base::in);
        nlohmann::json robot_json = nlohmann::json::parse(robot_description);

        PiecewiseCurve original_trajectory;
        for(nlohmann::json piece_json
                : robot_json["original_trajectory"]["pieces"]) {
            if(piece_json["type"] == "BEZIER") {
                Bezier piece(piece_json["duration"]);
                for(const std::vector<double>& cpt
                        : piece_json["control_points"]) {
                    assert(cpt.size() == 3);
                    piece.appendControlPoint({ cpt[0], cpt[1], cpt[2] });
                }
                original_trajectory.addPiece(piece);
            }
        }

        PiecewiseCurveQPGenerator qp_generator;
        nlohmann::json pieces_json;
        if(config_json.contains("plan_for_trajectory")) {
            // override robot json
            pieces_json = config_json["plan_for_trajectory"]["pieces"];
        } else {
            pieces_json = robot_json["plan_for_trajectory"]["pieces"];
        }


        for(const nlohmann::json& plan_for_piece: pieces_json) {
            if(plan_for_piece["type"] == "BEZIER") {
                qp_generator.addBezier(plan_for_piece["num_control_points"], 0);
            }
        }

        double search_step = robot_json["search_step"];
        double rescaling_multiplier = robot_json["rescaling_multiplier"];
        unsigned int max_rescaling_count = robot_json["max_rescaling_count"];
        double desired_time_horizon = robot_json["desired_time_horizon"];
        unsigned int continuity_upto_degree
            = robot_json["continuity_upto_degree"];
        std::vector<std::pair<unsigned int, double>>
                maximum_derivative_magnitudes;
        for(const std::pair<unsigned int, double>& der_mag
                : robot_json["maximum_derivative_magnitudes"]) {
            maximum_derivative_magnitudes.push_back(der_mag);
        }
        std::vector<std::pair<unsigned int, double>>
                integrated_squared_derivative_weights;
        for(const std::pair<unsigned int, double>& deg_weight
                : robot_json["integrated_squared_derivative_weights"]) {
            integrated_squared_derivative_weights.push_back(deg_weight);
        }
        std::vector<double> piece_endpoint_cost_weights;
        for(const double weight: robot_json["piece_endpoint_cost_weights"]) {
            piece_endpoint_cost_weights.push_back(weight);
        }

        VectorDIM colshapemin(
            robot_json["collision_shape_at_zero"][0][0],
            robot_json["collision_shape_at_zero"][0][1],
            robot_json["collision_shape_at_zero"][0][2]
        );
        VectorDIM colshapemax(
            robot_json["collision_shape_at_zero"][1][0],
            robot_json["collision_shape_at_zero"][1][1],
            robot_json["collision_shape_at_zero"][1][2]
        );
        auto albox_col_shape = std::make_shared<AlignedBoxCollisionShape>(
                AlignedBox(colshapemin, colshapemax));
        auto collision_shape = std::static_pointer_cast<CollisionShape>(
                albox_col_shape);

        VectorDIM workspacemin(
            robot_json["workspace"][0][0],
            robot_json["workspace"][0][1],
            robot_json["workspace"][0][2]
        );
        VectorDIM workspacemax(
            robot_json["workspace"][1][0],
            robot_json["workspace"][1][1],
            robot_json["workspace"][1][2]
        );

        AlignedBox workspace(workspacemin, workspacemax);

        // const PiecewiseCurve& orig_traj,
        // const PiecewiseCurveQPGenerator& generator,
        // std::shared_ptr<CollisionShape> col_shape,
        // const AlignedBox& workspace,
        // T planning_horizon,
        // T safe_upto,
        // T search_step,
        // unsigned int continuity_upto,
        // T rescaling_multiplier,
        // unsigned int max_rescaling_count,
        // const std::vector<std::pair<unsigned int, T>>& lambda_integrated,
        // const std::vector<T> theta_pos_at,
        // const std::vector<std::pair<unsigned int, T>>& max_d_mags

        planners.push_back({
            original_trajectory,
            qp_generator,
            collision_shape,
            workspace,
            desired_time_horizon,
            replanning_period,
            search_step,
            continuity_upto_degree,
            rescaling_multiplier,
            max_rescaling_count,
            integrated_squared_derivative_weights,
            piece_endpoint_cost_weights,
            maximum_derivative_magnitudes
        });
    }

    unsigned int num_robots = planners.size();
    std::vector<StdVectorVectorDIM> states(num_robots);
    for(unsigned int i = 0; i < num_robots; i++) {
        for(unsigned int j = 0; j <= planners[i].continuityUpto(); j++) {
            states[i].push_back(planners[i].originalTrajectory().eval(0, j));
        }
    }

    double current_time = 0;
    std::vector<PiecewiseCurve> trajectories(num_robots);
    std::vector<double> trajectory_current_times(num_robots, 0);

    while(!allRobotsReachedFinalStates(planners, states)) {
        rlss::debug_message("starting the loop for time ", current_time, "...");

        rlss::debug_message("collecting robot shapes...");
        std::vector<AlignedBox> robot_collision_boxes(num_robots);
        for(std::size_t i = 0; i < num_robots; i++) {
            robot_collision_boxes[i]
                = planners[i].collisionShape()->boundingBox(states[i][0]);
        }

        for(std::size_t i = 0; i < planners.size(); i++) {
            rlss::debug_message("planning for robot ", i, "...");
            std::vector<AlignedBox> other_robot_collision_boxes
                    = robot_collision_boxes;
            other_robot_collision_boxes.erase(
                    other_robot_collision_boxes.begin() + i);

            std::optional<PiecewiseCurve> curve
                    = planners[i].plan(
                            current_time,
                            states[i],
                            other_robot_collision_boxes,
                            occupancy_grid
            );

            if(curve) {
                rlss::debug_message(
                    rlss::internal::debug::colors::GREEN,
                    "planning successful.",
                    rlss::internal::debug::colors::RESET
                );
                trajectories[i] = *curve;
                trajectory_current_times[i] = 0;
            } else {
                rlss::debug_message(
                    rlss::internal::debug::colors::RED,
                    "planning failed.",
                    rlss::internal::debug::colors::RESET
                );
                trajectory_current_times[i] += replanning_period;
            }

            for(std::size_t j = 0; j < states[i].size(); j++) {
                states[i][j] = trajectories[i].eval(
                        std::min(
                                trajectory_current_times[i] + replanning_period,
                                trajectories[i].maxParameter()
                        ),
                        j
                );
            }
        }

        current_time += replanning_period;
    }


    return 0;
}