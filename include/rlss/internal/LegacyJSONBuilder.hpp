#ifndef RLSS_LEGACY_JSON_BUILDER_HPP
#define RLSS_LEGACY_JSON_BUILDER_HPP

#include "../../../third_party/json.hpp"
#include <rlss/internal/Util.hpp>
#include <fstream>
#include <splx/curve/PiecewiseCurve.hpp>

namespace rlss {
    namespace internal {

        namespace jsonbuilder {
            long long int file_count = 0;
        }


#ifdef ENABLE_RLSS_LEGACY_JSON_BUILDER
        template<typename T, unsigned int DIM>
        class LegacyJSONBuilder {
        public:

            using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
            using Bezier = splx::Bezier<T, DIM>;

            LegacyJSONBuilder() {
                m_frame["step"] = 0;
            }

            void save(const std::string& filename) const {
                std::ofstream file(filename, std::ios_base::out);
                file << m_json;
                file.close();
                debug_message("written ", filename);
            }

            void save() const {
                std::string filename
                        = "json" + std::to_string(jsonbuilder::file_count++) + ".json";
                this->save(filename);
            }

            void setRobotCount(long long int count) {
                m_json["robot_count"] = count;
            }

            void setRobotRadius(T rad) {
                m_json["robot_radius"] = rad;
            }

            void addOriginalTrajectory(
                std::size_t robot_id,
                const PiecewiseCurve& pwisecurve
            ) {
                nlohmann::json otraj_json;
                otraj_json["robot_id"] = robot_id;
                otraj_json["trajectory"] = this->toJSON(pwisecurve);

                if(m_json.contains("original_trajectories")) {
                    auto& otrajs = m_json["original_trajectories"];
                    for(std::size_t i = 0; i < otrajs.size(); i++) {
                        auto& otraj = otrajs[i];
                        if(otraj["robot_id"] == robot_id) {
                            otraj = otraj_json;
                            return;
                        }
                    }
                }

                m_json["original_trajectories"].push_back(otraj_json);
            }

            void setFrameDt(T dt) {
                m_json["frame_dt"] = dt;
            }

            void nextFrame() {
                m_json["frames"].push_back(m_frame);
                long long int step = m_frame["step"];
                m_frame.clear();
                m_frame["step"] = step + 1;
            }

            void addTrajectoryToCurrentFrame(
                std::size_t robot_id,
                const PiecewiseCurve& pwisecurve,
                const StdVectorVectorDIM<T, DIM>& discrete_path
            ) {
                nlohmann::json curves_json = this->toJSON(pwisecurve);
                nlohmann::json path_json = this->toJSON(discrete_path);

                nlohmann::json trajectory_json;
                trajectory_json["robot_id"] = robot_id;
                trajectory_json["trajectory"] = curves_json;
                trajectory_json["discrete_path"] = path_json;

                if(m_frame.contains("trajectories")) {
                    for(auto& traj: m_frame["trajectories"]) {
                        if(traj["robot_id"] == robot_id) {
                            traj = trajectory_json;
                            return;
                        }
                    }
                }
                m_frame["trajectories"].push_back(trajectory_json);
            }



            void addTrajectoryToCurrentFrame(
                    std::size_t robot_id,
                    const PiecewiseCurve& pwisecurve
            ) {
                nlohmann::json curves_json = this->toJSON(pwisecurve);

                nlohmann::json trajectory_json;
                trajectory_json["robot_id"] = robot_id;
                trajectory_json["trajectory"] = curves_json;

                if(m_frame.contains("trajectories")) {
                    for(auto& traj: m_frame["trajectories"]) {
                        if(traj["robot_id"] == robot_id) {
                            traj = trajectory_json;
                            return;
                        }
                    }
                }
                m_frame["trajectories"].push_back(trajectory_json);
            }

            void addOccupancyGridToCurrentFrame(
                const OccupancyGrid<T, DIM>& grid
            ) {
                for(auto it = grid.begin(); it != grid.end(); ++it) {
                    nlohmann::json min_json = this->toJSON((*it).min());
                    nlohmann::json max_json = this->toJSON((*it).max());
                    nlohmann::json cell_json;
                    cell_json["min"] = min_json;
                    cell_json["max"] = max_json;
                    m_frame["occupied_cells"].push_back(cell_json);
                }
            }

            void setRobotPositionInCurrentFrame(
                std::size_t robot_id,
                const VectorDIM<T, DIM>& position
            ) {
                nlohmann::json pos_json;
                pos_json["robot_id"] = robot_id;
                pos_json["position"] = this->toJSON(position);

                if(m_frame.contains("robot_positions")) {
                    for(auto& pos: m_frame["robot_positions"]) {
                        if(pos["robot_id"] == robot_id) {
                            pos = pos_json;
                            return;
                        }
                    }
                }

                m_frame["robot_positions"].push_back(pos_json);
            }

            void addRobotSafetyHyperplanesToCurrentFrame(
                std::size_t robot_id,
                const std::vector<Hyperplane<T, DIM>>& hps
            ) {
                nlohmann::json robot_hps;
                robot_hps["robot_id"] = robot_id;
                for(const auto& hp: hps) {
                    robot_hps["hyperplanes"].push_back(
                        this->toJSON(hp)
                    );
                }

                if(m_frame.contains("voronoi_hyperplanes")) {
                    for(auto& hp_json: m_frame["voronoi_hyperplanes"]) {
                        if(hp_json["robot_id"] == robot_id) {
                            hp_json = robot_hps;
                            return;
                        }
                    }
                }

                m_frame["voronoi_hyperplanes"].push_back(robot_hps);
            }

            void addObstacleSafetyHyperplanesToCurrentFrame(
                std::size_t robot_id,
                std::size_t piece_idx,
                const std::vector<Hyperplane<T, DIM>>& hps
            ) {
                // TODO
            }



        private:
            nlohmann::json m_json;
            nlohmann::json m_frame;
            long long int frame_step;


            nlohmann::json toJSON(const VectorDIM<T, DIM>& vec) const {
                nlohmann::json vec_json;
                for(unsigned int i = 0; i < DIM; i++) {
                    vec_json.push_back(vec(i));
                }
                return vec_json;
            }

            nlohmann::json toJSON(const PiecewiseCurve& pwisecurve) const {
                nlohmann::json pwise_json;
                for(std::size_t i = 0; i < pwisecurve.numPieces(); i++) {
                    nlohmann::json piece_json;
                    piece_json["min_parameter"] = 0;
                    piece_json["max_parameter"] = 1;

                    if(pwisecurve.type(i)
                       == splx::PiecewiseCurve<T, DIM>::CurveType::BEZIER
                            ) {
                        nlohmann::json controlpoints;
                        const Bezier& bez = pwisecurve.getPiece(i);

                        for(std::size_t j = 0; j < bez.numControlPoints(); j++) {
                            nlohmann::json cpt = this->toJSON(bez[j]);
                            controlpoints.push_back(cpt);
                        }
                        piece_json["type"] = "bezier";
                        piece_json["controlpoints"] = controlpoints;
                    } else {
                        throw std::domain_error (
                                absl::StrCat(
                                        "curve type not handled"
                                )
                        );
                    }
                    pwise_json.push_back(piece_json);
                }
                return pwise_json;
            }
            nlohmann::json toJSON(
                const StdVectorVectorDIM<T, DIM>& segments
            ) const {
                nlohmann::json seg_json;
                for(const auto& pt: segments) {
                    seg_json.push_back(this->toJSON(pt));
                }
            }

            nlohmann::json toJSON(
                const Hyperplane<T, DIM>& hp
            ) const {
                nlohmann::json hp_json;
                hp_json["normal"] = this->toJSON(hp.normal());
                hp_json["distance"] = -hp.offset();
                return hp_json;
            }
        };
#else
        template<typename T, unsigned int DIM>
        class LegacyJSONBuilder {
        public:

            using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
            using Bezier = splx::Bezier<T, DIM>;

            LegacyJSONBuilder() {
            }

            void setRobotCount(long long int count) {
            }

            void setRobotRadius(T rad) {
            }

            void addOriginalTrajectory(
                std::size_t robot_id,
                const PiecewiseCurve& pwisecurve
            ) {
            }

            void setFrameDt(T dt) {
            }

            void nextFrame() {
            }

            void addTrajectoryToCurrentFrame(
                std::size_t robot_id,
                const PiecewiseCurve& pwisecurve
            ) {

            }

            void addTrajectoryToCurrentFrame(
                std::size_t robot_id,
                const PiecewiseCurve& pwisecurve,
                const StdVectorVectorDIM<T, DIM>& discrete_path
            ) {
            }

            void addOccupancyGridToCurrentFrame(
                const OccupancyGrid<T, DIM>& grid
            ) {
            }

            void setRobotPositionInCurrentFrame(
                std::size_t robot_id,
                const VectorDIM<T, DIM>& position
            ) {
            }

            void addRobotSafetyHyperplanesToCurrentFrame(
                std::size_t robot_id,
                const std::vector<Hyperplane<T, DIM>>& hps
            ) {
            }

            void addObstacleSafetyHyperplanesToCurrentFrame(
                std::size_t robot_id,
                std::size_t piece_idx,
                const std::vector<Hyperplane<T, DIM>>& hps
            ) {
            }



            void save(const std::string& filename) const {
            }

            void save() const {
            }

        };
#endif
    }
}

#endif // RLSS_LEGACY_JSON_BUILDER_HPP