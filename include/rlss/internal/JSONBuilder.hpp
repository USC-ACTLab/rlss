#ifndef RLSS_JSON_BUILDER_HPP
#define RLSS_JSON_BUILDER_HPP

#include "../../../third_party/json.hpp"
#include <rlss/internal/Util.hpp>
#include <fstream>
#include <splx/curve/PiecewiseCurve.hpp>

namespace rlss {
    namespace internal {

        namespace jsonbuilder {
            long long int file_count = 0;
        }


#ifdef ENABLE_RLSS_JSON_BUILDER
        template<typename T, unsigned int DIM>
        class JSONBuilder {
        public:

            using AlignedBox = internal::AlignedBox<T, DIM>;
            using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
            using Bezier = splx::Bezier<T, DIM>;
            using VectorDIM = internal::VectorDIM<T, DIM>;

            JSONBuilder() {
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
                m_json["robot_shapes"] = std::vector<nlohmann::json>(count);
                m_json["original_trajectories"] = std::vector<nlohmann::json>(count);
            }

            void setRobotShape(std::size_t robot_idx, const AlignedBox& box) {
                nlohmann::json item;
                item["robot_id"] = robot_idx;
                item["shape"] = this->toJSON(box);
                m_json["robot_shapes"][robot_idx] = item;
            }

            void setRobotShape(std::size_t robot_idx, const VectorDIM& center, T radius) {
                nlohmann::json item;
                item["robot_id"] = robot_idx;
                item["shape"] = this->toJSON(center, radius);
                m_json["robot_shapes"][robot_idx] = item;
            }

            void setOriginalTrajectory(
                std::size_t robot_idx,
                const PiecewiseCurve& pwisecurve
            ) {
                nlohmann::json otraj_json;
                otraj_json["robot_id"] = robot_idx;
                otraj_json["trajectory"] = this->toJSON(pwisecurve);

                m_json["original_trajectories"][robot_idx] = otraj_json;
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
                    nlohmann::json aabb_json = this->toJSON((*it));
                    m_frame["obstacles"].push_back(aabb_json);
                }
            }

            void setRobotPositionInCurrentFrame(
                std::size_t robot_id,
                const VectorDIM& position
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

            void addHyperplanesToCurrentFrame(
                std::size_t robot_id,
                std::size_t piece_idx,
                const std::vector<Hyperplane<T, DIM>>& hps
            ) {
                nlohmann::json robot_hps;
                robot_hps["robot_id"] = robot_id;
                for(const auto& hp: hps) {
                    nlohmann::json hp_json;
                    hp_json["piece_id"] = this->toJSON(piece_idx);
                    hp_json["hyperplane"] = this->toJSON(hp);

                    robot_hps["hyperplanes"].push_back(
                        hp_json
                    );
                }

                for(auto& hp_json: m_frame["hyperplanes"]) {
                    if(hp_json["robot_id"] == robot_id) {
                        hp_json = robot_hps;
                        return;
                    }
                }

                m_frame["hyperplanes"].push_back(robot_hps);
            }

        private:
            nlohmann::json m_json;
            nlohmann::json m_frame;


            nlohmann::json toJSON(const VectorDIM& vec) const {
                nlohmann::json vec_json;
                for(unsigned int i = 0; i < DIM; i++) {
                    vec_json.push_back(vec(i));
                }
                return vec_json;
            }

            nlohmann::json toJSON(const AlignedBox& box) const {
                nlohmann::json aabb_json;

                aabb_json["min"] = this->toJSON(box.min());
                aabb_json["max"] = this->toJSON(box.max());

                return aabb_json;
            }

            nlohmann::json toJSON(const VectorDIM& vec, T radius) const {
                nlohmann::json sphere_json;

                sphere_json["center"] = this->toJSON(vec);
                sphere_json["radius"] = radius;

                return sphere_json;
            }

            nlohmann::json toJSON(const Bezier& bez) const {
                nlohmann::json bez_json;
                bez_json["max_parameter"] = 1;
                bez_json["type"] = "bezier";

                for(std::size_t j = 0; j < bez.numControlPoints(); j++) {
                    bez_json["controlpoints"].push_back(this->toJSON(bez[j]));
                }

                return bez_json;
            }

            nlohmann::json toJSON(const PiecewiseCurve& pwisecurve) const {
                nlohmann::json pwise_json;
                for(std::size_t i = 0; i < pwisecurve.numPieces(); i++) {
                    const Bezier& bez = pwisecurve[i];
                    pwise_json.push_back(this->toJSON(bez));
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
        class JSONBuilder {
        public:

            using AlignedBox = AlignedBox<T, DIM>;
            using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
            using Bezier = splx::Bezier<T, DIM>;
            using VectorDIM = VectorDIM<T, DIM>;

            JSONBuilder() {
            }

            void save(const std::string& filename) const {
            }

            void save() const {
            }

            void setRobotCount(long long int count) {
            }

            void setRobotShape(std::size_t robot_idx, const AlignedBox& box) {
            }

            void setRobotShape(std::size_t robot_idx, const VectorDIM& center, T radius) {
            }

            void setOriginalTrajectory(
                    std::size_t robot_idx,
                    const PiecewiseCurve& pwisecurve
            ) {
            }

            void setFrameDt(T dt) {
            }

            void nextFrame() {
            }

            void addTrajectoryToCurrentFrame(
                    std::size_t robot_id,
                    const PiecewiseCurve& pwisecurve,
                    const StdVectorVectorDIM<T, DIM>& discrete_path
            ) {
            }



            void addTrajectoryToCurrentFrame(
                    std::size_t robot_id,
                    const PiecewiseCurve& pwisecurve
            ) {
            }

            void addOccupancyGridToCurrentFrame(
                    const OccupancyGrid<T, DIM>& grid
            ) {
            }

            void setRobotPositionInCurrentFrame(
                    std::size_t robot_id,
                    const VectorDIM& position
            ) {
            }

            void addHyperplanesToCurrentFrame(
                    std::size_t robot_id,
                    std::size_t piece_idx,
                    const std::vector<Hyperplane<T, DIM>>& hps
            ) {
            }

        private:
            nlohmann::json toJSON(const VectorDIM& vec) const {
            }

            nlohmann::json toJSON(const AlignedBox& box) const {
            }

            nlohmann::json toJSON(const VectorDIM& vec, T radius) const {
            }

            nlohmann::json toJSON(const Bezier& bez) const {
            }

            nlohmann::json toJSON(const PiecewiseCurve& pwisecurve) const {
            }

            nlohmann::json toJSON(
                    const StdVectorVectorDIM<T, DIM>& segments
            ) const {
            }

            nlohmann::json toJSON(
                    const Hyperplane<T, DIM>& hp
            ) const {
            }
        };
#endif
    }
}

#endif // RLSS_LEGACY_JSON_BUILDER_HPP