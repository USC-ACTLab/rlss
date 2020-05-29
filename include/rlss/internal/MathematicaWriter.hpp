#ifndef RLSS_MATHEMATICA_WRITER_HPP
#define RLSS_MATHEMATICA_WRITER_HPP

#include <fstream>
#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/curve/Bezier.hpp>
#include <absl/strings/str_cat.h>
#include <iomanip>

namespace rlss {

    namespace internal {

        namespace mathematica {
            long long int file_count = 0;
        }

        template<typename T, unsigned int DIM>
        class MathematicaWriter {
        public:

            using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
            using Bezier = splx::Bezier<T, DIM>;

            MathematicaWriter(){}
            MathematicaWriter(const std::string filename) {}
            const std::string& fileName() const {}
            void save() {}
            void robotCollisionAvoidanceHyperplane(
                    const Hyperplane<T, DIM>& hp) {}
            void obstacleCollisionAvoidanceHyperplane(
                    const Hyperplane<T, DIM>& hp) {}
            void selfCollisionBox(const AlignedBox<T, DIM>& box) {}
            void otherRobotCollisionBox(const AlignedBox<T, DIM>& box) {}
            void obstacleCollisionBox(const AlignedBox<T, DIM>& box) {}
            void discretePath(const StdVectorVectorDIM<T, DIM>& segments) {}
            void bezier(const Bezier& bez) {}
            void piecewiseCurve(const PiecewiseCurve& pwisecurve) {}
        };
#ifdef ENABLE_RLSS_MATHEMATICA_OUTPUT
        template<typename T>
        class MathematicaWriter<T, 3U> {
        public:

            using PiecewiseCurve = splx::PiecewiseCurve<T, 3U>;
            using Bezier = splx::Bezier<T, 3U>;


            MathematicaWriter(const std::string fname)
                    : filename(fname),
                      file(filename, std::ios_base::out),
                      rcah_count(0),
                      ocah_count(0),
                      orcb_count(0),
                      ocb_count(0),
                      dp_count(0),
                      bez_count(0)
            {
                file << std::fixed
                     << std::setprecision(std::numeric_limits<T>::max_digits10)
                     << std::showpoint;
            }

            MathematicaWriter() :
                filename("mathematica"
                         + std::to_string(mathematica::file_count)
                         + ".commands"),
                file(filename, std::ios_base::out),
                rcah_count(0),
                ocah_count(0),
                orcb_count(0),
                ocb_count(0),
                dp_count(0),
                bez_count(0)
            {
                file << std::fixed
                     << std::setprecision(std::numeric_limits<T>::max_digits10)
                     << std::showpoint;
                ++mathematica::file_count;
            }

            ~MathematicaWriter() {
                if(file.is_open())
                    save();
            }

            const std::string& fileName() const {
                return filename;
            }

            void save() {
                this->flushShowList();
                debug_message("written ", filename);
                file.close();
            }

            void robotCollisionAvoidanceHyperplane(const Hyperplane<T, 3U>& hp) {
                std::string prefix = "rcah" + std::to_string(rcah_count++);
                std::string normal = prefix + "normal";
                std::string arrowstart = prefix + "arrowstart";
                std::string arrowend = prefix + "arrowend";
                std::string arrow = prefix + "arrow";
                std::string hyperplane = prefix + "hp";
                std::string graphics = prefix + "graphics";

                file << normal << " = {" << hp.normal()(0) << ", "
                     << hp.normal()(1) << ", " << hp.normal()(2)
                     << "};" << std::endl;

                file << arrowstart << " = (" << -hp.offset()
                     << "/Dot[" << normal <<", " << normal <<"]) * "
                     << normal << ";" << std::endl;

                file << arrowend << " = " << arrowstart << "+ {";
                VectorDIM<T, 3U> normal_normalized = hp.normal();
                normal_normalized.normalize();
                file << normal_normalized(0) << ", "
                     << normal_normalized(1) << ", "
                     << normal_normalized(2) << "};" << std::endl;


                file << arrow << " = Arrow[{"
                     << arrowstart << ", " << arrowend << "}];" << std::endl;

                file << hyperplane << " = Hyperplane[" << normal << ", "
                     << -hp.offset() << "];" << std::endl;

                file << graphics << " = Graphics3D[{{Red, Opacity[0.25], "
                     << hyperplane << "}}];" << std::endl;

                add_to_show_list.push_back(graphics);
            }

            void obstacleCollisionAvoidanceHyperplane(const Hyperplane<T, 3U>& hp) {
                std::string prefix = "ocah" + std::to_string(ocah_count++);
                std::string normal = prefix + "normal";
                std::string arrowstart = prefix + "arrowstart";
                std::string arrowend = prefix + "arrowend";
                std::string arrow = prefix + "arrow";
                std::string hyperplane = prefix + "hp";
                std::string graphics = prefix + "graphics";

                file << normal << " = {" << hp.normal()(0) << ", "
                     << hp.normal()(1) << ", " << hp.normal()(2)
                     << "};" << std::endl;

                file << arrowstart << " = (" << -hp.offset()
                     << "/Dot[" << normal <<", " << normal <<"]) * "
                     << normal << ";" << std::endl;

                file << arrowend << " = " << arrowstart << "+ {";
                VectorDIM<T, 3U> normal_normalized = hp.normal();
                normal_normalized.normalize();
                file << normal_normalized(0) << ", "
                     << normal_normalized(1) << ", "
                     << normal_normalized(2) << "};" << std::endl;


                file << arrow << " = Arrow[{"
                     << arrowstart << ", " << arrowend << "}];" << std::endl;

                file << hyperplane << " = Hyperplane[" << normal << ", "
                     << -hp.offset() << "];" << std::endl;

                file << graphics << " = Graphics3D[{{Cyan, Opacity[0.25], "
                     << hyperplane << "}}];" << std::endl;

                add_to_show_list.push_back(graphics);
            }

            void selfCollisionBox(const AlignedBox<T, 3U>& box) {
                std::string prefix = "selfbox";
                std::string cuboid = prefix + "cuboid";
                std::string graphics = prefix + "graphics";

                const auto& min = box.min();
                const auto& max = box.max();

                file << cuboid << " = Cuboid[{" << min(0)
                     << ", " << min(1) << ", " << min(2)
                     << "}, {" << max(0) << ", " << max(1)
                     << ", " << max(2) << "}];" << std::endl;

                file << graphics << " = Graphics3D[{{Blue, Opacity[0.5],"
                     << cuboid << "}}];" << std::endl;

                add_to_show_list.push_back(graphics);
            }

            void otherRobotCollisionBox(const AlignedBox<T, 3U>& box) {
                std::string prefix = "orcb" + std::to_string(orcb_count++);
                std::string cuboid = prefix + "cuboid";
                std::string graphics = prefix + "graphics";

                const auto& min = box.min();
                const auto& max = box.max();

                file << cuboid << " = Cuboid[{" << min(0)
                     << ", " << min(1) << ", " << min(2)
                     << "}, {" << max(0) << ", " << max(1)
                     << ", " << max(2) << "}];" << std::endl;

                file << graphics << " = Graphics3D[{{Red, Opacity[0.5],"
                     << cuboid << "}}];" << std::endl;

                add_to_show_list.push_back(graphics);
            }

            void obstacleCollisionBox(const AlignedBox<T, 3U>& box) {
                std::string prefix = "ocb" + std::to_string(ocb_count++);
                std::string cuboid = prefix + "cuboid";
                std::string graphics = prefix + "graphics";

                const auto& min = box.min();
                const auto& max = box.max();

                file << cuboid << " = Cuboid[{" << min(0)
                     << ", " << min(1) << ", " << min(2)
                     << "}, {" << max(0) << ", " << max(1)
                     << ", " << max(2) << "}];" << std::endl;

                file << graphics << " = Graphics3D[{{Pink, Opacity[0.5],"
                     << cuboid << "}}];" << std::endl;

                add_to_show_list.push_back(graphics);
            }

            void discretePath(const StdVectorVectorDIM<T, 3U>& segments) {
                std::string prefix = "dp" + std::to_string(dp_count++);
                std::string line = prefix + "line";
                std::string graphics = prefix + "graphics";

                file << line << " = Line[{";
                for(std::size_t i = 0; i < segments.size(); i++) {
                    file << "{" << segments[i][0] << ", "
                                << segments[i][1] << ", "
                                << segments[i][2] << "}";
                    if(i != segments.size() - 1)
                        file << ",";
                }
                file << "}];" << std::endl;

                file << graphics << " = Graphics3D[{{Green, Thickness[0.0075],"
                     << line << "}}];" << std::endl;

                add_to_show_list.push_back(graphics);
            }

            void bezier(const Bezier& bez) {
                std::string prefix = "bez" + std::to_string(bez_count++);
                std::string points = prefix + "points";
                std::string function = prefix + "function";
                std::string graphics = prefix + "graphics";

                file << points << " = {";
                for(unsigned int i = 0; i < bez.numControlPoints(); i++) {
                    file << "{" << bez[i](0) << ", "
                                << bez[i](1) << ", "
                                << bez[i](2) << "}";
                    if(i != bez.numControlPoints() - 1) {
                        file << ", ";
                    }
                }
                file << "};" << std::endl;

                file << function << "[t_] := Sum[Binomial[Length["
                                  + points + "] - 1, i ] * "
                                  + points + "[[i + 1]]*t^i*(1 - t)^(Length["
                                  + points + "] - 1 - i), {i, 0, Length["
                                  + points + "] - 1}];" << std::endl;


                file << graphics << " = ParametricPlot3D["
                     << function << "[t], {t, 0, 1}, PlotStyle -> "
                     << "{Thickness[0.01], Opacity[0.5]}];" << std::endl;

                add_to_show_list.push_back(graphics);
            }

            void piecewiseCurve(const PiecewiseCurve& pwisecurve) {
                for(std::size_t i = 0; i < pwisecurve.numPieces(); i++) {
                    if(pwisecurve.type(i)
                        == splx::ParametricCurve<T,3U>::CurveType::BEZIER) {
                        bezier(pwisecurve.getPiece(i));
                    } else {
                        throw std::domain_error(
                            absl::StrCat(
                                "curve type not handled"
                            )
                        );
                    }
                }
            }


        private:
            std::string filename;
            std::ofstream file;
            std::vector<std::string> add_to_show_list;

            // robot collision avoidance hyperplane count
            long long int rcah_count;
            long long int ocah_count;
            long long int orcb_count;
            long long int ocb_count;
            long long int dp_count;
            long long int bez_count;

            void flushShowList() {
                file << "Show[";
                for(std::size_t i = 0; i + 1 < add_to_show_list.size(); i++) {
                    file << add_to_show_list[i] << ", ";
                }
                if(!add_to_show_list.empty()) {
                    file << add_to_show_list.back();
                }
                file << "];" << std::endl;

                add_to_show_list.clear();
            }
        };
#endif

    } // namespace internal

} // namespace rlss

#endif // RLSS_MATHEMATICA_WRITER_HPP