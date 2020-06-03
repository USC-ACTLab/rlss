#ifndef RLSS_STATISTICS_COLLECTOR_HPP
#define RLSS_STATISTICS_COLLECTOR_HPP

#include <rlss/internal/Util.hpp>
#include <json.hpp>
#include <fstream>

namespace rlss {

namespace internal {


namespace statistics {
template<typename T>
T mean(const std::vector<T>& data) {
    T sum = std::accumulate(data.begin(), data.end(), 0);
    return data.empty() ? 0 : (sum / data.size());
}

template<typename T>
T variance(const std::vector<T>& data) {
    if(data.size() <= 1)
        return 0;

    T m = mean(data);
    T sqsum = std::accumulate(data.begin(), data.end(), 0, [m] (T sum, T val) {
        return sum + std::pow(val-m, 2);
    });

    return sqsum / (data.size() - 1);
}


template<typename T>
T stddev(const std::vector<T>& data) {
    T var = variance(data);
    return std::sqrt(var);
}


template<typename T>
T min(const std::vector<T>& data) {
    T m = std::numeric_limits<T>::max();
    for(auto v: data) {
        m = std::min(m, v);
    }
    return m;
}


template<typename T>
T max(const std::vector<T>& data) {
    T m = std::numeric_limits<T>::lowest();
    for(auto v: data) {
        m = std::max(m, v);
    }
    return m;
}

static long long int storage_file_count = 0;

} // namespace statistics

#ifdef ENABLE_RLSS_STATISTICS

/*
 * Durations statistics for one run of RLSS::plan
 */
template<typename T>
class DurationStatistics {
public:
    DurationStatistics() {

    }

    void setGoalSelectionDuration(T gsd) {
        m_goal_selection_duration = gsd;
    }

    void setDiscreteSearchDuration(T dsd) {
        m_discrete_search_duration = dsd;
    }

    void addTrajectoryOptimizationDuration(T tod) {
        m_trajectory_optimization_durations.push_back(tod);
    }

    void addValidityCheckDuration(T vcd) {
        m_validity_check_durations.push_back(vcd);
    }

    void setPlanningDuration(T pd) {
        m_planning_duration = pd;
    }

    void addSvmDuration(T sd) {
        m_svm_durations.push_back(sd);
    }

    void addBFSDuration(T bd) {
        m_bfs_durations.push_back(bd);
    }

    nlohmann::json toJSON() const {
        nlohmann::json result;
        result["goal_selection_duration"] = m_goal_selection_duration;
        result["discrete_search_duration"] = m_discrete_search_duration;
        result["planning_duration"] = m_planning_duration;
        for(const auto& r: m_trajectory_optimization_durations) {
            result["trajectory_optimization_durations"].push_back(r);
        }
        for(const auto& r: m_validity_check_durations) {
            result["validity_check_durations"].push_back(r);
        }
        for(const auto& r: m_svm_durations) {
            result["svm_durations"].push_back(r);
        }
        for(const auto& r: m_bfs_durations) {
            result["bfs_durations"].push_back(r);
        }

        return result;
    }

    T goalSelectionDuration() const {
        return m_goal_selection_duration;
    }

    T discreteSearchDuration() const {
        return m_discrete_search_duration;
    }

    T planningDuration() const {
        return m_planning_duration;
    }

    const std::vector<T>& trajectoryOptimizationDurations() const {
        return m_trajectory_optimization_durations;
    }

    const std::vector<T>& validityCheckDurations() const {
        return m_validity_check_durations;
    }

    const std::vector<T>& svmDurations() const {
        return m_svm_durations;
    }

    const std::vector<T>& bfsDurations() const {
        return m_bfs_durations;
    }

    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
            trajectoryOptimizationDurationsStatistics() const {
        return std::make_tuple(
                statistics::min(m_trajectory_optimization_durations),
                statistics::max(m_trajectory_optimization_durations),
                statistics::mean(m_trajectory_optimization_durations),
                statistics::stddev(m_trajectory_optimization_durations),
                m_trajectory_optimization_durations.size()
        );
    }

    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
    validityCheckDurationsStatistics() const {
        return std::make_tuple(
                statistics::min(m_validity_check_durations),
                statistics::max(m_validity_check_durations),
                statistics::mean(m_validity_check_durations),
                statistics::stddev(m_validity_check_durations),
                m_validity_check_durations.size()
        );
    }


    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
    svmDurationsStatistics() const {
        return std::make_tuple(
                statistics::min(m_svm_durations),
                statistics::max(m_svm_durations),
                statistics::mean(m_svm_durations),
                statistics::stddev(m_svm_durations),
                m_svm_durations.size()
        );
    }

    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
    bfsDurationsStatistics() const {
        return std::make_tuple(
                statistics::min(m_bfs_durations),
                statistics::max(m_bfs_durations),
                statistics::mean(m_bfs_durations),
                statistics::stddev(m_bfs_durations),
                m_bfs_durations.size()
        );
    }


private:
    T m_goal_selection_duration;
    T m_discrete_search_duration;
    std::vector<T> m_trajectory_optimization_durations;
    std::vector<T> m_validity_check_durations;
    T m_planning_duration;

    std::vector<T> m_svm_durations;
    std::vector<T> m_bfs_durations;
}; // DurationStatistics


/*
 * Success failure statistics for one round of RLSS::plan
 */
template<typename T>
class SuccessFailureStatistics {
public:

    void setGoalSelectionSuccessFail(bool r) {
        m_goal_selection_success_fail = r;
    }
    void setDiscreteSearchSuccessFail(bool r) {
        m_discrete_search_success_fail = r;
    }
    void addTrajectoryOptimizationSuccessFail(bool r) {
        m_trajectory_optimization_success_fail.push_back(r);
    }
    void setPlanningSuccessFail(bool r) {
        m_planning_success_fail= r;
    }
    void addSVMSuccessFail(bool r) {
        m_svm_success_fail.push_back(r);
    }

    nlohmann::json toJSON() const {
        nlohmann::json result;
        result["goal_selection_success_fail"] = m_goal_selection_success_fail;
        result["discrete_search_success_fail"] = m_discrete_search_success_fail;
        result["planning_success_fail"] = m_planning_success_fail;
        for(bool r: m_trajectory_optimization_success_fail) {
            result["trajectory_optimization_success_fail"].push_back(r);
        }
        for(bool r: m_svm_success_fail) {
            result["svm_success_fail"].push_back(r);
        }

        return result;
    }

    bool goalSelectionSuccessFail() const {
        return m_goal_selection_success_fail;
    }

    bool discreteSearchSuccessFail() const {
        return m_discrete_search_success_fail;
    }

    const std::vector<bool>& trajectoryOptimizationSuccessFail() const {
        return m_trajectory_optimization_success_fail;
    }

    bool planningSuccessFail() const {
        return m_planning_success_fail;
    }

    const std::vector<bool>& svmSuccessFail() const {
        return m_svm_success_fail;
    }

    // returns [num successes, num failures]
    std::tuple<unsigned int, unsigned int>
            trajectoryOptimizationStatistics() const {
        unsigned int success_cnt
            = std::accumulate(
                m_trajectory_optimization_success_fail.begin(),
                m_trajectory_optimization_success_fail.end(),
                0,
                [](unsigned int sum, bool v) {
                    return sum + v;
                }
        );

        return std::make_tuple(
            success_cnt,
            m_trajectory_optimization_success_fail.size() - success_cnt
        );
    }


    // returns [num successes, num failures]
    std::tuple<unsigned int, unsigned int>
    svmStatistics() const {
        unsigned int success_cnt
                = std::accumulate(
                        m_svm_success_fail.begin(),
                        m_svm_success_fail.end(),
                        0,
                        [](unsigned int sum, bool v) {
                            return sum + v;
                        }
                );

        return std::make_tuple(
                success_cnt,
                m_svm_success_fail.size() - success_cnt
        );
    }

private:
    bool m_goal_selection_success_fail;
    bool m_discrete_search_success_fail;
    std::vector<bool> m_trajectory_optimization_success_fail;
    bool m_planning_success_fail;
    std::vector<bool> m_svm_success_fail;

}; // SuccessFailureStatistics


/*
 * Statistics storage class for several calls to RLSS::plan
 */
template<typename T>
class StatisticsStorage {
public:
    using DurationStatistics_ = DurationStatistics<T>;
    using SuccessFailureStatistics_ = SuccessFailureStatistics<T>;

    StatisticsStorage() {

    }

    ~StatisticsStorage() {

    }

    void add(const DurationStatistics_& ds) {
        m_durations_statistics.push_back(ds);
    }

    void add(const SuccessFailureStatistics_& sf) {
        m_sf_statistics.push_back(sf);
    }

    const std::vector<DurationStatistics_>& durationStatistics() const {
        return m_durations_statistics;
    }

    const std::vector<SuccessFailureStatistics_>& sfStatistics() const {
        return m_sf_statistics;
    }

    void save(const std::string& filename) {
        nlohmann::json stats;
        for(const auto& ds: m_durations_statistics) {
            stats["duration_statistitcs"].push_back(ds.toJSON());
        }

        for(const auto& sfs: m_sf_statistics) {
            stats["success_failure_statistics"].push_back(sfs.toJSON());
        }

        std::ofstream file(filename, std::ios_base::out);
        file << stats;
        file.close();
    }

    void save() const {
        save("statistics" + std::to_string(statistics::storage_file_count++));
    }

private:
    std::vector<DurationStatistics_> m_durations_statistics;
    std::vector<SuccessFailureStatistics_> m_sf_statistics;
};

#else

/*
 * Durations statistics for one run of RLSS::plan
 */
template<typename T>
class DurationStatistics {
public:
    DurationStatistics() {

    }

    void setGoalSelectionDuration(T gsd) {
    }

    void setDiscreteSearchDuration(T dsd) {
    }

    void addTrajectoryOptimizationDuration(T tod) {
    }

    void addValidityCheckDuration(T vcd) {
    }

    void setPlanningDuration(T pd) {
    }

    void addSvmDuration(T sd) {
    }

    void addBFSDuration(T bd) {
    }

    nlohmann::json toJSON() const {
        return nlohmann::json();
    }

    T goalSelectionDuration() const {
        return 0;
    }

    T discreteSearchDuration() const {
        return 0;
    }

    T planningDuration() const {
        return 0;
    }

    const std::vector<T>& trajectoryOptimizationDurations() const {
        return {};
    }

    const std::vector<T>& validityCheckDurations() const {
        return {};
    }

    const std::vector<T>& svmDurations() const {
        return {};
    }

    const std::vector<T>& bfsDurations() const {
        return {};
    }

    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
            trajectoryOptimizationDurationsStatistics() const {
        return {};
    }

    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
    validityCheckDurationsStatistics() const {
        return {};
    }


    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
    svmDurationsStatistics() const {
        return {};
    }

    // returns [min, max, mean, stddev, num data points]
    std::tuple<T, T, T, T, std::size_t>
    bfsDurationsStatistics() const {
        return {};
    }
}; // DurationStatistics


/*
* Success failure statistics for one round of RLSS::plan
*/
template<typename T>
class SuccessFailureStatistics {
public:

    void setGoalSelectionSuccessFail(bool r) {
    }
    void setDiscreteSearchSuccessFail(bool r) {
    }
    void addTrajectoryOptimizationSuccessFail(bool r) {
    }
    void setPlanningSuccessFail(bool r) {
    }
    void addSVMSuccessFail(bool r) {
    }

    nlohmann::json toJSON() const {
        return nlohmann::json();
    }

    bool goalSelectionSuccessFail() const {
        return false;
    }

    bool discreteSearchSuccessFail() const {
        return false;
    }

    const std::vector<bool>& trajectoryOptimizationSuccessFail() const {
        return {};
    }

    bool planningSuccessFail() const {
        return false;
    }

    const std::vector<bool>& svmSuccessFail() const {
        return {};
    }

    // returns [num successes, num failures]
    std::tuple<unsigned int, unsigned int>
    trajectoryOptimizationStatistics() const {
        return {};
    }

    // returns [num successes, num failures]
    std::tuple<unsigned int, unsigned int>
    svmStatistics() const {
        return {};
    }

}; // SuccessFailureStatistics


/*
* Statistics storage class for several calls to RLSS::plan
*/
template<typename T>
class StatisticsStorage {
public:
    using DurationStatistics_ = DurationStatistics<T>;
    using SuccessFailureStatistics_ = SuccessFailureStatistics<T>;

    StatisticsStorage() {

    }

    ~StatisticsStorage() {

    }

    void add(const DurationStatistics_& ds) {
    }

    void add(const SuccessFailureStatistics_& sf) {
    }

    const std::vector<DurationStatistics_>& durationStatistics() const {
        return {};
    }

    const std::vector<SuccessFailureStatistics_>& sfStatistics() const {
        return {};
    }

    void save(const std::string& filename) {
    }

    void save() const {
    }
};

#endif

} // namespace internal

} // namespace rlss

#endif // RLSS_STATISTICS_COLLECTOR_HPP