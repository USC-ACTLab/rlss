#ifndef RLSS_STATISTICS_COLLECTOR_HPP
#define RLSS_STATISTICS_COLLECTOR_HPP

#include <rlss/internal/Util.hpp>
#include "../../../third_party/json.hpp"
#include <fstream>

namespace rlss {

namespace internal {


namespace statistics {
template<typename T, typename U>
[[nodiscard]] U mean(const std::vector<T>& data) {
    U sum = std::accumulate(data.begin(), data.end(), U(0));
    return data.empty() ? 0 : (sum / data.size());
}

template<typename T, typename U>
[[nodiscard]] U variance(const std::vector<T>& data) {
    if(data.size() <= 1)
        return 0;

    U m = mean<T, U>(data);
    U sqsum = std::accumulate(data.begin(), data.end(), U(0), [&] (U sum, T val) {
        return sum + std::pow(val-m, 2);
    });

    U sqsum2 = 0;
    for(auto it = data.begin(); it != data.end(); it++) {
        sqsum2 += std::pow((*it-m), 2);
    }

    return sqsum / (data.size() - 1);
}


template<typename T, typename U>
[[nodiscard]] U stddev(const std::vector<T>& data) {
    U var = variance<T, U>(data);
    return std::sqrt(var);
}


template<typename T>
[[nodiscard]] T min(const std::vector<T>& data) {
    T m = std::numeric_limits<T>::max();
    for(auto v: data) {
        m = std::min(m, v);
    }
    return m;
}


template<typename T>
[[nodiscard]] T max(const std::vector<T>& data) {
    T m = std::numeric_limits<T>::lowest();
    for(auto v: data) {
        m = std::max(m, v);
    }
    return m;
}

// min, max, avg, stddev, num elements
// T data type, U statistics type
template<typename T, typename U>
using Stats = std::tuple<T, T, U, U, std::size_t>;


enum StatIndex {
    imin = 0,
    imax = 1,
    imean = 2,
    istddev = 3,
    ipopsize = 4
};


template<typename T, typename U>
[[nodiscard]] nlohmann::json toJSON(const Stats<T, U>& stats) {
    nlohmann::json result;
    result["min"] = std::get<imin>(stats);
    result["max"] = std::get<imax>(stats);
    result["mean"] = std::get<imean>(stats);
    result["stddev"] = std::get<istddev>(stats);
    result["popsize"] = std::get<ipopsize>(stats);
    return result;
}

template<typename T, typename U>
[[nodiscard]] Stats<T, U> createStats(const std::vector<T>& data) {
    return std::make_tuple(
            statistics::min(data),
            statistics::max(data),
            statistics::mean<T, U>(data),
            statistics::stddev<T, U>(data),
            data.size()
    );
}

template<typename T, typename U>
[[nodiscard]] Stats<T, U> zeroStats() {
    return std::make_tuple(
            std::numeric_limits<T>::max(),
            std::numeric_limits<T>::lowest(),
            U(0),
            U(0),
            0
    );
}

template<typename T, typename U>
[[nodiscard]] Stats<T, U> combineStats(
    const Stats<T, U>& f,
    const Stats<T, U>& s
) {
    std::size_t new_population_size = std::get<ipopsize>(f)
                                    + std::get<ipopsize>(s);
    U new_mean = new_population_size == 0 ?
                0 :
                ((
                       std::get<imean>(f)
                     * std::get<ipopsize>(f)
                     + std::get<imean>(s)
                     * std::get<ipopsize>(s)
                 )
                 / new_population_size);
    T new_min = std::min(std::get<imin>(f),
                         std::get<imin>(s)
    );
    T new_max = std::max(std::get<imax>(f),
                         std::get<imax>(s)
    );

    U new_stddev = new_population_size <= 1 ?
        0 :
        (std::sqrt(
            (
                (std::get<ipopsize>(f)-1)
                * std::get<istddev>(f)
                * std::get<istddev>(f)
                + (std::get<ipopsize>(s)-1)
                * std::get<istddev>(s)
                * std::get<istddev>(s)
                + std::get<ipopsize>(f)
                * (std::get<imean>(f) - new_mean)
                * (std::get<imean>(f) - new_mean)
                + std::get<ipopsize>(s)
                * (std::get<imean>(s) - new_mean)
                * (std::get<imean>(s) - new_mean)
            ) /
            (new_population_size - 1)
        )
    );


    return std::make_tuple(
        new_min,
        new_max,
        new_mean,
        new_stddev,
        new_population_size
    );
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

    statistics::Stats<T, T>
    goalSelectionStatistics() const {
        std::vector<T> data {m_goal_selection_duration};
        return statistics::createStats<T, T>(data);
    }

    statistics::Stats<T, T>
    discreteSearchStatistics() const {
        std::vector<T> data {m_discrete_search_duration};
        return statistics::createStats<T, T>(data);
    }

    statistics::Stats<T, T>
    trajectoryOptimizationDurationsStatistics() const {
        return statistics::createStats<T, T>(
                m_trajectory_optimization_durations);
    }

    statistics::Stats<T, T>
    validityCheckDurationsStatistics() const {
        return statistics::createStats<T, T>(m_validity_check_durations);
    }

    statistics::Stats<T, T>
    planningStatistics() const {
        std::vector<T> data {m_planning_duration};
        return statistics::createStats<T, T>(data);
    }

    statistics::Stats<T, T>
    svmDurationsStatistics() const {
        return statistics::createStats<T, T>(m_svm_durations);
    }

    statistics::Stats<T, T>
    bfsDurationsStatistics() const {
        return statistics::createStats<T, T>(m_bfs_durations);
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
    std::pair<unsigned int, unsigned int>
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

        return std::make_pair(
            success_cnt,
            m_trajectory_optimization_success_fail.size() - success_cnt
        );
    }


    // returns [num successes, num failures]
    std::pair<unsigned int, unsigned int>
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

        return std::make_pair(
                success_cnt,
                m_svm_success_fail.size() - success_cnt
        );
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

    nlohmann::json durationSummaryJSON() const {
        using Stats = statistics::Stats<T, T>;
        Stats goal_selection_stats
            = statistics::zeroStats<T, T>();
        Stats discrete_search_stats
            = statistics::zeroStats<T, T>();
        Stats trajectory_optimization_stats
            = statistics::zeroStats<T, T>();
        Stats validity_check_stats
            = statistics::zeroStats<T, T>();
        Stats planning_stats
            = statistics::zeroStats<T, T>();
        Stats svm_stats
            = statistics::zeroStats<T, T>();
        Stats bfs_stats
            = statistics::zeroStats<T, T>();

        for(const DurationStatistics_& ds: m_durations_statistics) {
            goal_selection_stats = statistics::combineStats<T, T>(
                    goal_selection_stats, ds.goalSelectionStatistics());
            discrete_search_stats = statistics::combineStats<T, T>(
                    discrete_search_stats, ds.discreteSearchStatistics());
            trajectory_optimization_stats = statistics::combineStats<T, T>(
                    trajectory_optimization_stats,
                    ds.trajectoryOptimizationDurationsStatistics());
            validity_check_stats = statistics::combineStats<T, T>(
                    validity_check_stats,
                    ds.validityCheckDurationsStatistics());
            planning_stats = statistics::combineStats<T, T>(
                    planning_stats, ds.planningStatistics());
            svm_stats = statistics::combineStats<T, T>(
                    svm_stats, ds.svmDurationsStatistics());
            bfs_stats = statistics::combineStats<T, T>(
                    bfs_stats, ds.bfsDurationsStatistics());
        }


        nlohmann::json summary;
        summary["goal_selection"] = statistics::toJSON(goal_selection_stats);
        summary["discrete_search"] = statistics::toJSON(discrete_search_stats);
        summary["trajectory_optimization"] = statistics::toJSON(trajectory_optimization_stats);
        summary["validity_check"] = statistics::toJSON(validity_check_stats);
        summary["planning"] = statistics::toJSON(planning_stats);
        summary["svm"] = statistics::toJSON(svm_stats);
        summary["bfs"] = statistics::toJSON(bfs_stats);

        return summary;
    }

    nlohmann::json successFailureSummaryJSON() const {
        /*
         *     bool m_goal_selection_success_fail;
    bool m_discrete_search_success_fail;
    std::vector<bool> m_trajectory_optimization_success_fail;
    bool m_planning_success_fail;
    std::vector<bool> m_svm_success_fail;
         */
        std::pair<unsigned int, unsigned int> goal_selection
                = std::make_pair(0, 0);
        std::pair<unsigned int, unsigned int> discrete_search
                = std::make_pair(0, 0);
        std::pair<unsigned int, unsigned int> traj_opt
                = std::make_pair(0, 0);
        std::pair<unsigned int, unsigned int> planning
                = std::make_pair(0, 0);
        std::pair<unsigned int, unsigned int> svm
                = std::make_pair(0, 0);

        // how many times trajectory optimization is retried
        std::vector<std::size_t> num_traj_opt_when_planning_successful;

        for(const SuccessFailureStatistics_& sf: m_sf_statistics) {
            if(sf.goalSelectionSuccessFail()) {
                goal_selection.first++;
            } else {
                goal_selection.second++;
            }

            if(sf.discreteSearchSuccessFail()) {
                discrete_search.first++;
            } else {
                discrete_search.second++;
            }

            auto tr = sf.trajectoryOptimizationStatistics();
            traj_opt.first += tr.first;
            traj_opt.second += tr.second;

            if(sf.planningSuccessFail()) {
                planning.first++;
                num_traj_opt_when_planning_successful.push_back(
                        sf.trajectoryOptimizationSuccessFail().size());
            } else {
                planning.second++;
            }

            auto sv = sf.svmStatistics();
            svm.first += sv.first;
            svm.second += sv.second;

        }

        statistics::Stats<T, T> num_traj_opt_stats
            = statistics::createStats<std::size_t, T>(
                    num_traj_opt_when_planning_successful);

        nlohmann::json result;

        result["goal_selection"]["success"] = goal_selection.first;
        result["goal_selection"]["fail"] = goal_selection.second;
        result["discrete_search"]["success"] = discrete_search.first;
        result["discrete_search"]["fail"] = discrete_search.second;
        result["trajectory_optimization"]["success"] = traj_opt.first;
        result["trajectory_optimization"]["fail"] = traj_opt.second;
        result["trajectory_optimization"]
              ["retry_count_when_planning_successful"]
            = statistics::toJSON(num_traj_opt_stats);
        result["planning"]["success"] = planning.first;
        result["planning"]["fail"] = planning.second;
        result["svm"]["success"] = svm.first;
        result["svm"]["fail"] = svm.second;

        return result;
    }

    void save(const std::string& filename) const {
        nlohmann::json stats;
        for(const auto& ds: m_durations_statistics) {
            stats["duration_statistics"].push_back(ds.toJSON());
        }

        for(const auto& sfs: m_sf_statistics) {
            stats["success_failure_statistics"].push_back(sfs.toJSON());
        }

        stats["duration_summary"] = this->durationSummaryJSON();
        stats["success_failure_summary"] = this->successFailureSummaryJSON();

        std::ofstream file(filename, std::ios_base::out);
        file << stats.dump();
        file.close();
    }

    void save() const {
        save("statistics"
            + std::to_string(statistics::storage_file_count++)
            + ".json"
        );
    }

    StatisticsStorage<T>& operator+=(const StatisticsStorage<T>& rhs) {
        m_durations_statistics.insert(
                m_durations_statistics.end(),
                rhs.m_durations_statistics.begin(),
                rhs.m_durations_statistics.end());
        m_sf_statistics.insert(
                m_sf_statistics.end(),
                rhs.m_sf_statistics.begin(),
                rhs.m_sf_statistics.end());

        return *this;

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