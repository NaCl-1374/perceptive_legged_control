/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>

namespace ocs2 {

class TrajectorySpreading final {
 public:
  struct Status {
    bool willTruncate = false;
    bool willPerformTrajectorySpreading = false;
  };

  /**
   * Constructor
   * @param [in] debugCaching: To activate the debug print.
   */
  explicit TrajectorySpreading(bool debugPrint = false) : debugPrint_(debugPrint) {}

  /**
*初始化轨迹传播策略。在调用其他成员函数之前运行它
*
*@param [in] oldModeSchedule：与应调整的轨迹相关联的旧模式时间表。
*@param [in] newModeSchedule：应该适应的新模式时间表。
*@param [in] oldTimeTrajectory：与旧模式时间表相关联的旧时间轨迹。
*@returns 设计的轨迹传播策略的状态。
*/
  Status set(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule, const scalar_array_t& oldTimeTrajectory);

  /** Gets the trajectory-spreading strategy report. */
  const Status& getStatus() const { return status_; }

  /**
*调整连续时间轨迹。
*
*@tparam 数据类型。
*@param [in, out] 轨迹：校正轨迹。
*/
  template <typename T>
  void adjustTrajectory(std::vector<T>& trajectory) const;

  /**
*提取事件时间数据。
*
*@tparam 数据类型。
*@param [in] array: 用于整改的输入数组。
*@return: 输出整流数组。
*/
  template <typename T>
  std::vector<T> extractEventsArray(const std::vector<T>& array) const;

  /**
   * Adjust time stamp of the trajectories and post event indices of the trajectories.
   *
   * @param [in, out] timeTrajectory: The time stamp of the trajectories.
   */
  void adjustTimeTrajectory(scalar_array_t& timeTrajectory) const;

  /**
   * Get adjusted post event index array.
   *
   * @return The post event indices of the trajectories.
   */
  const size_array_t& getPostEventIndices() const { return updatedPostEventIndices_; }

 private:
  /**
   * Computing spreading strategy based on the correspondence between the old event time and the new event time.
   *
   * @param oldTimeTrajectory: The old time trajectories that is associated with the old mode schedule.
   * @param oldMatchedEventTimes: The old event time that need adjustment potentially.
   * @param newMatchedEventTimes: The corresponding new event time.
   */
  void computeSpreadingStrategy(const scalar_array_t& oldTimeTrajectory, const scalar_array_t& oldMatchedEventTimes,
                                const scalar_array_t& newMatchedEventTimes);

  // helper function: get index of the iterator returned by upper_bound function
  int upperBoundIndex(const scalar_array_t& timeTrajectory, scalar_t queryTime) const {
    return std::distance(timeTrajectory.begin(), std::upper_bound(timeTrajectory.begin(), timeTrajectory.end(), queryTime));
  }

  // helper function: get index of the iterator returned by lower_bound function
  int lowerBoundIndex(const scalar_array_t& timeTrajectory, scalar_t queryTime) const {
    return std::distance(timeTrajectory.begin(), std::lower_bound(timeTrajectory.begin(), timeTrajectory.end(), queryTime));
  }

  /**
   * Finds post event indices of an array of event time for the given time trajectory.
   *
   * @param eventTimes: An array of event time.
   * @param timeTrajectory: Time trajectory.
   * @return An array containing post event indices of the given event time.
   */
  size_array_t findPostEventIndices(const scalar_array_t& eventTimes, const scalar_array_t& timeTrajectory) const {
    size_array_t postEventIndices(eventTimes.size());
    for (std::size_t i = 0; i < eventTimes.size(); i++) {
      if (i == eventTimes.size() - 1 && eventTimes[i] == timeTrajectory.back()) {
        postEventIndices[i] = timeTrajectory.size() - 1;
      } else {
        postEventIndices[i] = upperBoundIndex(timeTrajectory, eventTimes[i]);
      }
    }
    return postEventIndices;
  }

  /***********
   * Variables
   ***********/
  const bool debugPrint_;

  Status status_;
  size_t eraseFromIndex_;                             /**< The first index to erase **/
  std::pair<size_t, size_t> keepEventDataInInterval_; /**< Keep event data within [first, second) **/

  size_array_t beginIndices_;
  size_array_t endIndices_;
  size_array_t spreadingValueIndices_;

  size_array_t updatedPostEventIndices_;
  scalar_array_t updatedMatchedEventTimes_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
std::vector<T> TrajectorySpreading::extractEventsArray(const std::vector<T>& array) const {
  const auto firstItr = array.begin() + keepEventDataInInterval_.first;
  const auto lastItr = array.begin() + keepEventDataInInterval_.second;
  std::vector<T> out(firstItr, lastItr);
  return out;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
void TrajectorySpreading::adjustTrajectory(std::vector<T>& trajectory) const {
  // erase segment of trajectory associated to mismatched modes
  trajectory.erase(trajectory.begin() + eraseFromIndex_, trajectory.end());

  // extract spreading values beforehand since they might be overridden
  std::vector<T> spreadingValues(spreadingValueIndices_.size());
  std::transform(spreadingValueIndices_.begin(), spreadingValueIndices_.end(), spreadingValues.begin(),
                 [&](const size_t& ind) { return trajectory[ind]; });

  // spread
  for (size_t i = 0; i < spreadingValueIndices_.size(); i++) {
    for (size_t j = beginIndices_[i]; j < endIndices_[i]; j++) {
      trajectory[j] = spreadingValues[i];
    }  // end of i loop
  }    // end of j loop
}

}  // namespace ocs2
