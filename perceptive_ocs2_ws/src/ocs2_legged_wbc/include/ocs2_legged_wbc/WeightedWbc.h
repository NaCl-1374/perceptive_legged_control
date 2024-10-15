/*
 * @Author: NaCl
 * @Date: 2024-04-08 14:20:31
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-12 14:17:58
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_wbc/include/ocs2_legged_wbc/WeightedWbc.h
 * @Description: 
 * 
 */
//
// Created by qiayuan on 22-12-23.
//

#include "ocs2_legged_wbc/WbcBase.h"

namespace legged {

class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

 private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};

}  // namespace legged