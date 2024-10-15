/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_centroidal_model/PinocchioCentroidalDynamics.h"

#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioCentroidalDynamics::PinocchioCentroidalDynamics(CentroidalModelInfo info)
      : pinocchioInterfacePtr_(nullptr), mapping_(std::move(info)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioCentroidalDynamics::PinocchioCentroidalDynamics(const PinocchioCentroidalDynamics &rhs)
      : pinocchioInterfacePtr_(nullptr), mapping_(rhs.mapping_.getCentroidalModelInfo()) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t PinocchioCentroidalDynamics::getValue(scalar_t time, const vector_t &state, const vector_t &input)
  {
    const auto &interface = *pinocchioInterfacePtr_;
    const auto &info = mapping_.getCentroidalModelInfo();
    assert(info.stateDim == state.rows());

    vector_t f(info.stateDim);
    f << getNormalizedCentroidalMomentumRate(interface, info, input), mapping_.getPinocchioJointVelocity(state, input);

    return f;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation PinocchioCentroidalDynamics::getLinearApproximation(scalar_t time, const vector_t &state,
                                                                                        const vector_t &input)
  {
    const auto &info = mapping_.getCentroidalModelInfo();
    assert(info.stateDim == state.rows());
    assert(info.inputDim == input.rows());

    auto dynamics = ocs2::VectorFunctionLinearApproximation::Zero(info.stateDim, info.stateDim, info.inputDim);
    dynamics.f = getValue(time, state, input);

    // Partial derivatives of the normalized momentum rates
    computeNormalizedCentroidalMomentumRateGradients(state, input);
    // 只计算动量部分，其他部分为0
    matrix_t dfdq = matrix_t::Zero(info.stateDim, info.generalizedCoordinatesNum);
    matrix_t dfdv = matrix_t::Zero(info.stateDim, info.generalizedCoordinatesNum);
    dfdq.topRows<6>() << normalizedLinearMomentumRateDerivativeQ_, normalizedAngularMomentumRateDerivativeQ_; // dhdot/dqPinocchio state中动量的部分 非动量部分如dotp dotrpy 与state无关 为0
    dfdv.bottomRows(info.generalizedCoordinatesNum).setIdentity();                                            // dhdot/dvPinocchio state的后generalizedCoordinatesNum行和vPinocchio是一样的所以为单位阵

    std::tie(dynamics.dfdx, dynamics.dfdu) = mapping_.getOcs2Jacobian(state, dfdq, dfdv);//将dotstate=f(state,input)方程中除力相关的偏导数进行映射

    // 添加 f 对 u 的偏导数，因为 f 的一部分明确取决于输入（接触力 + 扭矩）
    dynamics.dfdu.topRows<3>() += normalizedLinearMomentumRateDerivativeInput_;//dhldot/du
    dynamics.dfdu.middleRows(3, 3) += normalizedAngularMomentumRateDerivativeInput_;//dhwdot/du

    return dynamics;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void PinocchioCentroidalDynamics::computeNormalizedCentroidalMomentumRateGradients(const vector_t &state, const vector_t &input)
  {
    const auto &interface = *pinocchioInterfacePtr_;
    const auto &info = mapping_.getCentroidalModelInfo();
    assert(info.stateDim == state.rows());
    assert(info.inputDim == input.rows());

    // compute partial derivatives of the center of robotMass acceleration and normalized angular momentum
    normalizedLinearMomentumRateDerivativeQ_.setZero(3, info.generalizedCoordinatesNum);
    normalizedAngularMomentumRateDerivativeQ_.setZero(3, info.generalizedCoordinatesNum);
    normalizedLinearMomentumRateDerivativeInput_.setZero(3, info.inputDim);
    normalizedAngularMomentumRateDerivativeInput_.setZero(3, info.inputDim);
    Matrix3 f_hat, p_hat;

    for (size_t i = 0; i < info.numThreeDofContacts; i++)
    {
      const Vector3 contactForceInWorldFrame = centroidal_model::getContactForces(input, i, info);
      f_hat = skewSymmetricMatrix(contactForceInWorldFrame) / info.robotMass;
      const auto J = getTranslationalJacobianComToContactPointInWorldFrame(interface, info, i);
      normalizedAngularMomentumRateDerivativeQ_.noalias() -= f_hat * J;//角动量表示部分，负号为叉乘交换导致，J*dq=dPfoot -[f]*J*dq=-[f]*dPfoot=dhadot 积分得 -[f]*Pfoot=hadot
      normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i).diagonal().array() = 1.0 / info.robotMass;
      p_hat = skewSymmetricMatrix(getPositionComToContactPointInWorldFrame(interface, info, i)) / info.robotMass;
      normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i) = p_hat;//线动量部分直接是力求和
    }

    for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++)
    {
      const size_t inputIdx = 3 * info.numThreeDofContacts + 6 * (i - info.numThreeDofContacts);
      const Vector3 contactForceInWorldFrame = centroidal_model::getContactForces(input, i, info);
      f_hat = skewSymmetricMatrix(contactForceInWorldFrame) / info.robotMass;
      const auto J = getTranslationalJacobianComToContactPointInWorldFrame(interface, info, i);
      normalizedAngularMomentumRateDerivativeQ_.noalias() -= f_hat * J;
      normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, inputIdx).diagonal().array() = 1.0 / info.robotMass;
      p_hat = skewSymmetricMatrix(getPositionComToContactPointInWorldFrame(interface, info, i)) / info.robotMass;
      normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx) = p_hat;
      normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx + 3).diagonal().array() = 1.0 / info.robotMass;
    }
  }

} // namespace ocs2
