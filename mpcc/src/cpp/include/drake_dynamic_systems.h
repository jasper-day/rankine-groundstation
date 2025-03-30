#pragma once

#include <drake/systems/framework/framework_common.h>
#include <drake/systems/framework/leaf_system.h>

#include "dynamics.h"

namespace mpcc {

namespace systems {

// Adapt FDM dynamics to Drake system

template <typename T>
class FDM_2D_Plant final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FDM_2D_Plant);

  /// Construct a system
  explicit FDM_2D_Plant(drake::Vector<T, 4> const& roll_params,
                        drake::Vector<T, 1> const& v_A,
                        drake::Vector<T, 1> const& g);

  /// Scalar-converting copy constructor
  template <typename U>
  explicit FDM_2D_Plant(FDM_2D_Plant<U> const& other);

 protected:
  void CopyStateOut(drake::systems::Context<T> const& context,
                    drake::systems::BasicVector<T>* output) const;
  void DoCalcTimeDerivatives(
      drake::systems::Context<T> const& context,
      drake::systems::ContinuousState<T>* derivatives) const override;

 private:
  dynamics::FDM_2D<T> fdm_;
  drake::systems::InputPort<T>& phi_ref_port_;
  drake::systems::InputPort<T>& wind_NED_port_;
  int roll_params_parameter_index_;
  int g_parameter_index_;
  int v_A_parameter_index_;
};

}  // namespace systems

}  // namespace mpcc