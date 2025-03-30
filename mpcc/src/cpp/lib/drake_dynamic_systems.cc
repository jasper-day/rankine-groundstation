#pragma once

#include "drake_dynamic_systems.h"

#include <drake/systems/framework/framework_common.h>

template <typename T>
mpcc::systems::FDM_2D_Plant<T>::FDM_2D_Plant(
    drake::Vector<T, 4> const& roll_params, drake::Vector<T, 1> const& v_A,
    drake::Vector<T, 1> const& g)
    : drake::systems::LeafSystem<T>(
          drake::systems::SystemTypeTag<mpcc::systems::FDM_2D_Plant>{}) {
  // input vector: just reference phi
  phi_ref_port_ = this->DeclareVectorInputPort("phi_ref", 1);
  // wind
  wind_NED_port_ = this->DeclareVectorInputPort("wind_NED", 2);
  // Define parameters
  roll_params_parameter_index_ = this->DeclareNumericParameter(roll_params);
  g_parameter_index_ = this->DeclareNumericParameter(g);
  v_A_parameter_index_ = this->DeclareNumericParameter(v_A);
  // simulation state
  this->DeclareContinuousState(5);
  // outputs
  this->DeclareVectorOutputPort("state", 5,
                                &mpcc::systems::FDM_2D_Plant<T>::CopyStateOut);
}

template <typename T>
template <typename U>
mpcc::systems::FDM_2D_Plant<T>::FDM_2D_Plant(
    mpcc::systems::FDM_2D_Plant<U> const& other)
    : mpcc::systems::FDM_2D_Plant<T>() {}

template <typename T>
void mpcc::systems::FDM_2D_Plant<T>::CopyStateOut(
    drake::systems::Context<T> const& context,
    drake::systems::BasicVector<T>* output) const {
  const drake::systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  output->set_value(continuous_state_vector.CopyToVector());
}

template <typename T>
void mpcc::systems::FDM_2D_Plant<T>::DoCalcTimeDerivatives(
    drake::systems::Context<T> const& context,
    drake::systems::ContinuousState<T>* derivatives) const {
  // Get current state from context
  drake::Vector<T, 5> const& state = context.get_continuous_state_vector();
  // Obtain the structure we need to write into
  drake::systems::VectorBase<T>& derivatives_vector =
      derivatives->get_mutable_vector();
  // Get the controlled roll angle
  drake::Vector<T, 1> const& phi_ref =
      this->EvalVectorInput(context, phi_ref_port_);
  drake::Vector<T, 2> const& wind_NED =
      this->EvalVectorInput(context, wind_NED_port_);
  // Calculate with the fdm
  derivatives_vector.SetFromVector(fdm_.dynamics(state, controls, wind_NED);)
}

// TODO: Deal with parameter changes