#include "dynamics.h"

#include <cmath>

#include <drake/common/eigen_types.h>

template <typename T>
drake::Vector<T, 5> mpcc::dynamics::FDM_2D<T>::dynamics(
    drake::Vector<T, 5> const& state, drake::Vector<T, 1> const& controls,
    drake::Vector<T, 2> const& wind_NED) {
  using std::cos;
  using std::sin;
  using std::tan;
  drake::Vector<T, 5> dstate;

  // STATE VARIABLES
  T const& phi = state(0);
  T const& dphi = state(1);
  T const& xi = state(2);
  T const& north = state(3);
  T const& east = state(4);

  // CONTROL VARIABLES
  T const& phi_ref = controls(0);

  // DERIVATIVES
  // Second order roll dynamics model phi_r / r = b0 / s^2 + a_1 s + a_0
  T ddphi, dxi, dnorth, deast;
  ddphi = roll_params_(0) * phi_ref - roll_params_(1) * phi -
          roll_params_(2) * dphi;
  dxi = g_ * tan(phi) / v_A_;
  dnorth = v_A_ * cos(xi) + wind_NED(0);
  deast = v_A_ * sin(xi) + wind_NED(1);
  dstate << dphi, ddphi, dxi, dnorth, deast;
  return dstate;
}

template <typename T>
drake::Vector<T, 9> mpcc::dynamics::FDM_3D<T>::dynamics(
    drake::Vector<T, 9> const& state,
    drake::Vector<T, 4> const& controls) const {
  using std::atan2;
  using std::cos;
  using std::pow;
  using std::sin;
  drake::Vector<T, 9> dstate;

  // STATE VARIABLES
  // roll
  T const& phi = state(0);
  // horizon angle
  T const& th = state(1);
  // roll rate
  T const& p = state(2);
  // pitch rate
  T const& q = state(3);
  // yaw rate
  T const& r = state(4);
  // airspeed
  T const& v_A = state(5);
  // flight path angle
  T const& gam = state(6);
  // heading
  T const& xi = state(7);
  // thrust
  T const& delta_T = state(8);

  // CONTROL VARIABLES
  // reference roll
  T const& phi_ref = controls(0);
  // reference horizon angle
  T const& th_ref = controls(1);
  // skip reference heading
  T const& u_T = controls(3);

  // CALCULATIONS

  // approximate value for alpha, neglecting sideslip
  T alpha, power, thrust, qbarS, lift_coeff, drag_coeff, lift, drag, dphi, dth,
      dp, dq, dr, dv_A, dgam, dxi, ddelta_T;
  alpha = th - gam;
  // thrust = power / velocity at propeller
  power = C_T1() * delta_T + C_T2() * pow(delta_T, T(2)) +
          C_T3() * pow(delta_T, T(3));
  thrust = power / (v_A * cos(alpha));
  // standard equations of lift and drag
  lift_coeff = C_L0() + C_L1() * alpha + C_L2() * pow(alpha, T(2));
  drag_coeff = C_D0() + C_D1() * alpha + C_D2() * pow(alpha, T(2));

  // Assume coordinated turn, ie v_A is all parallel to wings
  qbarS = T(0.5) * rho_ * S_ * pow(v_A, T(2));

  lift = lift_coeff * qbarS;
  drag = drag_coeff * qbarS;

  // LATERAL DERIVATIVES
  // Stastny, eq. 1
  dphi = p;
  dth = q * cos(phi) - r * sin(phi);
  dp = l_p() * p + l_r() * r + l_e_phi() * (phi_ref - phi);
  dq = pow(v_A, T(2)) *
       (m_0() + m_alpha() * alpha + m_q() * q + m_e_th() * (th_ref - th));
  dr = n_r() * r + n_phi() * phi + n_phi_ref() * phi_ref;

  // LONGITUDINAL DERIVATIVES
  // Stastny, eq. 2
  dv_A = (thrust * cos(alpha) - drag) / m_ - g_ * sin(gam);
  dgam = (thrust * sin(alpha) + lift) * cos(phi) / (m_ * v_A) -
         g_ * cos(phi) / v_A;
  dxi = (thrust * sin(alpha) + lift) * sin(phi) / (m_ * v_A * cos(gam));
  ddelta_T = (u_T - delta_T) / tau_T();

  dstate << phi, dth, dp, dq, dr, dv_A, dgam, dxi, ddelta_T;
  return dstate;
}

template <typename T>
drake::Vector<T, 3> mpcc::dynamics::FDM_3D<T>::kinematics(
    drake::Vector<T, 9> const& state, drake::Vector<T, 3> const& wind_NED) {
  using std::cos;
  using std::sin;
  drake::Vector<T, 3> dr;
  T const& gam = state(6);
  T const& xi = state(7);
  T const& v_A = state(5);
  dr.setZero();

  dr << cos(gam) * cos(xi),  // N heading
      cos(gam) * sin(xi),    // E heading
      -sin(gam);             // D heading
  dr = dr * v_A + wind_NED;
  return dr;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dynamics::FDM_2D);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dynamics::FDM_3D);