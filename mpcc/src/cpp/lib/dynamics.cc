#include "dynamics.h"

#include <cmath>

#include <drake/common/eigen_types.h>

template <typename T>
drake::Vector<T, 9> mpcc::dynamics::FDM<T>::dynamics(
    drake::Vector<T, 9> const& state,
    drake::Vector<T, 4> const& controls) const {
  using std::atan2;
  using std::cos;
  using std::pow;
  using std::sin;
  drake::Vector<T, 9> dstate;

  // STATE VARIABLES
  // roll
  T phi = state(0);
  // horizon angle
  T th = state(1);
  // roll rate
  T p = state(2);
  // pitch rate
  T q = state(3);
  // yaw rate
  T r = state(4);
  // airspeed
  T v_A = state(5);
  // flight path angle
  T gam = state(6);
  // heading
  T xi = state(7);
  // thrust
  T delta_T = state(8);

  // CONTROL VARIABLES
  // reference roll
  T phi_ref = controls(0);
  // reference horizon angle
  T th_ref = controls(1);
  // skip reference heading
  T u_T = controls(3);

  // CALCULATIONS

  // approximate value for alpha, neglecting sideslip
  T alpha = th - gam;
  // thrust = power / velocity
  T thrust = C_T1() * delta_T + C_T2() * pow(delta_T, T(2)) +
             C_T3() * pow(delta_T, T(3));
  // standard equations of lift and drag
  T lift = C_L0() + C_L1() * alpha + C_L2 * pow(alpha, 2);
  T drag = C_D0() + C_D1() * alpha + C_D2 * pow(alpha, 2);

  // LATERAL DERIVATIVES
  // Stastny, eq. 1
  T dphi = p;
  T dth = q * cos(phi) - r * sin(phi);
  T dp = l_p() * p + l_r() * r + l_e_phi() * (phi_ref - phi);
  T dq = pow(v_A, T(2)) *
         (m_0() + m_alpha() * alpha + m_q() * q + m_e_th() * (th_ref - th));
  T dr = n_r() * r + n_phi() * phi + n_phi_ref() * phi_ref;

  // LONGITUDINAL DERIVATIVES
  // Stastny, eq. 2
  T dv_A = (thrust * cos(alpha) - drag) / m_ - g_ * sin(gam);
  T dgam = (thrust * sin(alpha) + lift) * cos(phi) / (m_ * v_A) -
           g_ * cos(phi) / v_A;
  T dxi = (thrust * sin(alpha) + L) * sin(phi) / (m_ * v_A * cos(gam));
  T ddelta_T = (u_T - delta_T) / tau_T();

  dstate << phi, dth, dp, dq, dr, dv_a, dgam, dxi, ddelta_T;
  return dstate;
}

template <typename T>
drake::Vector<T, 3> mpcc::dynamics::FDM<T>::kinematics(
    drake::Vector<T, 9> const& state, drake::Vector<T, 3> const& wind_NED) {
  using std::cos;
  using std::sin;
  drake::Vector<T, 3> dr;
  T gam = state(6);
  T xi = state(7);
  T v_A = state(5);
  dr.setZero();

  dr << cos(gam) * cos(xi),  // N heading
      cos(gam) * sin(xi),    // E heading
      -sin(gam);             // D heading
  dr = dr * v_A + wind_NED;
  return dr;
}
