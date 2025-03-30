#pragma once

#include <drake/common/eigen_types.h>

namespace mpcc {

namespace dynamics {

/**
 * Flight dynamic model implemented according to
 *
 * Thomas J. Stastny, Adyasha Dash, and Roland Siegwart, “Nonlinear MPC for
 * Fixed-Wing UAV Trajectory Tracking: Implementation and Flight Experiments,”
 * in AIAA Guidance, Navigation, and Control Conference (AIAA Guidance,
 * Navigation, and Control Conference, Grapevine, Texas: American Institute of
 * Aeronautics and Astronautics, 2017), https://doi.org/10.2514/6.2017-1512.
 *
 * This FDM implements lateral (horizontal) dynamics only.
 *
 * The controller-in-the-loop roll rate transfer function is represented by
 * the second order equation.
 *
 * \f$ {\phi \over \phi_r} = {b_0 \over s^2 + a_1 s + a_0} \f$
 *
 * This is set by roll_params, in order
 * roll_params(0) = b_0
 * roll_params(1) = a_0
 * roll_params(2) = a_1
 *
 */
template <typename T>
class FDM_2D {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FDM_2D);

  FDM_2D() = default;
  ~FDM_2D() = default;

  FDM_2D(drake::Vector<T, 3> roll_params, T g)
      : g_(g), roll_params_(roll_params) {}

  void set_g(T g) { g_ = g; }
  void set_v_A(T v_A) { v_A_ = v_A; }
  void set_params(drake::Vector<T, 3> params) { params = params }

  /// Scalar-converting copy constructor
  template <typename U>
  FDM_2D(FDM_2D<U> const& other) : FDM_2D(other.roll_params_, other.g) {}

  template <typename U>
  friend class FDM_2D<U>;

  /**
   * Flight dynamics and kinematics
   *
   * The controller-in-the-loop roll rate transfer function is represented by
   * the second order equation.
   *
   * \f$ {\phi \over \phi_r} = {b_0 \over s^2 + a_1 s + a_0} @\f$
   *
   * INPUTS:
   * State vector (Vector<T, 5>)
   * phi (rad)  : bank angle
   * dphi (rad) : bank angle rate
   * xi (rad)   : heading angle (from north)
   * n (m)      : northing
   * e (m)      : easting
   *
   * Controls (Vector<T, 1>)
   * phi_ref (rad) : reference bank angle
   *
   * wind_NED (Vector<T, 2>)
   * w_n (m/s)  : windspeed north (pushes vehicle north)
   * w_e (m/s)  : windspeed east (pushes vehicle east)
   *
   * v_A (T) (m/s) : Airspeed
   *
   * OUTPUTS:
   * Derivative state vector (Vector<T, 5>)
   * dphi (rad/s)   : bank angle rate
   * ddphi (rad/s^2): bank angle acceleration
   * dxi (rad/s)    : heading angle rate
   * dn (m/s)       : velocity north
   * de (m/s)       : velocity east
   *
   */
  drake::Vector<T, 5> dynamics(drake::Vector<T, 5> const& state,
                               drake::Vector<T, 1> const& controls,
                               drake::Vector<T, 2> const& wind_NED);

 private:
  /// Acceleration due to gravity (m/s^2)
  T g_;
  /// (Constant) plane velocity
  T v_A_;
  drake::Vector<T, 3> roll_params_;
};

/**
 * Flight dynamic model implemented according to
 *
 * Thomas Stastny and Roland Siegwart, “Nonlinear Model Predictive Guidance for
 * Fixed-Wing UAVs Using Identified Control Augmented Dynamics,” in 2018
 * International Conference on Unmanned Aircraft Systems (ICUAS) (2018
 * International Conference on Unmanned Aircraft Systems (ICUAS), Dallas, TX:
 * IEEE, 2018), 432–42, https://doi.org/10.1109/ICUAS.2018.8453377.
 *
 * This flight dynamic model is intended to control a craft with a low-level
 * controller stabilizing the attitude and attitude rates. The low-level
 * controller (eg., PX4, Ardupilot) tracks commanded attitudes and provides
 * coordinated turns. In offboard control mode, however, the low-level
 * controller does not provide altitude or velocity stabilization. Our
 * controller will therefore have to include flight dynamics separately.
 *
 * This FDM implements lateral (horizontal) dynamics and longitudinal (vertical)
 * dynamics.
 *
 * Technical notes:
 *
 * The flight dynamic model is implemented here as a generic class over
 * drake scalars, allowing for automatic differentiation.
 */
template <typename T>
class FDM_3D {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FDM_3D);

  FDM_3D() = default;
  ~FDM_3D() = default;

  // copying the arguments doesn't matter and we don't have to worry about
  // lifetimes
  FDM_3D(drake::Vector<T, 10> coeffs_cl, drake::Vector<T, 10> coeffs_ol, T m,
         T g = T(9.81))
      : coeffs_cl_(coeffs_cl), coeffs_ol_(coeffs_ol), m_(m), g_(g) {}

  /// Scalar-converting copy constructor
  template <typename U>
  FDM_3D(FDM_3D<U> const& other)
      : FDM_3D<T>(other.coeffs_cl_, other.coeffs_ol_, other.m_, other.g_) {}

  /**
   * Flight dynamics
   *
   * Input:
   * state vector (Vector<T, 9>)
   * phi (rad)  : roll angle
   * theta (rad): angle to horizon
   * p (rad/s)  : body x rate
   * q (rad/s)  : body y rate
   * r (rad/s)  : body z rate
   * v_A (m/s): Airspeed (relative to air mass)
   * gamma (rad): flight path angle
   * xi (rad): heading angle (equal to yaw angle in coordinated turns)
   * delta_T: throttle internal state (simple lag model)
   *
   * controls vector (Vector<T, 2>)
   * phi_ref (rad): Reference roll angle
   * th_ref (rad) : Reference horizon angle
   * xi_ref (rad) : Reference heading (not used in dynamics)
   * u_T          : Reference thrust
   *
   * Output:
   * state vector derivative
   * dphi (rad/s)   : roll rate
   * dtheta (rad/s) : horizon angle rate
   * dp (rad/s^2)   : roll acceleration
   * dq (rad/s^2)   : pitch acceleration
   * dr (rad/s^2)   : yaw acceleration
   * dv_A (m/s^2): Airspeed derivative
   * dgamma (rad/s^2): flight path angle rate
   * dxi (rad/s^2): heading angle rate
   * ddelta_T: throttle internal rate
   */
  drake::Vector<T, 9> dynamics(drake::Vector<T, 9> const& state,
                               drake::Vector<T, 4> const& controls) const;

  /**
   * 3-DOF Kinematics
   *
   * Input:
   * state (Vector<T, 9>)
   * Same argument as FDM::dynamics
   *
   * wind_NED (Vector<T, 3>)
   * north, east, and down components of inertial wind
   *
   * Output:
   * dr_NED (Vector<T, 3>)
   * north, east, and down components of inertial velocity
   *
   */
  drake::Vector<T, 3> kinematics(drake::Vector<T, 9> const& state,
                                 drake::Vector<T, 3> const& wind_NED);

  // closed loop coefficients
  T l_p() const { return coeffs_cl_(0); }
  T l_r() const { return coeffs_cl_(1); }
  T l_e_phi() const { return coeffs_cl_(2); }
  T m_0() const { return coeffs_cl_(3); }
  T m_alpha() const { return coeffs_cl_(4); }
  T m_q() const { return coeffs_cl_(5); }
  T m_e_th() const { return coeffs_cl_(6); }
  T n_r() const { return coeffs_cl_(7); }
  T n_phi() const { return coeffs_cl_(8); }
  T n_phi_ref() const { return coeffs_cl_(9); }

  // open loop coefficients
  T C_T1() const { return coeffs_ol_(0); }
  T C_T2() const { return coeffs_ol_(1); }
  T C_T3() const { return coeffs_ol_(2); }
  T tau_T() const { return coeffs_ol_(3); }
  T C_D0() const { return coeffs_ol_(4); }
  T C_D1() const { return coeffs_ol_(5); }
  T C_D2() const { return coeffs_ol_(6); }
  T C_L0() const { return coeffs_ol_(7); }
  T C_L1() const { return coeffs_ol_(8); }
  T C_L2() const { return coeffs_ol_(9); }

  void set_m(T m) { m_ = m; }
  void set_g(T g) { g_ = g; }
  void set_coeffs_cl(drake::Vector<T, 10> coeffs_cl) { coeffs_cl_ = coeffs_cl; }
  void set_coeffs_ol(drake::Vector<T, 10> coeffs_ol) { coeffs_ol_ = coeffs_ol; }

  template <typename U>
  friend class FDM_3D;

 private:
  /// Plane mass
  T m_;
  /// Gravity
  T g_;
  /// closed loop coefficients l_p, l_r, l_e_phi, m_0, m_alpha, m_q, m_e_phi,
  /// n_r, n_phi, n_phi_ref
  drake::Vector<T, 10> const coeffs_cl_;
  /// open loop coefficients C_T1, C_T2, C_T3, tau_T, C_D0, C_Dalpha, C_Dalpha2,
  /// C_L0, C_Lalpha, C_Lalpha2
  drake::Vector<T, 10> const coeffs_ol_;
};

}  // namespace dynamics

}  // namespace mpcc