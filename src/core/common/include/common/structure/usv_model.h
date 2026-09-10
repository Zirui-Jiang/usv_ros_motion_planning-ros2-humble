/**
 * *********************************************************
 *
 * @file: usv_model.h
 * @brief: USV model parameters
 * @author: Yang Haodong
 * @date: 2025-4-8
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_STRUCTURE_USV_MODEL_H_
#define RMP_COMMON_STRUCTURE_USV_MODEL_H_

#include <string>

namespace rmp
{
namespace common
{
namespace structure
{
enum USV_TYPE
{
  Omni,
  Differential,
};

struct USV_MODEL
{
  std::string ns{ "none" };                 // ship namespace
  std::string name{ "none" };               // ship name
  USV_TYPE type{ USV_TYPE::Differential };  // ship type
  double cruising_speed{ 0.0 };             // cruising_speed
  double max_force{ 0.0 };                  // max force forward and reverse
  double mass{ 0.0 };                       // mass
  double i_z{ 0.0 };                        // moment of inertia about the z-axis
  double L{ 0.0 };                          // length
  double W{ 0.0 };                          // width
  double D{ 0.0 };                          // distance from propeller to main shaft

  // physical limitation
  double velocity_min{ 0.0 };
  double velocity_max{ 0.0 };
  double acceleration_min{ 0.0 };
  double acceleration_max{ 0.0 };
  double jerk_min{ 0.0 };
  double jerk_max{ 0.0 };

  // ship acceleration-induced added mass coefficient
  double x_dot_u{ 0.0 };
  double x_dot_v{ 0.0 };
  double x_dot_r{ 0.0 };
  double y_dot_u{ 0.0 };
  double y_dot_v{ 0.0 };
  double y_dot_r{ 0.0 };
  double n_dot_u{ 0.0 };
  double n_dot_v{ 0.0 };
  double n_dot_r{ 0.0 };

  // ship velocity-induced linear hydrodynamic damping coefficient
  double x_u{ 0.0 };
  double x_v{ 0.0 };
  double x_r{ 0.0 };
  double y_u{ 0.0 };
  double y_v{ 0.0 };
  double y_r{ 0.0 };
  double n_u{ 0.0 };
  double n_v{ 0.0 };
  double n_r{ 0.0 };

  // ship velocity-induced 2nd hydrodynamic damping coefficient
  double x_u_u{ 0.0 };
  double y_v_v{ 0.0 };
  double n_r_r{ 0.0 };
};

struct USV_STATE
{
  double x{ 0.0 };
  double y{ 0.0 };
  double psi{ 0.0 };
  double u{ 0.0 };
  double v{ 0.0 };
  double r{ 0.0 };
  double x_dot{ 0.0 };
  double y_dot{ 0.0 };
  double psi_dot{ 0.0 };
};

}  // namespace structure
}  // namespace common
}  // namespace rmp
#endif