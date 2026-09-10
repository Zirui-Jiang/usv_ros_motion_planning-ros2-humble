// Calls the existing baseline shared libraries. No algorithm formulas copied.
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include "path_planner/path_planner.h"
#include "path_planner/path_prune/ramer_douglas_peucker.h"
#include "path_planner/path_smooth/savitzky_golay.h"
#include "trajectory_planner/trajectory.h"
#include "controller/pid_controller/otter_pid_controller.h"
#include "controller/lqr_controller/otter_lqr_controller.h"

using rmp::common::geometry::Points3d;
using rmp::trajectory_planner::Trajectory;
using rmp::trajectory_planner::TrajectoryCell;

Trajectory trajectory(const YAML::Node& points) {
  Trajectory result;
  for (const auto& p : points) result.push({p[0].as<double>(),p[1].as<double>(),p[2].as<double>(),p[3].as<double>(),p[4].as<double>(),p[5].as<double>(),p[6].as<double>(),p[7].as<double>()});
  return result;
}
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  auto fixture = YAML::LoadFile(argv[1]);
  std::ofstream out(argv[2]);
  if (!out) return 3;
  out << std::setprecision(17);
  rclcpp::init(0, nullptr);
  for (const auto& c : fixture["cases"]) {
    const auto op = c["operation"].as<std::string>();
    out << c["id"].as<std::string>();
    if (op == "resample" || op == "rdp" || op == "sg") {
      Points3d input, result;
      for (const auto& p : c["points"]) input.emplace_back(p[0].as<double>(),p[1].as<double>(),p[2].as<double>());
      if (op == "resample") {
        if (!rmp::path_planner::PathPlanner::resample(input, result, c["spacing"].as<double>())) throw std::runtime_error("resample failed");
      } else if (op == "rdp") {
        rmp::path_planner::RDPPathProcessor p(c["epsilon"].as<double>(), c["max_interval"].as<double>());
        p.process(input,result);
      } else {
        rmp::path_planner::SavitzkyGolayPathProcessor p;
        p.process(input,result);
      }
      for (const auto& p : result) out << '\t' << p.x() << '\t' << p.y() << '\t' << p.theta();
    } else if (op == "trajectory_time" || op == "trajectory_distance") {
      auto input = trajectory(c["points"]);
      for (const auto& q : c["queries"]) {
        TrajectoryCell cell;
        bool ok = op == "trajectory_time" ? input.getReferencWithTime(q.as<double>(), cell) : input.getReferencWithDistance(q.as<double>(), cell);
        if (!ok) throw std::runtime_error("reference failed");
        out << '\t' << cell.time << '\t' << cell.s << '\t' << cell.x << '\t' << cell.y << '\t' << cell.theta << '\t' << cell.curv << '\t' << cell.velocity << '\t' << cell.acceleration;
      }
    } else if (op == "pid" || op == "lqr") {
      rmp::common::structure::USV_MODEL m;
      m.ns="parity"; m.name="otter";
      auto model = fixture["model"];
      m.mass=model["mass"].as<double>(); m.i_z=model["i_z"].as<double>();
      m.D=model["D"].as<double>(); m.max_force=model["max_force"].as<double>();
      m.x_dot_u=model["x_dot_u"].as<double>(); m.y_dot_v=model["y_dot_v"].as<double>(); m.n_dot_r=model["n_dot_r"].as<double>();
      m.x_u=model["x_u"].as<double>(); m.y_v=model["y_v"].as<double>(); m.n_r=model["n_r"].as<double>();
      auto cfg=c["config"];
      std::shared_ptr<rmp::controller::Controller> control;
      if (op=="pid") control=std::make_shared<rmp::controller::PIDControllerOtter>(10,m,nullptr,nullptr,0.1,cfg["lookahead_time"].as<double>(),cfg["k_p_speed"].as<double>(),cfg["k_i_speed"].as<double>(),cfg["k_d_speed"].as<double>(),cfg["k_p_course"].as<double>(),cfg["k_i_course"].as<double>(),cfg["k_d_course"].as<double>());
      else {
        auto q=cfg["Q_matrix_diag"].as<std::vector<double>>(); auto r=cfg["R_matrix_diag"].as<std::vector<double>>();
        control=std::make_shared<rmp::controller::LQRControllerOtter>(10,m,nullptr,nullptr,0.1,cfg["lookahead_time"].as<double>(),cfg["max_iterations"].as<int>(),cfg["discrete_dt"].as<double>(),cfg["solve_eps"].as<double>(),q,r);
      }
      control->setGlobalPlan({{0,0,0},{100,0,0}});
      auto t=trajectory(c["points"]); control->setLocalPlan(t);
      for (const auto& s:c["states"]) {
        rmp::common::structure::USV_STATE state;
        state.x=s[0].as<double>(); state.y=s[1].as<double>(); state.psi=s[2].as<double>(); state.u=s[3].as<double>(); state.v=s[4].as<double>(); state.r=s[5].as<double>();
        state.x_dot=std::cos(state.psi)*state.u-std::sin(state.psi)*state.v;
        state.y_dot=std::sin(state.psi)*state.u+std::cos(state.psi)*state.v; state.psi_dot=state.r;
        control->setState(state); rmp::controller::ControlCommand command;
        if (!control->computeControlCommand(command)) throw std::runtime_error("controller failed");
        out << '\t' << command.thrust_cmd_left() << '\t' << command.thrust_cmd_right() << '\t' << command.thrust_cmd_lateral();
      }
    } else throw std::runtime_error("unknown operation");
    out << '\n'; out.flush();
  }
  rclcpp::shutdown();
}
