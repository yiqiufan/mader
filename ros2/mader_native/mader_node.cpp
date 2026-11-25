#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <type_traits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mader_msgs/msg/dyn_traj.hpp"
#include "mader_msgs/msg/who_plans.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "snapstack_msgs/msg/goal.hpp"
#include "snapstack_msgs/msg/state.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "elapsed_timer.hpp"
#include "mader.hpp"
#include "mader_handoff_interfaces/srv/propose_trajectory.hpp"

namespace
{
using geometry_msgs::msg::PoseStamped;
using mader_msgs::msg::DynTraj;
using mader_msgs::msg::PieceWisePolTraj;
using mader_msgs::msg::WhoPlans;
using snapstack_msgs::msg::Goal;
using snapstack_msgs::msg::State;

template<typename T>
void read_parameter(rclcpp::Node& node, const char* name, T& value)
{
  const auto parameter = node.get_parameter(name);
  if constexpr (std::is_same_v<T, double>) {
    value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
      static_cast<double>(parameter.as_int()) : parameter.as_double();
  } else if constexpr (std::is_same_v<T, int>) {
    value = static_cast<int>(parameter.as_int());
  } else {
    value = parameter.get_value<T>();
  }
}

void verify(bool condition, const char* requirement)
{
  if (!condition) {
    throw std::invalid_argument(requirement);
  }
}

mt::parameters read_configuration(rclcpp::Node& node)
{
  mt::parameters par{};
  std::vector<double> v_max_tmp, a_max_tmp, j_max_tmp;
  read_parameter(node, "use_ff", par.use_ff);
  read_parameter(node, "visual", par.visual);
  read_parameter(node, "color_type", par.color_type);
  read_parameter(node, "n_agents", par.n_agents);
  read_parameter(node, "dc", par.dc);
  read_parameter(node, "goal_radius", par.goal_radius);
  read_parameter(node, "drone_radius", par.drone_radius);
  read_parameter(node, "Ra", par.Ra);
  read_parameter(node, "w_max", par.w_max);
  read_parameter(node, "alpha_filter_dyaw", par.alpha_filter_dyaw);
  read_parameter(node, "x_min", par.x_min);
  read_parameter(node, "x_max", par.x_max);
  read_parameter(node, "y_min", par.y_min);
  read_parameter(node, "y_max", par.y_max);
  read_parameter(node, "z_min", par.z_min);
  read_parameter(node, "z_max", par.z_max);
  read_parameter(node, "v_max", v_max_tmp);
  read_parameter(node, "a_max", a_max_tmp);
  read_parameter(node, "j_max", j_max_tmp);
  read_parameter(node, "factor_alloc", par.factor_alloc);
  read_parameter(node, "factor_alloc_close", par.factor_alloc_close);
  read_parameter(node, "dist_factor_alloc_close", par.dist_factor_alloc_close);
  read_parameter(node, "factor_alpha", par.factor_alpha);
  read_parameter(node, "num_pol", par.num_pol);
  read_parameter(node, "deg_pol", par.deg_pol);
  read_parameter(node, "weight", par.weight);
  read_parameter(node, "epsilon_tol_constraints", par.epsilon_tol_constraints);
  read_parameter(node, "xtol_rel", par.xtol_rel);
  read_parameter(node, "ftol_rel", par.ftol_rel);
  read_parameter(node, "solver", par.solver);
  read_parameter(node, "upper_bound_runtime_snlopt", par.upper_bound_runtime_snlopt);
  read_parameter(node, "lower_bound_runtime_snlopt", par.lower_bound_runtime_snlopt);
  read_parameter(node, "kappa", par.kappa);
  read_parameter(node, "mu", par.mu);
  read_parameter(node, "a_star_samp_x", par.a_star_samp_x);
  read_parameter(node, "a_star_samp_y", par.a_star_samp_y);
  read_parameter(node, "a_star_samp_z", par.a_star_samp_z);
  read_parameter(node, "a_star_fraction_voxel_size", par.a_star_fraction_voxel_size);
  read_parameter(node, "allow_infeasible_guess", par.allow_infeasible_guess);
  read_parameter(node, "a_star_bias", par.a_star_bias);
  read_parameter(node, "basis", par.basis);
  read_parameter(node, "res_plot_traj", par.res_plot_traj);
  read_parameter(node, "alpha", par.alpha);
  read_parameter(node, "beta", par.beta);
  read_parameter(node, "gamma", par.gamma);
  read_parameter(node, "alpha_shrink", par.alpha_shrink);
  read_parameter(node, "fov_horiz_deg", par.fov_horiz_deg);
  read_parameter(node, "fov_vert_deg", par.fov_vert_deg);
  read_parameter(node, "fov_depth", par.fov_depth);
  auto vector3 = [](const std::vector<double>& values) {
    verify(values.size() == 3, "Expected three axis limits");
    return Eigen::Vector3d(values[0], values[1], values[2]);
  };
  par.v_max = vector3(v_max_tmp);
  par.a_max = vector3(a_max_tmp);
  par.j_max = vector3(j_max_tmp);

  // The checks and operators are the original MaderRos configuration checks.
  verify(par.gamma > 0, "Expected gamma > 0");
  verify(par.beta >= 0 || par.alpha >= 0, "Expected beta >= 0 or alpha >= 0");
  verify(par.factor_alloc >= 1.0, "Expected factor_alloc >= 1.0");
  verify(par.kappa > 0 || par.mu > 0, "Expected kappa > 0 or mu > 0");
  verify(par.kappa + par.mu <= 1, "Expected kappa + mu <= 1");
  verify(par.a_star_fraction_voxel_size >= 0.0 || par.a_star_fraction_voxel_size <= 1.0,
         "a_star_fraction_voxel_size");
  verify(par.epsilon_tol_constraints < 0.02, "Expected epsilon_tol_constraints < 0.02");
  return par;
}

template<typename T>
void assign_xyz(T& message, const Eigen::Vector3d& values)
{
  message.x = values.x();
  message.y = values.y();
  message.z = values.z();
}

template<typename Wire>
void to_wire(const std::vector<Eigen::Vector4d>& coefficients, Wire& wire)
{
  for (const auto& values : coefficients) {
    typename Wire::value_type msg;
    msg.a = values(0);
    msg.b = values(1);
    msg.c = values(2);
    msg.d = values(3);
    wire.push_back(msg);
  }
}

template<typename Wire>
void from_wire(const Wire& wire, std::vector<Eigen::Vector4d>& coefficients)
{
  for (const auto& msg : wire) {
    coefficients.emplace_back(msg.a, msg.b, msg.c, msg.d);
  }
}

class MaderNode final : public rclcpp::Node
{
public:
  MaderNode()
  : Node("mader", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    par_(read_configuration(*this)),
    id_(get_parameter("vehicle_id").as_int()),
    explicit_handoff_(get_parameter_or<bool>("explicit_handoff", false)),
    planner_(par_, [this] { return now().seconds(); },
      [this](const char* phase, const mt::dynTrajCompiled& peer, double opt_start) {
        RCLCPP_INFO(get_logger(),
          "PEER_USED phase=%s own=%d peer=%d begin=%.9f end=%.9f receive=%.9f opt_start=%.9f",
          phase, id_, peer.id, peer.pwp.times.front(), peer.pwp.times.back(), peer.time_received, opt_start);
      })
  {
    // Match the three original ROS 1 callback queues.
    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ref_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    plan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions io;
    io.callback_group = io_group_;
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    peer_sub_ = create_subscription<DynTraj>("/trajs", rclcpp::QoS(20).reliable().transient_local(),
      [this](DynTraj::ConstSharedPtr msg) {
        if (msg->is_agent) {
          on_trajectory(msg);
        }
      }, io);
    // Upstream obstacle publishers send live, non-latched updates on /trajs.
    obstacle_sub_ = create_subscription<DynTraj>("/trajs", rclcpp::QoS(20).reliable().durability_volatile(),
      [this](DynTraj::ConstSharedPtr msg) {
        if (!msg->is_agent) {
          on_trajectory(msg);
        }
      }, io);
    if (explicit_handoff_) {
      proposal_ = create_service<Propose>("propose_trajectory",
        [this](Propose::Request::SharedPtr req, Propose::Response::SharedPtr res) { on_propose(*req, *res); },
        rmw_qos_profile_services_default, plan_group_);
      return;
    }
    goals_ = create_publisher<Goal>("goal", 1);
    outgoing_ = create_publisher<DynTraj>("/trajs", latched);
    authority_sub_ = create_subscription<WhoPlans>("who_plans", latched,
      [this](WhoPlans::ConstSharedPtr msg) { on_authority(*msg); }, io);

    const auto period = rclcpp::Duration::from_seconds(par_.dc);
    send_ref_ = rclcpp::create_timer(this, get_clock(), period, [this] { send_reference(); }, ref_group_);
    replan_ = rclcpp::create_timer(this, get_clock(), period, [this] { replan(); }, plan_group_);
    send_ref_->cancel();
    replan_->cancel();
    rebroadcast_.Reset();
  }

private:
  using Propose = mader_handoff_interfaces::srv::ProposeTrajectory;

  void on_propose(const Propose::Request& request, Propose::Response& response)
  {
    mt::state initial, target;
    initial.setPos(request.position.x, request.position.y, request.position.z);
    initial.setVel(request.velocity.x, request.velocity.y, request.velocity.z);
    initial.setAccel(request.acceleration.x, request.acceleration.y, request.acceleration.z);
    target.setPos(request.target.x, request.target.y, request.target.z);
    const double start = rclcpp::Time(request.valid_from).seconds();
    if (!initial.pos.allFinite() || !initial.vel.allFinite() || !initial.accel.allFinite() ||
        !target.pos.allFinite()) {
      response.reason = "NONFINITE_BOUNDARY";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      position_ = initial.pos;
      handoff_position_known_ = true;
      for (const auto& entry : pending_trajectories_) {
        const auto& pending = entry.second;
        update_trajectory(*pending.message, pending.received, position_);
      }
      pending_trajectories_.clear();
    }
    mt::PieceWisePol candidate;
    if (!planner_.propose(initial, target, start, candidate)) {
      response.reason = "NO_CANDIDATE";
      return;
    }
    if (now().seconds() >= start) {
      response.reason = "MISSED_CUTOVER";
      return;
    }
    response.trajectory.times = candidate.times;
    to_wire(candidate.coeff_x, response.trajectory.coeff_x);
    to_wire(candidate.coeff_y, response.trajectory.coeff_y);
    to_wire(candidate.coeff_z, response.trajectory.coeff_z);
    response.success = true;
  }

  void on_authority(const WhoPlans& msg)
  {
    const bool requested = msg.value == WhoPlans::MADER;
    if (active_.exchange(requested) == requested) {
      return;
    }
    if (requested) {
      rclcpp::SubscriptionOptions io;
      io.callback_group = io_group_;
      const auto latched = rclcpp::QoS(1).reliable().transient_local();
      state_sub_ = create_subscription<State>("state", latched,
        [this](State::ConstSharedPtr msg) { on_state(*msg); }, io);
      term_sub_ = create_subscription<PoseStamped>("term_goal", 1,
        [this](PoseStamped::ConstSharedPtr msg) { on_target(*msg); }, io);
      send_ref_->reset();
      replan_->reset();
      RCLCPP_INFO(get_logger(), "MADER STARTED id=%d", id_);
    } else {
      state_sub_.reset();
      term_sub_.reset();
      send_ref_->cancel();
      replan_->cancel();
      std::scoped_lock lock(ref_mutex_, plan_mutex_);
      planner_.resetInitialization();
    }
  }

  void on_state(const State& msg)
  {
    if (!active_) {
      return;
    }
    mt::state state;
    state.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
    state.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
    state.setAccel(0, 0, 0);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(msg.quat.x, msg.quat.y, msg.quat.z, msg.quat.w)).getRPY(roll, pitch, yaw);
    state.setYaw(yaw);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      position_ = state.pos;
    }
    planner_.updateState(state);
    if (!initial_published_) {
      last_.times = {now().seconds(), now().seconds() + 1e10};
      last_.coeff_x.emplace_back(0.0, 0.0, 0.0, state.pos.x());
      last_.coeff_y.emplace_back(0.0, 0.0, 0.0, state.pos.y());
      last_.coeff_z.emplace_back(0.0, 0.0, 0.0, state.pos.z());
      publish(last_);
      initial_published_ = true;
    }
  }

  void on_target(const PoseStamped& msg)
  {
    if (active_) {
      const auto& p = msg.pose.position;
      mt::state goal;
      goal.setPos(p.x, p.y, std::abs(p.z) < 1e-5 ? 1.0 : p.z);
      planner_.setTerminalGoal(goal);
    }
  }

  struct PendingTrajectory
  {
    DynTraj::ConstSharedPtr message;
    double received;
  };

  void on_trajectory(const DynTraj::ConstSharedPtr& message)
  {
    const auto received = now().seconds();
    const auto& msg = *message;
    if (msg.id == id_) {
      return;
    }
    const auto& curve = msg.pwp;
    const auto segments = curve.coeff_x.size();
    if (msg.function.size() != 3 || msg.bbox.size() != 3) {
      RCLCPP_ERROR(get_logger(), "Malformed trajectory id=%d", msg.id);
      return;
    }
    if (msg.is_agent && (segments == 0 || curve.times.size() != segments + 1 ||
        curve.coeff_y.size() != segments || curve.coeff_z.size() != segments)) {
      RCLCPP_ERROR(get_logger(), "Malformed peer trajectory id=%d", msg.id);
      return;
    }
    if (explicit_handoff_) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!handoff_position_known_) {
        pending_trajectories_.insert_or_assign(msg.id, PendingTrajectory{message, received});
        return;
      }
      update_trajectory(msg, received, position_);
      return;
    }
    update_trajectory(msg, received, position());
  }

  void update_trajectory(const DynTraj& msg, double received, const Eigen::Vector3d& local_position)
  {
    const Eigen::Vector3d peer_position(msg.pos.x, msg.pos.y, msg.pos.z);
    if ((local_position - peer_position).norm() > 4 * par_.Ra) {
      return;
    }
    const auto& curve = msg.pwp;
    mt::dynTraj trajectory;
    trajectory.id = msg.id;
    trajectory.is_agent = msg.is_agent;
    trajectory.function = msg.function;
    trajectory.bbox << msg.bbox[0], msg.bbox[1], msg.bbox[2];
    trajectory.time_received = received;
    if (msg.is_agent) {
      trajectory.pwp.times = curve.times;
      from_wire(curve.coeff_x, trajectory.pwp.coeff_x);
      from_wire(curve.coeff_y, trajectory.pwp.coeff_y);
      from_wire(curve.coeff_z, trajectory.pwp.coeff_z);
      RCLCPP_INFO(get_logger(), "PEER_RECEIVED own=%d peer=%d begin=%.9f end=%.9f receive=%.9f",
        id_, msg.id, curve.times.front(), curve.times.back(), received);
    }
    planner_.updateTrajObstacles(trajectory, explicit_handoff_ ? &local_position : nullptr);
  }

  Eigen::Vector3d position()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return position_;
  }

  void publish(const mt::PieceWisePol& trajectory)
  {
    DynTraj msg;
    msg.function.resize(3);
    msg.bbox.assign(3, static_cast<float>(2 * par_.drone_radius));
    assign_xyz(msg.pos, position());
    msg.id = id_;
    msg.is_agent = true;
    msg.pwp.times = trajectory.times;
    to_wire(trajectory.coeff_x, msg.pwp.coeff_x);
    to_wire(trajectory.coeff_y, msg.pwp.coeff_y);
    to_wire(trajectory.coeff_z, msg.pwp.coeff_z);
    outgoing_->publish(msg);
  }

  void replan()
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (!active_ || !initial_published_) {
      return;
    }
    mt::Edges obstacles;
    std::vector<mt::state> plan;
    std::vector<Hyperplane3D> planes;
    mt::PieceWisePol trajectory;
    int linear_programs = 0;
    int quadratic_programs = 0;
    if (planner_.replan(obstacles, plan, planes, linear_programs, quadratic_programs, trajectory)) {
      publish(trajectory);
      last_ = std::move(trajectory);
    } else if (rebroadcast_.ElapsedMs() > 500.0) {
      publish(last_);
      rebroadcast_.Reset();
    }
  }

  void send_reference()
  {
    std::lock_guard<std::mutex> lock(ref_mutex_);
    mt::state reference;
    if (!active_ || !planner_.getNextGoal(reference)) {
      return;
    }
    Goal msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    assign_xyz(msg.p, reference.pos);
    assign_xyz(msg.v, reference.vel);
    assign_xyz(msg.a, reference.accel);
    assign_xyz(msg.j, reference.jerk);
    msg.psi = reference.yaw;
    msg.dpsi = reference.dyaw;
    msg.power = true;
    goals_->publish(msg);
  }

  const mt::parameters par_;
  const int id_;
  const bool explicit_handoff_;
  Mader planner_;
  rclcpp::Service<Propose>::SharedPtr proposal_;
  std::atomic<bool> active_{false}, initial_published_{false};
  std::mutex state_mutex_, ref_mutex_, plan_mutex_;
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  bool handoff_position_known_ = false;
  std::map<int, PendingTrajectory> pending_trajectories_;
  mt::PieceWisePol last_;
  MADER_timers::Timer rebroadcast_;
  rclcpp::CallbackGroup::SharedPtr io_group_, ref_group_, plan_group_;
  rclcpp::TimerBase::SharedPtr send_ref_, replan_;
  rclcpp::Publisher<Goal>::SharedPtr goals_;
  rclcpp::Publisher<DynTraj>::SharedPtr outgoing_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr term_sub_;
  rclcpp::Subscription<DynTraj>::SharedPtr peer_sub_, obstacle_sub_;
  rclcpp::Subscription<WhoPlans>::SharedPtr authority_sub_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3);
  auto node = std::make_shared<MaderNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}