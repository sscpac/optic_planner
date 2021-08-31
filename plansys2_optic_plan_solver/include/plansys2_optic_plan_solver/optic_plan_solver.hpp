#ifndef PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_
#define PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_

#include <optional>
#include <memory>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"

namespace plansys2
{

class OPTICPlanSolver : public PlanSolverBase
{
private:
  std::string parameter_name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;

public:
  OPTICPlanSolver();

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &);

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

  std::string check_domain(
    const std::string & domain,
    const std::string & node_namespace = "");
};

}  // namespace plansys2

#endif  // PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_
