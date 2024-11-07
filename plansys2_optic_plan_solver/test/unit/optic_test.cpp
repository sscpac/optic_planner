#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_optic_plan_solver/optic_plan_solver.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "plansys2_core/PlanSolverBase.hpp"

void test_plan_generation(const std::string & argument = "")
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_optic_plan_solver");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str((
      std::istreambuf_iterator<char>(problem_ifs)),
    std::istreambuf_iterator<char>());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto planner = std::make_shared<plansys2::OPTICPlanSolver>();
  planner->configure(node, "OPTIC");
  node->set_parameter(rclcpp::Parameter("OPTIC.arguments", argument));

  auto plan = planner->getPlan(domain_str, problem_str, "generate_plan_good");

  ASSERT_TRUE(plan);
  ASSERT_EQ(plan.value().items.size(), 3);
  ASSERT_EQ(plan.value().items[0].action, "(move leia kitchen bedroom)");
  ASSERT_EQ(plan.value().items[1].action, "(approach leia bedroom jack)");
  ASSERT_EQ(plan.value().items[2].action, "(talk leia jack jack m1)");
}

TEST(optic_plan_solver, generate_plan_good)
{
  test_plan_generation();
}

TEST(optic_plan_solver, load_optic_plugin)
{
  try {
    pluginlib::ClassLoader<plansys2::PlanSolverBase> lp_loader(
      "plansys2_core", "plansys2::PlanSolverBase");
    plansys2::PlanSolverBase::Ptr plugin =
      lp_loader.createUniqueInstance("plansys2/OPTICPlanSolver");
    ASSERT_TRUE(true);
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
    ASSERT_TRUE(false);
  }
}

// TEST(optic_plan_solver, check_1_ok_domain)
// {
//   std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_optic_plan_solver");
//   std::ifstream domain_ifs(pkgpath + "/pddl/domain_1_ok.pddl");
//   std::string domain_str((
//       std::istreambuf_iterator<char>(domain_ifs)),
//     std::istreambuf_iterator<char>());

//   auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
//   auto planner = std::make_shared<plansys2::OPTICPlanSolver>();
//   planner->configure(node, "OPTIC");

//   auto result = planner->check_domain(domain_str, "check_1_ok_domain");

//   ASSERT_TRUE(result.empty());
// }

// TEST(optic_plan_solver, check_2_error_domain)
// {
//   std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_optic_plan_solver");
//   std::ifstream domain_ifs(pkgpath + "/pddl/domain_2_error.pddl");
//   std::string domain_str((
//       std::istreambuf_iterator<char>(domain_ifs)),
//     std::istreambuf_iterator<char>());

//   auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
//   auto planner = std::make_shared<plansys2::OPTICPlanSolver>();
//   planner->configure(node, "OPTIC");

//   auto result = planner->check_domain(domain_str, "check_2_error_domain");

//   ASSERT_FALSE(result.empty());
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
