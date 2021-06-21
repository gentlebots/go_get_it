#include "GoGetIt_executor.hpp"

GoGetIt_executor::GoGetIt_executor() 
{
  node_ = rclcpp::Node::make_shared("gogetit_graph");
  init();
}

void GoGetIt_executor::init()
{
  graph_ = ros2_knowledge_graph::GraphFactory::getInstance(node_);
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  initKnowledge();
}

void GoGetIt_executor::initKnowledge() 
{
  problem_expert_->addInstance(plansys2::Instance{"jarvis", "robot"});
  problem_expert_->addInstance(plansys2::Instance{"object1", "object"});
  problem_expert_->addPredicate(plansys2::Predicate("(is_free jarvis)"));
  problem_expert_->addPredicate(plansys2::Predicate("(is_target object1)"));

  // mfc : faltaría añadir:
  //  - Qué objeto buscamos: target
  //  - A qué persona se la entregamos
}

std::string 
GoGetIt_executor::statusToString(int8_t status) {
    switch (status)
    {
    case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
      return "NOT_EXECUTED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
      return "EXECUTING";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
      return "FAILED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
      return "SUCCEEDED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
      return "CANCELLED";
      break;
    default:
      return "UNKNOWN";
      break;
    }
}

bool
GoGetIt_executor::getResult()
{
  RCLCPP_INFO(get_logger(), "========================= PLAN FINISHED =========================");
  auto result = executor_client_->getResult();
  if (result.has_value()) {
    RCLCPP_INFO_STREAM(get_logger(), "Plan succesful: " << result.value().success);
    for (const auto & action_info : result.value().action_execution_status) 
    {
      std::string args;
      rclcpp::Time start_stamp = action_info.start_stamp;
      rclcpp::Time status_stamp = action_info.status_stamp;
      for (const auto & arg : action_info.arguments) 
      {
        args = args + " " + arg;
      } 
      RCLCPP_INFO_STREAM(get_logger(), "Action: " << action_info.action << args << " " << statusToString(action_info.status) << " " << (status_stamp - start_stamp).seconds() << " secs");
    }

    return result.value().success;
  } else {
    RCLCPP_WARN(get_logger(), "No result for this plan");
  }
  return -1;
}

void GoGetIt_executor::Init_code_iterative()
{
  if (!executor_client_->execute_and_check_plan()) 
  {
    //plan_result_ = getResult();
  }
}

void GoGetIt_executor::Init_code_once()
{
  init();
  succesful_plan_ = false;
  problem_expert_->clearGoal();
  problem_expert_->setGoal(plansys2::Goal("(and(is_delivered object1))"));

  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (plan.has_value()) {
    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
  } else {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),"Could not find plan to reach goal " <<
      parser::pddl::toString(problem_expert_->getGoal()));
  }
}

// Search
void GoGetIt_executor::SearchObject_code_iterative()
{
  return;
}

void GoGetIt_executor::SearchObject_code_once()
{
  return;
}

// Pick
void GoGetIt_executor::PickObject_code_iterative()
{
  return;
}

void GoGetIt_executor::PickObject_code_once()
{
  return;
}

// Deliver
void GoGetIt_executor::DeliverObject_code_iterative()
{
  return;
}

void GoGetIt_executor::DeliverObject_code_once()
{
  return;
}

bool GoGetIt_executor::Init_2_SearchObject()
{
  return false;
}

bool GoGetIt_executor::DeliverObject_2_SearchObject()
{
  return false;
}

bool GoGetIt_executor::PickObject_2_DeliverObject()
{
  return false;
}

bool GoGetIt_executor::SearchObject_2_PickObject()
{
  return false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  GoGetIt_executor node;
  node.init();
  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node.get_node_base_interface());
  }
  rclcpp::shutdown();
  return 0;
}
