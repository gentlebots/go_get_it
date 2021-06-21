// NOLINT (legal/copyright)

#include "GoGetItHFSM.hpp"

namespace cascade_hfsm
{
GoGetItHFSM::GoGetItHFSM()
: CascadeLifecycleNode("GoGetItHFSM"), state_(INIT), myBaseId_("GoGetItHFSM")
{
  declare_parameter("frequency");

  state_ts_ = now();
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
}

GoGetItHFSM::~GoGetItHFSM()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GoGetItHFSM::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  deactivateAllDeps();

  state_ = INIT;
  state_ts_ = now();

  Init_activateDeps();
  Init_code_once();


  double frequency = 5.0;
  get_parameter_or<double>("frequency", frequency, 5.0);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double, std::ratio<1>>(1.0 / frequency),
    std::bind(&GoGetItHFSM::tick, this));

  state_pub_->on_activate();
  return CascadeLifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GoGetItHFSM::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  loop_timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(previous_state);
}

void GoGetItHFSM::tick()
{
  std_msgs::msg::String msg;

  switch (state_) {
    case SEARCHOBJECT:
      SearchObject_code_iterative();

      msg.data = "SearchObject";
      state_pub_->publish(msg);

      if (SearchObject_2_PickObject()) {
        deactivateAllDeps();

        state_ = PICKOBJECT;
        state_ts_ = now();

        PickObject_activateDeps();
        PickObject_code_once();
      }
      break;
    case PICKOBJECT:
      PickObject_code_iterative();

      msg.data = "PickObject";
      state_pub_->publish(msg);

      if (PickObject_2_DeliverObject()) {
        deactivateAllDeps();

        state_ = PLACEOBJECT;
        state_ts_ = now();

        DeliverObject_activateDeps();
        DeliverObject_code_once();
      }
      break;
    case DELIVEROBJECT:
      DeliverObject_code_iterative();

      msg.data = "DeliverObject";
      state_pub_->publish(msg);

      if (DeliverObject_2_SearchObject()) {
        deactivateAllDeps();

        state_ = SEARCHOBJECT;
        state_ts_ = now();

        SearchObject_activateDeps();
        SearchObject_code_once();
      }
      break;
    case INIT:
      Init_code_iterative();

      msg.data = "Init";
      state_pub_->publish(msg);

      if (Init_2_SearchObject()) {
        deactivateAllDeps();

        state_ = SEARCHOBJECT;
        state_ts_ = now();

        SearchObject_activateDeps();
        SearchObject_code_once();
      }
      break;
  }
}

void
GoGetItHFSM::deactivateAllDeps()
{
  remove_activation("vision");
  remove_activation("attention");
}

void
GoGetItHFSM::SearchObject_activateDeps()
{
  add_activation("vision");
  add_activation("attention");
}
void
GoGetItHFSM::PickObject_activateDeps()
{
  add_activation("attention");
}
void
GoGetItHFSM::DeliverObject_activateDeps()
{
  add_activation("attention");
}
void
GoGetItHFSM::Init_activateDeps()
{
}


}  // namespace cascade_hfsm
