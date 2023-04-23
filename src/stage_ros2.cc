#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "stage/stage.hh"
#include "stage_ros2/camera_wrapper.hpp"
#include "stage_ros2/position_wrapper.hpp"
#include "stage_ros2/ranger_wrapper.hpp"
#include "stage_ros2/robot_wrapper.hpp"

class StageWrapper
{
public:
  void init(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::uninstall_signal_handlers();
    Stg::Init(&argc, &argv);

    node_ = rclcpp::Node::make_shared("stage_ros2");
    auto context = node_->get_node_base_interface()->get_context();

    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
    node_->declare_parameter<bool>("gui");
    auto gui = node_->get_parameter("gui");
    node_->declare_parameter<std::string>("world");
    auto world_file = node_->get_parameter("world");

    executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
    executor_->add_node(node_);

    world_ = gui.get_value<bool>() ? std::make_shared<Stg::WorldGui>(600, 400, "stage_ros2")
                                   : std::make_shared<Stg::World>();

    world_->Load(world_file.get_value<std::string>());
    world_->AddUpdateCallback(
      [](Stg::World * world, void * user) {
        return static_cast<StageWrapper *>(user)->world_callback(world);
      },
      this);
    world_->ForEachDescendant(
      [](Stg::Model * mod, void * user) {
        return static_cast<StageWrapper *>(user)->search_and_init_robot(mod);
      },
      this);
    world_->Start();
    ros2_thread_ = std::make_shared<std::thread>([this]() { executor_->spin(); });
  }

  ~StageWrapper() { rclcpp::shutdown(); }

private:
  std::shared_ptr<std::thread> ros2_thread_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::shared_ptr<Stg::World> world_;
  std::unordered_map<Stg::Model *, std::shared_ptr<RobotWrapper>> robots_;

  int search_and_init_robot(Stg::Model * mod)
  {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "[search robots] token: " << mod->Token() << ", type: " << mod->GetModelType());
    if (mod->GetModelType() == "position") {
      robots_[mod] = std::make_shared<RobotWrapper>(executor_, mod->Token());
      robots_[mod]->wrap(mod);
      mod->ForEachDescendant(
        [](Stg::Model * mod, void * user) {
          return static_cast<StageWrapper *>(user)->search_and_init_sensor(mod);
        },
        this);
    }
    return 0;
  }

  int search_and_init_sensor(Stg::Model * mod)
  {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "[search sensors] token: " << mod->Token() << ", type: " << mod->GetModelType());
    robots_[mod->Parent()]->wrap(mod);
    return 0;
  }

  int world_callback(Stg::World * world_)
  {
    rosgraph_msgs::msg::Clock clock;
    rclcpp::Time now(world_->SimTimeNow() * 1000);
    clock.clock = now;
    clock_pub_->publish(clock);

    for (auto & pair : robots_) {
      auto robot = pair.second;
      robot->publish(now);
    }
    // We return false to indicate that we want to be called again (an
    // odd convention, but that's the way that Stage works).
    return 0;
  }
};

int main(int argc, char ** argv)
{
  StageWrapper stage_wrapper;
  stage_wrapper.init(argc, argv);
  Stg::World::Run();
  int i = 1;
  while (true) {
    i++;
  }

  rclcpp::shutdown();
  std::cerr << "EXITTTT" << i << std::endl;
  return 0;
}
