#include <yasmin_ros/basic_outcomes.hpp>

#include "guide_dog_mission/main_fsm.hpp"
#include "guide_dog_mission/states/idle_state.hpp"
#include "guide_dog_mission/states/patrol_state.hpp"
#include "guide_dog_mission/states/scan_state.hpp"
#include "guide_dog_mission/states/align_state.hpp"
#include "guide_dog_mission/states/greet_state.hpp"
#include "guide_dog_mission/states/guide_state.hpp"
#include "guide_dog_mission/states/arrive_state.hpp"

GuideDogMission::GuideDogMission() : Node("guide_dog_mission_node") 
{
    setup_state_machine();
}

void GuideDogMission::run()
{
    auto blackboard = std::make_shared<yasmin::Blackboard>();
    RCLCPP_INFO(this->get_logger(), "Booting up Guide Dog FSM...");
    std::string outcome = main_sm_->execute(blackboard);
    RCLCPP_INFO(this->get_logger(), "FSM finished with outcome: %s", outcome.c_str());
}

void GuideDogMission::setup_state_machine() 
{ 
    // Passing SUCCEED and ABORT as the final outcomes of the ENTIRE machine.
    main_sm_ = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::ABORT}
    );

    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    // 1. IDLE
    main_sm_->add_state("IDLE", std::make_shared<IdleState>(node_ptr), {
        {"START", "PATROL"}
    });

    // 2. PATROL
    main_sm_->add_state("PATROL", std::make_shared<PatrolState>(node_ptr), {
        {yasmin_ros::basic_outcomes::SUCCEED, "SCAN"},     // Reached waypoint safely
        {yasmin_ros::basic_outcomes::ABORT, "PATROL"},     // Blocked! Try the next waypoint
        {yasmin_ros::basic_outcomes::CANCEL, "ALIGN"}      // Face detected mid-walk!
    });

    // 3. SCAN
    main_sm_->add_state("SCAN", std::make_shared<ScanState>(node_ptr), {
        {"FACE_DETECTED", "ALIGN"},
        {"NO_FACE", "PATROL"}                               // 360 complete, no one here. Keep walking.
    });

    // 4. ALIGN
    main_sm_->add_state("ALIGN", std::make_shared<AlignState>(node_ptr), {
        {"ALIGNED", "GREET"},
        {"LOST_FACE", "SCAN"}                               // Person moved away, spin to find them
    });

    // 5. GREET
    main_sm_->add_state("GREET", std::make_shared<GreetState>(node_ptr), {
        {"SPOKEN", "GUIDE"},
        {"FAILED", "GUIDE"}                                 // If audio fails, just guide them anyway
    });

    // 6. GUIDE
    main_sm_->add_state("GUIDE", std::make_shared<GuideState>(node_ptr), {
        {yasmin_ros::basic_outcomes::SUCCEED, "ARRIVE"},    // Reached the Lab!
        {yasmin_ros::basic_outcomes::ABORT, "IDLE"},        // Hallway completely blocked, abort mission.
        {yasmin_ros::basic_outcomes::CANCEL, "IDLE"}
    });

    // 7. ARRIVE
    main_sm_->add_state("ARRIVE", std::make_shared<ArriveState>(node_ptr), {
        {"DONE", "IDLE"}                                    // Mission complete, wait for next start command
    });
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuideDogMission>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spin_thread([&executor]() { executor.spin(); });
    node->run();         
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}