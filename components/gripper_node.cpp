#include "ecn_baxter/gripper/gripper_node.hpp"
namespace ECNBaxter {
    /*
     * ========================================
     *               Gripper Node
     *               Implementing
     * ========================================
     */
    GripperNode::GripperNode(const rclcpp::NodeOptions& options) : Node("gripper", options) {
        // Loading the right gripper
        _side = declare_parameter<std::string>("gripper_side", "left");
        this->topics_init();

        cmd = EndEffectorCommand();

        // TOPIC IO Making
        act_sub = create_subscription<BaxterAction>(
                ACTION_TOPIC,
                QOS,
                [this](BaxterAction::UniquePtr msg) {
                    // Check if action destined to this gripper
                    if (msg->component == _side + "_gripper")
                        this->action_process(msg);
                });
        state_sub = create_subscription<EndEffectorState>(
                STATE_TOPIC,
                QOS,
                [this](EndEffectorState::UniquePtr msg) {
                    this->state_process(msg);
                });
        cmd_pub = create_publisher<EndEffectorCommand>(CMD_TOPIC, QOS);
    }

    /**
     * Init the topics according to the chosen _side
     */
    void GripperNode::topics_init() {
        if (_side == "left" || _side == "right") {
            CMD_TOPIC = "/robot/end_effector/" + _side + "_gripper/command";
            STATE_TOPIC = "/robot/end_effector/" + _side + "_gripper/state";
        }
        // If not a _side, kill the node
        else {
            RCLCPP_FATAL(get_logger(), "Parameter 'gripper_side' should be 'right' or 'left', was instead %s", _side.c_str());
            exit(-1);
        }
    }

    /**
     * Process the received action to the gripper
     * @param msg the received message
     */
    void GripperNode::action_process(const BaxterAction::UniquePtr &msg) {
        // Check if gripper properties are set and calibrated before launching any action
        if (!_initialized || !_calibrated)
            return;
        action(msg->action);
    }

    /**
     * Process the gripper latest_state
     * @param msg the received latest_state
     */
    void GripperNode::state_process(const EndEffectorState::UniquePtr &msg) {
        // Latest
        RCLCPP_INFO(get_logger(), "Timestamp %d", msg->timestamp.sec);
        latest_cmd = msg->command;
        latest_state_src = msg->state;

        if (!_initialized)
            this->initialize(msg);
        _calibrated = msg->calibrated;
    }

    /**
     * Set basic gripper properties
     * @param msg the latest_state of the gripper
     */
    void GripperNode::initialize(const EndEffectorState::UniquePtr &msg) {
        RCLCPP_INFO(get_logger (), "Initializing %s gripper", _side.c_str());
        cmd.id = msg->id;

        // Set gripper type
        parse_state();
        _type = (latest_state.HasMember("vacuum")) ? VACUUM : CLAMP;
        switch (_type) {
            case VACUUM:
                RCLCPP_INFO(get_logger(), "%s_gripper is a vacuum one !", _side.c_str());
                checker = AsyncTimer([this](){
                    vacuum_check();
                }, timer_duration);
                break;
            case CLAMP:
                RCLCPP_INFO(get_logger(), "%s_gripper is a clamp one !", _side.c_str());
                break;
        };

        reset();
        sleep_for(500ms);
        calibrate();
        _initialized = true;
    }

    /**
     * Decoding the latest json latest_state
     * @param state
     */
    void GripperNode::parse_state() {
        latest_state.Parse(&latest_state_src[0]);

        if (latest_state.HasParseError()) {
            RCLCPP_FATAL(get_logger(), "%s_gripper has a JSON parsing error.", _side.c_str());
        }
    }

    void GripperNode::vacuum_check() {
        // If we grip, we check
        if (latest_cmd != "grip")
            return;
        // Waiting for the suction to appear

        sleep_for(check_delay);

        while (latest_cmd != "release") {
            parse_state();
            if (latest_state.HasMember(VACUUM_SENSOR) && latest_state.HasMember(VACUUM_THRESHOLD)) {
                unsigned char sensor = latest_state[VACUUM_SENSOR].GetUint();
                unsigned char threshold = latest_state[VACUUM_THRESHOLD].GetUint();

                RCLCPP_INFO(get_logger(), "Vacuum : %1u / %2u", sensor, threshold);

                if (sensor < threshold) {
                    release();
                }
            }
            sleep_for(timer_loop_time);
        }
    }

    void GripperNode::action(const std::string &action_id) {
        cmd.command = action_id;
        cmd_pub->publish(cmd);
    }
}

/*
 * ========================================
*               Launching Node
 * ========================================
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ECNBaxter::GripperNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}