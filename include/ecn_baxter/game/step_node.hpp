#ifndef BUILD_STEP_NODE_HPP
#define BUILD_STEP_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>

namespace ecn_baxter {

    using namespace rclcpp;
    using namespace std::chrono_literals;
    using namespace std::chrono;

    // Check & action functions
    using cfunc = std::function<bool(void)>;
    using afunc = std::function<void(void)>;

    template <typename E>
    class StepNode : public Node {
    public:
        explicit StepNode(const std::string &name, NodeOptions &opts) : Node(name, opts) {
            step = static_cast<E>(0);
            timer = create_wall_timer(sample_time, std::bind(&StepNode<E>::iterate_step, this));
        }
    private:
        // At init time, the robot arm is idling
        E step;
        TimerBase::SharedPtr timer;
        milliseconds sample_time = 100ms;

        E check(E actual_step) {
            if (const auto it = check_map.find(actual_step); it != check_map.end()) {
                auto f = check_map.at(actual_step);
                if (f())
                    increment_step();
            }
            return actual_step;
        }
        void action(E actual_step) {
            if (const auto it = check_map.find(actual_step); it != check_map.end()) {
                auto f = check_map.at(actual_step);
                f();
            }
        }

        void iterate_step() {
            step = check(step);
            action(step);
        }
    protected:
        virtual E check_step(E act_step) = 0;
        inline void increment_step() { step = check_step(step); }
        std::map<E, cfunc> check_map = std::map<E, cfunc>();
        std::map<E, afunc> action_map = std::map<E, afunc>();
    };
}

#endif //BUILD_STEP_NODE_HPP
