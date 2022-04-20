// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node {
   public:
    PatrollingController()
        : rclcpp::Node("patrolling_controller"), state_(STARTING) {
    }

    void init() {
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();

        init_knowledge();
    }

    void init_knowledge() {
        problem_expert_->addInstance(plansys2::Instance{"kbot", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"room1", "room"});
        problem_expert_->addInstance(plansys2::Instance{"object1", "object"});
        problem_expert_->addInstance(plansys2::Instance{"gripper1", "gripper"});

        problem_expert_->addPredicate(plansys2::Predicate("(gripper_at gripper1 kbot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(gripper_free gripper1)"));

        problem_expert_->addPredicate(plansys2::Predicate("(robot_at kbot room1)"));
        problem_expert_->addPredicate(plansys2::Predicate("(object_at object1 room1)"));
    }

    void step() {
      if (!ejecutado){
        // Set the goal for next state
        problem_expert_->setGoal(plansys2::Goal("(and(robot_carry kbot object1))"));

        // Compute the plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }

        // Execute the plan
        if (executor_client_->start_plan_execution(plan.value())) {
            std::cout << "plan ejecutado" << std::endl;
            state_ = PATROL_WP1;
        }
        ejecutado = true;
      }
    }

   private:
    typedef enum { STARTING,
                   PATROL_WP1,
                   PATROL_WP2,
                   PATROL_WP3,
                   PATROL_WP4 } StateType;
    StateType state_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;

    bool ejecutado = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrollingController>();

    node->init();

    rclcpp::Rate rate(5);
    while (rclcpp::ok()) {
        node->step();

        rate.sleep();
        rclcpp::spin_some(node->get_node_base_interface());
    }

    rclcpp::shutdown();

    return 0;
}
