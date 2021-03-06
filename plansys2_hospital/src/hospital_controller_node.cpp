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
    PatrollingController() : rclcpp::Node("patrolling_controller"), state_(STARTING) {}

    void init() {
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();

        init_knowledge();
    }

    void init_knowledge() {
        // Rooms, doors, zones and their connections
        for (int i = 1; i < 19; i++) {
            std::string room = "room" + std::to_string(i);
            std::string door = "door" + std::to_string(i);
            std::string zone = "zone" + std::to_string(i);
            problem_expert_->addInstance(plansys2::Instance{room, "room"});
            problem_expert_->addInstance(plansys2::Instance{door, "door"});
            problem_expert_->addInstance(plansys2::Instance{zone, "zone"});

            problem_expert_->addPredicate(
                plansys2::Predicate("(connected_through " + room + " " + zone + " " + door + ")"));
            problem_expert_->addPredicate(
                plansys2::Predicate("(connected_through " + zone + " " + room + " " + door + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(door_open " + door + ")"));
        }

        problem_expert_->addInstance(plansys2::Instance{"room19", "room"});
        problem_expert_->addInstance(plansys2::Instance{"room20", "room"});

        problem_expert_->addInstance(plansys2::Instance{"door19A", "door"});
        problem_expert_->addInstance(plansys2::Instance{"door19B", "door"});
        problem_expert_->addInstance(plansys2::Instance{"door20A", "door"});
        problem_expert_->addInstance(plansys2::Instance{"door20B", "door"});

        problem_expert_->addPredicate(plansys2::Predicate("(door_closed door19A)"));
        problem_expert_->addPredicate(plansys2::Predicate("(door_closed door19B)"));
        problem_expert_->addPredicate(plansys2::Predicate("(door_closed door20A)"));
        problem_expert_->addPredicate(plansys2::Predicate("(door_closed door20B)"));

        problem_expert_->addInstance(plansys2::Instance{"zone19A", "zone"});
        problem_expert_->addInstance(plansys2::Instance{"zone19B", "zone"});
        problem_expert_->addInstance(plansys2::Instance{"zone20A", "zone"});
        problem_expert_->addInstance(plansys2::Instance{"zone20B", "zone"});

        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through zone19A room19 door19A)"));
        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through room19 zone19A door19A)"));

        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through zone19B room19 door19B)"));
        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through room19 zone19B door19B)"));

        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through zone20A room20 door20A)"));
        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through room20 zone20A door20A)"));

        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through zone20B room20 door20B)"));
        problem_expert_->addPredicate(
            plansys2::Predicate("(connected_through room20 zone20B door20B)"));

        // Corridors
        for (int i = 1; i < 10; i++) {
            std::string corridor = "corridor" + std::to_string(i);
            problem_expert_->addInstance(plansys2::Instance{corridor, "corridor"});
        }

        // Connect corridors with the zones

        // --- Corridor1
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 zone11)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone11 corridor1)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 zone12)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone12 corridor1)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 zone20B)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone20B corridor1)"));

        // --- Corridor2
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 zone1)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone1 corridor2)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 zone2)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone2 corridor2)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 zone19B)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone19B corridor2)"));

        // --- Corridor4
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor4 zone13)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone13 corridor4)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor4 zone16)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone16 corridor4)"));

        // --- Corridor5
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor5 zone14)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone14 corridor5)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor5 zone17)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone17 corridor5)"));

        // --- Corridor6
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor6 zone15)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone15 corridor6)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor6 zone18)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone18 corridor6)"));

        // --- Corridor7
        for (int i = 3; i < 11; i++) {
            std::string zone = "zone" + std::to_string(i);
            problem_expert_->addPredicate(plansys2::Predicate("(connected corridor7 " + zone + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(connected " + zone + " corridor7)"));
        }

        // --- Corridor9
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor9 zone19A)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone19A corridor9)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor9 zone20A)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected zone20A corridor9)"));

        // Between corridors
        // -- Corridor1
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor3)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor3 corridor1)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor4)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor4 corridor1)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor5)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor5 corridor1)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor1 corridor6)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor6 corridor1)"));

        // -- Corridor2
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor3)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor3 corridor2)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor4)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor4 corridor2)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor5)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor5 corridor2)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor6)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor6 corridor2)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor2 corridor9)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor9 corridor2)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor7 corridor8)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor8 corridor7)"));

        // -- Corridor6
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor6 corridor7)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected corridor7 corridor6)"));

        // Robot
        problem_expert_->addInstance(plansys2::Instance{"kbot", "robot"});
        problem_expert_->addPredicate(plansys2::Predicate("(robot_at kbot corridor6)"));

        // Gripper
        problem_expert_->addInstance(plansys2::Instance{"gripper1", "gripper"});
        problem_expert_->addPredicate(plansys2::Predicate("(gripper_at gripper1 kbot)"));
        problem_expert_->addPredicate(plansys2::Predicate("(gripper_free gripper1)"));

        // Object
        problem_expert_->addInstance(plansys2::Instance{"ball", "object"});
        problem_expert_->addPredicate(plansys2::Predicate("(object_at ball room3)"));
    }

    void step() {
        switch (state_) {
            case STARTING: {
                // Set the goal for next state
                problem_expert_->setGoal(plansys2::Goal("(and(robot_at kbot corridor7))"));

                // Compute the plan
                auto domain = domain_expert_->getDomain();
                auto problem = problem_expert_->getProblem();
                auto plan = planner_client_->getPlan(domain, problem);

                if (!plan.has_value()) {
                    std::cout << "Could not find plan to reach goal "
                              << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                    break;
                }

                // Execute the plan
                if (executor_client_->start_plan_execution(plan.value())) {
                    state_ = GO_TO_ROOM;
                }
            } break;
            case GO_TO_ROOM: {
                auto feedback = executor_client_->getFeedBack();

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                    if (executor_client_->getResult().value().success) {
                        std::cout << "Successful finished " << std::endl;

                        // Set the goal for next state
                        std::string room = "room" + std::to_string(counter);
                        problem_expert_->setGoal(plansys2::Goal("(and(robot_at kbot " + room + "))"));

                        std::cout << "(and(robot_at kbot " + room + "))" << std::endl;

                        // Compute the plan
                        auto domain = domain_expert_->getDomain();
                        auto problem = problem_expert_->getProblem();
                        auto plan = planner_client_->getPlan(domain, problem);

                        if (!plan.has_value()) {
                            std::cout << "Could not find plan to reach goal "
                                      << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                            break;
                        }

                        // Execute the plan
                        if (executor_client_->start_plan_execution(plan.value())) {
                            state_ = PICK_OBJECT;
                        }
                    } else {
                        for (const auto& action_feedback : feedback.action_execution_status) {
                            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                                std::cout << "[" << action_feedback.action
                                          << "] finished with error: " << action_feedback.message_status
                                          << std::endl;
                            }
                        }

                        // Replan
                        auto domain = domain_expert_->getDomain();
                        auto problem = problem_expert_->getProblem();
                        auto plan = planner_client_->getPlan(domain, problem);

                        if (!plan.has_value()) {
                            std::cout << "Unsuccessful replan attempt to reach goal "
                                      << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                            break;
                        }

                        // Execute the plan
                        executor_client_->start_plan_execution(plan.value());
                    }
                }
            } break;
            case PICK_OBJECT: {
                auto feedback = executor_client_->getFeedBack();

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                    if (executor_client_->getResult().value().success) {
                        std::cout << "Successful finished " << std::endl;

                        // Set the goal for next state
                        std::string room = "room" + std::to_string(counter + 1);
                        problem_expert_->setGoal(plansys2::Goal("(and(object_at ball " + room + "))"));

                        std::cout << "(and(object_at ball " + room + "))" << std::endl;

                        // Compute the plan
                        auto domain = domain_expert_->getDomain();
                        auto problem = problem_expert_->getProblem();
                        auto plan = planner_client_->getPlan(domain, problem);

                        if (!plan.has_value()) {
                            std::cout << "Could not find plan to reach goal "
                                      << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                            break;
                        }

                        // Execute the plan
                        if (executor_client_->start_plan_execution(plan.value())) {
                            state_ = CLOSE_DOOR;
                        }
                    } else {
                        for (const auto& action_feedback : feedback.action_execution_status) {
                            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                                std::cout << "[" << action_feedback.action
                                          << "] finished with error: " << action_feedback.message_status
                                          << std::endl;
                            }
                        }

                        // Replan
                        auto domain = domain_expert_->getDomain();
                        auto problem = problem_expert_->getProblem();
                        auto plan = planner_client_->getPlan(domain, problem);

                        if (!plan.has_value()) {
                            std::cout << "Unsuccessful replan attempt to reach goal "
                                      << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                            break;
                        }

                        // Execute the plan
                        executor_client_->start_plan_execution(plan.value());
                    }
                }
            } break;
            case CLOSE_DOOR: {
                auto feedback = executor_client_->getFeedBack();

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                    if (executor_client_->getResult().value().success) {
                        std::cout << "Successful finished " << std::endl;

                        // Set the goal for next state
                        std::string door = "door" + std::to_string(counter);
                        problem_expert_->setGoal(plansys2::Goal("(and(door_closed " + door + "))"));

                        std::cout << "(and(door_closed" + door + "))" << std::endl;

                        // Compute the plan
                        auto domain = domain_expert_->getDomain();
                        auto problem = problem_expert_->getProblem();
                        auto plan = planner_client_->getPlan(domain, problem);

                        if (!plan.has_value()) {
                            std::cout << "Could not find plan to reach goal "
                                      << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                            break;
                        }

                        // Execute the plan
                        if (executor_client_->start_plan_execution(plan.value())) {
                            state_ = GO_TO_ROOM;
                            counter++;
                            if (counter > 18) {
                                counter = 1;
                            }
                        }
                    } else {
                        for (const auto& action_feedback : feedback.action_execution_status) {
                            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                                std::cout << "[" << action_feedback.action
                                          << "] finished with error: " << action_feedback.message_status
                                          << std::endl;
                            }
                        }

                        // Replan
                        auto domain = domain_expert_->getDomain();
                        auto problem = problem_expert_->getProblem();
                        auto plan = planner_client_->getPlan(domain, problem);

                        if (!plan.has_value()) {
                            std::cout << "Unsuccessful replan attempt to reach goal "
                                      << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                            break;
                        }

                        // Execute the plan
                        executor_client_->start_plan_execution(plan.value());
                    }
                }
            } break;
        }
    }

   private:
    typedef enum { STARTING,
                   GO_TO_ROOM,
                   PICK_OBJECT,
                   CLOSE_DOOR } StateType;
    StateType state_;
    int counter = 3;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;

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
