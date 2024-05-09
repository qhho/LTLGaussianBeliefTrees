#include "agent_gbt/GBT.h"
#include <chrono>
#include <functional>
#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <fstream>
#include <filesystem>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

ob::StateSpacePtr constructStateSpace(void)
{
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new R2BeliefSpace(2.0));
    return state_space;
}

ob::StateSpacePtr constructRealVectorStateSpace(void)
{
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
    return state_space;
}

class MyGoalRegion : public ompl::base::GoalRegion
{
    public:
        MyGoalRegion(const oc::SpaceInformationPtr &si) : ompl::base::GoalRegion(si)
        {
            setThreshold(16.0);
        }
    
        virtual double distanceGoal(const State *st) const
            {

            double dx = st->as<R2BeliefSpace::StateType>()->getX() - 90.0;
            double dy = st->as<R2BeliefSpace::StateType>()->getY() - 90.0;

            return (dx*dx + dy*dy);
    }
};


void GBT::SaveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string stringpath)
{   
    std::ofstream outputsolution;

    outputsolution.open(stringpath, std::ios::out | std::ios::trunc);
    outputsolution << "x,y,cov" << std::endl;
    
    std::vector<ob::State*> path_control_states;
    path_control_states = path_control.getStates();

    for (size_t i = path_control_states.size()-1; i > 0; i--)
    {
        ob::State *s = space->allocState();
        space->copyState(s, path_control_states[i]);
        path_states_.push_back(s);

        double x_pose = s->as<R2BeliefSpace::StateType>()->getX();
        double y_pose = s->as<R2BeliefSpace::StateType>()->getY();
        Mat cov = s->as<R2BeliefSpace::StateType>()->getCovariance();

        outputsolution << x_pose << ","  << y_pose << "," <<  cov.trace() << std::endl;

    }
    outputsolution << "10.0,10.0,10.0" << std::endl;

    outputsolution.close();

}

std::string GBT::planWithSimpleSetup()
{
    std::cout<<"TEST"<<std::endl;
    ob::StateSpacePtr space(constructStateSpace()); //space should probably be a class attribute
    ob::RealVectorBounds bounds_se_n(scene_.scene_bounds_.size());
    for(int i = 0; i < scene_.scene_bounds_.size(); i++)
    {
        bounds_se_n.setLow(i, scene_.scene_bounds_[i].first);
        bounds_se_n.setHigh(i, scene_.scene_bounds_[i].second);
    }
    space->as<R2BeliefSpace>()->setBounds(bounds_se_n); //TODO: currently setup for se2 how to make se^n?
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, system_.control_bounds_.size()));
    ob::RealVectorBounds bounds(system_.control_bounds_.size());
    for(int i = 0; i < system_.control_bounds_.size(); i++)
    {
        bounds.setLow(i, system_.control_bounds_[i].first);
        bounds.setHigh(i, system_.control_bounds_[i].second);
    }
    cspace->setBounds(bounds);
    oc::SpaceInformationPtr si(new oc::SpaceInformation(space, cspace));
    si->setMinMaxControlDuration(system_.control_duration_.first, system_.control_duration_.second);
    si->setPropagationStepSize(system_.propagation_size_);

    ob::PlannerPtr planner;
    planner = ob::PlannerPtr(new oc::SSBT(si));
    planner->as<oc::SSBT>()->setGoalBias(system_.goal_bias_);
    planner->as<oc::SSBT>()->setSelectionRadius(system_.selection_radius_);
    planner->as<oc::SSBT>()->setPruningRadius(system_.pruning_radius_);
    planner->as<oc::SSBT>()->setSamplingBias(system_.sampling_bias_);
    planner->as<oc::SSBT>()->setDistanceFunction(1);
    
    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    // create a start state
    ob::ScopedState<> start(space);
    for(int i = 0; i < system_.starting_configuration_.size(); i++)
    {
        start[i] = system_.starting_configuration_[i];
    }
    // create a goal state
    ob::ScopedState<> goal(space);
    for(int i = 0; i < system_.goal_configuration_.size(); i++)
    {
        goal[i] = system_.goal_configuration_[i];
    }

    si->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si, system_.Q_,system_.R_)));

    ob::StateValidityCheckerPtr om_stat_val_check;
    // om_stat_val_check = ob::StateValidityCheckerPtr(new Scenario2ValidityChecker(si));
    om_stat_val_check = ob::StateValidityCheckerPtr(new StateValidityCheckerBlackmore(scene_, si, system_.p_safe_));
    // simple_setup_->setStateValidityChecker(om_stat_val_check);
    si->setStateValidityChecker(om_stat_val_check);

    si->setup();

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<MyGoalRegion>(si));
    pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));
    pdef->getOptimizationObjective()->setCostThreshold(ob::Cost(system_.cost_threshold_));

    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->solve(system_.planning_time_);

    ob::PlannerData planner_data(si);
    planner->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    std::ofstream outputfile;
    outputfile.open("fixedK_03_tree_60sec.csv", std::ios::out | std::ios::trunc);
    outputfile << "to,from,x,y,cov,cost" << std::endl;
    
    for(unsigned int i = 1; i < planner_data.numVertices(); ++i) {

        double x = planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getX();
        double y = planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getY();
        Mat cov = planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getCovariance();
        if (planner_data.getIncomingEdges(i, edgeList) > 0){
            outputfile << i << "," << edgeList[0] << "," << x << "," << y << "," << cov.trace() << "," << planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getCost() << std::endl; 
        }
    }

    outputfile.close();

    if (planner->getProblemDefinition()->hasExactSolution()){
        const ompl::base::PathPtr &path_5sec = planner->getProblemDefinition()->getSolutionPath(); 
        oc::PathControl path_control = static_cast<oc::PathControl&>(*path_5sec);
        this->SaveSolutionPath(path_control, space, "solution_60sec_fixedK_03.csv");
        return "solution_60sec_fixedK_03.csv";
    }
    else{
        std::cout << "No solution for 10 secs" << std::endl;
    }
}

void GBT::readAgentPositions(const std::string& solution_path) {
    int endPadding = 100; // Number of end states so rviz does not repeat immediately. 
    int interpVal = 20; // Number of interpolation values between each point.
    std::ifstream file(solution_path);
    std::string line;
    std::getline(file, line);
    Eigen::Vector3d lastPos(
        system_.starting_configuration_.size() > 0 ? system_.starting_configuration_[0] : 0.0,
        system_.starting_configuration_.size() > 1 ? system_.starting_configuration_[1] : 0.0,
        system_.starting_configuration_.size() > 2 ? system_.starting_configuration_[2] : 0.0
    );

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<double> row;

        while (std::getline(iss, token, ',')) {
            row.push_back(std::stod(token));
        }
        double z = row.size() > 2 ? row[2] : 0.0;
        Eigen::Vector3d newPos(row[0], row[1], z);
        for (int i = 1; i <= interpVal; ++i) {
            double t = i / double(interpVal);
            Eigen::Vector3d interpolatedPos = lastPos + t * (newPos - lastPos);
            agent_positions_.push_back(interpolatedPos);
        }
        lastPos = newPos;
    }
    agent_positions_.push_back(lastPos);
    for (int i = 0; i < endPadding; ++i) {
        agent_positions_.push_back(lastPos);
    }
}

GBT::GBT(const std::string& scene_config, const std::string& system_config)
: Node("gbt_publisher")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    scene_ = Scene(scene_config);
    system_ = System(system_config);
    initial_covariance_ = 1.0*Eigen::MatrixXd::Identity(2, 2);
    obstacle_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array/obstacles", 10);
    agent_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array/agent", 10);
    solution_ = planWithSimpleSetup();
    readAgentPositions(solution_);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&GBT::timer_callback, this));
}

void GBT::timer_callback()
{       
    visualization_msgs::msg::MarkerArray obstacle_marker_array;
    visualization_msgs::msg::MarkerArray agent_marker_array;

    // obstacle visualization
    int marker_id = 0;
    for (const auto& obstacle : scene_.obstacles_limits_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE; 
        marker.action = visualization_msgs::msg::Marker::ADD;
        auto& min_point = obstacle.first;
        auto& max_point = obstacle.second;
        marker.pose.position.x = (min_point[0] + max_point[0]) / 2.0f;
        marker.pose.position.y = (min_point[1] + max_point[1]) / 2.0f;
        marker.pose.position.z = (min_point[2] + max_point[2]) / 2.0f;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::abs(max_point[0] - min_point[0]);
        marker.scale.y = std::abs(max_point[1] - min_point[1]);
        marker.scale.z = std::abs(max_point[2] - min_point[2]);
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        obstacle_marker_array.markers.push_back(marker);
    }
    // agent visualization 
    visualization_msgs::msg::Marker agent_marker;
    agent_marker.header.frame_id = "world";
    agent_marker.header.stamp = this->now();
    agent_marker.ns = "agent";
    agent_marker.id = 0; 
    agent_marker.type = visualization_msgs::msg::Marker::SPHERE;
    agent_marker.action = visualization_msgs::msg::Marker::ADD;
    const auto& pos = agent_positions_[current_pos_ind_ % agent_positions_.size()];
    agent_marker.pose.position.x = pos.x();
    agent_marker.pose.position.y = pos.y();
    agent_marker.pose.position.z = pos.z();
    agent_marker.pose.orientation.x = 0.0;
    agent_marker.pose.orientation.y = 0.0;
    agent_marker.pose.orientation.z = 0.0;
    agent_marker.pose.orientation.w = 1.0;
    agent_marker.scale.x = 3.0;
    agent_marker.scale.y = 3.0; 
    agent_marker.scale.z = 3.0; 
    agent_marker.color.r = 0.0;
    agent_marker.color.g = 1.0;
    agent_marker.color.b = 1.0;
    agent_marker.color.a = 1.0; 
    current_pos_ind_++;
    agent_marker_array.markers.push_back(agent_marker);
    
    // Publish Agent Trajectory and Obstacles
    agent_publisher_->publish(agent_marker_array);
    obstacle_publisher_->publish(obstacle_marker_array);
}

int main(int argc, char** argv)
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path scene_config = currentPath  / ".." / "configurations" / "scenes" / "scene5.yaml";
    std::filesystem::path system_config = currentPath  / ".." / "configurations" / "systems" / "system1.yaml";
    std::cout << "Scene Path: " << scene_config << std::endl;
    std::cout << "System Path: " << system_config << std::endl;
    std::cout << "Starting GBT planner node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GBT>(scene_config, system_config));
	rclcpp::shutdown();
	return 0;
}