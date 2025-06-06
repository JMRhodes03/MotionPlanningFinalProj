///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Jessica M. Rhodes, Sunny Kang
//////////////////////////////////////

#include <iostream>
#include <vector>
#include <limits>
#include <math.h>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/PlannerData.h>


// Your implementation of RG-RRT
#include "RG-RRT-Kang.h"

// Define global variables
int n = 0;
double* joint_vel_limit = nullptr;
double* torque_limit = nullptr;
double* link_lengths = nullptr;
double* masses = nullptr;

class ManipulatorGoalRegion : public ompl::base::GoalRegion
{
public:
    ManipulatorGoalRegion(const ompl::base::SpaceInformationPtr &si, const std::vector<double> &goal_center, double goal_radius)
        : ompl::base::GoalRegion(si), goal_center_(goal_center), goal_radius_(goal_radius)
    {
        setThreshold(goal_radius); // Set the goal threshold (radius)
    }

    // Override the distanceGoal method
    double distanceGoal(const ompl::base::State *state) const override
    {
        const auto *cstate = state->as<ompl::base::CompoundState>();
        std::vector<double> joint_angles(n);

        // Extract joint angles from the state
        for (int i = 0; i < n; ++i)
        {
            const ompl::base::SO2StateSpace::StateType *joint_angle = cstate->as<ompl::base::SO2StateSpace::StateType>(i);
            joint_angles[i] = joint_angle->value;
        }

        // Compute the Euclidean distance to the goal center
        double distance = 0.0;
        for (int i = 0; i < n; ++i)
        {
            distance += std::pow(joint_angles[i] - goal_center_[i], 2);
        }
        return std::sqrt(distance);
    }

private:
    std::vector<double> goal_center_; // Center of the goal region
    double goal_radius_;              // Radius of the goal region
};

// Your projection for the Manipulator
class ManipulatorProjection : public ompl::base::ProjectionEvaluator
{
public:
    ManipulatorProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the Manipulator
        return 2*n;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the Manipulator
        const auto *cstate = state->as<ompl::base::CompoundState>();
        for (int i = 0; i < n; ++i)
        {
            const ompl::base::SO2StateSpace::StateType *joint_angle = cstate->as<ompl::base::SO2StateSpace::StateType>(i);
            const ompl::base::RealVectorStateSpace::StateType *joint_velocity = cstate->as<ompl::base::RealVectorStateSpace::StateType>(n);
            
            projection(i) = joint_angle->value / M_PI;
            projection(i + n) = joint_velocity->values[i] / joint_vel_limit[i];
        }
    }
};

std::vector<double> computePhi(const std::vector<double> &q)
{
    std::vector<double> phi(n, 0.0);
    for (int i = 0; i < n; ++i){
        if (i == 0)
            phi[i] = q[i];
        else
            phi[i] = phi[i-1] + q[i];
    }
    return phi;
}

std::pair<std::vector<double>, std::vector<double>> computeXY(const std::vector<double> &phi)
{
    std::vector<double> x(n, 0.0);
    std::vector<double> y(n, 0.0);
    for (int i = n - 1; i >= 0; --i){
        if (i == n - 1){
            x[i] = link_lengths[i] * cos(phi[i]);
            y[i] = link_lengths[i] * sin(phi[i]);
        }
        else{
            x[i] = x[i+1] + link_lengths[i] * cos(phi[i]);
            y[i] = y[i+1] + link_lengths[i] * sin(phi[i]);
        }
    }
    return {x, y};
}

std::pair<std::vector<double>, std::vector<double>> computeXY_links(const std::vector<double> &phi)
{
    std::vector<double> x_l(n, 0.0);
    std::vector<double> y_l(n, 0.0);
    for (int i = 0; i < n; ++i){
        if (i == 0){
            x_l[i] = link_lengths[i] * cos(phi[i]);
            y_l[i] = link_lengths[i] * sin(phi[i]);
        }
        else{
            x_l[i] = x_l[i - 1] + link_lengths[i] * cos(phi[i]);
            y_l[i] = y_l[i - 1] + link_lengths[i] * sin(phi[i]);
        }
    }
    return {x_l, y_l};
}

std::pair<std::vector<double>, std::vector<double>> computeXY_COM(const std::vector<double> &phi, const std::vector<double> &x, const std::vector<double> &y)
{
    std::vector<double> x_c(n, 0.0);
    std::vector<double> y_c(n, 0.0);
    for (int i = 0; i < n; ++i){
        x_c[i] = x[0] - x[i] + link_lengths[i]/2 * cos(phi[i]);
        y_c[i] = y[0] - y[i] + link_lengths[i]/2 * sin(phi[i]);
    }
    return {x_c, y_c};
}

Eigen::MatrixXd ComputeJL_i(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &phi, int i)
{
    Eigen::MatrixXd JL_i = Eigen::MatrixXd::Zero(2, n);
    for (int j = 0; j < n; ++j) {
        if (j <= i){
            JL_i(0, j) = -1 * y[j] + y[i] - link_lengths[i] / 2 * sin(phi[i]);
            JL_i(1, j) = x[j] - x[i] + link_lengths[i] / 2 * cos(phi[i]);
        }
    }
    return JL_i;
}

Eigen::MatrixXd compute_dJL_i_dq_k(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &phi, int i, int k)
{
    Eigen::MatrixXd dJL_i_dq_k = Eigen::MatrixXd::Zero(2, n);
    for (int j = 0; j < n; ++j) {
        if (j <= i && k <= i)
            dJL_i_dq_k(0, j) = -1 * x[std::max(j, k)] + x[i] - link_lengths[i] / 2 * cos(phi[i]);
            dJL_i_dq_k(1, j) = -1 * y[std::max(j, k)] + y[i] - link_lengths[i] / 2 * sin(phi[i]);
    }
    return dJL_i_dq_k;
}

std::vector<Eigen::MatrixXd> computePsi_i(const std::vector<double>& phi, const std::vector<double>& x, const std::vector<double>& y, int i)
{
    std::vector<Eigen::MatrixXd> Psi_i(n, Eigen::MatrixXd::Zero(n, n));

    for (int j = 0; j < n; ++j)
    {
        // Compute Jacobian dJL_i_dq_k
        auto dJL_i_dq_k = compute_dJL_i_dq_k(x, y, phi, i, j);
        // Compute Jacobian JL_i for the CoM of link i
        auto JL_i = ComputeJL_i(x, y, phi, i);

        // Construct Psi_j^(i) matrices
        Psi_i[j] = dJL_i_dq_k.transpose() * JL_i + JL_i.transpose() * dJL_i_dq_k;
    }

    return Psi_i;
}

Eigen::MatrixXd ComputeInertiaMatrix(const std::vector<double> &q)
{
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);

    // Precompute phi_i = sum q[0] to q[i]
    auto phi = computePhi(q);

    // Precompute x_i and y_i
    auto [x, y] = computeXY(phi);

    for (int i = 0; i < n; ++i) {
        // Compute Jacobian JL_i for the CoM of link i
        auto JL_i = ComputeJL_i(x, y, phi, i);

        // Compute the JA_i^T * I_i * JA_i term
        double MoI_i = 1.0/12.0 * masses[i] * link_lengths[i] * link_lengths[i];
        Eigen::VectorXd v = Eigen::VectorXd::Zero(n);  // Create a zero vector
        v.head(i).setOnes();  // Set the first i values to 1

        // Add the term m_i * JL^T * JL
        M += masses[i] * JL_i.transpose() * JL_i + MoI_i * v * v.transpose();
    }

    return M;
}

Eigen::VectorXd computeCoriolisCentrifugalVector(const std::vector<double>& q, const std::vector<double>& qd)
{
    Eigen::VectorXd h = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd qdot = Eigen::Map<const Eigen::VectorXd>(qd.data(), qd.size());

    // Precompute phi_i = sum q[0] to q[i]
    auto phi = computePhi(q);

    // Precompute x_i and y_i
    auto [x, y] = computeXY(phi);

    for (int i = 0; i < n; ++i)
    {
        // Initialize h_i
        Eigen::VectorXd h_i = Eigen::VectorXd::Zero(n);

        // Compute Psi_i for every i (link)
        auto Psi_i = computePsi_i(phi, x, y, i); // n x n matrix

        // Compute sumPsiQ term 
        Eigen::MatrixXd sumPsiQ = Eigen::MatrixXd::Zero(n, n);
        for (int j = 0; j < n; ++j)
        {
            // Sum up the Psi*qdot terms 
            sumPsiQ += Psi_i[j] * qdot[j];
        }

        // Compute qtPsi term
        Eigen::MatrixXd qtPsi = Eigen::MatrixXd::Zero(n, n);
        for (int k = 0; k < n; ++k)
        {
            Eigen::RowVectorXd qtPsi_k = qdot.transpose() * Psi_i[k];
            for (int j = 0; j < n; ++j)
                qtPsi(k, j) = qtPsi_k[j];
        }

        h_i = masses[i] * (sumPsiQ * qdot - 0.5 * qtPsi * qdot);
        h += h_i;
    }
    return h;
}

Eigen::VectorXd computeGravityVector(const std::vector<double>& q)
{
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);

    auto phi = computePhi(q);

    for (int i = n - 1; i >= 0; --i) {
        if (i == n - 1)
            g[i] = 9.81 * masses[i] * link_lengths[i] / 2 * cos(phi[i]);
        else{
            double sumMassLink = 0;
            for (int k = i + 1; k < n; ++k){
                sumMassLink += masses[k] * link_lengths[i] * cos(phi[i]);
            }
            g[i] = g[i + 1] + 9.81 * (masses[i] * link_lengths[i] / 2 * cos(phi[i]) + sumMassLink);
        }
    }
    return g;
}

void ManipulatorODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control *control,
    ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the Manipulator's dynamics
    // std::cout << "ManipulatorODE called" << std::endl;
    qdot.resize(q.size());
    // Extract the first n elements of q into q_vec
    std::vector<double> q_vec(q.begin(), q.begin() + n);
    // Extract the last n elements of q into qd_vec
    std::vector<double> qd_vec(q.begin() + n, q.end());

    const double* u_values = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    Eigen::VectorXd u = Eigen::Map<const Eigen::VectorXd>(u_values, n);    
    Eigen::MatrixXd M = ComputeInertiaMatrix(q_vec);
    Eigen::VectorXd C = computeCoriolisCentrifugalVector(q_vec, qd_vec);
    Eigen::VectorXd V = computeGravityVector(q_vec);
    Eigen::VectorXd qdotdot = M.inverse() * (u - C - V);

    for (int i = 0; i < n; i++){
        qdot[i] = q[n + i];
        qdot[n + i] = qdotdot[i];
    }
}

bool checkLineIntersection(double x1, double y1, double x2, double y2,
    double x3, double y3, double x4, double y4)
{
    // Compute the direction of the lines
    double det = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3);
    if (det == 0.0) {
        return false; // Lines are parallel
    }

    double t = ((x3 - x1) * (y4 - y3) - (y3 - y1) * (x4 - x3)) / det;
    double u = ((x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)) / det;

    return (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0);
}

bool checkLineRectangleIntersection(double x1, double y1, double x2, double y2,
    double rect_x_min, double rect_y_min,
    double rect_x_max, double rect_y_max)
{
    // Check if either endpoint is inside the rectangle
    if ((x1 >= rect_x_min && x1 <= rect_x_max && y1 >= rect_y_min && y1 <= rect_y_max) ||
    (x2 >= rect_x_min && x2 <= rect_x_max && y2 >= rect_y_min && y2 <= rect_y_max)) 
    {
        return true; // One endpoint is inside the rectangle
    }

    // Check for intersection with each edge of the rectangle
    return checkLineIntersection(x1, y1, x2, y2, rect_x_min, rect_y_min, rect_x_max, rect_y_min) || // Bottom edge
    checkLineIntersection(x1, y1, x2, y2, rect_x_min, rect_y_max, rect_x_max, rect_y_max) || // Top edge
    checkLineIntersection(x1, y1, x2, y2, rect_x_min, rect_y_min, rect_x_min, rect_y_max) || // Left edge
    checkLineIntersection(x1, y1, x2, y2, rect_x_max, rect_y_min, rect_x_max, rect_y_max);   // Right edge
}

bool isStateValid(const ompl::base::State *state)
{
    const ompl::base::CompoundState *cstate = state->as<ompl::base::CompoundState>();
    const ompl::base::RealVectorStateSpace::StateType *omega = cstate->as<ompl::base::RealVectorStateSpace::StateType>(n);

    // Check joint velocity limits
    for (int i = 0; i < n; i++) {
        if (omega->values[i] < -1 * joint_vel_limit[i] || omega->values[i] > joint_vel_limit[i])
            return false;
    }

    // Compute the positions of the manipulator's links
    std::vector<double> joint_angles(n);
    for (int i = 0; i < n; ++i) {
        const ompl::base::SO2StateSpace::StateType *joint_angle = cstate->as<ompl::base::SO2StateSpace::StateType>(i);
        joint_angles[i] = joint_angle->value;
    }

    auto phi = computePhi(joint_angles);
    auto [x, y] = computeXY_links(phi);


    // Check for collision with the rectangle
    double rect_x_min = -1.;
    double rect_y_min = -1.;
    double rect_x_max = rect_x_min + 0.5;
    double rect_y_max = rect_y_min + 0.5;

    for (int i = 0; i < n; ++i) {
        double x1 = (i == 0) ? 0.0 : x[i - 1];
        double y1 = (i == 0) ? 0.0 : y[i - 1];
        double x2 = x[i];
        double y2 = y[i];
        
        if (checkLineRectangleIntersection(x1, y1, x2, y2, rect_x_min, rect_y_min, rect_x_max, rect_y_max)) {
            return false; // Collision detected
        }
    }

    return true; // No collision
}

void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result)
{
    // Access the compound state
    ompl::base::CompoundState *compoundState = result->as<ompl::base::CompoundState>();
    // Access the orientation component (theta)
    for (int i = 0; i < n; i++){
        // Access the SO2 state
        ompl::base::SO2StateSpace SO2;
        ompl::base::SO2StateSpace::StateType *theta = compoundState->as<ompl::base::SO2StateSpace::StateType>(i);
        SO2.enforceBounds(theta);
    }
}

ompl::control::SimpleSetupPtr createManipulator(int n)
{
    // TODO: Create and setup the Manipulator's state space, control space, validity checker, everything you need for
    // planning.

    // create state space
    auto compoundSpace = std::make_shared<ompl::base::CompoundStateSpace>();
    for (int i = 0; i < n; i++){
        // create a SO2 space for joint angle
        compoundSpace->addSubspace(std::make_shared<ompl::base::SO2StateSpace>(), 1.0);
    }
    // create real vector spaces for joint velocities
    auto omegaSpace = std::make_shared<ompl::base::RealVectorStateSpace>(n);
    ompl::base::RealVectorBounds bounds(n);
    for (int i = 0; i < n; i++){
        bounds.setLow(i, -1 * joint_vel_limit[i]);
        bounds.setHigh(i, joint_vel_limit[i]);
    }
    omegaSpace->setBounds(bounds);
    compoundSpace->addSubspace(omegaSpace, 1.0);

    ompl::base::StateSpacePtr space = compoundSpace;

    std::cout << "State space dimensionality: " << space->getDimension() << std::endl;
    
    // create control space
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, n);
    ompl::base::RealVectorBounds cbounds(n);
    for (int i = 0; i < n; i++) {
        cbounds.setLow(i, -1 * torque_limit[i]); // Set lower bound for control input i
        cbounds.setHigh(i, torque_limit[i]);    // Set upper bound for control input i
    }
    cspace->setBounds(cbounds);

    // create a simple setup object
    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);   
    ss->setStateValidityChecker(isStateValid);

    // set the state propagation routine
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &ManipulatorODE);
    // ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    // create the start and goal states
    ompl::base::ScopedState<> start(space);
    std::vector<double> goal_center(n, 0.0);

    start[0] = 0.;
    goal_center[0] = -M_PI;
    for (int i = 1; i < 2*n; i++){
        start[i] = 0.;
        goal_center[i] = 0.;
    }
    double goal_radius = 0.1;
    auto goal_region = std::make_shared<ManipulatorGoalRegion>(ss->getSpaceInformation(), goal_center, goal_radius);
    
    ss->setStartState(start);
    ss->setGoal(goal_region);

    std::cout << "Manipulator setup complete" << std::endl;

    return ss;
}


void saveExploredStates(const ompl::control::SimpleSetupPtr &ss, int n)
{
    std::cout << "Saving explored states..." << std::endl;
    ompl::base::PlannerData pd(ss->getSpaceInformation());
    ss->getPlanner()->getPlannerData(pd);

    std::ofstream exploredFile("explored_states.txt");
    if (!exploredFile.is_open())
    {
        std::cerr << "Failed to open explored_states.txt" << std::endl;
        return;
    }

    exploredFile << "n = " << n << std::endl;

    for (unsigned int i = 0; i < pd.numVertices(); ++i)
    {
        const ompl::base::State *state = pd.getVertex(i).getState();
        auto compound = state->as<ompl::base::CompoundState>();

        // Assuming joint angles and velocities are stored as two RealVectorStateSpaces
        const auto *velocities = compound->as<ompl::base::RealVectorStateSpace::StateType>(n);

        // Write joint angles
        for (int j = 0; j < n; ++j)
        {
            const ompl::base::SO2StateSpace::StateType *joint_angle = compound->as<ompl::base::SO2StateSpace::StateType>(j);            
            exploredFile << joint_angle->value << " ";
        }

        // Write joint velocities
        for (int j = 0; j < n; ++j)
        {
            exploredFile << velocities->values[j];
            if (j < n - 1)
                exploredFile << " ";
        }

        exploredFile << std::endl;
    }

    exploredFile.close();
    std::cout << "Explored states saved to explored_states.txt!" << std::endl;
}

double computeEnergy(const std::vector<double> &q, const std::vector<double> &qd)
{
    // Eigen::VectorXd q_vec = Eigen::Map<Eigen::VectorXd>(q.data(), q.size());
    Eigen::Map<const Eigen::VectorXd> qd_vec(qd.data(), qd.size());
    auto M = ComputeInertiaMatrix(q);
    
    //compute energy
    auto phi = computePhi(q);
    auto [x, y] = computeXY(phi);
    auto [x_c, y_c] = computeXY_COM(phi, x, y);

    double E_mh = 0.0;
    for (int i = 0; i < n; i++){
        double height_i = y_c[i]+0.5*n;
        E_mh += masses[i] * height_i;
    }
    double E = 0.5 * qd_vec.transpose() * M * qd_vec + 9.81 * E_mh;

    return E;
}

void planManipulator(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the Manipulator
    // choice is what planner to use.
    if (choice == 1)
    {
        // RRT
        ss->getSpaceInformation()->setPropagationStepSize(0.02);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else if (choice == 2)
    {
        // KPIECE1
        ss->getSpaceInformation()->setPropagationStepSize(0.02);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->getStateSpace()->registerProjection("ManipulatorProjection", ompl::base::ProjectionEvaluatorPtr(new ManipulatorProjection(ss->getStateSpace().get())));
        planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("ManipulatorProjection");
        ss->setPlanner(planner);
    }
    else if (choice == 3)
    {
        // RG-RRT
        ss->getSpaceInformation()->setPropagationStepSize(0.07);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else
        std::cerr << "Invalid choice of planner." << std::endl;

    ss->setup();
    std::cout << "Plnner setup complete" << std::endl;

    ompl::base::PlannerStatus solved = ss->solve(120.0);

    if (solved)
    {
        // print path
        std::cout << "Solution found!" << std::endl;
        ompl::control::PathControl &path = ss->getSolutionPath();
        path.asGeometric().printAsMatrix(std::cout);
    
        // save path to file
        std::ofstream pathFile("Manipulator_path.txt");
        if (pathFile.is_open())
        {
            // Write number of joints
            pathFile << n << std::endl;
    
            // Write link lengths
            for (size_t i = 0; i < n; ++i)
            {
                pathFile << link_lengths[i];
                if (i < n - 1)
                    pathFile << " ";
            }
            pathFile << std::endl;
    
            // Write the path
            path.asGeometric().printAsMatrix(pathFile);
    
            std::cout << "saved to Manipulator_path.txt!" << std::endl;
        }
        else
        {
            std::cerr << "Failed to open Manipulator_path.txt for writing!" << std::endl;
        }
    }
    else if (ss->getProblemDefinition()->hasApproximateSolution())
    {
        std::cout << "Approximate solution found!" << std::endl;
        ompl::control::PathControl &path = ss->getSolutionPath();
        path.asGeometric().printAsMatrix(std::cout);
    }
    else
    {
        saveExploredStates(ss, n);
        std::cout << "No solution found" << std::endl;
    }

}

void benchmarkManipulator(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the Manipulator
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(*ss, "Manipulator");

    // RRT
    {
        ss->getSpaceInformation()->setPropagationStepSize(0.02);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        planner->setName("RRT");
        b.addPlanner(planner);
    }
    // KPIECE1
    {
        ss->getSpaceInformation()->setPropagationStepSize(0.02);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->getStateSpace()->registerProjection("ManipulatorProjection", ompl::base::ProjectionEvaluatorPtr(new ManipulatorProjection(ss->getStateSpace().get())));
        planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("ManipulatorProjection");
        planner->setName("KPIECE1");
        b.addPlanner(planner);
    }
    // RG-RRT
    {
        ss->getSpaceInformation()->setPropagationStepSize(0.07);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        planner->setName("RG-RRT");
        b.addPlanner(planner);
    }
    
    // Run the benchmark
    std::cout << "Running benchmark..." << std::endl;
    b.benchmark(request);
    b.saveResultsToFile("Manipulator.db");
    std::cout << "Benchmarking complete!" << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 3);

    // Get the number of joints
    do{
        std::cout << "Number of joints? " << std::endl;
        std::cin >> n;
    } while (n == 0);
    std::cout << "Number of joints: " << n << std::endl;

    // Get link lengths and masses
    link_lengths = new double[n];
    masses = new double[n];
    torque_limit = new double[n];
    joint_vel_limit = new double[n];
    for (int i = 0; i < n; i++){
        std::cout << "Joint " << i+1 << " torque limit?" << std::endl;
        std::cin >> torque_limit[i];
        std::cout << "Joint " << i+1 << " velocity limit?" << std::endl;
        std::cin >> joint_vel_limit[i];
        std::cout << "Link " << i+1 << " length?" << std::endl;
        std::cin >> link_lengths[i];
        std::cout << "Link " << i+1 << " mass?" << std::endl;
        std::cin >> masses[i];
    }

    for (int i = 0; i < n; i++) {
        std::cout << "Joint " << i + 1 << " torque limit: " << torque_limit[i] << std::endl;
        std::cout << "Joint " << i + 1 << " velocity limit: " << joint_vel_limit[i] << std::endl;
        std::cout << "Link " << i + 1 << " length: " << link_lengths[i] << std::endl;
        std::cout << "Link " << i + 1 << " mass: " << masses[i] << std::endl;
    }

    ompl::control::SimpleSetupPtr ss = createManipulator(n);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planManipulator(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkManipulator(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
