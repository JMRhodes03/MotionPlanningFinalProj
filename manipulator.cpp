///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
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

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/tools/benchmark/Benchmark.h>


// Your implementation of RG-RRT
#include "RG-RRT-Rhodes.h"

// Define global variables
int n = 0;
double joint_vel_limit = 0.0;
double torque_limit;
double* link_lengths = nullptr;
double* masses = nullptr;

// Environment obstacles (example rectangular obstacles)
struct Obstacle {
    double x_min, x_max, y_min, y_max;
};
std::vector<Obstacle> obstacles = {
    {1.0, 2.0, -0.5, 0.5},  // Example obstacle
    {-2.0, -1.0, -1.0, 1.0} // Another example obstacle
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
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override 
    {
        const auto *cstate = state->as<ompl::base::CompoundState>();
        
        double x = 0, y = 0;
        for (int i = 0; i < n; i++) {
            const auto *theta = cstate->as<ompl::base::SO2StateSpace::StateType>(i);
            x += link_lengths[i] * cos(theta->value);
            y += link_lengths[i] * sin(theta->value);
        }
        
        projection(0) = x;
        projection(1) = y;
    }
};

// Helper functions for dynamics calculations
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

Eigen::MatrixXd ComputeJL_i(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &phi, int i) {
    Eigen::MatrixXd JL_i = Eigen::MatrixXd::Zero(2, n);
    for (int j = 0; j < n; ++j) {
        JL_i(0, j) = -1 * y[j] + y[i] - link_lengths[i] / 2 * sin(phi[i]);
        JL_i(1, j) = x[j] - x[i] + link_lengths[i] / 2 * cos(phi[i]);
    }
    return JL_i;
}

Eigen::MatrixXd compute_dJL_i_dq_k(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &phi, int i, int k) {
    Eigen::MatrixXd dJL_i_dq_k = Eigen::MatrixXd::Zero(2, n);
    for (int j = 0; j < n; ++j) {
        if (j <= i && k <= i) {
            dJL_i_dq_k(0, j) = -1 * x[std::max(j, k)] + x[i] - link_lengths[i] / 2 * cos(phi[i]);
            dJL_i_dq_k(1, j) = -1 * y[std::max(j, k)] + y[i] - link_lengths[i] / 2 * sin(phi[i]);
        }
    }
    return dJL_i_dq_k;
}

std::vector<Eigen::MatrixXd> computePsi_i(const std::vector<double>& phi, const std::vector<double>& x, const std::vector<double>& y, int i) {
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

Eigen::MatrixXd ComputeInertiaMatrix(const std::vector<double> &q) {
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);
    auto phi = computePhi(q);
    auto [x, y] = computeXY(phi);

    for (int i = 0; i < n; ++i) {
        auto JL_i = ComputeJL_i(x, y, phi, i);
        double MoI_i = 1.0/12.0 * masses[i] * link_lengths[i] * link_lengths[i];
        Eigen::VectorXd v = Eigen::VectorXd::Zero(n);
        v.head(i+1).setOnes();
        M += masses[i] * JL_i.transpose() * JL_i + MoI_i * v * v.transpose();
    }

    return M;
}

Eigen::VectorXd computeCoriolisCentrifugalVector(const std::vector<double>& q, const std::vector<double>& qd) {
    Eigen::VectorXd h = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd qdot = Eigen::Map<const Eigen::VectorXd>(qd.data(), qd.size());
    auto phi = computePhi(q);
    auto [x, y] = computeXY(phi);

    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd h_i = Eigen::VectorXd::Zero(n);
        auto Psi_i = computePsi_i(phi, x, y, i);

        Eigen::MatrixXd sumPsiQ = Eigen::MatrixXd::Zero(n, n);
        for (int j = 0; j < n; ++j) {
            sumPsiQ += Psi_i[j] * qdot[j];
        }

        Eigen::MatrixXd qtPsi = Eigen::MatrixXd::Zero(n, n);
        for (int k = 0; k < n; ++k) {
            Eigen::RowVectorXd qtPsi_k = qdot.transpose() * Psi_i[k];
            for (int j = 0; j < n; ++j)
                qtPsi(k, j) = qtPsi_k[j];
        }

        h_i = masses[i] * (sumPsiQ * qdot - 0.5 * qtPsi * qdot);
        h += h_i;
    }
    return h;
}

Eigen::VectorXd computeGravityVector(const std::vector<double>& q) {
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
    auto phi = computePhi(q);
    
    for (int i = 0; i < n; ++i) {
        double sum = 0;
        for (int k = i + 1; k <= n; ++k) {
            sum += masses[k-1];
        }
        g[i] = 9.81 * (masses[i] * link_lengths[i]/2 + sum * link_lengths[i]) * cos(phi[i]);
    }
    
    return g;
}

// Check if line segment (x1,y1)-(x2,y2) intersects with any obstacle
bool checkCollision(double x1, double y1, double x2, double y2) {
    for (const auto& obs : obstacles) {
        // Simple AABB collision check (for demo purposes)
        if ((x1 > obs.x_min && x1 < obs.x_max && y1 > obs.y_min && y1 < obs.y_max) ||
            (x2 > obs.x_min && x2 < obs.x_max && y2 > obs.y_min && y2 < obs.y_max)) {
            return true;
        }
    }
    return false;
}

bool isStateValid(const ompl::base::State *state) {
    const ompl::base::CompoundState *cstate = state->as<ompl::base::CompoundState>();
    const ompl::base::RealVectorStateSpace::StateType *omega = cstate->as<ompl::base::RealVectorStateSpace::StateType>(n);

    // Check velocity limits
    for (int i = 0; i < n; i++) {
        if (fabs(omega->values[i]) > joint_vel_limit)
            return false;
    }

    // Check for collisions
    std::vector<double> angles;
    for (int i = 0; i < n; i++) {
        const auto *theta = cstate->as<ompl::base::SO2StateSpace::StateType>(i);
        angles.push_back(theta->value);
    }

    double x = 0, y = 0;
    for (int i = 0; i < n; i++) {
        double new_x = x + link_lengths[i] * cos(angles[i]);
        double new_y = y + link_lengths[i] * sin(angles[i]);
        
        if (checkCollision(x, y, new_x, new_y)) {
            return false;
        }
        
        x = new_x;
        y = new_y;
    }
    
    return true;
}

void ManipulatorODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control *control,
    ompl::control::ODESolver::StateType & qdot) {
    qdot.resize(q.size());
    std::vector<double> q_vec(q.begin(), q.begin() + n);
    std::vector<double> qd_vec(q.begin() + n, q.end());

    const double* u_values = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    Eigen::MatrixXd M = ComputeInertiaMatrix(q_vec);
    Eigen::VectorXd C = computeCoriolisCentrifugalVector(q_vec, qd_vec);
    Eigen::VectorXd V = computeGravityVector(q_vec);
    Eigen::VectorXd u = Eigen::Map<const Eigen::VectorXd>(u_values, n);    
    
    // Regularize M to avoid numerical issues
    M += Eigen::MatrixXd::Identity(n, n) * 1e-6;
    Eigen::VectorXd qdotdot = M.llt().solve(u - C - V);

    for (int i = 0; i < n; i++) {
        qdot[i] = q[n + i];
        qdot[n + i] = qdotdot[i];
    }
}

void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, 
                  const double duration, ompl::base::State* result) {
    ompl::base::CompoundState *compoundState = result->as<ompl::base::CompoundState>();
    for (int i = 0; i < n; i++) {
        ompl::base::SO2StateSpace SO2;
        ompl::base::SO2StateSpace::StateType *theta = compoundState->as<ompl::base::SO2StateSpace::StateType>(i);
        SO2.enforceBounds(theta);
    }
}

void plotManipulator(const std::vector<double>& angles, const std::string& filename = "manipulator_plot.txt") {
    std::ofstream plot(filename);
    double x = 0, y = 0;
    plot << x << " " << y << "\n";
    
    for (int i = 0; i < n; i++) {
        x += link_lengths[i] * cos(angles[i]);
        y += link_lengths[i] * sin(angles[i]);
        plot << x << " " << y << "\n";
    }
}

ompl::control::SimpleSetupPtr createManipulator(double torque, int n) {
    auto compoundSpace = std::make_shared<ompl::base::CompoundStateSpace>();
    for (int i = 0; i < n; i++) {
        compoundSpace->addSubspace(std::make_shared<ompl::base::SO2StateSpace>(), 1.0);
    }
    
    auto omegaSpace = std::make_shared<ompl::base::RealVectorStateSpace>(n);
    ompl::base::RealVectorBounds bounds(n);
    bounds.setLow(-joint_vel_limit);
    bounds.setHigh(joint_vel_limit);
    omegaSpace->setBounds(bounds);
    compoundSpace->addSubspace(omegaSpace, 1.0);

    ompl::base::StateSpacePtr space = compoundSpace;
    
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, n);
    ompl::base::RealVectorBounds cbounds(n);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);

    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);
    ss->setStateValidityChecker(isStateValid);

    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &ManipulatorODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);

    start[0] = M_PI/2;
    goal[0] = -M_PI;
    for (int i = 1; i < 2*n; i++) {
        start[i] = 0.;
        goal[i] = 0.;
    }
    ss->setStartAndGoalStates(start, goal, 0.2);

    return ss;
}

void planManipulator(ompl::control::SimpleSetupPtr &ss, int choice) {
    if (choice == 1) {
        ss->getSpaceInformation()->setPropagationStepSize(0.01);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else if (choice == 2) {
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->getStateSpace()->registerProjection("ManipulatorProjection", 
            ompl::base::ProjectionEvaluatorPtr(new ManipulatorProjection(ss->getStateSpace().get())));
        planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("ManipulatorProjection");
        ss->setPlanner(planner);
    }
    else if (choice == 3) {
        ss->getSpaceInformation()->setPropagationStepSize(0.15);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else {
        std::cerr << "Invalid choice of planner." << std::endl;
        return;
    }

    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(60.0);

    if (solved) {
        std::cout << "Solution found!" << std::endl;
        ompl::control::PathControl &path = ss->getSolutionPath();
        
        // Save path and plot final configuration
        std::ofstream pathFile("Manipulator_path.txt");
        path.asGeometric().printAsMatrix(pathFile);
        
        // Plot final configuration
        auto final_state = path.getStates().back();
        const auto *cstate = final_state->as<ompl::base::CompoundState>();
        std::vector<double> final_angles;
        for (int i = 0; i < n; i++) {
            const auto *theta = cstate->as<ompl::base::SO2StateSpace::StateType>(i);
            final_angles.push_back(theta->value);
        }
        plotManipulator(final_angles);
        
        std::cout << "Path saved to Manipulator_path.txt and final configuration to manipulator_plot.txt" << std::endl;
    }
    else {
        std::cout << "No solution found" << std::endl;
    }
}

void visualizePath(const ompl::control::PathControl& path) {
    std::ofstream out("manipulator_path_animation.txt");
    
    // Write header with number of joints
    out << n << "\n";
    
    // Create a non-const copy for iteration
    ompl::control::PathControl pathCopy(path);
    const auto& states = pathCopy.getStates();
    
    for (const auto& state : states) {
        const auto* cstate = state->as<ompl::base::CompoundState>();
        
        // Write joint angles
        for (int j = 0; j < n; j++) {
            const auto* theta = cstate->as<ompl::base::SO2StateSpace::StateType>(j);
            out << theta->value << " ";
        }
        out << "\n";
    }
    std::cout << "Animation data saved to manipulator_path_animation.txt" << std::endl;
}

void benchmarkManipulator(ompl::control::SimpleSetupPtr &ss) {
    double runtime_limit = 20, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(*ss, "Manipulator");

    // RRT
    {
        ss->getSpaceInformation()->setPropagationStepSize(0.01);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        planner->setName("RRT");
        b.addPlanner(planner);
    }
    // KPIECE1
    {
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->getStateSpace()->registerProjection("ManipulatorProjection", 
            ompl::base::ProjectionEvaluatorPtr(new ManipulatorProjection(ss->getStateSpace().get())));
        planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("ManipulatorProjection");
        planner->setName("KPIECE1");
        b.addPlanner(planner);
    }
    // RG-RRT
    {
        ss->getSpaceInformation()->setPropagationStepSize(0.1);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        planner->setName("RG-RRT");
        b.addPlanner(planner);
    }
    
    b.benchmark(request);
    b.saveResultsToFile("Manipulator.db");
    std::cout << "Benchmarking complete! Results saved to Manipulator.db" << std::endl;
}

int main(int /* argc */, char ** /* argv */) {
    int choice;
    do {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    do {
        std::cout << "Number of joints? " << std::endl;
        std::cin >> n;
    } while (n == 0);

    link_lengths = new double[n];
    masses = new double[n];
    for (int i = 0; i < n; i++) {
        std::cout << "Link " << i+1 << " length? ";
        std::cin >> link_lengths[i];
        std::cout << "Link " << i+1 << " mass? ";
        std::cin >> masses[i];
    }

    do {
        std::cout << "Torque limit? ";
        std::cin >> torque_limit;
        std::cout << "Joint velocity limit? ";
        std::cin >> joint_vel_limit;
    } while (torque_limit < 0 || joint_vel_limit < 0);

    ompl::control::SimpleSetupPtr ss = createManipulator(torque_limit, n);

    if (choice == 1) {
        int planner;
        do {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;
            std::cin >> planner;
        } while (planner < 1 || planner > 3);
        planManipulator(ss, planner);
        visualizePath(ss->getSolutionPath());
    }
    else {
        benchmarkManipulator(ss);
    }

    delete[] link_lengths;
    delete[] masses;
    return 0;
}