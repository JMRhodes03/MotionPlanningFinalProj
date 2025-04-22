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
#include "RG-RRT-Kang.h"

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
        // TODO: Your projection for the Manipulator
        // const auto *cstate = state->as<ompl::base::CompoundState>();
        // const ompl::base::SO2StateSpace::StateType *rot = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
        // const ompl::base::RealVectorStateSpace::StateType *vel = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        // projection(0) = rot->value;
        // projection(1) = vel->values[0];
    }
};

void ManipulatorODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the Manipulator's dynamics
    const double theta = q[0];
    const double omega = q[1];

    qdot.resize(q.size(), 0);
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    qdot[0] = omega;
    qdot[1] = -9.81*cos(theta) + u[0];
}

bool isStateValid(const ompl::base::State *state, int n)
{
    const ompl::base::CompoundState *cstate = state->as<ompl::base::CompoundState>();
    for (int i = 0; i < n; i++) {
        const ompl::base::RealVectorStateSpace::StateType *omega = cstate->as<ompl::base::RealVectorStateSpace::StateType>(n+i);
        if (omega->values[0] < -10 || omega->values[0] > 10)
            return false;
    }
    return true;
}

void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result)
{
    ompl::base::SO2StateSpace SO2;
    // Access the compound state
    ompl::base::CompoundState *compoundState = result->as<ompl::base::CompoundState>();
    // Access the orientation component (theta)
    ompl::base::SO2StateSpace::StateType *theta = compoundState->as<ompl::base::SO2StateSpace::StateType>(0);
    SO2.enforceBounds(theta);
}


ompl::control::SimpleSetupPtr createManipulator(double torque, int n)
{
    // TODO: Create and setup the Manipulator's state space, control space, validity checker, everything you need for
    // planning.

    // create the state space
    ompl::base::StateSpacePtr space;
    for (int i = 0; i < n; i++){
        // create a SO2 space for joint angle
        ompl::base::StateSpacePtr so2 = std::make_shared<ompl::base::SO2StateSpace>();
        // create a real vector space for joint vel
        ompl::base::StateSpacePtr r = std::make_shared<ompl::base::RealVectorStateSpace>(1);
        ompl::base::RealVectorBounds bounds(1);
        bounds.setLow(-10);
        bounds.setHigh(10);
        r->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
        space = space + so2;
        space = space + r;
    }

    std::cout << "State space dimensionality: " << space->getDimension() << std::endl;
    
    // create the control space
    ompl::control::ControlSpacePtr cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, n);
    ompl::base::RealVectorBounds cbounds(n);
    cbounds.setLow(-1*torque);
    cbounds.setHigh(torque);
    cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);

    // create a simple setup object
    ompl::control::SimpleSetupPtr ss = std::make_shared<ompl::control::SimpleSetup>(cspace);
    ss->setStateValidityChecker([n](const ompl::base::State *state) {
        return isStateValid(state, n);
    });

    // set the state propagation routine
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &ManipulatorODE);
    // ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    // create the start and goal states
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    start[0] = 0.;
    goal[0] = M_PI/2;
    for (int i = 1; i < 2*n; i++){
        start[i] = 0.;
        goal[i] = 0.;
    }
    ss->setStartAndGoalStates(start, goal, 0.1);

    return ss;
}

void planManipulator(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the Manipulator
    // choice is what planner to use.
    if (choice == 1)
    {
        // RRT
        ss->getSpaceInformation()->setPropagationStepSize(0.01);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else if (choice == 2)
    {
        // KPIECE1
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->getStateSpace()->registerProjection("ManipulatorProjection", ompl::base::ProjectionEvaluatorPtr(new ManipulatorProjection(ss->getStateSpace().get())));
        planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("ManipulatorProjection");
        ss->setPlanner(planner);
    }
    else if (choice == 3)
    {
        // RG-RRT
        ss->getSpaceInformation()->setPropagationStepSize(0.1);
        ompl::base::PlannerPtr planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else
        std::cerr << "Invalid choice of planner." << std::endl;

    ss->setup();

    ompl::base::PlannerStatus solved = ss->solve(20.0);

    if (solved)
    {
        // print path
        std::cout << "Solution found!" << std::endl;
        ompl::control::PathControl &path = ss->getSolutionPath();
        path.asGeometric().printAsMatrix(std::cout);

        // save path to file
        std::ofstream pathFile("Manipulator_path.txt");
        ss->getSolutionPath().asGeometric().printAsMatrix(pathFile);
        std::cout << "saved to Manipulator_path.txt!" << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

}

void benchmarkManipulator(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the Manipulator
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
        ss->getStateSpace()->registerProjection("ManipulatorProjection", ompl::base::ProjectionEvaluatorPtr(new ManipulatorProjection(ss->getStateSpace().get())));
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
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque limit? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    int n = 0;
    do{
        std::cout << "Number of joints? " << std::endl;
        std::cin >> n;
    } while (n == 0);

    ompl::control::SimpleSetupPtr ss = createManipulator(torque, n);

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
