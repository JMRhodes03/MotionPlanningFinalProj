///////////////////////////////////////
// RBE550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "RG-RRT-Kang.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <limits>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// TODO: Implement RGRRT as described

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    addIntermediateStates_ = false;
    lastGoalMotion_ = NULL;

    goalBias_ = 0.1;
  
    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates, "0,1");

    // discretizes the control space 
    // divides the control space into RSIZE parts
    // control_offset stores the coputed step sizes 
    const std::vector<double> diff = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().getDifference();
    for (double d: diff)
        control_offset_.push_back(d/double(this->RSIZE));
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}


void ompl::control::RGRRT::setup(void)
{
    base::Planner::setup();
    if (!nn_)
    {
        nn_ = std::make_shared<ompl::NearestNeighborsLinear<Motion *>>();
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    }
    std::cout << "RGRRT::setup: Nearest neighbor data structure initialized." << std::endl;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            // Free reachable motions
            for (auto &reachableMotion : motion->reachable)
            {
                if (reachableMotion->state)
                    si_->freeState(reachableMotion->state);
                if (reachableMotion->control)
                    siC_->freeControl(reachableMotion->control);
                delete reachableMotion;
            }

            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
    nn_.reset(); // Reset the nearest neighbor data structure
}

double ompl::control::RGRRT::distanceFunction(const Motion *a, const Motion *b) const
{
    double const qnear_to_qrand = si_->distance(a->state, b->state);
    // std::cout<< "qnear_to_qrand: " << qnear_to_qrand << std::endl;

    double min_distance = qnear_to_qrand;
    for (const auto &reachable_motion : a->reachable)
    {
        double reachable_distance = si_->distance(reachable_motion->state, b->state);
        // std::cout<< "reachable to qrand: " << reachable_distance << std::endl;
        // if a reachable motion is closer to b than a, return the distance
        if (reachable_distance < min_distance)
        {
            // return distance;
            min_distance = reachable_distance;
        }
    }
    if (min_distance < qnear_to_qrand)
    {
        // std::cout<< "found! " << std::endl;
        // std::cout<< "min_distance: " << min_distance << std::endl;
        return qnear_to_qrand;
    }else
    {
        return std::numeric_limits<double>::infinity();
    }
}

void ompl::control::RGRRT::setupReachableSet(Motion* const m)
{
    const std::vector<double> &low_bound = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().low;
    const std::vector<double> &high_bound = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().high;

    for (int i = 0; i < this->RSIZE; i++)
    {
        Motion *motion = new Motion(siC_);
        siC_->copyControl(motion->control, m->control);
        double *controls = motion->control->as<RealVectorControlSpace::ControlType>()->values;
        // for car
        for (int j = 0; j < low_bound.size(); j++)
        {
            controls[j] = low_bound[j] + control_offset_[j] * (i + 1);
        }

        motion->steps = siC_->propagateWhileValid(m->state, motion->control, siC_->getMinControlDuration(), motion->state);

        if (motion->steps != 0)
        {
            m->reachable.push_back(motion);
        }
        else
        {
            si_->freeState(motion->state);
            siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

int ompl::control::RGRRT::selectReachableMotion(const Motion* qnear, const Motion* qrand)
{
    const double near_distance = si_->distance(qnear->state, qrand->state);
    double min_distance = near_distance;
    const std::vector<Motion*>& reachable = qnear->reachable;

    int min_index = -1;
    for (int i = 0; i < reachable.size(); i++)
    {
        if (!reachable[i] || !reachable[i]->state)
        {
            std::cerr << "Error: Invalid motion in reachable set at index " << i << std::endl;
            continue;
        }

        double distance = si_->distance(reachable[i]->state, qrand->state);
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }
    return min_index;
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    if (!nn_)
    {
        std::cerr << "Error: Nearest neighbor data structure is not initialized." << std::endl;
        setup();
        std::cout << "RGRRT::solve: Calling setup() before solve." << std::endl;
    }

    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    goal->print(std::cout);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        setupReachableSet(motion);
        if (!nn_)
        {
            std::cerr << "Error: Nearest neighbor data structure is not initialized." << std::endl;
            return base::PlannerStatus::INVALID_START;
        }
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
 

    if (!sampler_){
        sampler_ = si_->allocStateSampler();
    }
    if (!controlSampler_){
        controlSampler_ = siC_->allocDirectedControlSampler();
    }
    

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
 
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
 
    // define the random state and control
    auto *rmotion = new Motion(siC_); // qRand
    base::State *rstate = rmotion->state; // qRand state
    Control *rctrl = rmotion->control; // qRand control
    base::State *xstate = si_->allocState();
 
    OMPL_INFORM("Starting solve loop...");
    while (!ptc)
    {
        int id = -1;
        Motion *nmotion = nullptr;
        unsigned int cd = 0;
        while (id == -1){
            /* sample random state (with goal biasing) */
            if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                goal_s->sampleGoal(rstate);
            else
                sampler_->sampleUniform(rstate);

            // OMPL_INFORM("PRINTING DISTANCES FOR NN SEARCH", getName().c_str(), nn_->size());
            /* find nearest state in the tree */
            nmotion = nn_->nearest(rmotion);

            // Select a reachable motion from the nearest state that is closer to the random state
            id = selectReachableMotion(nmotion, rmotion);
        }
        // std::cout << "sampling done, qNear found, reachable point found" << std::endl;
        /* sample a random control that attempts to go towards the random state, and also sample a control duration */

        // if id is not -1, then copy the reachable state, control and step
        // unsigned int cd = nmotion->reachable[id]->steps;
        cd = nmotion->reachable[id]->steps;
 
        if (cd >= siC_->getMinControlDuration())
        {
            // create a new motion and copy the state and control from the reachable motion
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, nmotion->reachable[id]->state);
            siC_->copyControl(motion->control, nmotion->reachable[id]->control);
            motion->steps = cd;
            // connect qNear to qReachable
            motion->parent = nmotion;
            setupReachableSet(motion);
            nn_->add(motion);
            // OMPL_INFORM("%s: Now have %u states in datastructure", getName().c_str(), nn_->size());

            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            if (solv)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    } // end while (!ptc)
 
    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
        if (solution == nullptr)
        {
            OMPL_INFORM("%s: Unable to find any solution", getName().c_str());
            return base::PlannerStatus::TIMEOUT;
        }
        OMPL_INFORM("%s: Approximate solution found", getName().c_str());
        OMPL_INFORM("%s: Approximate solution distance: %f", getName().c_str(), approxdif);

    }
 
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
 
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
 
        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }
 
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    si_->freeState(xstate);
 
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
 
    return {solved, approximate};
}

void ompl::control::RGRRT::getPlannerData(ompl::base::PlannerData &data) const
{
    ompl::base::Planner::getPlannerData(data);
  
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(ompl::base::PlannerDataVertex(m->parent->state), ompl::base::PlannerDataVertex(m->state),
                            ompl::control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(ompl::base::PlannerDataVertex(m->parent->state), ompl::base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(ompl::base::PlannerDataVertex(m->state));
    }
}
  