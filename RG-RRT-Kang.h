///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Sunny Kang
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include <vector>
#include <limits>
#include <iostream>

namespace ompl
{
    namespace control
    {
        // TODO: Implement RGRRT as described

        class RGRRT : public ompl::base::Planner
        {
            public:
                RGRRT(const ompl::control::SpaceInformationPtr &si);

                ~RGRRT() override;

                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

                void clear() override;

                void setup() override;

                void setGoalBias(double goalBias){
                    goalBias_ = goalBias;
                }

                double getGoalBias() const{
                    return goalBias_;
                }

                bool getIntermediateStates() const{
                    return addIntermediateStates_;
                }

                void setIntermediateStates(bool addIntermediateStates){
                    addIntermediateStates_ = addIntermediateStates;
                }

                void getPlannerData(ompl::base::PlannerData &data) const override;
  
                template <template <typename T> class NN>
                void setNearestNeighbors()
                {
                    if (nn_ && nn_->size() != 0)
                        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                    clear();
                    nn_ = std::make_shared<ompl::NearestNeighborsLinear<Motion *>>();
                    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
                }
    
            protected:
                class Motion
                {
                    public:
                        Motion(const ompl::control::SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(nullptr)
                        {
                        }

                        Motion(void) : state(nullptr), control(nullptr), steps(0), parent(nullptr)
                        {
                        }

                        ~Motion()
                        {
                        }

                        ompl::base::State *state;
                        ompl::control::Control *control;
                        unsigned int steps;
                        Motion *parent;
                        std::vector<Motion *> reachable;
                };

                void freeMemory();

                double distanceFunction(const Motion* a, const Motion* b) const;

                void setupReachableSet(Motion* const m);

                int selectReachableMotion(const Motion* qnear, const Motion* qrand);

                base::StateSamplerPtr sampler_;
  
                DirectedControlSamplerPtr controlSampler_;
    
                const SpaceInformation *siC_;
    
                std::shared_ptr<NearestNeighborsLinear<Motion *>> nn_;
    
                double goalBias_{0.05};
    
                bool addIntermediateStates_{false};
    
                RNG rng_;
    
                Motion *lastGoalMotion_{nullptr};

                const int RSIZE = 10;
                
                std::vector<double> control_offset_;
        };


    }  // namespace control 
}  // namespace ompl

#endif
