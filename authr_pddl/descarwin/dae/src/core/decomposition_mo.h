
#ifndef __DECOMPOSITION_MO_H__
#define __DECOMPOSITION_MO_H__

#include <eo>
#include <moeo>

#include "goal_mo.h"
#include "decomposition.h"
#include "../core/strategies.h"

namespace daex {

/**
 * Definition of the objective vector traits for multi-objective for planning problems
 */
class DecompoMOTraits : public moeoObjectiveVectorTraits
{public:
    /**
     * Returns true if the _ith objective have to be minimzed
     * @param _i index of the objective
     */
    static bool minimizing( int ) { return true; }

    /**
     * Returns true if the _ith objective have to be maximzed
     * @param _i index of the objective
     */
    static bool maximizing( int ) { return false; }

    /**
     * Returns the number of objectives
     */
    static unsigned int nObjectives() { return 2; }
};

/**
 * Definition of the objective vector for multi-objective planning problems: a vector of doubles
 *
 * We use a dual fitness because we want to maintain unfeasible out of scope.
 * We use a maximizing dual fitness because it is the default in EO and that the traits are in charge of minimizing.
 *
 * Replacement of population is determined on the fitness, not the objectives.
 */
typedef moeoDualRealObjectiveVector<DecompoMOTraits, eoMaximizingDualFitness> DecompoMOObjectives;

class DecompositionMO : public DecompositionBase<
        GoalMO, // goal type
        MOEO< // base type
            DecompoMOObjectives, /* ObjectiveVector */
            eoMaximizingDualFitness, /*MOEOFitness*/
            double /*MOEODiversity*/
        >
    >
{
public:
    typedef DecompoMOObjectives ObjectiveVector ;
    typedef eoMaximizingDualFitness Fitness; // default maximizing fitness with feasibility
    typedef double Diversity;
    typedef MOEO<ObjectiveVector,Fitness,Diversity> MOEOType;

    /*    DecompositionMO() : _strategy(set_strategy()) {};
    Strategies::Type set_strategy() {
      std::string strat = parser.valueOf<std::string>("strategy");
      double proba_strategy = parser.valueOf<double>("proba-strategy");
      if (strat == "random") {eo::log << eo::progress << " = random" << std::endl;
	std::vector<Strategies::Type> default_strategies;
	default_strategies.push_back(Strategies::length);
	default_strategies.push_back(Strategies::cost);
	default_strategies.push_back(Strategies::makespan_max);
	default_strategies.push_back(Strategies::makespan_add);
	std::vector<double> rates;
	double rate = 1.0/default_strategies.size();
	for (unsigned int i=0; i < default_strategies.size(); ++i) {rates.push_back(rate);}
	return default_strategies[rng.roulette_wheel(rates)];}
      else if (strat == "flip-decomposition") {eo::log << eo::progress << " = flip-decomposition" << std::endl;
	if (rng.flip(proba_strategy)) {return Strategies::makespan_add;}
	else {return Strategies::cost;}}
      else if (strat == "length") {eo::log << eo::progress << " = length" << std::endl;
	return Strategies::length;}
      else if (strat == "cost") {eo::log << eo::progress << " = cost" << std::endl;
	return Strategies::cost;}
      else if (strat == "makespan-max") {eo::log << eo::progress << " = makespan-max" << std::endl;
	return Strategies::makespan_max;}
      else if (strat == "makespan-add") {eo::log << eo::progress << " = makespan-add" << std::endl;
	return Strategies::makespan_add;}
      else {throw std::runtime_error("Unknown MO search strategy!");}
      }*/

    std::string strategy() {return _strategy;}
    void strategy(std::string s) {_strategy = s;}
    double proba_strategy() {return _proba_strategy;}
    void proba_strategy (double p) {_proba_strategy = p;}

    eoserial::Object* pack(void) const
    {
        eoserial::Object* json = new eoserial::Object;

        json->add( "goals_number", eoserial::make( this->size() ) );
        json->add( "goals_evaluated", eoserial::make(_k) );
        json->add( "goals_useful", eoserial::make(_u) );

        // list<GoalMO>
        eoserial::Array* listGoal = eoserial::makeArray
            < std::list<GoalMO>, eoserial::SerializablePushAlgorithm >
            ( *this );
        json->add( "goals", listGoal );

        // subplans
        eoserial::Array* subplans = eoserial::makeArray
            < std::vector< Plan >, eoserial::SerializablePushAlgorithm >
            ( _plans_sub );
        json->add( "subplans", subplans );

        json->add( "b_max", eoserial::make(_b_max) );
        json->add( "attempts", eoserial::make(_B) );

        bool invalid_objective_vector = MOEOType::invalidObjectiveVector();
        json->add( "invalid_objective_vector", eoserial::make(invalid_objective_vector) );
        if( !invalid_objective_vector ) {
            json->add( "feasible_objectives", eoserial::make( this->objectiveVector().is_feasible() ) );

            // must be double, or else the direct serialization would not work
            // thus we use the implicit casting of eoDualFitness
            double objective_makespan = this->objectiveVector(0);
            json->add( "objective_makespan", eoserial::make(objective_makespan) );

            double objective_cost = this->objectiveVector(1);
            json->add( "objective_cost", eoserial::make(objective_cost) );
        }

        bool invalid_fitness = MOEOType::invalidFitness();
        json->add( "invalid_fitness", eoserial::make(invalid_fitness) );
        if( !invalid_fitness ) {
            json->add( "feasible_fitness", eoserial::make( this->MOEOType::fitness().is_feasible() ) );
            json->add( "fitness", eoserial::make(this->MOEOType::fitness() ) );
            if( this->MOEOType::fitness().is_feasible() ) {
                // specific members
                json->add( "plan_global", &_plan_global );
            }
        }

        bool invalid_diversity = MOEOType::invalidDiversity();
        json->add( "invalid_diversity", eoserial::make(invalid_diversity) );
        if( !invalid_diversity ) {
            Diversity diversity = this->diversity();
            json->add( "diversity", eoserial::make(diversity) );
        }

        return json;
    }

    void unpack( const eoserial::Object* json )
    {

        eoserial::unpack( *json, "goals_evaluated", _k );
        eoserial::unpack( *json, "goals_useful", _u );

        // list<Goal>
        clear();
        eoserial::unpackArray
            < std::list< GoalMO >, eoserial::Array::UnpackObjectAlgorithm >
            ( *json, "goals", *this );

        // _plans_sub
        _plans_sub.clear();
        eoserial::unpackArray
            < std::vector< daex::Plan >, eoserial::Array::UnpackObjectAlgorithm >
            ( *json, "subplans", _plans_sub );

        eoserial::unpack( *json, "b_max", _b_max );
        eoserial::unpack( *json, "attempts", _B );

        bool invalid_objective_vector;
        eoserial::unpack( *json, "invalid_objective_vector", invalid_objective_vector );
        if (invalid_objective_vector) {
            MOEOType::invalidateObjectiveVector();
        } else {
            // in MOEO, the fitness is computed only by the Pareto archive
            // here, we only need the objectives and the diversity
            double objective_makespan;
            eoserial::unpack( *json, "objective_makespan", objective_makespan );

            double objective_cost;
            eoserial::unpack( *json, "objective_cost", objective_cost);

            bool feasible_objectives;
            eoserial::unpack( *json, "feasible_objectives", feasible_objectives );

            this->objectiveVector(0, std::pair<double,bool>(objective_makespan,feasible_objectives));
            this->objectiveVector(1, std::pair<double,bool>(objective_cost,    feasible_objectives));
            this->objectiveVector().is_feasible( feasible_objectives );
        }

        bool invalid_fitness;
        eoserial::unpack( *json, "invalid_fitness", invalid_fitness );
        if( invalid_fitness ) {
            MOEOType::invalidateFitness();

        } else {
            double fit;
            bool feasible_fitness;
            eoserial::unpack( *json, "feasible_fitness", feasible_fitness );
            eoserial::unpack( *json, "fitness", fit );
            this->fitness( std::make_pair<double,bool>(fit,feasible_fitness) );
            if( feasible_fitness ) {
                // specific members
                eoserial::unpackObject( *json, "plan_global", _plan_global );
            }
        }

        bool invalid_diversity;
        eoserial::unpack( *json, "invalid_diversity", invalid_diversity );
        if( invalid_diversity ) {
            MOEOType::invalidateDiversity();
        } else {
            Diversity diversity;
            eoserial::unpack( *json, "diversity", diversity );
            this->diversity( diversity );
        }
    }

    virtual void readFrom(std::istream & is) {eoserial::readFrom( *this, is );}
    virtual void printOn(std::ostream & out) const {eoserial::printOn( *this, out );}

 protected:
    //! The objective search strategy to be used by YAHSP at evaluation time: either length, cost, makespan_max or makespan_add.
    //    Strategies::Type _strategy;
    std::string _strategy;
    double _proba_strategy;
};

} // namespace daex

#endif

