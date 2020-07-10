 
#ifndef _MAKE_CONTINUE_DAE_H_
#define _MAKE_CONTINUE_DAE_H_

#include <vector>

#include <eo>
#include <moeo>

#include <daex.h>
//#include <cli.h>

namespace daex {

void do_make_continue_param( eoParser & parser )
{
    unsigned int mingen = parser.createParam( (unsigned int)10, "gen-min", 
            "Minimum number of iterations", 'n', "Stopping criterions" ).value();
        eo::log << eo::logging << FORMAT_LEFT_FILL_W_PARAM << "mingen" << mingen << std::endl;

    unsigned int steadygen = parser.createParam( (unsigned int)50, "gen-steady", 
            "Number of iterations without improvement", 's', "Stopping criterions" ).value();
        eo::log << eo::logging << FORMAT_LEFT_FILL_W_PARAM << "steadygen" << steadygen << std::endl;

    unsigned int maxgens = parser.createParam( (unsigned int)1000, "gen-max", 
            "Maximum number of iterations", 'x', "Stopping criterions" ).value();
        eo::log << eo::logging << FORMAT_LEFT_FILL_W_PARAM << "maxgens" << maxgens << std::endl;

#ifdef DAE_MO
    int t[5] = {4,1,2,3,4};
    std::vector<double> target( &t[0], &t[0]+5 );
    eoValueParam< std::vector<double> > & pareto_target = parser.createParam( target, "pareto-target", // FIXME better unreachable default value?
            "Stop when this feasible Pareto set is reached",'p', "Stopping criterion");
        eo::log << eo::logging << FORMAT_LEFT_FILL_W_PARAM << "pareto-target";
            for( unsigned int i=0; i<pareto_target.value().size(); ++i){ eo::log << pareto_target.value().at(i) << ", "; }
            eo::log << std::endl;
#endif

}


template <class EOT>
eoCombinedContinue<EOT> & do_make_continue_op( eoParser & parser, eoState & state
#ifdef DAE_MO
        , moeoArchive<EOT>& archive
#endif
    )
{
#ifndef DAE_MO
    unsigned int mingen = parser.valueOf<unsigned int>("gen-min");
    unsigned int steadygen = parser.valueOf<unsigned int>("gen-steady");
#endif
    unsigned int maxgens = parser.valueOf<unsigned int>("gen-max");

    eoGenContinue<EOT>* maxgen = new eoGenContinue<EOT>( maxgens );
    state.storeFunctor( maxgen );

    // combine the continuators
    eoCombinedContinue<EOT>* continuator = new eoCombinedContinue<EOT>( *maxgen );
    state.storeFunctor( continuator );


#ifdef DAE_MO
    typedef typename EOT::ObjectiveVector::Type OVT;
    // if the user asked for a pareto-target continuator
    if ( parser.isItThere("pareto-target") ) {
        std::vector<OVT> pareto_target = parser.valueOf< std::vector<OVT> >("pareto-target");
        // NOTE: we assume that we always want to reach a feasible targeted front
        eoContinue<EOT>* hypcont = new moeoDualHypContinue<EOT>( pareto_target, /*feasible*/true, archive, /*normalize*/true, /*rho*/1.1 ); // FIXME: do we want to normalize?
        state.storeFunctor(hypcont);
        continuator->add(*hypcont);
    }
#else
    eoContinue<EOT>* steadyfit = new eoSteadyFitContinue<EOT>( mingen, steadygen );
    state.storeFunctor(steadyfit);
    continuator->add(*steadyfit);
#endif
    return *continuator;
}


template <class EOT>
eoContinue<EOT> & do_make_continue( eoParser & parser, eoState & state
#ifdef DAE_MO
        , moeoArchive<EOT>& archive
#endif
    )
{
    do_make_continue_param( parser );
    return do_make_continue_op<EOT>( parser, state
#ifdef DAE_MO
        , archive
#endif
        );
}



} // namespace daex

#endif // _MAKE_CONTINUE_DAE_H_
