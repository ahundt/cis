#ifndef _TERMINATION_CRITERIA_HPP
#define _TERMINATION_CRITERIA_HPP 


#include <cmath>
#include <vector>
#include <iostream>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/rolling_variance.hpp>

namespace acc = boost::accumulators;

struct TerminationCriteriaParams {
    TerminationCriteriaParams():
        meanErrorThreshold(0.01),
        maxErrorThreshold(0.1),
        minVarianceInMeanErrorBetweenIterations(1e-10),
        maxIterationCount(300),
        minIterationCount(10),
        trackProgressEveryNIterations(10){}
    
    double meanErrorThreshold;
    double maxErrorThreshold;
    double minVarianceInMeanErrorBetweenIterations;
    int    maxCount;
    int    maxIterationCount;
    int    minIterationCount;
    
    int    trackProgressEveryNIterations; // 0 means don't track progress
    
};

/// TerminationCirteria is used to evaluate when the ICP algorithm should stop
/// Also evaluates other statistics about the algorighm's execution
struct TerminationCriteria {
    typedef acc::accumulator_set< double, acc::features< acc::tag::min, acc::tag::max, acc::tag::mean, acc::tag::variance, acc::tag::count, acc::tag::rolling_mean, acc::tag::rolling_variance > > accumulator_type;
    
    TerminationCriteria(TerminationCriteriaParams tcp = TerminationCriteriaParams()):
        m_tcp(tcp),
        m_acc(new accumulator_type(acc::tag::rolling_window::window_size = m_tcp.minIterationCount)),
        m_iterationMeanAcc(new accumulator_type(acc::tag::rolling_window::window_size = m_tcp.minIterationCount)){}
    
    void operator()(double i){
        (*m_acc)(i);
    }
    
    bool shouldTerminate(){
        // should have run at least once, plus both the mean and max error should be below a threshold
        bool shouldTerminate_b =
                   (
                      m_tcp.minIterationCount                            < acc::count(*m_iterationMeanAcc) // must meet minimum iteration count
                   && acc::mean(*m_acc)                            < m_tcp.meanErrorThreshold              // the mean error should be sufficiently low
                   && acc::extract_result< acc::tag::max >(*m_acc) < m_tcp.maxErrorThreshold               // the max error should be sufficiently low
                   )
                || (  acc::count(*m_iterationMeanAcc)              > m_tcp.maxIterationCount )             // don't exceed max iterations
                || (
                      m_tcp.minIterationCount                            < acc::count(*m_iterationMeanAcc) // must meet minimum iteration count
                   && acc::rolling_variance(*m_iterationMeanAcc)   < m_tcp.minVarianceInMeanErrorBetweenIterations // if the optimization is having no effect (small rolling variance), terminate
                   );
        
        if(m_tcp.trackProgressEveryNIterations && shouldTerminate_b) {
			std::cout << "\n\n>> Algorithm " << description << " complete. << Final Stats:";
			PrintIterationStats();
			std::cout << "\n";
		}
        
        return shouldTerminate_b;
    }
	
	void PrintIterationStats(){
        if(m_tcp.trackProgressEveryNIterations) {
            std::cout << "\n"
                  <<  "iter: " << acc::count(*m_iterationMeanAcc)                      << "/" << m_tcp.maxIterationCount
				  << " Error"
                  << " mean: "      << acc::mean(*m_acc)                            << "/" << m_tcp.meanErrorThreshold
                  << " max: "       << acc::extract_result< acc::tag::max >(*m_acc) << "/" << m_tcp.maxErrorThreshold
                  << " rVarOfMean: "  << acc::rolling_variance(*m_iterationMeanAcc) << "/" << m_tcp.minVarianceInMeanErrorBetweenIterations
                  << " File: "            << description                                  << "\n";
        }
		
	}
    
    /// resets accumulated statistics, not termination criteria
    void nextIteration(){
       (*m_iterationMeanAcc)(acc::mean(*m_acc));
	   if((acc::count(*m_iterationMeanAcc) % m_tcp.trackProgressEveryNIterations == 0)) PrintIterationStats();
       m_acc.reset(new accumulator_type(acc::tag::rolling_window::window_size = m_tcp.minIterationCount));
    }
    
    
    TerminationCriteriaParams m_tcp;
    std::string description;
    
    boost::shared_ptr<accumulator_type> m_acc;
    boost::shared_ptr<accumulator_type> m_iterationMeanAcc;
    
    
};

#endif