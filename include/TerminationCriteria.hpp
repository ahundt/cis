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

/// parameters for the TerminationCriteria object
/// this struct is useful to facilitate reading
/// parameters from the command line or other sources
struct TerminationCriteriaParams {
    TerminationCriteriaParams():
        meanErrorThreshold(0.01),
        maxErrorThreshold(0.1),
        minVarianceInMeanErrorBetweenIterations(1e-10),
        maxIterationCount(300),
        minIterationCount(10),
        trackProgressEveryNIterations(10){}
    
    /// Stop the algorithm once the mean error is *below* this amount
    double meanErrorThreshold;
    
    /// Stop the algorithm once the max error is *below* this amount
    double maxErrorThreshold;
    
    /// The mean error is calculated on each iteration.
    /// Over a series of iterations, a rolling variance is calculated
    /// for the mean error. If the variance becomes too small, this means
    /// that the mean is no longer improving, so the algorithm should stop.
    ///
    /// Typically use a very small value like 1e-10
    double minVarianceInMeanErrorBetweenIterations;
    
    /// the maximum number of iterations to run
    int    maxIterationCount;
    
    /// the minimum number of iterations to run
    int    minIterationCount;
    
    /// Print out the progress and stats every Nth iteration.
    /// 0 means don't track progress.
    int    trackProgressEveryNIterations;
    
};

/// TerminationCirteria is used to evaluate when the ICP algorithm should stop
/// Also evaluates other statistics about the algorighm's execution
struct TerminationCriteria {
    typedef acc::accumulator_set< double, acc::features< acc::tag::min, acc::tag::max, acc::tag::mean, acc::tag::variance, acc::tag::count, acc::tag::rolling_mean, acc::tag::rolling_variance > > accumulator_type;
    
    /// constructor
    TerminationCriteria(TerminationCriteriaParams tcp = TerminationCriteriaParams()):
        m_tcp(tcp),
        m_acc(new accumulator_type(acc::tag::rolling_window::window_size = m_tcp.minIterationCount)),
        m_iterationMeanAcc(new accumulator_type(acc::tag::rolling_window::window_size = m_tcp.minIterationCount)){}
    
    /// operator() add a new error data point
    ///
    /// @param error the amount of error found in the data point
    void operator()(double error){
        BOOST_VERIFY(!boost::math::isnan(error));
        (*m_acc)(error);
    }
    
    /// Returns true if the termination criteria has been met
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
	
    /// prints various stats about the collected error data for the most recent iteration
	void PrintIterationStats(){
        if(m_tcp.trackProgressEveryNIterations) {
            std::cout << "\n"
                  <<  "iter: "        << acc::count(*m_iterationMeanAcc)              << "/" << m_tcp.maxIterationCount
				  << " Error"
                  << " mean: "        << acc::mean(*m_acc)                            << "/" << m_tcp.meanErrorThreshold
                  << " max: "         << acc::extract_result< acc::tag::max >(*m_acc) << "/" << m_tcp.maxErrorThreshold
                  << " var: "         << acc::variance(*m_acc)
                  << " rVarOfMean: "  << acc::rolling_variance(*m_iterationMeanAcc)   << "/" << m_tcp.minVarianceInMeanErrorBetweenIterations
                  << " File: "        << description                                  << "\n";
        }
		
	}
    
    /// resets accumulated statistics, not termination criteria
    void nextIteration(){
       double meanErrorForIteration = acc::mean(*m_acc);
       BOOST_VERIFY(!boost::math::isnan(meanErrorForIteration));
       (*m_iterationMeanAcc)(meanErrorForIteration);
	   if((acc::count(*m_iterationMeanAcc) % m_tcp.trackProgressEveryNIterations == 0)) PrintIterationStats();
       m_acc.reset(new accumulator_type(acc::tag::rolling_window::window_size = m_tcp.minIterationCount));
    }
    
    
    TerminationCriteriaParams m_tcp;
    std::string description;
    
    boost::shared_ptr<accumulator_type> m_acc;
    boost::shared_ptr<accumulator_type> m_iterationMeanAcc;
    
    
};

#endif