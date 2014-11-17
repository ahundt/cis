
// Library includes
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <cmath>
#include <thread>

// Project includes
#include "parseCSV_CIS_pointCloud.hpp"
#include "parseCommandLineOptions.hpp"
#include "hornRegistration.hpp"
#include "PointData.hpp"
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"
#include "hwDataConstants.hpp"
#include "DistortionCalibration.hpp"
#include "PA2.hpp"
#include "ICP.hpp"

namespace po = boost::program_options;


/// read the command line options from argc,argv and load them into the params object
/// @see boost::program_options for details on how the command line parsing library works.
bool readCommandLine(int argc, char* argv[], ParsedCommandLineCommands & pclp){


    static const bool optional = false; // false means parameters are not required, and therefore optional
    static const bool required = true;  // true means parameters are required


/**************************************************************************/
/**
 * @brief Main function
 *
 * @param argc  Number of input arguments
 * @param argv  Pointer to input arguments
 *
 * @return int
 */
int main(int argc,char**argv) {
	/////////////////////////////////////////
	// Code Outline once data has been parsed
	/////////////////////////////////////////
	
	std::vector<Eigen::Vector3d> ck;
	Eigen::Matrix4d Freg = Eigen::Matrix4d::Identity();
	for (int i=0; i<k; i++){
		Eigen::Matrix4d Fa = horn(a[k],A); // a: PA3-A-Debug-SampleReadingsTest A: Problem3-BodyA
		Eigen::Matrix4d Fb = horn(B,b[k]); // b: PA3-A-Debug-SampleReadingsTest B: Problem3-BodyB
		Eigen::Vector3d dk = Fb*Fa*Atip; // Atip: Problem3-BodyA (last line)
		for (int j=0; j<numTriangles; j++){
			ck.push_back = ICP(dk,std::vector<Eigen::Vector3d> Triangle_j);
		}
	}
	return 0;
}
