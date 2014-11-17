
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
	
	Eigen::Vector3d ckMin;
	double errorMin;
	std::<Eigen::Vector3d> dk;
	std::<Eigen::Vector3d> ck;
	std::<double> errork;
	Eigen::Matrix4d Freg = Eigen::Matrix4d::Identity();
	for (int i=0; i<k; i++){
		// Need to define a[k], b[k], A, and B from parser info
		Eigen::Matrix4d Fa = horn(a[k],A); // a: PA3-A-Debug-SampleReadingsTest A: Problem3-BodyA
		Eigen::Matrix4d Fb = horn(B,b[k]); // b: PA3-A-Debug-SampleReadingsTest B: Problem3-BodyB
		
		// Need to convert homogeneous matrices to .affine() to allow these matrices to multiply by a vector
		dk.push_back = Fb*Fa*Atip; // Atip: Problem3-BodyA (last line)
		Eigen::Vector3d sk = Freg*dk;
		for (int j=0; j<numTriangles; j++){
			Eigen::Vector3d ckTemp = ICP(dk,std::vector<Eigen::Vector3d> TriangleVertices_j);
			double errorTemp = (tempCk-dk).norm();
			if (errorTemp < errorMin || j == 0){
				ckMin = tempCk;
				errorMin = errorTemp;
			}
		}
		ck.push_back = ckMin;
		errork.push_back = errorMin;
	}
	
	// Need output file
	// for each k: dx, dy, dz, cx, cy, cz, error
	
	return 0;
}
