
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



/// Produce an output CIS CSV file
/// Note: I tried to make an output function but it probably has bugs
void output1CISCSV_PA3(std::ostream& ostr, const std::string& outputName = "name-output-3.txt", const std::vector<Eigen::Vector3d> & dk = std::vector<Eigen::Vector3d>(), const std::vector<Eigen::Vector3d> & ck = std::vector<Eigen::Vector3d>(), const std::vector<double> & error = std::vector<Eigen::double>()){
    
    ostr
    << dk.size() << " " << outputName << "\n";
    
    std::vector<Eigen::Vector3d>::const_iterator dIterator = dk.begin();
    std::vector<Eigen::Vector3d>::const_iterator cIterator = ck.begin();
    std::vector<double>::const_iterator eIterator = error.begin();
    for(; dIterator != dk.end() && cIterator != ck.end() && eIterator != error.end();
        ++lIt, ++uIt, ++nIt)
            ostr
            << dIterator << "   " << cIterator << "   " << eIterator << "\n";
        }
    }
    ostr.flush();
    
}


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
		Eigen::Matrix4d FbInverse = horn(B,b[k]); // b: PA3-A-Debug-SampleReadingsTest B: Problem3-BodyB
        Eigen::Affine3d FaAffine = Fa.matrix();
        Eigen::Affine3d FbInverseAffine = FbInverse.matrix();
		dk.push_back = FbInverseAffine*FaAffine*Atip; // Atip: Problem3-BodyA (last line)
		Eigen::Vector3d sk = Freg*dk;
		for (int j=0; j<numTriangles; j++){
			Eigen::Vector3d ckTemp = ICP(dk,std::vector<Eigen::Vector3d> TriangleVertices_j);
			double errorTemp = (tempCk-dk).norm();
			if (errorTemp < errorMin || j == 0){
				ckMin = tempCk;
				errorMin = errorTemp;
			}
		}
		ck.push_back() = ckMin;
		errork.push_back() = errorMin;
	}
	
	// Need output file
	// for each k: dx, dy, dz, cx, cy, cz, error
    /*
    std::string outputFilename =  dataFilenamePrefix + "-output1.txt";
    std::ofstream ofs (outputFilename, std::ofstream::out);
    output1CISCSV_PA3(ofs,outputFilename,ck,dk,error);
    
    ofs.close();
    */

	
	return 0;
}
