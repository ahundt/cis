
// Library includes
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <math.h>

// Project includes
#include "parseCSV_CIS_pointCloud.hpp"
#include "parseCommandLineOptions.hpp"
#include "hornRegistration.hpp"
#include "PointData.hpp"
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"
#include "hw1Constants.hpp"
#include "DistortionCalibration.hpp"

namespace po = boost::program_options;


/// read the command line options from argc,argv and load them into the params object
/// @see boost::program_options for details on how the command line parsing library works.
bool readCommandLine(int argc, char* argv[], ParsedCommandLineCommands & pclp){


    static const bool optional = false; // false means parameters are not required, and therefore optional
    static const bool required = true;  // true means parameters are required



    po::options_description generalOptions("General Options");
    generalOptions.add_options()
    ("responseFile", po::value<std::string>(), "File containing additional command line parameters")
    CLO_HELP
    CLO_DEBUG
    ;


    po::options_description algorithmOptions("Algorithm Options");


    // create algorithm command line options
    //algorithmOptions.add_options()
	// todo, add options for configuring algorithms

    po::options_description dataOptions("Data Options");


    std::string currentPath(boost::filesystem::path( boost::filesystem::current_path() ).string());

    // create algorithm command line options
    dataOptions.add_options()
		    ("dataFolderPath"                ,po::value<std::string>()->default_value(currentPath)       ,"folder containing data files, defaults to current working directory"   )
            ("dataFilenamePrefix"            ,po::value<std::vector<std::string> >()->default_value(HW1DataFilePrefixes(),""),"constant prefix of data filename path. Specify this multiple times to run on many data sources at once"   )
		  	("dataFileNameSuffix_calbody"    ,po::value<std::string>()->default_value(dataFileNameSuffix_calbody    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_calreadings",po::value<std::string>()->default_value(dataFileNameSuffix_calreadings),"suffix of data filename path"   )
		  	("dataFileNameSuffix_empivot"    ,po::value<std::string>()->default_value(dataFileNameSuffix_empivot    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_optpivot"   ,po::value<std::string>()->default_value(dataFileNameSuffix_optpivot   ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_output1"    ,po::value<std::string>()->default_value(dataFileNameSuffix_output1    ),"suffix of data filename path"   )
		  	("calbodyPath"                   ,po::value<std::string>() , "full path to data txt file")
		  	("calreadingsPath"               ,po::value<std::string>() , "full path to data txt file")
		  	("empivotPath"                   ,po::value<std::string>() , "full path to data txt file")
		  	("optpivotPath"                  ,po::value<std::string>() , "full path to data txt file")
		  	("output1Path"                   ,po::value<std::string>() , "full path to data txt file")
			  ;

    po::options_description allOptions;
    allOptions.add(generalOptions).add(algorithmOptions).add(dataOptions);

    po::variables_map vmap;

    try
    {
      po::store(po::command_line_parser(argc, argv).options(allOptions).run(), vmap);
      po::notify(vmap);
    }
    catch (std::exception& e)
    {
      std::cerr << "[Error] " << BOOST_CURRENT_FUNCTION << std::endl
      << "    " << e.what() << std::endl
      << std::endl
      << allOptions << std::endl;
      return false;
    }

    if (vmap.count(CLO_GET_ARG_STR2(CLO_HELP)) || argc < 2)
    {
      std::cout << allOptions << std::endl;
      return false;
    }

	parseResponseFiles(vmap,allOptions);

	// initalize string params
	std::string  dataFolderPath
				,dataFileNameSuffix_calbody
				,dataFileNameSuffix_calreadings
				,dataFileNameSuffix_empivot
				,dataFileNameSuffix_optpivot
				,dataFileNameSuffix_output1;

    DataSource datasource;

    std::vector<std::string> dataFilenamePrefixList;

	// load up parameter values from the variable map
	po::readOption(vmap,"dataFolderPath"                   ,dataFolderPath                     ,optional);
	po::readOption(vmap,"dataFilenamePrefix"               ,dataFilenamePrefixList             ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calbody"      ,dataFileNameSuffix_calbody         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calreadings"  ,dataFileNameSuffix_calreadings     ,optional);
	po::readOption(vmap, "dataFileNameSuffix_empivot"      ,dataFileNameSuffix_empivot         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_optpivot"     ,dataFileNameSuffix_optpivot        ,optional);
	po::readOption(vmap, "dataFileNameSuffix_output1"      ,dataFileNameSuffix_output1         ,optional);

    int prefixCount = dataFilenamePrefixList.size();
    if(!prefixCount){
        pclp.dataSources.push_back(DataSource());
    }

    if(prefixCount<=1){
        po::readOption(vmap,"calbodyPath"                      ,pclp.dataSources[0].calbodyPath        ,optional);
        po::readOption(vmap,"calreadingsPath"                  ,pclp.dataSources[0].calreadingsPath    ,optional);
        po::readOption(vmap,"empivotPath"                      ,pclp.dataSources[0].empivotPath        ,optional);
        po::readOption(vmap,"optpivotPath"                     ,pclp.dataSources[0].optpivotPath       ,optional);
        po::readOption(vmap,"output1Path"                      ,pclp.dataSources[0].output1Path        ,optional);
    }


	// check if the user supplied a full path, if not assemble a path
	// from the default paths and the defualt prefix/suffix combos
    for(auto&& prefix : dataFilenamePrefixList){
        DataSource dataSource;
        dataSource.filenamePrefix = prefix;
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_calbody       ,dataSource.calbodyPath,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_calreadings   ,dataSource.calreadingsPath,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_empivot       ,dataSource.empivotPath,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_optpivot      ,dataSource.optpivotPath,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_output1       ,dataSource.output1Path,optional);

        pclp.dataSources.push_back(dataSource);
    }

    pclp.debug = vmap.count(CLO_GET_ARG_STR2(CLO_DEBUG));

    return false;
}




/// Produce an output CIS CSV file
void outputCISCSV(std::ostream& ostr, const std::string& outputName = "NAME-OUTPUT-1.TXT", const Eigen::Vector3d & emProbe = Eigen::Vector3d(0,0,0), const Eigen::Vector3d & optProbe = Eigen::Vector3d(0,0,0), const csvCIS_pointCloudData::TrackerDevices & vTrackers = csvCIS_pointCloudData::TrackerDevices()){

    ostr
    << vTrackers.size() << "," << vTrackers.size() << "," << outputName << "\n"
    << emProbe(0) << "," << emProbe(1) << "," << emProbe(2) << "\n"
    << optProbe(0) << "," << optProbe(1) << "," << optProbe(2) << "\n";

    for(auto && tracker : vTrackers) {
        std::size_t rows = tracker.rows();
        for(std::size_t i = 0; i < rows; ++i) {
            ostr
            << tracker.block<1,1>(i,0) << "," << tracker.block<1,1>(i,1) << "," << tracker.block<1,1>(i,2) << "\n";
        }

    }

}

void hw1GenerateOutputFile(AlgorithmData ad, std::string dataFilenamePrefix, bool debug = false){

    Eigen::Vector3d emPivotPoint;

    ///////////////////////////////////////////
    // print pivot calibration data of empivot
    if(!ad.empivot.frames.empty()){
        csvCIS_pointCloudData::TrackerDevices trackerIndexedData;
        // Note: we know there is only one tracker in this data
        //       so we can run concat to combine the vectors and
        //       and do the calibration for it.
        trackerIndexedData = concat(ad.empivot.frames);
        Eigen::VectorXd result = pivotCalibration(trackerIndexedData,debug);
        emPivotPoint = result.block<3,1>(3,0);
        std::cout << "\n\nPivotCalibration result for " << ad.empivot.title << ":\n\n" << result << "\n\n";
    }

    Eigen::Vector3d optPivotPoint;

    if(!ad.optpivot.frames.empty()){
        std::vector<Eigen::MatrixXd> Gnew;
        Gnew = findNewGInEM(ad.optpivot.frames,ad.calbody.frames,debug);
        Eigen::VectorXd result = pivotCalibration(Gnew,debug);
        optPivotPoint = result.block<3,1>(3,0);
        std::cout << "\n\nPivotCalibration result for " << ad.optpivot.title << ":\n\n" << result << "\n\n";
    }

    std::vector<Eigen::MatrixXd> cExpected;

    if(!ad.calreadings.frames.empty() && !ad.calbody.frames.empty()){

        cExpected = estimateCExpected(ad.calreadings.frames,ad.calbody.frames,debug);

        std::cout << "\n\nsolveForCExpected results for "<< ad.calreadings.title << " and " << ad.calbody.title <<":\n\n";
        for (auto expected : cExpected)
            std::cout << expected << "\n";

    }


    std::string outputFilename = dataFilenamePrefix + "-output1.txt";
    std::ofstream ofs (outputFilename, std::ofstream::out);
    outputCISCSV(ofs,outputFilename,emPivotPoint,optPivotPoint,cExpected);

    ofs.close();


    /////////////////////////////////////////////////////////////////////
    // Adding Programming Assignment 2 Code to Main
    /////////////////////////////////////////////////////////////////////

    // q = Values returned by nagivational sensor -> C (from calreadings)
    // p = known 3D ground truth -> Cexpected (already calculated above)

    Eigen::MatrixXd cEM;

    // Stacks all of the C values given in calreadings and then normalize
    // Might want to find the max and min in each frame - need to ask Paul
    if(!ad.calreadings.frames.empty()){
        static const std::size_t NumMarkers = ad.calreadings.frames[0][2].rows();
        static const std::size_t NumFrames = ad.calreadings.frames.size();
        cEM.resize(NumMarkers*NumFrames,3);
        for (std::size_t outputRow = 0, i = 0; i < NumFrames; outputRow+=NumMarkers, i++){
            Eigen::MatrixXd markerTrackersOnCalBodyInEMFrame=ad.calreadings.frames[i][2];

            // @todo For some reason putting numMarkers in for 27 does not work
            cEM.block<27,3>(outputRow,0) = markerTrackersOnCalBodyInEMFrame;
        }
    Eigen::MatrixXd normalCEM = ScaleToBox(cEM);

        std::cout << "\n\nnormalC in EM results for "<< normalCEM << std::endl;
        std::cout << "\n\nC size is "<< cEM.rows() << std::endl;
        //std::cout << "\n\nCalreadings in first frame is " << ad.calreadings.frames[0][2];
        //std::cout << "\n\nthe size is "<< ad.calreadings.frames[0][2].rows() << std::endl;
    }

    // Testing
    double test = boost::math::binomial_coefficient<double>(3, 1);
    std::cout << "\n\nbinomial coefficient test is " << test << std::endl;
    double a=5.0;
    int b=3;
    int c=1;
    double Btest = BersteinPolynomial(a, b, c);
    std::cout << "\n\nBtest is " << Btest << std::endl;

    Eigen::Vector3d test2;
    test2 << 0, 0.5, 1;
    Eigen::MatrixXd TestF = FMatrix(test2);
    std::cout << "\n\nF is " << TestF.transpose() << std::endl;
    //std::cout << "\n\nThe size of F is " << TestF.rows() << "x" << TestF.cols() <<std::endl;

    Eigen::MatrixXd cEMFMatrix;
    for (int i=0; i<cEM.cols(); i++){
        Eigen::Vector3d vXYZ;
        vXYZ = cEM.block<1,3>(i,0);
        cEMFMatrix = FMatrix(v);
    }
    std::cout << "\n\ncEMFMatrix is " << cEMFMatrix << std::endl;
}

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
	ParsedCommandLineCommands pclp;
	readCommandLine(argc,argv,pclp);

    for(auto&& dataSource : pclp.dataSources){
        AlgorithmData ad;
        loadPointCloudFromFile(dataSource.calbodyPath       ,ad.calbody                    );
        loadPointCloudFromFile(dataSource.calreadingsPath   ,ad.calreadings                );
        loadPointCloudFromFile(dataSource.empivotPath       ,ad.empivot                    );
        loadPointCloudFromFile(dataSource.optpivotPath      ,ad.optpivot                   );
        loadPointCloudFromFile(dataSource.output1Path       ,ad.output1                    );

        hw1GenerateOutputFile(ad, dataSource.filenamePrefix,pclp.debug);
    }

	return 0;
}
