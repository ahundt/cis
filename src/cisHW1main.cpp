
// Library includes
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>

// Project includes
#include "parseCSV_CIS_pointCloud.hpp"
#include "parseCommandLineOptions.hpp"
#include "hornRegistration.hpp"
#include "PointData.hpp"
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"


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
		  	("dataFilenamePrefix"            ,po::value<std::string>()->default_value("pa1-debug-a"     ),"constant prefix of data filename path"   )
		  	("dataFileNameSuffix_calbody"    ,po::value<std::string>()->default_value("-calbody.txt"    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_calreadings",po::value<std::string>()->default_value("-calreadings.txt"),"suffix of data filename path"   )
		  	("dataFileNameSuffix_empivot"    ,po::value<std::string>()->default_value("-empivot.txt"    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_optpivot"   ,po::value<std::string>()->default_value("-optpivot.txt"   ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_output1"    ,po::value<std::string>()->default_value("-output1.txt"    ),"suffix of data filename path"   )
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

	// load up parameter values from the variable map
	po::readOption(vmap,"dataFolderPath"                   ,dataFolderPath                     ,optional);
	po::readOption(vmap,"dataFilenamePrefix"               ,pclp.dataFilenamePrefix            ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calbody"      ,dataFileNameSuffix_calbody         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calreadings"  ,dataFileNameSuffix_calreadings     ,optional);
	po::readOption(vmap, "dataFileNameSuffix_empivot"      ,dataFileNameSuffix_empivot         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_optpivot"     ,dataFileNameSuffix_optpivot        ,optional);
	po::readOption(vmap, "dataFileNameSuffix_output1"      ,dataFileNameSuffix_output1         ,optional);


    po::readOption(vmap,"calbodyPath"                      ,pclp.calbodyPath        ,optional);
    po::readOption(vmap,"calreadingsPath"                  ,pclp.calreadingsPath    ,optional);
	po::readOption(vmap,"empivotPath"                      ,pclp.empivotPath        ,optional);
	po::readOption(vmap,"optpivotPath"                     ,pclp.optpivotPath       ,optional);
	po::readOption(vmap,"output1Path"                      ,pclp.output1Path        ,optional);


	// check if the user supplied a full path, if not assemble a path
	// from the default paths and the defualt prefix/suffix combos
	assmblePathIfFullPathNotSupplied(dataFolderPath,pclp.dataFilenamePrefix,dataFileNameSuffix_calbody       ,pclp.calbodyPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,pclp.dataFilenamePrefix,dataFileNameSuffix_calreadings   ,pclp.calreadingsPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,pclp.dataFilenamePrefix,dataFileNameSuffix_empivot       ,pclp.empivotPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,pclp.dataFilenamePrefix,dataFileNameSuffix_optpivot      ,pclp.optpivotPath);
	assmblePathIfFullPathNotSupplied(dataFolderPath,pclp.dataFilenamePrefix,dataFileNameSuffix_output1       ,pclp.output1Path);

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

    AlgorithmData ad;
    loadPointCloudFromFile(pclp.calbodyPath       ,ad.calbody                    );
    loadPointCloudFromFile(pclp.calreadingsPath   ,ad.calreadings                );
    loadPointCloudFromFile(pclp.empivotPath       ,ad.empivot                    );
    loadPointCloudFromFile(pclp.optpivotPath      ,ad.optpivot                   );
    loadPointCloudFromFile(pclp.output1Path       ,ad.output1                    );

    Eigen::Vector3d emPivotPoint;

    ///////////////////////////////////////////
    // print pivot calibration data of empivot
    if(!ad.empivot.frames.empty()){
		csvCIS_pointCloudData::TrackerDevices trackerIndexedData;
		// Note: we know there is only one tracker in this data
		//       so we can run concat to combine the vectors and
		//       and do the calibration for it.
		trackerIndexedData = concat(ad.empivot.frames);
        Eigen::VectorXd result = pivotCalibration(trackerIndexedData,pclp.debug);
        emPivotPoint = result.block<3,1>(3,0);
		std::cout << "\n\nPivotCalibration result for " << ad.empivot.title << ":\n\n" << result << "\n\n";
	}

    Eigen::Vector3d optPivotPoint;
    
	if(!ad.optpivot.frames.empty()){
		csvCIS_pointCloudData::TrackerFrames trackerIndexedData(swapIndexing(ad.optpivot.frames));
		// Note: we know there is only one tracker in this data
		//       so we can run concat to combine the vectors and
		//       and do the calibration for it.
		Eigen::VectorXd result = pivotCalibrationTwoSystems(trackerIndexedData[0],trackerIndexedData[1],pclp.debug);
        optPivotPoint = result.block<3,1>(3,0);
		std::cout << "\n\nPivotCalibrationTwoSystems result for " << ad.optpivot.title << ":\n\n" << result << "\n\n";
	}

    std::vector<Eigen::MatrixXd> cExpected;
    
    if(!ad.calreadings.frames.empty() && !ad.calbody.frames.empty()){

        // a
        cExpected = estimateCExpected(ad.calreadings.frames,ad.calbody.frames,pclp.debug);


        std::cout << "\n\nsolveForCExpected results for "<< ad.calreadings.title << " and " << ad.calbody.title <<":\n\n";
        for (auto expected : cExpected)
            std::cout << expected << "\n";
    }
    
    
    std::string outputFilename = pclp.dataFilenamePrefix + "-OUTPUT1.txt";
    std::ofstream ofs (outputFilename, std::ofstream::out);
    
    outputCISCSV(ofs,outputFilename,emPivotPoint,optPivotPoint,cExpected);
    
    ofs.close();

	return 0;
}
