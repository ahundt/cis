
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
#include "hw1Constants.hpp"

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
        // Sorts through the input files and splits it into vectors of vectors
        // trackerIndexedData[0][i] is D and [1][i] is G
        csvCIS_pointCloudData::TrackerFrames trackerIndexedData(swapIndexing(ad.optpivot.frames));

//        Eigen::MatrixXd TestCase = registrationToFirstCloud(ad.optpivot.frames);
//        Eigen::MatrixXd FGTest = TestCase.block<4,4>(4,0);
//        auto littleGTest = std::begin(ad.optpivot.frames);
//        Eigen::MatrixXd = FGTest*littleGTest;

        // opticalMarkersOnEMDeviceInEMFrame = d from calbody
        Eigen::MatrixXd opticalMarkersOnEMDeviceInEMFrame(ad.calbody.frames[0][0]); // A

        // Intializes a vector of Eigen Matrices to store Gnew
        std::vector<Eigen::MatrixXd> Gnew;

        for (int i = 0; i< trackerIndexedData[0].size(); ++i){
            // Confirms that there are two data sets in trackerIndexedData (D and G)
            BOOST_VERIFY(trackerIndexedData.size()==2);
            // only copy first two falues in here
            // perform hornRegistration and add to back of vector

            // input points are in EM Tracker frame
            // output points are in Optical Tracker Frame
            // Find transform from the calibration object reference frame to the EM coordinates of the calibration object

            // trackerIndexedData[0][i] = D for each frame from optpivot
            Eigen::MatrixXd EMMarkersOnCalObjInCalObjFrame(trackerIndexedData[1][i]);

            // trackerIndexedData[0][i] = D for each frame from optpivot
            Eigen::MatrixXd OptMarkersOnEMDeviceInOptFrame(trackerIndexedData[0][i]); // D

            // Creates the Horn Transform from d to D (i.e. FDinv) in each frame
            Eigen::Affine3d OptToEMFrame(hornRegistration( // horn(D,A)
                                                          OptMarkersOnEMDeviceInOptFrame,
                                                          opticalMarkersOnEMDeviceInEMFrame
                                                          ));

            Eigen::MatrixXd calObjPtsInEMTrackerF(EMMarkersOnCalObjInCalObjFrame.rows(),3);
            for(std::size_t j = 0; j < EMMarkersOnCalObjInCalObjFrame.rows(); ++j){
                //  extract calibration Object EM Tracker Positions On Calibration Object into individual points
                Eigen::Vector3d EMMarkerOnCalObjInCalObjFrame(EMMarkersOnCalObjInCalObjFrame.row(j).transpose()); // Extract point from C

                // starting with point on calibration object in calObj frame -> optFrame -> EMFrame
                Eigen::Vector3d calObjEMTrackerPosInEMFrame = OptToEMFrame*EMMarkerOnCalObjInCalObjFrame;

                // would be .row(j) but it is MatrixXd so we need to use .block to specify the size
                // note that the output of the transform multiplication is already in row format
                // so it can be inserted directly

                // Need to make a vector of point clouds to store the new G (Gnew=FDinv*G)
                calObjPtsInEMTrackerF.block<1,3>(j,0) = calObjEMTrackerPosInEMFrame.transpose();
            }
            // Puts the Eigen Matrix Gnew into a vector corresponding to the frame
            Gnew.push_back(calObjPtsInEMTrackerF);
        }

        Eigen::VectorXd result = pivotCalibration(Gnew,debug);
        optPivotPoint = result.block<3,1>(3,0);
        std::cout << "\n\nPivotCalibrationTwoSystems result for " << ad.optpivot.title << ":\n\n" << result << "\n\n";
    }

    std::vector<Eigen::MatrixXd> cExpected;

    if(!ad.calreadings.frames.empty() && !ad.calbody.frames.empty()){

        // a
        cExpected = estimateCExpected(ad.calreadings.frames,ad.calbody.frames,debug);

        std::cout << "\n\nsolveForCExpected results for "<< ad.calreadings.title << " and " << ad.calbody.title <<":\n\n";
        for (auto expected : cExpected)
            std::cout << expected << "\n";

    }


    std::string outputFilename = dataFilenamePrefix + "-output1.txt";
    std::ofstream ofs (outputFilename, std::ofstream::out);
    outputCISCSV(ofs,outputFilename,emPivotPoint,optPivotPoint,cExpected);

    ofs.close();
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
