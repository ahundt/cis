
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
#include "parsePA1_2.hpp"
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"
#include "PA1_2_DataConstants.hpp"
#include "DistortionCalibration.hpp"
#include "PA2.hpp"

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
    ("debugParser","display debug information for data file parser")
    ;


    po::options_description algorithmOptions("Algorithm Options");
    algorithmOptions.add_options()
    ("threads","run each source data file in a separate thread");


    // create algorithm command line options
    //algorithmOptions.add_options()
	// todo, add options for configuring algorithms

    po::options_description dataOptions("Data Options");


    std::string currentPath(boost::filesystem::path( boost::filesystem::current_path() ).string());

    // create algorithm command line options
    dataOptions.add_options()
            ("pa1", "set automatic programming assignment 1 source data parameters, overrides DataFilenamePrefix, exclusive of pa1")
            ("pa2", "set automatic programming assignment 2 source data parameters, overrides DataFilenamePrefix, exclusive of pa2")
    
            ("dataFolderPath"                   ,po::value<std::string>()->default_value(currentPath)       ,"folder containing data files, defaults to current working directory"   )
            ("outputDataFolderPath"             ,po::value<std::string>()->default_value(currentPath)       ,"folder for output data files, defaults to current working directory"   )
            ("dataFilenamePrefix"               ,po::value<std::vector<std::string> >()->default_value(HW2DataFilePrefixes(),""),"constant prefix of data filename path. Specify this multiple times to run on many data sources at once"   )
		  	("dataFileNameSuffix_calbody"       ,po::value<std::string>()->default_value(dataFileNameSuffix_calbody     ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_calreadings"   ,po::value<std::string>()->default_value(dataFileNameSuffix_calreadings ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_empivot"       ,po::value<std::string>()->default_value(dataFileNameSuffix_empivot     ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_optpivot"      ,po::value<std::string>()->default_value(dataFileNameSuffix_optpivot    ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_output1"       ,po::value<std::string>()->default_value(dataFileNameSuffix_output1     ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_ct_fiducials"  ,po::value<std::string>()->default_value(dataFileNameSuffix_ct_fiducials),"suffix of data filename path"   )
		  	("dataFileNameSuffix_em_fiducials"  ,po::value<std::string>()->default_value(dataFileNameSuffix_em_fiducials),"suffix of data filename path"   )
		  	("dataFileNameSuffix_em_nav"        ,po::value<std::string>()->default_value(dataFileNameSuffix_em_nav      ),"suffix of data filename path"   )
		  	("dataFileNameSuffix_output2"       ,po::value<std::string>()->default_value(dataFileNameSuffix_output2     ),"suffix of data filename path"   )
			
				
				
		  	("calbodyPath"                   ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("calreadingsPath"               ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("empivotPath"                   ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("optpivotPath"                  ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("output1Path"                   ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("ct_fiducialsPath"  			 ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("em_fiducialsPath"  			 ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("em_navPath"        			 ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("output2Path"       			 ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
    
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
    
    pclp.debugParser = vmap.count("debugParser");

	parseResponseFiles(vmap,allOptions);

	// initalize string params
	std::string  dataFolderPath
				,dataFileNameSuffix_calbody
				,dataFileNameSuffix_calreadings
				,dataFileNameSuffix_empivot
				,dataFileNameSuffix_optpivot
				,dataFileNameSuffix_output1
				,dataFileNameSuffix_ct_fiducials
				,dataFileNameSuffix_em_fiducials
				,dataFileNameSuffix_em_nav
				,dataFileNameSuffix_output2
				;

    DataSource datasource;

    std::vector<std::string> dataFilenamePrefixList;

    // load up parameter values from the variable map
    po::readOption(vmap,"dataFolderPath"                   ,dataFolderPath                     ,optional);
    po::readOption(vmap,"outputDataFolderPath"             ,pclp.outputDataFolderPath          ,optional);
	po::readOption(vmap,"dataFilenamePrefix"               ,dataFilenamePrefixList             ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calbody"      ,dataFileNameSuffix_calbody         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_calreadings"  ,dataFileNameSuffix_calreadings     ,optional);
	po::readOption(vmap, "dataFileNameSuffix_empivot"      ,dataFileNameSuffix_empivot         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_optpivot"     ,dataFileNameSuffix_optpivot        ,optional);
	po::readOption(vmap, "dataFileNameSuffix_output1"      ,dataFileNameSuffix_output1         ,optional);
	po::readOption(vmap, "dataFileNameSuffix_ct_fiducials" ,dataFileNameSuffix_ct_fiducials    ,optional);
	po::readOption(vmap, "dataFileNameSuffix_em_fiducials" ,dataFileNameSuffix_em_fiducials    ,optional);
	po::readOption(vmap, "dataFileNameSuffix_em_nav"       ,dataFileNameSuffix_em_nav          ,optional);
	po::readOption(vmap, "dataFileNameSuffix_output2"      ,dataFileNameSuffix_output2         ,optional);
    
    // enable threads if specified
    pclp.threads = vmap.count("threads");
    
    if (vmap.count("pa1")) {
        dataFilenamePrefixList = HW1DataFilePrefixes();
    } else if (vmap.count("pa2")) {
        dataFilenamePrefixList = HW2DataFilePrefixes();
    }
    

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
        po::readOption(vmap,"ct_fiducialsPath"                 ,pclp.dataSources[0].ct_fiducialsPath   ,optional);
        po::readOption(vmap,"em_fiducialsPath"                 ,pclp.dataSources[0].em_fiducialsPath   ,optional);
        po::readOption(vmap,"em_navPath"                       ,pclp.dataSources[0].em_navPath         ,optional);
        po::readOption(vmap,"output2Path"                      ,pclp.dataSources[0].output2Path        ,optional);
    }


	// check if the user supplied a full path, if not assemble a path
	// from the default paths and the defualt prefix/suffix combos
    for(auto&& prefix : dataFilenamePrefixList){
        DataSource dataSource;
        dataSource.filenamePrefix = prefix;
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_calbody       ,dataSource.calbodyPath        ,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_calreadings   ,dataSource.calreadingsPath    ,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_empivot       ,dataSource.empivotPath        ,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_optpivot      ,dataSource.optpivotPath       ,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_output1       ,dataSource.output1Path        ,optional);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_ct_fiducials  ,dataSource.ct_fiducialsPath  ,optional);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_em_fiducials  ,dataSource.em_fiducialsPath  ,optional);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_em_nav        ,dataSource.em_navPath        ,optional);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix,dataFileNameSuffix_output2       ,dataSource.output2Path       ,optional);

        pclp.dataSources.push_back(dataSource);
    }

    pclp.debug = vmap.count(CLO_GET_ARG_STR2(CLO_DEBUG));

    return false;
}




/// Produce an output CIS CSV file
void output1CISCSV_PA1(std::ostream& ostr, const std::string& outputName = "name-output-1.txt", const Eigen::Vector3d & emProbe = Eigen::Vector3d(0,0,0), const Eigen::Vector3d & optProbe = Eigen::Vector3d(0,0,0), const csvCIS_pointCloudData::TrackerDevices & vTrackers = csvCIS_pointCloudData::TrackerDevices()){
    
    ostr
    << vTrackers[0].rows() << "," << vTrackers.size() << "," << outputName << "\n"
    << emProbe(0) << "," << emProbe(1) << "," << emProbe(2) << "\n"
    << optProbe(0) << "," << optProbe(1) << "," << optProbe(2) << "\n";
    
    for(auto && tracker : vTrackers) {
        std::size_t rows = tracker.rows();
        for(std::size_t i = 0; i < rows; ++i) {
            ostr
            << tracker.block<1,1>(i,0) << "," << tracker.block<1,1>(i,1) << "," << tracker.block<1,1>(i,2) << "\n";
        }
        
    }
    
    ostr.flush();
    
}




/// Produce an output CIS CSV file
void output2CISCSV_PA2(std::ostream& ostr, const std::string& outputName = "name-output-2.txt", std::vector<Eigen::Vector3d> probeTipPositions = std::vector<Eigen::Vector3d>()){
    
    ostr
    << probeTipPositions.size() << "," << outputName << "\n";
    
    for(auto && pos : probeTipPositions) {
        ostr
        << pos(0) << "," << pos(1) << "," << pos(2) << "\n";
        
    }
    
    ostr.flush();
}


void generateOutputFile(AlgorithmData ad, std::string outputDataFolderPath, std::string dataFilenamePrefix, bool debug = false){

    Eigen::Vector3d emPivotPoint;
    csvCIS_pointCloudData::TrackerDevices EMPtsInEMFrameOnProbe;

    ///////////////////////////////////////////
    // print pivot calibration data of empivot
    if(!ad.empivot.frames.empty()){
        // Note: we know there is only one tracker in this data
        //       so we can run concat to combine the vectors and
        //       and do the calibration for it.
        EMPtsInEMFrameOnProbe = concat(ad.empivot.frames);
        Eigen::VectorXd result = pivotCalibration(EMPtsInEMFrameOnProbe,debug);
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

    // cExpected contains *estimated* Optical Points in EM frame on Calibration Object
    std::vector<Eigen::MatrixXd> cExpected;

    if(!ad.calreadings.frames.empty() && !ad.calbody.frames.empty()){

        cExpected = estimateCExpected(ad.calreadings.frames,ad.calbody.frames,debug);

        if(debug ){
            std::cout << "\n\nsolveForCExpected results for "<< ad.calreadings.title << " and " << ad.calbody.title <<":\n\n";
            for (auto expected : cExpected)
                std::cout << expected << "\n";
        }

    }



    /////////////////////////////////////////////////////////////////////
    // Adding Programming Assignment 2 Code to Main
    /////////////////////////////////////////////////////////////////////

    // q = Values returned by nagivational sensor -> C (from calreadings)
    // p = known 3D ground truth -> Cexpected (already calculated above)

    Eigen::Vector3d dc;
    Eigen::Vector3d ppivotDistortionCorrected;

    // Stacks all of the C values given in calreadings and then normalize
    // Might want to find the max and min in each frame - need to ask Paul
    if(!ad.calreadings.frames.empty() && !ad.empivot.frames.empty()){
        std::size_t numRowsPerTracker = EMPtsInEMFrameOnProbe[0].rows();
        Eigen::MatrixXd undistortedEMPtsInEMFrameOnCalibrationObject = correctDistortionOnSourceData(ad.calreadings.frames,cExpected,EMPtsInEMFrameOnProbe);
        std::vector<Eigen::MatrixXd> splitUndistortedFrames = splitRows(undistortedEMPtsInEMFrameOnCalibrationObject,numRowsPerTracker);
        Eigen::VectorXd result = pivotCalibration(splitUndistortedFrames,debug);
        ppivotDistortionCorrected = result.block<3,1>(3,0);
        dc = result.block<3,1>(0,0);
        std::cout << "\n\nundistorted result:\n\n" << result << "\n\n";
    }
    
    std::vector<Eigen::Vector3d> fiducialPointinEMFrames;
    
    if (!ad.calreadings.frames.empty() && !ad.em_fiducials.frames.empty()){
        // Problem 4
        // Takes positions of EM tracker points on the EM probe in the EM frame when the tip is in a CT fiducial
        // Returns points of the CT fiducial locations in EM frame
        // Need to pass dc too when making it into a function
		
		// see PA2.hpp
        
        std::vector<Eigen::MatrixXd> Gvector = concat(ad.em_fiducials.frames);
        fiducialPointinEMFrames = fiducialPointInEMFrame(Gvector,ad.calreadings.frames,cExpected,dc);

    }
    
    Eigen::Matrix4d Freg;
    
    if (!ad.ct_fiducials.frames.empty()){
        // Problem 5
        // Takes the point clouds of the CT and EM measured at each fiducial
        // Calculates Freg which is the transformation between CT and EM coordinates
        
        // There is only one frame and "tracker" containing points to each CT fiducial in the CT Frame
        Eigen::MatrixXd fiducialPointCloudCT = ad.ct_fiducials.frames[0][0];
        
        Eigen::MatrixXd fiducialPointCloudEM = stackRangeTranspose(fiducialPointinEMFrames);
        
        Freg = hornRegistration(fiducialPointCloudEM,fiducialPointCloudCT);
        std::cout << "\n\nFreg is: \n" << Freg << std::endl;
    }
    
    Eigen::Affine3d freg2;
    freg2.matrix() = Freg;
    for(int i = 0; i < fiducialPointinEMFrames.size(); ++i){
        std::cout << freg2*fiducialPointinEMFrames[i] << "\n";
    }
    
    std::vector<Eigen::Vector3d> probeTipPointinCTFrames;
    
    if (!ad.calreadings.frames.empty() && !ad.em_nav.frames.empty()){
        // Problem 6
        // Takes positions of EM tracker points on the EM probe in the EM frame when the tip is in a CT fiducial
        // Returns points of the CT fiducial locations in the CT frame
        // Need to pass dc too when making it into a function
        
        std::vector<Eigen::MatrixXd> Gvector = concat(ad.em_nav.frames);
        
        Eigen::Affine3d affineFreg;
        affineFreg.matrix() = Freg;
        
        // see PA2.hpp
        probeTipPointinCTFrames = probeTipPointinCTFrame(Gvector,ad.calreadings.frames,cExpected,dc,affineFreg);
        
    }
    
    
    
    std::string outputFilename =  dataFilenamePrefix + "-output1.txt";
    std::ofstream ofs (outputFilename, std::ofstream::out);
    output1CISCSV_PA1(ofs,outputFilename,ppivotDistortionCorrected,optPivotPoint,cExpected);
    
    ofs.close();
    
    if (!ad.em_nav.frames.empty())
    {
        std::string outputFilename = dataFilenamePrefix + "-output2.txt";
        std::ofstream ofs (outputFilename, std::ofstream::out);
        output2CISCSV_PA2(ofs,outputFilename,probeTipPointinCTFrames);
    }
    
    

    //std::cout << "\n\ncEMFMatrix rows: " << TestF.rows() << " cols: " << TestF.cols() << " values:\n\n" << TestF << std::endl;
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
    
    // thread pool to speed up execution
    std::vector<std::thread> th;

    for(auto&& dataSource : pclp.dataSources){
        AlgorithmData ad;
        loadPointCloudFromFile(dataSource.calbodyPath       ,ad.calbody             ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.calreadingsPath   ,ad.calreadings         ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.empivotPath       ,ad.empivot             ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.optpivotPath      ,ad.optpivot            ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.output1Path       ,ad.output1             ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.ct_fiducialsPath  ,ad.ct_fiducials        ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.em_fiducialsPath  ,ad.em_fiducials        ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.em_navPath        ,ad.em_nav              ,pclp.debugParser       );
        loadPointCloudFromFile(dataSource.output2Path       ,ad.output2             ,pclp.debugParser       );

        if(pclp.threads) {
            // run all data sources in separate threads to speed up execution
            th.push_back(std::thread(generateOutputFile,ad, pclp.outputDataFolderPath, dataSource.filenamePrefix,pclp.debug));
        } else {
            generateOutputFile(ad, pclp.outputDataFolderPath, dataSource.filenamePrefix,pclp.debug);
        }
    }
    
    
    //Join the threads with the main thread
    for(auto &t : th){
        t.join();
    }

	return 0;
}
