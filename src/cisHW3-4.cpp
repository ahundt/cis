
// Library includes
#include <string>
#include <ostream>
#include <iostream>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/tokenizer.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <boost/filesystem.hpp>
#include <cmath>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

// Project includes
#include "parseCommandLineOptions.hpp"
#include "hornRegistration.hpp"
#include "PivotCalibration.hpp"
#include "PA3_4_DataConstants.hpp"
#include "parsePA3_4.hpp"
#include "DistortionCalibration.hpp"
#include "ICP.hpp"

namespace po = boost::program_options;



/// Produce an output CIS CSV file
/// Note: I tried to make an output function but it probably has bugs
template<typename T>
void output1CISCSV_PA3(std::ostream& ostr, const std::string& outputName = "name-output-3.txt", const T & dk = std::vector<Eigen::Vector3d>(), const T & ck = std::vector<Eigen::Vector3d>(), const std::vector<double> & error = std::vector<double>()){
    
    ostr << dk.size() << " " << outputName << "\n";
    
    typename T::const_iterator dIterator = dk.begin();
    typename T::const_iterator cIterator = ck.begin();
    std::vector<double>::const_iterator eIterator = error.begin();
    for(; dIterator != dk.end() && cIterator != ck.end() && eIterator != error.end();
        ++dIterator, ++cIterator, ++eIterator){
            ostr  << (*dIterator)(0) << "   "  << (*dIterator)(1) << "  "  << (*dIterator)(2) << "  " << (*cIterator)(0) << "   "  << (*cIterator)(1) << "  "  << (*cIterator)(2) << "  " << *eIterator << "\n";
    }

    ostr.flush();
    
}


/// read the command line options from argc,argv and load them into the params object
/// @see boost::program_options for details on how the command line parsing library works.
bool readCommandLine(int argc, char* argv[], ParsedCommandLineCommandsPA3_4 & pclp){


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
            ("pa3", "set automatic programming assignment 3 source data parameters, overrides DataFilenamePrefix, exclusive of pa4")
            ("pa4", "set automatic programming assignment 4 source data parameters, overrides DataFilenamePrefix, exclusive of pa3")
    
            ("dataFolderPath"                   ,po::value<std::string>()->default_value(currentPath)       ,"folder containing data files, defaults to current working directory"   )
            ("outputDataFolderPath"             ,po::value<std::string>()->default_value(currentPath)       ,"folder for output data files, defaults to current working directory"   )
            ("dataFilenamePrefix"               ,po::value<std::vector<std::string> >()->default_value(PA3DataFilePrefixes(),""),"constant prefix of data filename path. Specify this multiple times to run on many data sources at once"   )
			("dataFilenameProblemPrefix"               ,po::value<std::string >()->default_value(pa3problemPrefix),"constant prefix of data typically starting with \"Problem\" filename path. Specify this multiple times to run on many data sources at once"   )
		  	("suffixAnswer"    ,po::value<std::string>()->default_value(DefaultAnswer         ),"suffix of data filename path"   )
		  	("suffixOutput"    ,po::value<std::string>()->default_value(DefaultOutput         ),"suffix of data filename path"   )
		  	("suffixSample"    ,po::value<std::string>()->default_value(DefaultSampleReadings ),"suffix of data filename path"   )
		  	("suffixMesh"      ,po::value<std::string>()->default_value(DefaultMesh           ),"suffix of data filename path"   )
		  	("suffixBodyA"     ,po::value<std::string>()->default_value(DefaultBodyA          ),"suffix of data filename path"   )
		  	("suffixBodyB"     ,po::value<std::string>()->default_value(DefaultBodyB          ),"suffix of data filename path"   )
			
				
				
		  	("AnswerPath"                    ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("OutputPath"                    ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("SamplePath"                    ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("MeshPath"                      ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("BodyAPath"                     ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
		  	("BodyBPath"  			         ,po::value<std::string>() , "full path to data txt file, optional alternative to prefix+suffix name combination"   )
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
				,dataFileNameSuffix_AnswerPath  
				,dataFileNameSuffix_OutputPath   
				,dataFileNameSuffix_SamplePath  
				,dataFileNameSuffix_MeshPath  
				,dataFileNameSuffix_BodyAPath   
				,dataFileNameSuffix_BodyBPath 
				;

    DataSourcePA3_4 datasource;

    std::vector<std::string> dataFilenamePrefixList;
	std::string dataFilenameProblemPrefix;

    // load up parameter values from the variable map
    po::readOption(vmap, "dataFolderPath"                  ,dataFolderPath                     ,optional);
    po::readOption(vmap, "outputDataFolderPath"            ,pclp.outputDataFolderPath          ,optional);
	po::readOption(vmap, "dataFilenamePrefix"              ,dataFilenamePrefixList             ,optional);
	po::readOption(vmap, "dataFilenameProblemPrefix"       ,dataFilenameProblemPrefix          ,optional);
	po::readOption(vmap, "suffixAnswer"                    ,dataFileNameSuffix_AnswerPath      ,optional);
	po::readOption(vmap, "suffixOutput"                    ,dataFileNameSuffix_OutputPath      ,optional);
	po::readOption(vmap, "suffixSample"                    ,dataFileNameSuffix_SamplePath      ,optional);
	po::readOption(vmap, "suffixMesh"                      ,dataFileNameSuffix_MeshPath        ,optional);
	po::readOption(vmap, "suffixBodyA"                     ,dataFileNameSuffix_BodyAPath       ,optional);
	po::readOption(vmap, "suffixBodyB"                     ,dataFileNameSuffix_BodyBPath       ,optional);
    
    // enable threads if specified
    pclp.threads = vmap.count("threads");
    
    if (vmap.count("pa3")) {
        dataFilenamePrefixList = PA3DataFilePrefixes();
    } else if (vmap.count("pa4")) {
        dataFilenamePrefixList = PA3DataFilePrefixes();
    }
    

    int prefixCount = dataFilenamePrefixList.size();
    if(!prefixCount){
        pclp.dataSources.push_back(DataSourcePA3_4());
    }

    if(prefixCount<=1){
        po::readOption(vmap,"calbodyPath"                      ,pclp.dataSources[0].BodyA            ,optional);
        po::readOption(vmap,"calreadingsPath"                  ,pclp.dataSources[0].BodyB            ,optional);
        po::readOption(vmap,"empivotPath"                      ,pclp.dataSources[0].SampleReadings   ,optional);
        po::readOption(vmap,"optpivotPath"                     ,pclp.dataSources[0].Answer           ,optional);
        po::readOption(vmap,"output1Path"                      ,pclp.dataSources[0].Output           ,optional);
        po::readOption(vmap,"ct_fiducialsPath"                 ,pclp.dataSources[0].Mesh             ,optional);
    }


	// check if the user supplied a full path, if not assemble a path
	// from the default paths and the defualt prefix/suffix combos
    for(auto&& prefix : dataFilenamePrefixList){
        DataSourcePA3_4 dataSource;
        dataSource.filenamePrefix = prefix;
        dataSource.filenameProblemPrefix = dataFilenameProblemPrefix;
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenameProblemPrefix       ,dataFileNameSuffix_BodyAPath   ,dataSource.BodyA            ,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenameProblemPrefix       ,dataFileNameSuffix_BodyBPath   ,dataSource.BodyB            ,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix              ,dataFileNameSuffix_SamplePath  ,dataSource.SampleReadings   ,required);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix              ,dataFileNameSuffix_AnswerPath  ,dataSource.Answer           ,optional);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenamePrefix              ,dataFileNameSuffix_OutputPath  ,dataSource.Output           ,optional);
        assemblePathIfFullPathNotSupplied(dataFolderPath,dataSource.filenameProblemPrefix       ,dataFileNameSuffix_MeshPath    ,dataSource.Mesh             ,required);

        pclp.dataSources.push_back(dataSource);
    }

    pclp.debug = vmap.count(CLO_GET_ARG_STR2(CLO_DEBUG));

    return false;
}

void generateOutputFilePA3_4(AlgorithmDataPA3_4 ad, std::string outputDataFolderPath, std::string dataFilenamePrefix, bool debug = false){
	
	Eigen::Vector3d ckMin;
	std::vector<Eigen::Vector3d> dk;
	std::vector<Eigen::Vector3d> ck;
	std::vector<double> errork;
	Eigen::Affine3d Freg;
    Freg.setIdentity();
    
    for (int i=0; i<ad.sampleReadings.NA.size(); i++){
        double errorMin=std::numeric_limits<double>::max();
		Eigen::Affine3d FaAffine(hornRegistration(ad.bodyA.markerLEDs,ad.sampleReadings.NA[i])); // a: PA3-A-Debug-SampleReadingsTest A: Problem3-BodyA
		Eigen::Affine3d FbInverseAffine(hornRegistration(ad.sampleReadings.NB[i], ad.bodyB.markerLEDs)); // b: PA3-A-Debug-SampleReadingsTest B: Problem3-BodyB
        dk.push_back(Eigen::Vector3d(FbInverseAffine*FaAffine*ad.bodyA.tip)); // Atip: Problem3-BodyA (last line)
		Eigen::Vector3d sk = Freg*dk[i];
        for (auto&& triangle : ad.mesh.vertexTriangleNeighborIndex){
			Eigen::Vector3d ckTemp = FindClosestPoint(sk, ad.mesh.vertices, triangle);
			double errorTemp = (ckTemp-dk[i]).norm();
			if (errorTemp < errorMin){
				ckMin = ckTemp;
				errorMin = errorTemp;
			}
		}
		ck.push_back(ckMin);
		errork.push_back(errorMin);
	}
    
    std::string outputFilename =  dataFilenamePrefix + "-Output.txt";
    std::ofstream ofs (outputFilename, std::ofstream::out);
    output1CISCSV_PA3(ofs,outputFilename,dk,ck,errork);
    
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

	ParsedCommandLineCommandsPA3_4 pclp;
	readCommandLine(argc,argv,pclp);
    
    // thread pool to speed up execution
    std::vector<std::thread> th;

    for(auto&& dataSource : pclp.dataSources){
        AlgorithmDataPA3_4 ad;
        ad.bodyA          = parseProblemBody    (loadStringFromFile(dataSource.BodyA)            ,pclp.debugParser       );
        ad.bodyB          = parseProblemBody    (loadStringFromFile(dataSource.BodyB)            ,pclp.debugParser       );
		ad.mesh           = parseMesh           (loadStringFromFile(dataSource.Mesh)             ,pclp.debugParser       );
		ad.sampleReadings = parseSampleReadings (loadStringFromFile(dataSource.SampleReadings)   , ad.bodyA.markerLEDs.rows(), ad.bodyB.markerLEDs.rows() ,pclp.debugParser       );

        if(pclp.threads) {
            // run all data sources in separate threads to speed up execution
            th.push_back(std::thread(generateOutputFilePA3_4,ad, pclp.outputDataFolderPath, dataSource.filenamePrefix,pclp.debug));
        } else {
            generateOutputFilePA3_4(ad, pclp.outputDataFolderPath, dataSource.filenamePrefix,pclp.debug);
        }
    }
    
    
    //Join the threads with the main thread
    for(auto &t : th){
        t.join();
    }
	
	return 0;
}



