
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
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"
#include "PA1_2_DataConstants.hpp"
#include "DistortionCalibration.hpp"
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
