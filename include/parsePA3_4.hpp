#ifndef _PARSE_CSV_CIS_POINTCLOUD_HPP_
#define _PARSE_CSV_CIS_POINTCLOUD_HPP_

#include "parse.hpp"

struct cisMesh {
	std::vector<Eigen::Vector3d> vertices;
	std::vector<Eigen::VectorXd> vertexTriangleNeighborIndex;
};

struct problemBody {
	std::vector<Eigen::Vector3d> markerLEDs;
	Eigen::Vector3d tip;
};

struct sampleBody {
	std::string title;
	std::vector<std::vector<Eigen::Vector3d> > NA;
	std::vector<std::vector<Eigen::Vector3d> > NB;
	std::vector<std::vector<Eigen::Vector3d> > ND;
};

cisMesh parseMesh(std::string csv, bool debug = false){
    cisMesh outputData;
    if(csv.empty()) return outputData;
       
    std::vector<std::string> strs;
    
    // by default if no frame count is specified, there is only one frame
    int numFrames = 1;
    // split into separate strings by newlines
	boost::split(strs, csv, boost::is_any_of("\n"));
    
    if(debug) {
        //std::cout << "\n\n FULL FILE CONTENTS:\n\n" << csv;  // original file string
        std::cout << "\n\nNewly split vector of strings:\n\n"; // file string post split on newlines
        printStringVector(strs);
    }
    
    
	// parse first line differently
    std::vector<std::string>::iterator currentStringLineIterator = strs.begin();
	std::vector<std::string> firstLineStrings;
	
	boost::split(firstLineStrings, *currentStringLineIterator, boost::is_any_of(", "), boost::token_compress_on);
	++currentStringLineIterator;
	
	// read number of vertices
	int nVertices = boost::lexical_cast<int>(firstLineStrings[0]);
	for(int i = 0; i < nVerticies; ++i, ++currentStringLineIterator){
		outputData.vertices.push_back(Eigen::Vector3d(readPointString(*currentStringLineIterator)));
	}
	
	// read number of triangles
	int nTriangles = boost::lexical_cast<int>(firstLineStrings[0]);
	for(int i = 0; i < nVerticies; ++i, ++currentStringLineIterator){
		outputData.vertexTriangleNeighborIndex.push_back(readPointString(*currentStringLineIterator));
	}
	
	return outputData;
}



problemBody parseProblemBody(std::string csv, bool debug = false){
    problemBody outputData;
    if(csv.empty()) return outputData;
       
    std::vector<std::string> strs;
    
    // by default if no frame count is specified, there is only one frame
    int numFrames = 1;
    // split into separate strings by newlines
	boost::split(strs, csv, boost::is_any_of("\n"));
    
    if(debug) {
        //std::cout << "\n\n FULL FILE CONTENTS:\n\n" << csv;  // original file string
        std::cout << "\n\nNewly split vector of strings:\n\n"; // file string post split on newlines
        printStringVector(strs);
    }
    
    
	// parse first line differently
    std::vector<std::string>::iterator currentStringLineIterator = strs.begin();
	std::vector<std::string> firstLineStrings;
	
	boost::split(firstLineStrings, *currentStringLineIterator, boost::is_any_of(", "), boost::token_compress_on);
	++currentStringLineIterator;
	
	// read number of vertices
	int nVertices = boost::lexical_cast<int>(firstLineStrings[0]);
	for(int i = 0; i < nVerticies; ++i, ++currentStringLineIterator){
		outputData.markerLEDs.push_back(Eigen::Vector3d(readPointString(*currentStringLineIterator)));
	}
	
	outputData.tip = Eigen::Vector3d(readPointString(*currentStringLineIterator));
	
	return outputData;
}

template<typename T>
std::vector<Eigen::Vector3d> readNXrecords(T& iterator, T& end, int NS){
    std::vector<Eigen::Vector3d> sample;
	Eigen::Vector3d zero(0.0,0.0,0.0);
	bool previousWasZero = false;
	bool currentIsZero = false;
	
	// read NA records
	for(int i = 0; (i < NS) && iterator != end; ++i, ++currentStringLineIterator){
		Eigen::Vector3d point(readPointString(*currentStringLineIterator));
		currentIsZero = (point == zero);
		
		if(point != zero && previousWasZero) break; // stop iterating this loop when you hit the last zero points
		else if((point != zero) && (!currentIsZero)) sample.push_back(point); // don't add zero points to the list
		
		previousWasZero = currentIsZero;
	}
	
	return sample;
}


problemBody parseSampleBody(std::string csv, bool debug = false){
    sampleBody outputData;
	
    if(csv.empty()) return outputData;
       
    std::vector<std::string> strs;
    
    // by default if no frame count is specified, there is only one frame
    int numFrames = 1;
    // split into separate strings by newlines
	boost::split(strs, csv, boost::is_any_of("\n"));
    
    if(debug) {
        //std::cout << "\n\n FULL FILE CONTENTS:\n\n" << csv;  // original file string
        std::cout << "\n\nNewly split vector of strings:\n\n"; // file string post split on newlines
        printStringVector(strs);
    }
    
    
	// parse first line differently
    std::vector<std::string>::iterator currentStringLineIterator = strs.begin();
	std::vector<std::string> firstLineStrings;
	
	boost::split(firstLineStrings, *currentStringLineIterator, boost::is_any_of(", "), boost::token_compress_on);
	int NS =  boost::lexical_cast<int>(firstLineStrings[0]);
	int nSamps =  boost::lexical_cast<int>(firstLineStrings[1]);
	
	++currentStringLineIterator;
	
	for(int j = 0; j < nSamps; ++j){
		// read NA records
		outputData.NA.push_back(readNXrecords(currentStringLineIterator,strs.end(),NS));
		// read NB records
		outputData.NB.push_back(readNXrecords(currentStringLineIterator,strs.end(),NS));
		// read ND records
		outputData.ND.push_back(readNXrecords(currentStringLineIterator,strs.end(),NS));
	
    }
	
	return outputData;
}




/// Paths to the files containing source data
struct DataSourcePA3_4 {
	
	static const std::string BodyA = "BodyA";
	static const std::string BodyB = "BodyB";
	static const std::string SampleReadings = "SampleReadings";
	static const std::string Answer = "Answer";
	static const std::string Output = "Output";
	
    // filename prefix that goes in front of the filename string
    std::string  filenamePrefix;
    // map from file type to full paths to the files    
	std::map<std::string,std::string> dataTypeToPath;
	
};

struct ParsedCommandLineCommands {
    bool                    debug;
    bool                    debugParser;
    bool                    threads;
    std::string             outputDataFolderPath;
    std::vector<DataSource> dataSources;
};

#endif // _PARSE_CSV_CIS_POINTCLOUD_HPP_