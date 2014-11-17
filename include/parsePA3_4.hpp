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





#endif // _PARSE_CSV_CIS_POINTCLOUD_HPP_