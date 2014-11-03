#ifndef _PARSE_CSV_CIS_POINTCLOUD_HPP_
#define _PARSE_CSV_CIS_POINTCLOUD_HPP_

// system includes
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <ostream>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

// project includes
#include "matrixOperations.hpp"

template<typename T>
void Print (const std::vector<T>& v, bool newline = true, std::string description = std::string("")){
    if(!description.empty()) std::cout << description;
    if(newline) std::cout << "\n";
    
    for (int i=0; i<v.size();i++){
        std::cout << v[i] << std::endl;
        if(newline) std::cout << "\n"; // this will print all the contents of *features*
    }
}

/// print std::vector<std::string> for debugging
void printStringVector(std::vector<std::string> sv, bool newline = true, std::string description = std::string("")){
    if(!description.empty()) std::cout << description;
    if(newline) std::cout << "\n";
    
    for(std::vector<std::string>::const_iterator i = sv.begin(); i != sv.end(); ++i) {
        // process i
        std::cout << *i << " ";
        if(newline) std::cout << "\n"; // this will print all the contents of *features*
    }
}

/// trim whitespace on the ends of a string and convert the text to a double
double trimAndConvertToDouble(std::string str){
    
    // start by removing spaces and tabs
    boost::trim(str);
    // debug output
    // std::cout << "afterTrim:" << str << "\n";
    // convert string to double
    return boost::lexical_cast<double>(str);
}

Eigen::Vector3d readPointString(const std::string& pointString, bool debug = false){
    // there is data here, load the point in
    std::vector<std::string> pointStrings;
    boost::split( pointStrings, pointString, boost::is_any_of(","), boost::token_compress_on);
    
    // 3 values in a point
    BOOST_VERIFY(pointStrings.size() == 3);
    
    if(debug) printStringVector(pointStrings, "point:");
    
    return Eigen::Vector3d(trimAndConvertToDouble(pointStrings[0]), trimAndConvertToDouble(pointStrings[1]), trimAndConvertToDouble(pointStrings[2]));
}

/// Stores the parsed data from the csv text files
struct csvCIS_pointCloudData {
    typedef Eigen::MatrixXd TrackerPoints;
    typedef std::vector<TrackerPoints> TrackerDevices;
    typedef std::vector<TrackerDevices> TrackerFrames;
    
    /// filename of the file
	std::string title;
    
    /// the number of elements in each "cloud"
	std::vector<int> firstLine;
    
    /// "output" only field for testing
    /// @todo check if we need to do something special for these eigen values in a struct
    Eigen::Vector3d estElectromagneticPostPos;
    /// "output" only field for testing
    /// @todo check if we need to do something special for these eigen values in a struct
    Eigen::Vector3d estOpticalPostPos;
    
    /// vector containing each cloud
    TrackerFrames frames;
    
public:
    /// avoid issues with eigen alignment
    /// @see http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// parse the CIS HW1 point cloud string that was loaded directly from a txt file into a simpler struct
/// @see DataFileFormatDescription.svg for a visual explanation of the file format
///
///
/// There are additional special cases:
/// - 2 numbers -> trackerpoints1, numframes, filename
/// - 3 numbers -> trackerpoints1, trackerpoints2, numframes, filename
/// - 4 numbers -> trackerpoints1, trackerpoints2, trackerpoints3, numframes, filename
/// - "calbody" filename is different: 3 numbers -> trackerpoints1, trackerpoints2, trackerpoints3, filename
/// - "output" filename is differeint: first line is the same but next 2 lines are special points of estimated postion
csvCIS_pointCloudData parseCSV_CIS_pointCloud(std::string csv, bool debug = false){
    csvCIS_pointCloudData outputData;
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
    
    /// @todo the file format could be improved and shouldn't require these special cases
    bool isCtFiducials = boost::algorithm::contains(*strs.begin(), std::string("ct-fiducials"));
    bool isCalBody = boost::algorithm::contains(*strs.begin(), std::string("calbody"));
    // only isCalBody and is CtFiducials do not specify numframes, they assume a fixed value instead.
    bool isNumFramesSpecified = !(isCalBody || isCtFiducials);

	boost::split(firstLineStrings, *currentStringLineIterator, boost::is_any_of(", "), boost::token_compress_on);
    int firstLineIndex = 0;
    for(std::vector<std::string>::iterator beginFL = firstLineStrings.begin();
        beginFL != firstLineStrings.end(); ++beginFL, ++firstLineIndex ){
		if(boost::algorithm::contains(*beginFL, std::string("txt"))){
            // the first 3 numbers are the number of trackers on each object
            // there are expected to be 3 of these numbers (as opposed to trackers) as of HW1
			outputData.title = *beginFL;
        } else if(isNumFramesSpecified && firstLineIndex == firstLineStrings.size()-2) {
            // the number of Frames containing all the tracker markers,
            // essentially like timesteps, in each file.
            // This is the last number and second to last item in the line, except in the "calbody" case.
            if(debug) std::cout << *beginFL <<"\n";
            numFrames = boost::lexical_cast<int>(*beginFL);
            
        } else {
            if(debug) std::cout << *beginFL <<"\n";
			outputData.firstLine.push_back(boost::lexical_cast<int>(*beginFL));
		}
	}
    
    
    // if the filename contains the string output,
    // the first two lines are special estimated positions
    /// @todo the file format could be improved and shouldn't require these special cases
    if(boost::algorithm::contains(*strs.begin(), std::string("output1"))){
        ++currentStringLineIterator;
        outputData.estElectromagneticPostPos = readPointString(*currentStringLineIterator);
        ++currentStringLineIterator;
        outputData.estOpticalPostPos = readPointString(*currentStringLineIterator);
    } else if(boost::algorithm::contains(*strs.begin(), std::string("output2"))){
        // output2 does not specify number of elements per data frame, assume 1
        outputData.firstLine.push_back(1);
    }
    
    
    // move to second line
    ++currentStringLineIterator;
	// parse remaining lines
	
	// need to decrement each point cloud in the file using the number of points specified in the first line
    /// @bug this isn't accounted for correctly, somehow the values end up huge or negative
	std::vector<int> trackerCounterVec;
    // start out on a new frame
	std::vector<int>::iterator trackerCounterVecPointsRemainingIterator = trackerCounterVec.end();
    std::size_t currentPointIndexInThisTracker = 0;
    std::size_t trackerDeviceIndex = 0;
    csvCIS_pointCloudData::TrackerFrames::iterator currentFrameIterator(outputData.frames.end());
    
    /// @todo remove isNewFrame, it is confusing
    bool isNewFrame = true;
    
    
	// go through every string
	/// @todo there is a potential code safety issue here where if the sizes specified in the file don't match up with the length then we could run over the end of arrays.
	for(int stringVectorIndex = 0; currentStringLineIterator != strs.end(); ++currentStringLineIterator, ++stringVectorIndex){
        
        ////////////////////
        // Skip Empty lines
        ////////////////////
        if(currentStringLineIterator->empty()){
            // no data on this line, skip it
            continue; // skip empty lines, consider finding a way to remove the continue
            
        }
        
        /////////////////////////////////////////
        // Move to next Frame if needed
        /////////////////////////////////////////
        
        if(trackerCounterVecPointsRemainingIterator == trackerCounterVec.end() || (trackerCounterVecPointsRemainingIterator == trackerCounterVec.end()-1 && *trackerCounterVecPointsRemainingIterator == 0)){
            // done with this frame, move on to next one
            // back to the first tracker device for this new frame
            trackerDeviceIndex = 0;
            
            // create the counters for the next set of trackers
            trackerCounterVec = outputData.firstLine;
            trackerCounterVecPointsRemainingIterator = trackerCounterVec.begin();
            
            // add the next frame to the vector, which starts out empty
            csvCIS_pointCloudData::TrackerDevices tf;
            outputData.frames.push_back(tf);
            currentFrameIterator = outputData.frames.end()-1;
            isNewFrame = true;
            
            // add the next tracker point cloud to fill out, currently empty
            Eigen::MatrixXd nextCloud(*trackerCounterVecPointsRemainingIterator,3);
            currentFrameIterator->push_back(nextCloud);
            currentPointIndexInThisTracker = 0;
        }
        
        /////////////////////////////////////////
        // move to next Tracker if needed
        /////////////////////////////////////////
        
        if(!isNewFrame && *trackerCounterVecPointsRemainingIterator == 0 ) {
			// go to next tracker
            ++trackerCounterVecPointsRemainingIterator;
            BOOST_VERIFY(trackerCounterVecPointsRemainingIterator!=trackerCounterVec.end());
            // create a matrix large enough to store all the points for this tracker
            // and insert it into the vector
            /// @bug the length isn't accounted for correctly, somehow the values in *currentTrackerPointsRemainingIterato end up huge or negative
            Eigen::MatrixXd nextCloud(*trackerCounterVecPointsRemainingIterator,3);
			currentFrameIterator->push_back(nextCloud);
			++trackerDeviceIndex;
            currentPointIndexInThisTracker = 0;
	    }
        
        /////////////////////////////////////////
        // Add point to this tracker
        /////////////////////////////////////////
        if(*trackerCounterVecPointsRemainingIterator > 0 ){
            
            (*currentFrameIterator)[trackerDeviceIndex].block<1,3>(currentPointIndexInThisTracker,0) = readPointString(*currentStringLineIterator);
            
            (*trackerCounterVecPointsRemainingIterator)--; // decrement point cloud counter for this cloud
            currentPointIndexInThisTracker++;
        }
		
        // s
        isNewFrame = false;
	}
	
    if(debug) std::cout << "numframes: " <<numFrames << " outputdata.frames.size(): " << outputData.frames.size() << std::endl;
    BOOST_VERIFY(numFrames==outputData.frames.size());
    
	return outputData;
}



void loadPointCloudFromFile(std::string fullFilePath, csvCIS_pointCloudData& pointCloud, bool debug = false){
    
    std::stringstream ss;
    ss << std::ifstream( fullFilePath ).rdbuf();
    
    pointCloud = parseCSV_CIS_pointCloud(ss.str(),debug);
}

/// Swap indexing order of vector of vectors, so if it is row major the returned vv will be column major.
/// @todo this is pretty inefficient, but the lengthts can vary. Maybe store the data differently
template<typename T>
const std::vector<std::vector<T> > swapIndexing(const std::vector<std::vector<T> >& uv){
    std::vector<std::vector<T> > vu;
    for(auto uvi : uv){
        int i = 0;
        for(auto vi :uvi){
            if(i == vu.size()) vu.push_back(std::vector<T>());
            vu[i].push_back(vi);
            ++i;
        }
    }
    return vu;
}

/// @brief combine a vector<vector<T> > into a single vector<T>
template<typename T>
const std::vector<T> concat(const std::vector<std::vector<T> >& uv){
    std::vector<T> u;
    for(auto uvi : uv){
        u.insert(u.end(), uvi.begin(), uvi.end());
    }
    return u;
}



#endif // _PARSE_CSV_CIS_POINTCLOUD_HPP_