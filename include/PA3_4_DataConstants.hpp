#ifndef _HW_CONSTANTS_H_
#define _HW_CONSTANTS_H_

#include <string>
#include <vector>



static const std::string relativeDataPath("PA3-5/");
static const std::string relativeOutputDataPath("PA3-5-OUTPUT/");

/////////////////
// PA 3 Constants
/////////////////

static const std::string pa3problemPrefix("Problem3");

static const std::string pa3a("PA3-A-Debug");
static const std::string pa3b("PA3-B-Debug");
static const std::string pa3c("PA3-C-Debug");
static const std::string pa3d("PA3-D-Debug");
static const std::string pa3e("PA3-E-Debug");
static const std::string pa3f("PA3-F-Debug");
static const std::string pa3g("PA3-G-Unknown");
static const std::string pa3h("PA3-H-Unknown");
//static const std::string pa3i("PA3-I-Unknown"); // there is no I in the data set
static const std::string pa3j("PA3-J-Unknown");

std::vector<std::string> PA3DataFilePrefixes(){
    
    std::vector<std::string> defaultPrefixes;
    defaultPrefixes.push_back(pa3a);
    defaultPrefixes.push_back(pa3b);
    defaultPrefixes.push_back(pa3c);
    defaultPrefixes.push_back(pa3d);
    defaultPrefixes.push_back(pa3e);
    defaultPrefixes.push_back(pa3f);
    defaultPrefixes.push_back(pa3g);
    defaultPrefixes.push_back(pa3h);
    //defaultPrefixes.push_back(pa3i); // there is no I in the data set
    defaultPrefixes.push_back(pa3j);
	
	return defaultPrefixes;
}



const std::string DefaultBodyA = "-BodyA.txt";
const std::string DefaultBodyB = "-BodyB.txt";
const std::string DefaultSampleReadings = "-SampleReadingsTest.txt";
const std::string DefaultAnswer = "-Answer.txt";
const std::string DefaultOutput = "-Output.txt";
const std::string DefaultMesh = "Mesh.sur";

#endif // _HW_CONSTANTS_H_