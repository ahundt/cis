#ifndef _HW1_CONSTANTS_H_
#define _HW1_CONSTANTS_H_

#include <string>
#include <vector>

static const std::string relativeDataPath("PA1-2/");
static const std::string pa1debuga("pa1-debug-a");
static const std::string pa1debugb("pa1-debug-b");
static const std::string pa1debugc("pa1-debug-c");
static const std::string pa1debugd("pa1-debug-d");
static const std::string pa1debuge("pa1-debug-e");
static const std::string pa1debugf("pa1-debug-f");
static const std::string pa1debugg("pa1-debug-g");
static const std::string pa1unknownh("pa1-unknown-h");
static const std::string pa1unknowni("pa1-unknown-i");
static const std::string pa1unknownj("pa1-unknown-j");
static const std::string pa1unknownk("pa1-unknown-k");




std::vector<std::string> HW1DataFilePrefixes(){
    
    std::vector<std::string> defaultPrefixes;
    defaultPrefixes.push_back(pa1debuga);
    defaultPrefixes.push_back(pa1debugb);
    defaultPrefixes.push_back(pa1debugc);
    defaultPrefixes.push_back(pa1debugd);
    defaultPrefixes.push_back(pa1debuge);
    defaultPrefixes.push_back(pa1debugf);
    defaultPrefixes.push_back(pa1debugg);
    defaultPrefixes.push_back(pa1unknownh);
    defaultPrefixes.push_back(pa1unknowni);
    defaultPrefixes.push_back(pa1unknownj);
    defaultPrefixes.push_back(pa1unknownk);
	
	return defaultPrefixes;
}

#endif // _HW1_CONSTANTS_H_