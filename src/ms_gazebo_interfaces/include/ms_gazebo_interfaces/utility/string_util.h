// ***
// *** string operations
// *** Auther: xixun wang
// *** Purpose: easy tool of operate string
// *** update 20240424: create file
// *** 

#ifndef STRING_UTIL_H_
#define STRING_UTIL_H_

// *** std include
#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;

void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
	std::string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while(std::string::npos != pos2){
		v.push_back(s.substr(pos1, pos2-pos1));
		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if(pos1 != s.length()){
		v.push_back(s.substr(pos1));
	}
}
void String2Doubles(const std::string& s, std::vector<double>& n)
{
	n.clear();
	std::vector<std::string> v;
	SplitString(s, v, ",");
	for(int i=0; i<(int)v.size(); i++){
		double num = atof(v[i].c_str());
		n.push_back(num);
	}
}

std::string int2str(const int &int_temp){
    char s[255];
    snprintf(s, sizeof(s), "%d", int_temp);
    std::string string_temp=s;
    return string_temp;
}

std::string doulbe2str(double num)
{
    std::stringstream sstream;
    sstream << num;
    std::string num_str = sstream.str();
    return num_str;
}


#endif /* STRING_UTIL_H_ */

