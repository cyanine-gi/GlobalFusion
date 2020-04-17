#ifndef IO_MANAGER_GLOBALFUSION_H
#define IO_MANAGER_GLOBALFUSION_H
#include <iostream>
#include <fstream>
namespace GlobalFusion{
using namespace std;
shared_ptr<ofstream> pOutput = nullptr;

void outputResult(const string& output_type,vector<double> values)
{
    if(pOutput==nullptr)
    {//lazy initialization
        pOutput = shared_ptr<ofstream>(new ofstream("results.csv"));
    }
    (*pOutput)<<output_type;
    for(const double& val:values)
    {
        (*pOutput)<<","<<val;
    }
    (*pOutput)<<endl;
}




}





#endif
