#pragma once
#include <string>
#include <vector>
using namespace std;

typedef vector<float> fVector;
typedef vector<fVector> fMatrix;
typedef vector<fMatrix> fSuperMatrix;
typedef vector<double> dVector;
typedef vector<dVector> dMatrix;
typedef vector<dMatrix> dSuperMatrix;
typedef vector<int> iVector;
typedef vector<iVector> iMatrix;
typedef vector<iMatrix> iSuperMatrix;


class Defs{
public:
	static string configPath;
	static string modelsPath;
	static string resourcesPath;
};
