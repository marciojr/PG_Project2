#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include "Defs.h"

using namespace std;

double produtoEscalar(const dVector u, const dVector v);
dVector produtoVetorial(dVector u, dVector v);
double getNorma(const dVector v);
dVector normalize(dVector v);
dMatrix transpose(dMatrix m);
dVector ortogonalizacao(const dVector u, const dVector v);
dMatrix multiplicacaoNN(const dMatrix m1, const dMatrix m2);
dVector multiplicacaoN1(const dMatrix &m1, const dVector &m2);
dVector soma(dVector u, dVector v);
dVector subtracao(dVector u, dVector v);
dMatrix getRotationNN(dMatrix m);