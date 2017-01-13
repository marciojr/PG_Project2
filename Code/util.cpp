#include "util.h"

double produtoEscalar(const dVector u, const dVector v)
{
    double retorno  =0;
for(int i = 0;i< u.size();i++)    retorno+=u[i]*v[i];
	return retorno;
}
dMatrix transpose(dMatrix m){
	dMatrix t(m[0].size(), dVector(m.size()));
	for (int i = 0; i < m.size(); i++){
		for( int j = 0; j < m[0].size(); j++){
			t[i][j] = m[j][i];
		}
	}
	return t;
}
dMatrix getRotationNN(dMatrix m){
	dMatrix t(m.size(), dVector(m.size()));
	for (int i = 0; i < m.size()-1; i++){
		for (int j = 0; j < m.size()-1; j++){
			t[i][j] = m[i][j];
		}
	}
	t[m.size() - 1][m.size() - 1] = 1;
	return t;
}
double getNorma(const dVector v)
{
	double retorno = 0;
	for(int i = 0;i< v.size();i++)    retorno+=pow(v[i], 2);
    return (sqrt(retorno));
}

dVector normalize(dVector v)
{
    double norma = getNorma(v);
    for(int i = 0;i< v.size();i++)   v[i]=v[i]/norma;
    return v;
}

dVector ortogonalizacao(const dVector u, const dVector v)
{
    double escalar = produtoEscalar(u, v) / produtoEscalar(v, v);
    dVector vetor(u.size());
    for(int i = 0;i< u.size();i++)vetor[i]=u[i] - escalar*v[i];
    return vetor;
}

dMatrix multiplicacaoNN(const dMatrix m1, const dMatrix m2)
{
    int m1l = m1.size(), m1c = m1[0].size(), m2l=m2.size(), m2c = m2[0].size();
	dMatrix matrix(m1l, dVector(m2c, 0));
	for (int l = 0; l < m1l; l++){
		for (int c = 0; c < m1c; c++){
			for (int k = 0; k < m1c; k++){
				matrix[l][c] += m1[l][k] * m2[k][c];
			}
		}
	}
    return matrix;
}

dVector multiplicacaoN1(const dMatrix &m1, const dVector &v2)
{
    dVector vetor (m1[0].size());
	for (int i = 0; i < m1.size(); ++i)
        for (int j = 0; j < m1[0].size(); ++j)
            vetor[i] += m1[i][j] * v2[j];
	 return vetor;
}

dVector soma(dVector u, dVector v){
	dVector retorno(v.size());
	for (int i = 0; i < v.size(); i++)
		retorno[i] = u[i] + v[i];
	return retorno;
}

dVector subtracao(dVector u, dVector v){
	dVector retorno(v.size());
	for (int i = 0; i < v.size(); i++)
		retorno[i] = u[i] - v[i];
	return retorno;
}
dVector produtoVetorial(dVector u, dVector v){
	dVector ret(3);
	ret[0] = u[1] * v[2] - u[2] * v[1];
	ret[1] = u[2] * v[0] - u[0] * v[2];
	ret[2] = u[0] * v[1] - u[1] * v[0];
	return ret;
}