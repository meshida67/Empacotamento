#ifndef CIRCULOCALLBACK_H
#define CIRCULOCALLBACK_H
//-----------------------------------------------
#include <bits/stdc++.h>
#include "gurobi_c++.h"
//-----------------------------------------------
const double EPS = 1e-5;
//-----------------------------------------------
class circuloCallback: public GRBCallback
{
    double distancia(double x, double y);

    public:
        std::vector<GRBVar> a;
        std::vector<std::pair<GRBVar, GRBVar>> coord; // vetor para armazenar as coordenadas
        std::vector<std::pair<double, double>> tamanhos; // vetor para armazenar os tamanhos
        std::vector<std::pair<double, double>> sagit; // vetor para armazenar as sagitas
        double R;
        circuloCallback(std::vector<GRBVar> &a_,
                std::vector<std::pair<GRBVar, GRBVar>> &coord_,
                std::vector<std::pair<double, double>> &tamanhos_,
                std::vector<std::pair<double, double>> &sagit_,
                double R_)
        {
            a = a_;
            coord = coord_;
            tamanhos = tamanhos_;
            sagit = sagit_;
            R = R_;
        }

    protected:
        void callback();
};
//-----------------------------------------------
#endif

