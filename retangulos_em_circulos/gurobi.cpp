#include <bits/stdc++.h>
#include <fstream>
#include "gurobi_c++.h"
#include "circ.h"
//-----------------------------------------------
int main()
{
    std::ifstream dados;
    dados.open("entrada.txt");

    int n; // numero de variaveis
    double R; // raio do circulo
    dados >> n >> R;

    double M = 4 * R;

    // Variaveis para armazenar os valores
    std::vector<std::pair<double, double>> tamanhos;
    std::vector<std::pair<double, double>> sagit;

    double h, l, sl, sh;
    for (int i = 0; i < n; i++)
    {
        dados >> h >> l;
        sh = R - std::sqrt(R * R - l * l / 4.0);
        sl = R - std::sqrt(R * R - h * h / 4.0);
        sh *= sh;
        sl *= sl;
        tamanhos.push_back({h, l});
        sagit.push_back({sh, sl});
    }
    //-----------------------------------------------
    try
    {
        GRBEnv env = GRBEnv(true);
        env.start();

        GRBModel modelo = GRBModel(env);


        // Criando as variaveis do modelo
        std::vector<GRBVar> a;
        std::vector<std::vector<std::tuple<GRBVar, GRBVar, GRBVar, GRBVar>>> delta(n, std::vector<std::tuple<GRBVar, GRBVar, GRBVar, GRBVar>>(n));
        std::vector<std::pair<GRBVar, GRBVar>> coord;
        std::vector<std::vector<GRBVar>> z(n, std::vector<GRBVar>(n));
        for (int i = 0; i < n; i++)
        {
            a.push_back(modelo.addVar(0, 1, 0, GRB_BINARY));
            coord.push_back({modelo.addVar(0.0, 2 * R - tamanhos[i].second, 0.0,GRB_CONTINUOUS), modelo.addVar(0.0, 2 * R - tamanhos[i].first, 0.0, GRB_CONTINUOUS)});
            for (int j = i + 1; j < n; j++)
            {
                delta[i][j]= {modelo.addVar(0, 1, 0, GRB_BINARY), 
                        modelo.addVar(0, 1, 0, GRB_BINARY), 
                        modelo.addVar(0, 1, 0, GRB_BINARY), 
                        modelo.addVar(0, 1, 0, GRB_BINARY)};
                z[i][j] = modelo.addVar(0, 1, 0, GRB_BINARY);
            }
        }

        // Funcao a ser otimizada
        GRBLinExpr obj = 0;
        for (auto &x :a)
            obj += x;
        modelo.setObjective(obj, GRB_MAXIMIZE);

        // Restricoes quadrado
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                modelo.addConstr(a[i] + a[j] - 1 <= z[i][j]);
                modelo.addConstr(z[i][j] <= a[i]);
                modelo.addConstr(z[i][j] <= a[j]);
                modelo.addConstr(z[i][j] <= std::get<0>(delta[i][j]) + std::get<1>(delta[i][j]) + std::get<2>(delta[i][j]) + std::get<3>(delta[i][j]));
                modelo.addConstr(std::get<0>(delta[i][j]) + std::get<1>(delta[i][j]) + std::get<2>(delta[i][j]) + std::get<3>(delta[i][j]) <= 2 * z[i][j]);
                modelo.addConstr(coord[i].first + tamanhos[i].second <= coord[j].first + M * (1 - std::get<0>(delta[i][j])));
                modelo.addConstr(coord[i].first >= coord[j].first + tamanhos[j].second - M * (1 - std::get<1>(delta[i][j])));
                modelo.addConstr(coord[i].second + tamanhos[i].first <= coord[j].second + M * (1 - std::get<2>(delta[i][j])));
                modelo.addConstr(coord[i].second >= coord[j].second + tamanhos[j].first - M * (1 - std::get<3>(delta[i][j])));
            }
        }

        // Cortes para circulo
        modelo.set(GRB_IntParam_LazyConstraints, 1);

        circuloCallback cb(a, coord, tamanhos, sagit, R);
        modelo.setCallback(&cb);

        // Otimizando enfim :)
        modelo.optimize();


        int status = modelo.get(GRB_IntAttr_Status);
        std::cout << "Estado do modelo (IntAttr_Status): " << status << '\n';

        if (status == GRB_OPTIMAL)
        {
            std::cout << "Solucao otima: " << modelo.get(GRB_DoubleAttr_ObjVal) << '\n';
            for (int i = 0; i < n; i++)
            {
                std::cout << "Valor de a[" << i << "]: " << a[i].get(GRB_DoubleAttr_X);
                std::cout << "; Coordenadas "<< "(" << coord[i].first.get(GRB_DoubleAttr_X) << ", " << coord[i].second.get(GRB_DoubleAttr_X) << ")\n";
            }
        }
        else if (status == GRB_INFEASIBLE)
        {
            std::cout << "O modelo eh impossivel de resolver :,(\n";
            modelo.computeIIS();
            modelo.write("sei_la.ilp");
        }
    }
    catch(GRBException &e)
    {
        std::cout << e.getErrorCode() << '\n';
    }

    return 0;
}
