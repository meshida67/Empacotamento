#include <bits/stdc++.h>
#include "gurobi_c++.h"
#include "circ.h"
//------------------------------------------------
double circuloCallback::distancia(double x, double y)
{
    x -= R;
    y -= R;
    return std::sqrt(x * x + y * y);
}
//------------------------------------------------
void circuloCallback::callback()
{
    try
    {
        if (where == GRB_CB_MIPSOL)
        {
            int n = a.size();
            for (int i = 0; i < n; i++)
            {
                if (getSolution(a[i]) > EPS)
                {
                    double x = getSolution(coord[i].first);
                    double y = getSolution(coord[i].second);
                    double h = tamanhos[i].first;
                    double l = tamanhos[i].second;

                    double dOC1 = distancia(x + l, y + h);
                    double dOC2 = distancia(x, y + h);
                    double dOC3 = distancia(x, y);
                    double dOC4 = distancia(x + l, y);
                    double dMAX = std::max({dOC1, dOC2, dOC3, dOC4});

                    double xa, ya, m;


                    if (dOC1 == dMAX)
                    {
                        x += l;
                        y += h;
                    }
                    else if (dOC2 == dMAX)
                        y += h;
                    else if (dOC4 == dMAX)
                        x += l;

                    xa = R + (x - R) * R / dMAX;
                    ya = R + (y - R) * R / dMAX;
                    if (R == y)
                        m = 1e9;
                    else
                        m = (x - R) / (R - y);

                    GRBLinExpr cut = (coord[i].second + h);
                    GRBLinExpr cut2_ElectricBoogaloo = (ya + m * (coord[i].first + l -xa));
                    //std::cout << cut << " <= " << cut2_ElectricBoogaloo <<'\n';

                    if (dOC1 == dMAX)
                        addLazy(coord[i].second + h <= ya + m * (coord[i].first + l -xa));
                    else if (dOC2 == dMAX)
                        addLazy(coord[i].second + h <= ya + m * (coord[i].first - xa));
                    else if (dOC3 == dMAX)
                        addLazy(coord[i].second >= ya + m * (coord[i].first - xa));
                    else if (dOC4 == dMAX)
                        addLazy(coord[i].second >= ya + m * (coord[i].first + l - xa));
                }
            }
        }
    }
    catch (GRBException &e)
    {
        std::cout << "Erro no callback: " << e.getMessage() << '\n';
    }
}
