import gurobipy as gb
from gurobipy import GRB

with open("entrada.txt") as file:
    content = file.read()
valores = content.split()
n = int(valores[0])
R = float(valores[1]) # Radius of circular container
l = [] # Array containing the lengths of each rectangle
h = [] # Array containing the heights of each rectangle
sl = [] # Array containing the sagittas 
sh = []
M1 = []
M2 = []
M3 = []
M4 = []

for i in range(n):
    l.append(float(valores[2 * i + 2]))
    h.append(float(valores[2 * i + 3]))
    sl.append(R - (R ** 2 - (l[i] ** 2) / 4) ** 0.5)
    sh.append(R - (R ** 2 - (h[i] ** 2) / 4) ** 0.5)

for i in range(n):
    for j in range(n):
        M1.append(2 * R - sh[i] - sh[j] + 1)
        M2.append(2 * R - sh[i] - sh[j] + 1)
        M3.append(2 * R - sl[i] - sl[j] + 1)
        M4.append(2 * R - sl[i] - sl[j] + 1)

def getIndex(i, j):
    return i * n + j

try:
    model = gb.Model("Rectangles in circles")

    a = []
    delta1 = []
    delta2 = []
    delta3 = []
    delta4 = []
    z = []
    x = []
    y = []
    for i in range(n):
        a.append(model.addVar(vtype = GRB.BINARY))
        x.append(model.addVar(vtype = GRB.CONTINUOUS, lb = 0, ub = 2 * R - l[i]))
        y.append(model.addVar(vtype = GRB.CONTINUOUS, lb = 0, ub = 2 * R - h[i]))
        for j in range(n):
            z.append(model.addVar(vtype = GRB.BINARY))
            delta1.append(model.addVar(vtype = GRB.BINARY))
            delta2.append(model.addVar(vtype = GRB.BINARY))
            delta3.append(model.addVar(vtype = GRB.BINARY))
            delta4.append(model.addVar(vtype = GRB.BINARY))

    model.setObjective(sum(a), GRB.MAXIMIZE)

    for i in range(n):
        for j in range(n):
            if i < j:
                model.addConstr(a[i] + a[j] - 1 <= z[getIndex(i, j)])
                model.addConstr(z[getIndex(i, j)] <= a[i])
                model.addConstr(z[getIndex(i, j)] <= a[j])
                model.addConstr(z[getIndex(i, j)] <= delta1[getIndex(i, j)] + delta2[getIndex(i, j)] + delta3[getIndex(i, j)] + delta4[getIndex(i, j)])
                model.addConstr(delta1[getIndex(i, j)] + delta2[getIndex(i, j)] + delta3[getIndex(i, j)] + delta4[getIndex(i, j)] <= 2 * z[getIndex(i, j)])
                model.addConstr(x[i] + l[i] <= x[j]  + M1[getIndex(i, j)] * (1 - delta1[getIndex(i, j)]))
                model.addConstr(x[i] >= x[j] + l[j]  - M2[getIndex(i, j)] * (1 - delta2[getIndex(i, j)]))
                model.addConstr(y[i] + h[i] <= y[j]  + M3[getIndex(i, j)] * (1 - delta3[getIndex(i, j)]))
                model.addConstr(y[i] >= y[j] + h[j] - M4[getIndex(i, j)] * (1 - delta4[getIndex(i, j)]))



    model.optimize()

    if model.status == GRB.OPTIMAL:
        print(f"solução otima:{model.objVal}")
        l = len(x)
        for i in range(l):
            print(f"valor de x: {x[i].X}")
            print(f"valor de y: {y[i].X}")

except gb.GurobiError as e:
    print(f"error encountered: {e}")

