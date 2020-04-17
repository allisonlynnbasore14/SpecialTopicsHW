'''Code file for vehicle routing problem created for Advanced Algorithms
Spring 2020 at Olin College. These functions solve the vehicle routing problem
using an integer programming and then a local search approach. This code has
been adapted from functions written by Alice Paul.'''

import picos as pic
import numpy as np
from read_files import read_file_type_A, read_file_type_C

# Integer programming approach
def cvrp_ip(C,q,K,Q,obj=True):
    '''
    Solves the capacitated vehicle routing problem using an integer programming
    approach.

    C: matrix of edge costs, that represent distances between each node
    q: list of demands associated with each client node
    K: number of vehicles
    Q: capacity of each vehicle
    obj: whether to set objective (ignore unless you are doing local search)
    returns:
        objective_value: value of the minimum travel cost
        x: matrix representing number of routes that use each arc
    '''

    q = np.append(q,0)

    n = q.size-1

    B = np.zeros((q.size, q.size))
    B[:n,:n]= C
    B[n,:] = B[0,:]
    B[:,n] = B[:,0]

    nodes = []
    edges = []
    for x in range(0,q.size):
        nodes.append(x)
        for m in range(0,q.size):
            edges.append((x,m))

    # set up the picos problem
    prob = pic.Problem()

    x = prob.add_variable('x',B.shape,vtype='binary')
    u = prob.add_variable('u',q.size, vtype='continuous', upper=Q, lower=q)


    # State the potential inequalities.

    #K or fewer vehicles will leave the source
    prob.add_constraint( sum([x[0,j] for j in nodes]) <= K )

    # Routes cant end at the origin
    prob.add_constraint(sum([x[j,0] for j in nodes])==0)

    # Routes cant end at the origin
    prob.add_constraint(sum([x[j,j] for j in nodes])==0)

    # the number of vehicles that leave the origin has to equal the number that enter the destination
    prob.add_constraint( sum([x[0,j] for j in nodes]) == sum([x[j,n] for j in nodes]))

    # makes sure every node has a vehicle enter
    prob.add_list_of_constraints([sum([x[i,j] for j in nodes])==1 for i in nodes[1:-1]])

    # makes sure every node has a vehicle leave
    prob.add_list_of_constraints([sum([x[j,i] for j in nodes])==1 for i in nodes[1:-1]])

    # makes sure no cycles occur
    prob.add_list_of_constraints([(u[i]-u[j])+Q*x[i,j] <= Q-q[j] for (i,j) in edges])

    #make the objective_value
    prob.set_objective('min', pic.sum([B[e]*x[e] for e in edges]))

    print(prob)
    solution = prob.solve(solver='cplex', verbose=True)

    if (not "integer optimal solution" == solution['status']):
        return 0, x

    objective_value = prob.obj_value()
    return objective_value, x

def local_search(C,q,K,Q):
    '''
    Solves the capacitated vehicle routing problem using a local search
    approach.

    C: matrix of edge costs, that represent distances between each node
    q: list of demands associated with each client node
    K: number of vehicles
    Q: capacity of each vehicle
    returns:
        bestval: value of the minimum travel cost
        bestx: matrix representing number of routes that use each arc
    '''
    bestx = []
    bestval = 0

    return bestval, bestx


if __name__ == "__main__":

    # an example call to test your integer programming implementation
    C,q,K,Q = read_file_type_A('data/A-n016-k03.xml')

    travel_cost, x = cvrp_ip(C,q,K,Q)
    print("Travel cost: " + str(travel_cost))
    print("X: ", x)
