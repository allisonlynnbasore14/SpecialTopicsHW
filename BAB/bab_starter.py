import picos as pic
from picos import RealVariable
from copy import deepcopy
from heapq import *
import heapq as hq
import numpy as np
import itertools
import math
counter = itertools.count()

class BBTreeNode():
    def __init__(self, vars = [], constraints = [], objective='', prob=None):
        self.vars = vars
        self.constraints = constraints
        self.objective = objective
        self.prob = prob

    def __deepcopy__(self, memo):
        '''
        Deepcopies the picos problem
        This overrides the system's deepcopy method bc it doesn't work on classes by itself
        '''
        newprob = pic.Problem.clone(self.prob)
        return BBTreeNode(self.vars, newprob.constraints, self.objective, newprob)

    def buildProblem(self):
        '''
        Bulids the initial Picos problem
        '''
        prob=pic.Problem()

        prob.add_list_of_constraints(self.constraints)

        prob.set_objective('max', self.objective)
        self.prob = prob
        return self.prob

    def printSelf(self):
        print(self.prob)

    def is_integral(self):
        '''
        Checks if all variables (excluding the one we're maxing) are integers
        '''
        for v in self.vars[:-1]:
            if v.value == None or abs(round(v.value) - float(v.value)) > 1e-4 :
                return False
        return True

    def is_not_int(self, var):
        '''
        Checks if all variables (excluding the one we're maxing) are integers
        '''
        if var == None or abs(round(var) - float(var)) > 1e-4 :
            return True
        return False

    def branch_floor(self, branch_var):
        '''
        Makes a child where xi <= floor(xi)
        '''
        n1 = deepcopy(self)
        n1.prob.add_constraint( branch_var <= math.floor(branch_var.value) ) # add in the new binary constraint
        return n1

    def branch_ceil(self, branch_var):
        '''
        Makes a child where xi >= ceiling(xi)
        '''
        #print(self.prob.solve(solver='cvxopt'))

        n2 = deepcopy(self)
        n2.prob.add_constraint( branch_var >= math.ceil(branch_var.value) ) # add in the new binary constraint
        return n2


    def bbsolve(self):
        '''
        Use the branch and bound method to solve an integer program
        This function should return:
            return bestres, bestnode_vars

        where bestres = value of the maximized objective function
              bestnode_vars = the list of variables that create bestres
        '''

        # these lines build up the initial problem and adds it to a heap
        root = self
        res = root.buildProblem().solve(solver='cvxopt')
        heap = [(res, next(counter), root)]
        bestres = -1e20 # a small arbitrary initial best objective value
        bestnode_vars = root.vars # initialize bestnode_vars to the root vars
        keepGoing = True

        while(len(heap)):
            print("The heap has ", len(heap), " nodes")
            currValue = heap.pop(0)
            newNode = currValue[-1]
            newNode.printSelf()
            try:
                res = newNode.prob.solve(solver='cvxopt').value
                keepGoing = True
            except:
                print("Invesable solution")
                keepGoing = False
            if(res < bestres):
                print("Stopping this branch becuase it is less than current best")
                pass
            elif( keepGoing and newNode.is_integral() ):
                if( res > bestres ):
                    print("Result is int and greater than current result")
                    bestres = res
                    bestnode_vars = newNode.vars
                return bestres, bestnode_vars
            else:
                if(keepGoing):
                    varToBranch = newNode.vars[0]
                    for var in newNode.vars[:-1]:
                        if( newNode.is_not_int(var.value) ):
                            varToBranch = var
                    print("Adding to the heap with", varToBranch.value)
                    A = newNode.branch_floor(varToBranch)
                    B = newNode.branch_ceil(varToBranch)
                    heap.append((res, next(counter), A ))
                    heap.append((res, next(counter), B ))

        return bestres, bestnode_vars
