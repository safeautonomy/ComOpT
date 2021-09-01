#
#    ComOpT: COMbination and OPtimization for Testing autonomous driving systems
#
#    Copyright (C) 2021  Yuhang Chen, Chih-Hong Cheng, Changwen Li, Tiantian Sun, Rongjie Yan
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Affero General Public License as published
#    by the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Affero General Public License for more details.
#
#    You should have received a copy of the GNU Affero General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.
#


import numpy as np 
from ortools.linear_solver import pywraplp



def solveCP(lp_constraint):
    """ Solve the MILP constraint program.
    
    """

    # 5 seconds
    SOLVER_TIME_LIMIT = 5000

    # Instantiate a mixed-integer solver, naming it SolveIntegerProblem.
    solver = pywraplp.Solver('SolveIntegerProblem', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    variable_dict = dict()
    for var in lp_constraint.vars:
        variable_dict[var] = solver.IntVar(0, 1, var)

    for exp in lp_constraint.constraints:
        constraint = None
        if(exp.lowerbound == float('-inf') and exp.upperbound == float('inf')): 
            constraint = solver.Constraint(-solver.infinity(), solver.infinity())
        elif (exp.lowerbound == float('-inf')):
            constraint = solver.Constraint(-solver.infinity(), exp.upperbound)
        elif (exp.upperbound == float('inf')):
            constraint = solver.Constraint(exp.lowerbound, solver.infinity())            
        else:
            constraint = solver.Constraint(exp.lowerbound, exp.upperbound)   
        
        for i in range(len(exp.variables)):
            constraint.SetCoefficient(variable_dict[exp.variables[i]], exp.coefficients[i])

    objective = solver.Objective()
    for var in lp_constraint.occupyVars:
        objective.SetCoefficient(variable_dict[var], 1)
    objective.SetMaximization()        
            
    """Solve the problem and print the solution."""
    
    solver.SetTimeLimit(SOLVER_TIME_LIMIT)
    result_status = solver.Solve()
    # The problem has an optimal solution.
    if result_status == pywraplp.Solver.OPTIMAL:
        print("Optimal solution found")
    elif result_status == pywraplp.Solver.FEASIBLE:
        print("Timeout but feasible solution found in 10 seconds")
    else: 
        print(result_status)
        raise Exception("The solver can not find optimal or feasible solution within time bound in 10 seconds") 
    
    # The solution looks legit (when using solvers other than
    assert solver.VerifySolution(1e-7, True)

    #print('Number of variables =', solver.NumVariables())
    #print('Number of constraints =', solver.NumConstraints())

    variable_assignment = dict()
    for key, var in variable_dict.items():
        variable_assignment[key] = var.solution_value()
    
    
    return len(lp_constraint.occupyVars), solver.Objective().Value(), variable_assignment 

        
        
class TestCaseGenConstraint():

    def __init__(self):
        self.constraints = [];
        self.vars = [];
        self.occupyVars = [];   
    
    def print_constraint(self):
        print("All variables used in optimization objective\n\n")
        print(self.occupyVars)
        print("\n\nAll variables in the constraint system\n\n")
        print(self.vars)
        print("\n\nAll constraints\n\n\n")
        for exp in self.constraints:
            exp.print_expression()

class Expression():
    
    def __init__(self):
        self.name = "";
        self.lowerbound = float('-inf');
        self.upperbound = float("inf");    
        self.variables = [];
        self.coefficients= [];

    def print_expression(self):
        print(str(self.lowerbound) + " <= ", end='')
        for i in range(len(self.variables)):
            if self.coefficients[i] >= 0:
                print(" + "+ str(self.coefficients[i]) +" " +str(self.variables[i]), end='')
            else:
                print(" "+ str(self.coefficients[i]) + " "+ str(self.variables[i]), end='')
        print(" <= " + str(self.upperbound))
 
        
def create_value_assignment_variable(criteria, value):
    return "C" + str(criteria) + "_" + str(value)
         
