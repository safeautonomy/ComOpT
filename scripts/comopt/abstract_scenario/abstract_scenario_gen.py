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

from . import abstract_scenario_management
from . import milp

def propose_scenario_candidate(metric : abstract_scenario_management.ScenarioKProjectionMetricManager, verbose = False):
    """ Based on the current metric, propose a new scenario to maximally increase k-projection coverage.
    """
    lpConstraint = prepare_LP_constraint(metric)
    variable_assignment = solve_CP(lpConstraint, metric)

    for criterion in metric.sd.operating_condition_criteria:
        for assignment in metric.sd.operating_condition_items[criterion]: 
            var = milp.create_value_assignment_variable(metric.sd.operating_condition_criteria.index(criterion), metric.sd.operating_condition_items[criterion].index(assignment))
            if(variable_assignment[var] == 1 and verbose == True):
                print("for criterion "+str(criterion)+", set it to "+str(assignment))
    

    return variable_assignment
    
    
def solve_CP(lp_constraint, metric):
    """ Solve the MILP constraint program.
    
    """
    
    totalnum, max_improve, variable_assignment = milp.solveCP(lp_constraint)
    # The objective value of the solution.
    print('Maximum possibility for improvement =', totalnum)
    print('Optimal objective value computed from IP = %d' % max_improve)
    print() 

    return variable_assignment

def prepare_LP_constraint(metric):
    """ Prepare the MILP constraint for maximally improving neuron-k-activation-pattern coverage.
    """

    # kValue, numberOfNeuronsToTrack, k_Activation_record
    
    if not (type(metric) == abstract_scenario_management.ScenarioKProjectionMetricManager):
        raise TypeError("The method only takes input with type ScenarioKProjectionMetric")
    
       
    lp_constraint = milp.TestCaseGenConstraint()
   
    
    for i in range(metric.cm.number_of_categorizations):
        exp = milp.Expression()
        # Sum to be 1, therefore set upper and lower bound to be 1.
        exp.lowerbound = 1
        exp.upperbound = 1
        
        # The possible assignment can only be 0 or 1, so set j < 2
        for j in range(metric.cm.maxType[i]):
            exp.coefficients.append(1.0)
            exp.variables.append(milp.create_value_assignment_variable(i, j))
            lp_constraint.vars.append(milp.create_value_assignment_variable(i, j))
            
        lp_constraint.constraints.append(exp)
        

    occupy_variables = set()
    
    for projected_categorization, record in metric.cm.projection_records.items():
        for assignment, quantity in record.currently_occupied_entities.items():
            
            if (quantity < record.max_occupied_entities.get(assignment)):
                #  This item can be improved

                occupyVariable = "occupy_" + str(projected_categorization) + "_be_" + str(assignment)
                occupy_variables.add(occupyVariable)

                variables = projected_categorization.split('_')
                
                exp = milp.Expression()
                exp.lowerbound = 0
                exp.upperbound = metric.cm.kValue - 1
                for i in range(len(variables)):
                    # print("C" + variables[i] + "_" + assignment[2*i: 2*i + 1])
                    exp.variables.append("C" + variables[i] + "_" + assignment[2*i: 2*i + 1])
                    exp.coefficients.append(1.0)

                exp.variables.append(occupyVariable)
                exp.coefficients.append(-1.0 * metric.cm.kValue)

                lp_constraint.constraints.append(exp)


    # Add all domain restrictions
    lp_constraint.constraints.extend(metric.dr.domainRestrictions)
    
    lp_constraint.vars.extend(occupy_variables)
    lp_constraint.occupyVars.extend(occupy_variables)

    # lp_constraint.print_constraint()
    return lp_constraint
    
     
