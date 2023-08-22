#!/usr/bin/env python3

from __future__ import print_function

from downward_ros.srv import Plan  
import rospy
import os
from os.path import exists

#Service to call Fast-downward 
# by default FF is used
def downward_plan(req):
    print("\n================================")
    print("====STARTING downward planner===")
    print("================================")
    print("PDDL problem file ",req.problem)
    print("PDDL domain file ",req.domain)
    if(req.evaluator=="" and req.search=="" ):
        print("------ CALLING fast-downward as FF -------")
        evaluator = "\"hff=ff()\""
        search = "\"lazy_greedy([hff], preferred=[hff])\""
    else:
        evaluator = req.evaluator
        search = req.search
    print("evaluator used ",evaluator)
    print("search used ",search)

    f = open("problemfile", "w")
    f.write(req.problem)
    f.close()
    f = open("domainfile.pddl", "w")
    f.write(req.domain)
    f.close()

    command="fast-downward domainfile.pddl problemfile --evaluator " + evaluator + " --search " + search 
   
    if(exists("./sas_plan")):
        print("sas_plan exists - I will delete it")
        os.system("rm sas_plan") #remove previous plan, if any
    #call downward
    os.system(command)
    #check sas_plan has been obtained, otherwise report error
    if(not exists("./sas_plan")):
        print("Downward has not been able to generate sas_plan")
        Plan.plan = ["dummy"]
        Plan.response = False
        return [Plan.response, Plan.plan]

    f = open("sas_plan","r")
    lines = f.readlines()

    solution = []
    print("--- PLAN ---")
    for i in range(len(lines)-1):
        solution.append(lines[i].lstrip("(").rstrip(")\n").upper())#return as uppercase since this was what the FF package did...
        print(solution[i])
    #solution=["action1","action2"]
    Plan.plan = solution
    Plan.response = True
    return [Plan.response, Plan.plan]


if __name__ == "__main__":
    rospy.init_node('downward_server')
    s1 = rospy.Service('downward_service', Plan, downward_plan)
    print("downward server")
    rospy.spin()
