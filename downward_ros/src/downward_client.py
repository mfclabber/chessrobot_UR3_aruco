#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import rospkg
rospack =rospkg.RosPack()

from downward_ros.srv import *
import pathlib

def downward_plan_client(problem,domain, evaluator, search):
    rospy.wait_for_service('downward_service')
    downward_srv = Plan()
    downward_srv.problem = problem
    downward_srv.domain = domain
    downward_srv.evaluator = evaluator
    downward_srv.search = search
    try:
        downward_client = rospy.ServiceProxy('downward_service', Plan)
        resp1 = downward_client(downward_srv.problem,downward_srv.domain,downward_srv.evaluator,downward_srv.search)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    downward_ros_path = rospack.get_path("downward_ros")
    problemFile = downward_ros_path + "/pddl/block_world";
    domainFile = downward_ros_path + "/pddl/block_world.pddl";
    print("Requesting FF plan for problem file %s and domain file %s"%(problemFile, domainFile))

    strProblemFile=open(problemFile,"r").read()
    print("---------- ProblemFile ------------")
    print(strProblemFile)

    strDomainFile=open(domainFile,"r").read()
    print("---------- DomainFile -------------")
    print(strDomainFile)

    print("------ CALLING fast-downward as FF -------")
    #evaluator="\"hff=ff()\""
    #search="\"lazy_greedy([hff], preferred=[hff])\""
    #evaluator and search can be left empty for FF since the downward server fills them with the ff info
    evaluator=""
    search=""
    r=downward_plan_client(strProblemFile, strDomainFile, evaluator, search)
    if(r.response==True):
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)
        
    
    print("------ CALLING fast-downward using context-enhanced additive heuristic -------")
    evaluator="\"hcea=cea()\""
    search="\"lazy_greedy([hcea], preferred=[hcea])\""
    r=downward_plan_client(strProblemFile, strDomainFile, evaluator, search)
    if(r.response==True):
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)
