#!/usr/bin/env python

#####################################################################################
# IMPORT                                                                            #
#####################################################################################
import sys
import statistics
import math
import copy
import rospy
import random
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import SetPlannerParams
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetStateValidityRequest
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import rosgraph_msgs.msg
import matplotlib.pyplot as plt
import geometry_msgs.msg
import numpy as np


#####################################################################################
# CLASS: Planner                                                                    #
# Represent a planning algorithm and its parameters.                                #
#####################################################################################
class Planner:
    # -------------------------------------------------------------------------------
    # Constructor
    # -------------------------------------------------------------------------------
    def __init__(self):
        self.name = None
        self.parameters = moveit_msgs.msg.PlannerParams()


#####################################################################################
# CLASS: BenchmarkSettings                                                          #
# Contains a list of parameters for the benchmark algorithm.                        #
#####################################################################################
class BenchmarkSettings:
    # -------------------------------------------------------------------------------
    # Constructor
    # -------------------------------------------------------------------------------
    def __init__(self):
        # A list of planners to test.
        self.plannerList = []

        # The number of configurations to test. A configuration is determined by
        # the goal state of the robot and by its environment.
        self.numberOfConfiguration = 0

        # Number of plans to generate, for each configuration and for each planner.
        self.numberOfPlan = 0

        # Maximum planning time, in seconds.
        self.maximumPlanningTime = 0

        # Number of obstacles (boxes of size 1x1x2) that will be positioned randomly.
        self.numberOfObstacle = 0


#####################################################################################
# CLASS: Benchmark                                                                  #
# Class used to benchmark and compare the performance of planning algorithm.        #
#####################################################################################
class Benchmark:
    # -------------------------------------------------------------------------------
    # Constructor
    # args:
    #   benchmarkSettings : Object of type BenchmarkSettings.
    # -------------------------------------------------------------------------------
    def __init__(self, benchmarkSettings):
        self.benchmarkSettings = benchmarkSettings

        # Variables used to store benchmark results.
        self.planningTimeList = []
        self.pathLengthList = []
        self.pathDurationList = []
        self.currentAlgo = []
        self.fail_count = {}
        # Will be set to True if a collision is detected.
        # See __callbackGetLog() for more details.
        self.collisionFound = False

        # Instantiate a PlanningSceneInterface object.
        # This object is an interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object.
        # This enable us to generate plans for the selected planning group.
        self.moveGroup = moveit_commander.MoveGroupCommander("base")

        # Set the maximum amount of time to use when planning (in seconds).
        # If no plan is found before the time limit, the planning fails.
        self.moveGroup.set_planning_time(self.benchmarkSettings.maximumPlanningTime)

        # Subscribe to the /move_group/result topic. We get the planning time by listening to the
        # MoveGroupActionResult message that are published to this topic. These messages are published
        # by the planning system.
        rospy.Subscriber("/move_group/result", moveit_msgs.msg.MoveGroupActionResult, self.__callbackGetPlanningTime)

        # Subscribe to the /rosout topic. We listen on this topic to check if a collision is found
        # during planning.
        rospy.Subscriber("/rosout", rosgraph_msgs.msg.Log, self.__callbackGetLog)

        # Wait until the service set_planner_params is online.
        # This service is used to set the planner's parameters.
        rospy.wait_for_service("set_planner_params")

        # Create a service proxy for set_planner_params.
        self.set_planner_params = rospy.ServiceProxy("set_planner_params", SetPlannerParams)

        # Wait for RVIZ to initialize.
        rospy.sleep(5)

    # -------------------------------------------------------------------------------
    # run
    #   Run the benchmark.
    # -------------------------------------------------------------------------------
    def run(self):
        print("== Benchmark starting")

        # Set the starting position.
        joint_state_start = JointState()
        joint_state_start.name = ['base_footprint_joint_y', 'base_footprint_joint_x', 'base_footprint_joint_rev']
        joint_state_start.position = [-5.0, -5.0, 0.0]
        robot_state_start = RobotState()
        robot_state_start.joint_state = joint_state_start
        self.moveGroup.set_start_state(robot_state_start)

        # Set the ending position.
        joint_state_end = JointState()
        joint_state_end.name = ['base_footprint_joint_y', 'base_footprint_joint_x', 'base_footprint_joint_rev']
        joint_state_end.position = [5.0, 5.0, 0.0]
        self.moveGroup.set_joint_value_target(joint_state_end)

        rospy.sleep(1)

        configNumber = 0
        for planner in self.benchmarkSettings.plannerList:
            self.fail_count[planner.name] = 0
        while configNumber < self.benchmarkSettings.numberOfConfiguration:

            print("")
            print("======== Configuration #" + str(configNumber))

            # Generate the environment.
            self.__generateObstacles()
            self.collisionFound = False

            # Wait for RVIZ to display the environment.
            rospy.sleep(1)

            # TODO:  For each planner in *self.benchmarkSettings.plannerList*, generate
            # TODO:  *self.benchmarkSettings.numberOfPlan* plans.
            # TODO:
            # TODO:  Every time a valid plan is computed, record the following information:
            # TODO:  - the planning time (i.e. the time taken to compute the plan)
            # TODO:  - the path length
            # TODO:  - the path duration (i.e. the time necessary to execute the plan)
            # TODO:  Note: A valid plan contains a trajectory.
            # TODO:  Note: An invalid plan is caused by the following situation:
            # TODO:        - the planning algorithm failed to a find a plan in the allowed
            # TODO:          time (see the maximum planning time setting)
            # TODO:        - the plan causes a collision

            for planner in self.benchmarkSettings.plannerList:
                for j in range(self.benchmarkSettings.numberOfPlan):
                    self.moveGroup.set_planner_id(planner.name)
                    # c = self.moveGroup.go()
                    # self.moveGroup.stop()
                    p = self.moveGroup.plan()
                    traj = p.joint_trajectory
                    if len(traj.points) == 0 or self.collisionFound:
                        print("wrong plan ", planner.name)
                        self.fail_count[planner.name] += 1
                        continue
                    self.pathLengthList.append(self.__computePathLength(p))
                    duration = traj.points[-1].time_from_start
                    self.pathDurationList.append(duration.nsecs * 10e-9 + duration.secs)
                    self.currentAlgo.append(planner.name)
                    # Clear target pose.
                    self.moveGroup.clear_pose_targets()

            # Clear the environment
            self.scene.remove_world_object()

            configNumber += 1

        print("== Benchmark completed")

    # -------------------------------------------------------------------------------
    # displayStatistic
    #   Display the result of the benchmark.
    # -------------------------------------------------------------------------------
    def displayStatistic(self):

        # TODO:  Using the data you collected during each iteration of the
        # TODO:  benchmark algorithm, generate relevant plots and figures.
        # TODO:  You can use the Matplotlib library (which is already imported)
        # TODO:  or any other plotting library compatible with Python 2.7.
        data = {}

        Tableau = [] 


        names_algos = [a.name for a in self.benchmarkSettings.plannerList]

        DicoEchecs = {key : self.fail_count[key] for key in names_algos}


        """
        for planner in self.benchmarkSettings.plannerList:
            algo = planner.name
            data[algo] = {"duration": [], "length": [], "planning": []}
            for i in range(len(self.currentAlgo)):
                if self.currentAlgo[i] == algo:
                    data[algo]["duration"].append(self.pathDurationList[i])
                    data[algo]["length"].append(self.pathLengthList[i])
                    data[algo]["planning"].append(self.planningTimeList[i])

            print(algo, self.fail_count[algo])
            fig1, ax1 = plt.subplots()
            for variable in ["duration", "length", "planning"]:
                y = data[algo][variable]
                if len(y) == 0:
                    continue
                # ax1.plot(range(len(y)), y)
                bx = ax1.boxplot(y)
                array = np.array(y)
                print(variable, array.mean(), array.std())
                ax1.set_title(algo + " " + variable)
                
                plt.show()
        plt.show()
        """
        for variable in ["duration", "length", "planning"]:
                data={}

                Tableau.append([])


                for planner in self.benchmarkSettings.plannerList:



                    Tableau[-1].append([])

                    algo = planner.name
                    data[algo] = []


                    for i in range(len(self.currentAlgo)):
                        if self.currentAlgo[i] == algo:
                            if variable=="duration":
                                data[algo].append(self.pathDurationList[i])
                            elif variable=="length":
                                data[algo].append(self.pathLengthList[i])
                            else:
                                data[algo].append(self.planningTimeList[i])

                    dataray = np.array(data[algo])
                    metrics = [ dataray.mean(), np.median(dataray), dataray.std(), dataray.min(), dataray.max()   ]
                    for k in range(len(metrics)):
                        Tableau[-1][-1].append(metrics[k])


                


                fig=plt.figure(figsize=(20,20))

                plt.boxplot([data[algname] for algname in names_algos], labels=names_algos )

                plt.title("Boxplot de '" + variable+ "' avec "+ str(settings.numberOfConfiguration) +" configs / " + str(settings.numberOfPlan) + " plans / " + str(settings.maximumPlanningTime) + " Temps max et " +str(settings.numberOfObstacle) +" obstacles") 
                

                
                plt.show()
        plt.show()


        print(DicoEchecs)

        print("----------------------------------")

        print(Tableau)

        print(np.array(Tableau).shape  )
        


    # -------------------------------------------------------------------------------
    # __generateObstacles
    #   Generate an environment for the robot.
    #   The environment consists of multiple boxes of size 1x1x2 randomly positioned.
    # -------------------------------------------------------------------------------
    def __generateObstacles(self):
        for i in range(self.benchmarkSettings.numberOfObstacle):
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = self.moveGroup.get_planning_frame()
            pose.pose.position.z = 1.0

            positionValid = False
            while not positionValid:
                pose.pose.position.x = random.uniform(-5.0, 5.0)
                pose.pose.position.y = random.uniform(-5.0, 5.0)

                # The robot start at position (-5.0, -5.0).
                # We must keep a clearance of 1,5 unit radius around the starting position where no
                # obstacle will spawn. This is to ensure that the robot is not in collision
                # at his starting position. Same thing for the goal position.
                distanceStart = math.sqrt((pose.pose.position.x + 5.0) ** 2 + (pose.pose.position.y + 5.0) ** 2)
                distanceGoal = math.sqrt((pose.pose.position.x - 5.0) ** 2 + (pose.pose.position.y - 5.0) ** 2)

                if distanceStart >= 1.5 and distanceGoal >= 1.5:
                    positionValid = True

            self.scene.add_box("block" + str(i), pose, (1.0, 1.0, 2))

    # -------------------------------------------------------------------------------
    # __computePathLength
    #   Compute the path length for a given plan.
    # args:
    #   plan : Object of type RobotTrajectory.
    # return: The path length.
    # -------------------------------------------------------------------------------
    def __computePathLength(self, plan):
        pathLength = 0

        # TODO:  Using the *plan* object, compute the length of the plan's path.
        # TODO:  Use the euclidean distance.
        points = plan.joint_trajectory.points
        for i in range(len(points) - 1):
            sub_list = [(x - y) ** 2 for (x, y) in zip(points[i].positions, points[i + 1].positions)]
            pathLength += math.sqrt(sum(sub_list))
        return pathLength

    # -------------------------------------------------------------------------------
    # __callbackGetPlanningTime
    #   Callback for the /move_group/result topic subscriber.
    #   This callback is used to extract the planning time.
    # args:
    #   data : Object of type MoveGroupActionResult.
    # -------------------------------------------------------------------------------
    def __callbackGetPlanningTime(self, data):

        # TODO:  Using the *data* object, check if the planning has succeeded.
        # TODO:  If the planning was successful, get the planning time and add it to the
        # TODO:  *self.planningTimeList* list. If the planning has failed, do not record
        # TODO:  the planning time.
        if data.status.SUCCEEDED == data.status.status:
            self.planningTimeList.append(data.result.planning_time)

    # -------------------------------------------------------------------------------
    # __callbackGetLog
    #   Callback for the /rosout topic subscriber.
    #   This callback is used to detect collision.
    # args:
    #   data : Object of type Log.
    # -------------------------------------------------------------------------------
    def __callbackGetLog(self, data):
        # When a collision occurs during planning, a message is published on the
        # /rosout topic. We check for messages that include the following string:
        # "Found a contact between".
        if "Found a contact between" in data.msg:
            self.collisionFound = True


#####################################################################################
# PROGRAM ENTRY POINT                                                               #
#####################################################################################
if __name__ == '__main__':
    try:
        # Initialize moveit_commander.
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node.
        rospy.init_node('algorithm_benchmark', anonymous=True)

        settings = BenchmarkSettings()

        # TODO:  Configure each planner that you want to test (see the *Planner* class).
        # TODO:  Add each planner to the *BenchmarkSettings* object planner list
        # TODO:  by calling *settings.plannerList.append()*.
        for algo in ["RRTkConfigDefault", "RRTConnectkConfigDefault", "PRMkConfigDefault", "ESTkConfigDefault"]:
            p = Planner()
            p.name = algo
            settings.plannerList.append(p)

        # Default settings for the benchmark algorithm.
        # See the *BenchmarkSettings* class for more details.
        # You should change these values to ensure that you record enough data.
        settings.numberOfConfiguration = 2
        settings.numberOfPlan = 2
        settings.maximumPlanningTime = 5.0
        settings.numberOfObstacle = 5

        benchmark = Benchmark(settings)
        benchmark.run()
        benchmark.displayStatistic()

        # Shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
