#! /usr/bin/env python
#Code initiated by txa on 20 November 2020
#Comments added on 27 September 2021
#Saved under https://github.com/ThomasCarstens/cfScripts/blob/master/ros_action_server.py

# OUT OF CLASS: USAGE #############################################################################
# sm0 = self.execOctogonAndTrajOnCollision (self.ids)
# self.start_sm_on_thread(sm0)
# NOTE: RUNS specified command in packaged state machine.

# WITHIN STATE MACHINE: USAGE #####################################################################
# 
# self.fig8_sm = concurrent_trajs(selected_drones = ids, traj_id = 8)
# StateMachine.add('FIG8_EXECUTE', self.fig8_sm, transitions={'succeeded' : 'land_all', 
#                                                             'aborted' : 'land_all', 
#                                                         'preempted' : 'land_all'})
# NOTE: FUNCTION concurrent_trajs(args[]) readapts actionlib to smach.

# WITHIN FUNCTIONS:                      ########################## ##############################

# def concurrent_trajs(self, selected_drones, traj_id):
#     """FIG8_EXECUTE""" 
#     figure = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
#     with figure:
#         for drone_id in selected_drones:
#             Concurrence.add('FIG8_EXECUTE_drone'+str(id),
#             SimpleActionState('fig8_drone'+str(id),
#                             doTrajAction, goal = doTrajGoal(shape = traj_id, id = drone_id)))
#     return figure
# NOTE: create containers for the actionlib functions, accessible at 
# https://github.com/leonard-de-vinci/intelligent-drone-lab/blob/master/ros_ws/src/crazyswarm/scripts/ros_action_server.py

from __future__ import print_function
from numpy.lib.function_base import select
import rospy

import threading
from math import sqrt, pow
import smach_ros
import smach

import actionlib
from actionlib_tutorials.msg import my_newAction, my_newGoal, MachineAction, FibonacciAction, FibonacciGoal, doTrajAction, doTrajGoal
from geometry_msgs.msg import Point
from smach import StateMachine, Concurrence
from smach_ros import ActionServerWrapper, ServiceState, SimpleActionState, MonitorState, IntrospectionServer
import std_srvs.srv
import turtlesim.srv
import turtlesim.msg
import turtle_actionlib.msg
import sys

class smlib(my_ids):
    def __init__(self, my_ids):


        self.ids = my_ids


        # SECTION 1: define the differents points
        self.test_point= Point()
        self.test_point.x = 0
        self.test_point.y = 0
        self.test_point.z = 0.5

        self.home_points_fo8 = [Point(), Point(), Point()]

        self.home_points_fo8[0].x = 0.0
        self.home_points_fo8[0].y = 0.0
        self.home_points_fo8[0].z = 0.3

        self.home_points_fo8[1].x = 0.3
        self.home_points_fo8[1].y = 0.0
        self.home_points_fo8[1].z = 0.3

        self.home_points_fo8[2].x = -0.3
        self.home_points_fo8[2].y = 0.0
        self.home_points_fo8[2].z = 0.3


        self.home_points_heli = [Point(), Point(), Point()]

        self.home_points_heli[0].x = 0.0
        self.home_points_heli[0].y = 0.0
        self.home_points_heli[0].z = 0.3

        self.home_points_heli[1].x = 0.0
        self.home_points_heli[1].y = 0.9-0.4
        self.home_points_heli[1].z = 0.9

        self.home_points_heli[2].x = -0.3
        self.home_points_heli[2].y = 0.0
        self.home_points_heli[2].z = 0.3

        self.octogon_points = [Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()]

        self.octogon_points[0].x = 0.5
        self.octogon_points[0].y = -0.5
        self.octogon_points[0].z = 0.5

        self.octogon_points[1].x = 0.7
        self.octogon_points[1].y = 0.0
        self.octogon_points[1].z = 0.5

        self.octogon_points[2].x = 0.5
        self.octogon_points[2].y = 0.5
        self.octogon_points[2].z = 0.5

        self.octogon_points[3].x = 0
        self.octogon_points[3].y = 0.7
        self.octogon_points[3].z = 0.5

        self.octogon_points[4].x = -0.5
        self.octogon_points[4].y = 0.5
        self.octogon_points[4].z = 0.5

        self.octogon_points[5].x =  -0.7
        self.octogon_points[5].y = 0
        self.octogon_points[5].z = 0.5

        self.octogon_points[6].x = -0.5
        self.octogon_points[6].y = -0.5
        self.octogon_points[6].z = 0.5

        self.octogon_points[7].x = 0
        self.octogon_points[7].y = -0.7
        self.octogon_points[7].z = 0.5

        # monitor an external flag and preempt if detected
        def agent_far_away(ud, msg):
            if msg.data == True:
                return True
            return False

        self.activate_on_true = lambda ud,msg: not agent_far_away(ud,msg)

    ###############################################
    # SECTION 2 | FUNCTIONS: create containers for the server functions.

    # CONTAINER: MOVE DRONE [id] to waypoint [index]
    def move_drone(self, drone_id, traj_waypoint):
        figure = SimpleActionState('drone'+str(drone_id)+'detect_perimeter', 
                        my_newAction, 
                        goal = my_newGoal(point = traj_waypoint, id = drone_id )),
        return figure

    def monitor_general(self, monitor_topic, monitor_type, truth_function):
            monitor_s = MonitorState(monitor_topic, monitor_type,
                                cond_cb = truth_function)
            return monitor_s


    # FULL LAND CONTAINER # Land all the drones to their respective points
    def land_group(self, selected_drones, traj_waypoint):
        figure = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
        with figure:
            for drone_id in selected_drones:
                Concurrence.add('LAND_DRONE'+str(id),
                            SimpleActionState('land_drone'+str(drone_id),
                                            my_newAction, goal = my_newGoal(point = traj_waypoint[drone_id], id = drone_id)  )) 
        return figure

    # CONCURRENT Fo8s CONTAINER # Using all the ids currently running.
    def concurrent_trajs(self, selected_drones, traj_id):
        """FIG8_EXECUTE""" 
        figure = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
        with figure:
            for drone_id in selected_drones:
                Concurrence.add('FIG8_EXECUTE_drone'+str(id),
                SimpleActionState('fig8_drone'+str(id),
                                doTrajAction, goal = doTrajGoal(shape = traj_id, id = drone_id)))
        return figure

    # MONITOR Collisions CONTAINER # Allows for internal_sm
        # EXAMPLE USE:

    #     StateMachine.add('MOVE_AND_MONITOR',
    #             move_monitor_cc,
    #             {'succeeded':'STATE_IF_SUCCEED', 
    #                             'preempted':'STATE_IF_PREEMPTED'}) 


    # monitor_sm = MonitorState('/collision_detection', std_msgs.Bool,
    #                     cond_cb = lambda ud,msg: not agent_far_away(ud,msg))

    # monitored_trajs(self, selected_drones, traj_id, internal_sm, internal_name = 'MOVE_AGENT1', monitor_sm, monitor_name='MONITOR_COLLISIONS')



    def monitored_trajs(self, internal_sm, internal_name, monitor_sm, monitor_name):
        figure = Concurrence(
                    ['succeeded','aborted','preempted','interrupted'],
                    'aborted', 
                    # default state
                    child_termination_cb = lambda so: True, 
                    # stop Concurrence if any substates stops
                    outcome_map = {
                        'succeeded':{'MOVE_AGENT1':'succeeded'},
                        'preempted':{'MOVE_AGENT1':'preempted',
                                            'MONITOR_AGENT1':'invalid'},
                        'interrupted':{'MONITOR_AGENT1':'invalid'}})
            
        with figure:
            Concurrence.add(internal_name,
                    internal_sm)
        
            Concurrence.add(monitor_name,
                monitor_sm,)

        return figure

    """OCTOGON"""
    # OCTOGON CONTAINER # move DRONE 2 (id[1]) along the octogon. Attach a LAND independent from the other drones.
    def octogon_all_drones(self, selected_drones, waypoint_array, order_array):
        sm1 = Concurrence(['succeeded', 'aborted', 'preempted'],
                'succeeded',
                )
        with sm1:
            for drone_id in selected_drones:
                figure = StateMachine(outcomes=['succeeded','aborted','preempted']) 
                with figure:
            
                    order = [(5, 6, 7, 0, 1, 2, 3, 4),
                             (3, 4, 5, 6, 7, 0, 1, 2 ),
                             (1, 2, 3, 4, 5, 6, 7, 0)]
                    self.octogon_order = order
                    for i in range(7):
                        point_for_state = Point()
                        point_for_state.x = self.octogon_points[self.octogon_order[drone_id-1][i]].x
                        point_for_state.y = self.octogon_points[self.octogon_order[drone_id-1][i]].y
                        if i <= 4 :
                            point_for_state.z = self.octogon_points[self.octogon_order[drone_id-1][i]].z + (i*0.1)
                        else :
                            point_for_state.z = self.octogon_points[self.octogon_order[drone_id-1][i]].z - (i*0.1) + 0.8 
                            
                        StateMachine.add('DRONE'+str(drone_id)+'-' + str(self.octogon_order[drone_id-1][i]),
                                            SimpleActionState('drone'+str(drone_id)+'detect_perimeter',
                                                                my_newAction, goal = my_newGoal(point = point_for_state, id = drone_id)),
                                            transitions={'succeeded' : 'DRONE'+str(drone_id)+'-' + str(self.octogon_order[i+1]), 
                                                         'aborted' : 'LAND_DRONE'+str(drone_id), 
                                                         'preempted' : 'LAND_DRONE'+str(drone_id)})

                    #Set up a continous loop
                    smach.StateMachine.add('DRONE'+str(drone_id)+'-' + str(self.octogon_order[drone_id-1][-1]),
                                    SimpleActionState('drone1detect_perimeter',
                                                        my_newAction, goal = my_newGoal(point = self.octogon_points[self.octogon_order[-1]], id = drone_id)),
                                    transitions={'succeeded' : 'DRONE1'+str(drone_id)+'-'  + str(self.octogon_order[0]), 
                                                 'aborted' : 'LAND_DRONE'+str(drone_id), 
                                                 'preempted' : 'LAND_DRONE'+str(drone_id)})


                    #Land Drone If Aborted
                    smach.StateMachine.add('LAND_DRONE'+str(drone_id)+'',
                                    SimpleActionState('land_drone'+str(drone_id),
                                                        my_newAction, goal = my_newGoal(point = self.octogon_points[3], id = drone_id)),
                                    transitions={'succeeded' : 'LAND_DRONE'+str(drone_id)})

                #add drone's traj in concurrence with other drone trajs.
                Concurrence.add('DRONE1', figure)
        return sm1

        # END OF OCTOGON CONTAINER. ######################################################################################


    #############################################
    # COMPILATION SCRIPT

    def start_sm_on_thread(self, sm_name):
        # Attach a SMACH introspection server
        sis = IntrospectionServer('smach_usecase_01', sm_name, '/TEMPLATE')
        sis.start()

        # Set preempt handler
        smach_ros.set_preempt_handler(sm_name)

        # Execute SMACH tree in a separate thread so that we can ctrl-c the script
        smach_thread = threading.Thread(target = sm_name.execute)
        smach_thread.start()


    def oneDrone_ToAndFro (self, single_id):

        sm_top = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        with sm_top:
            
            # move drone 1
            move_sm1 = self.move_drone(drone_id= single_id, traj_waypoint=self.home_points_fo8[1])
            StateMachine.add('TO',
                            move_sm1,
                            transitions={'succeeded' : 'AND FRO', 
                                            'aborted' : 'land_all', 
                                            'preempted' : 'land_all'})
            # move drone 2
            move_sm2 = self.move_drone(drone_id= single_id, traj_waypoint=self.home_points_fo8[2])
            StateMachine.add('AND FRO',
                            move_sm2,
                            transitions={'succeeded' : 'TO', 
                                            'aborted' : 'land_all', 
                                            'preempted' : 'land_all'})
            # generalized landing
            land_sm = self.land_group(selected_drones = [single_id], traj_waypoint = self.home_points_fo8)
            StateMachine.add('land_all', land_sm)

        return sm_top



    def execOctogonAndTrajOnCollision (self, ids_involved):

        sm_top = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        with sm_top:
            

            # move drone 1
            move_sm1 = self.move_drone(drone_id= self.ids[1], traj_waypoint=self.home_points_fo8[2])
            StateMachine.add('initDrone2-FIG8',
                            move_sm1,
                            transitions={'succeeded' : 'initDrone3-FIG8', 
                                            'aborted' : 'land_all', 
                                            'preempted' : 'land_all'})
            # move drone 2
            move_sm2 = self.move_drone(drone_id= self.ids[2], traj_waypoint=self.home_points_fo8[2])
            StateMachine.add('initDrone3-FIG8',
                            move_sm2,
                            transitions={'succeeded' : 'FIG8_EXECUTE', 
                                            'aborted' : 'land_all', 
                                            'preempted' : 'land_all'})
            # concurrent figures of eight
            fig8_sm = self.concurrent_trajs(selected_drones = self.ids, traj_id = 8)
            StateMachine.add('FIG8_EXECUTE', fig8_sm, transitions={'succeeded' : 'land_all', 
                                                                        'aborted' : 'land_all', 
                                                                    'preempted' : 'land_all'})

            # 3 drone octogon
            octogon_sm = self.octogon_all_drones(selected_drones = self.ids, 
                                                    waypoint_array= self.octogon_points, 
                                                    order_array = self.octogon_order)                        
            StateMachine.add('octogon_all', octogon_sm,
                                        transitions={'succeeded' : 'FIG8_EXECUTE', 
                                                     'aborted' : 'land_all', 
                                                     'preempted' : 'land_all'})

            # generalized landing
            land_sm = self.land_group(selected_drones = self.ids, traj_waypoint = self.home_points_fo8)
            StateMachine.add('land_all', land_sm)

            # monitor ros topic and setup concurrence
            monitor_s = self.monitor_general(self, '/collision_detection', std_msgs.Bool, self.activate_on_true)
            move_monitor_cc = self.monitored_trajs(internal_sm = octogon_sm, internal_name = 'DRONE_CHORE', 
                                                                    monitor_s, monitor_name='MONITOR_COLLISIONS')

            StateMachine.add('MOVE_AND_MONITOR',
                    move_monitor_cc,
                    {'succeeded':'land_all', 
                     'preempted':'HELI_EXECUTE'}) 


            # concurrent helis
            heli_sm = self.concurrent_trajs(selected_drones = self.ids, traj_id = 5)
            StateMachine.add('HELI_EXECUTE', heli_sm, transitions={'succeeded' : 'land_all', 
                                                                        'aborted' : 'land_all', 
                                                                    'preempted' : 'land_all'})

        return sm_top
  






if __name__ == '__main__':
    rospy.init_node('smach_drone_template')

    # t1 = threading.Thread(target=polygonial)
    # t1.start()
    # rospy.spin()
    all_current_ids = [3, 1, 4] # ideally from Parameter Server.
    incarnationSm = smlib(all_current_ids)

    # mention the group that gets activated.
    sm0 = incarnationSm.execOctogonAndTrajOnCollision (incarnationSm.ids)
    
    # mention the group that gets activated.
    sm0 = incarnationSm.oneDrone_ToAndFro (incarnationSm.ids)


    incarnationSm.start_sm_on_thread(sm0)

