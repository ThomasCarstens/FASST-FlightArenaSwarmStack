#! /usr/bin/env python
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


class polygonial():
    def __init__(self):

        self.ids = [3, 1, 4]
        #define the differents points
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

    ###############################################
    # FUNCTIONS: create containers linked to the action server.

        # SEE USE EXAMPLE HERE: for instance, using concurrent_trajs(args[]):
        # self.fig8_sm = concurrent_trajs(selected_drones = ids, traj_id = 8)
        # StateMachine.add('FIG8_EXECUTE', self.fig8_sm, transitions={'succeeded' : 'land_all', 
        #                                                             'aborted' : 'land_all', 
        #                                                         'preempted' : 'land_all'})

    # CONTAINER: MOVE DRONE [id] to waypoint [index]
    def move_drone(self, drone_id, traj_waypoint):
        figure = SimpleActionState('drone'+str(drone_id)+'detect_perimeter', 
                        my_newAction, 
                        goal = my_newGoal(point = traj_waypoint, id = drone_id )),
        return figure

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



    def compile_sm (self, ids_involved, sm_ref):

        sm_ref = StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        with sm_ref:
            

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


            # monitor an external flag and preempt if detected
            def agent_far_away(ud, msg):
                if msg.data == True:
                    return True
                return False

            monitor_s = MonitorState('/collision_detection', std_msgs.Bool,
                                cond_cb = lambda ud,msg: not agent_far_away(ud,msg))

            move_monitor_cc = self.monitored_trajs(internal_sm = octogon_sm, internal_name = 'DRONE_CHORE', monitor_s, monitor_name='MONITOR_COLLISIONS')

            StateMachine.add('MOVE_AND_MONITOR',
                    move_monitor_cc,
                    {'succeeded':'land_all', 
                     'preempted':'land_all'}) 


        return sm_ref
        
    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    #progressively add drones
    with sm0:

        """
        StateMachine.add('HELI_START',
                         SimpleActionState('drone1detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_heli[0], id = ids[0] )),
                        transitions={'succeeded' : 'HELI_EXECUTE', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        StateMachine.add('HELI_EXECUTE',
                         SimpleActionState('trajectory_action',
                                            doTrajAction, goal = doTrajGoal(shape = 1, id = ids[0] )),
                         transitions={'succeeded' : 'HELI_END', 'aborted' : 'land_all', 'preempted' : 'land_all'})



        StateMachine.add('HELI_END',
                        SimpleActionState('land_drone1',
                                             my_newAction, goal = my_newGoal(point = octogon_points[3], id = ids[0]) ),
                                             transitions={'succeeded' : 'initDrone2-FIG8', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        """


        # MOVE THE DRONES INDEPENDENTLY # To the first Fo8 points.
        #

        StateMachine.add('initDrone2-FIG8',
                         SimpleActionState('drone2detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_fo8[2], id = ids[1] )),
                        transitions={'succeeded' : 'initDrone3-FIG8', 'aborted' : 'land_all', 'preempted' : 'land_all'})
        StateMachine.add('initDrone3-FIG8',
                         SimpleActionState('drone3detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_fo8[1], id = ids[2] )),
                        transitions={'succeeded' : 'FIG8_EXECUTE', 'aborted' : 'land_all', 'preempted' : 'land_all'})


        """FIG8_EXECUTE""" 
        ###############################################
        # CONTAINER: SIMULTANEOUS Fo8s # Using all the ids currently running.
        #
        fig8_sm = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
        with fig8_sm:
            for id in ids:
                Concurrence.add('FIG8_EXECUTE_drone'+str(id),
                SimpleActionState('fig8_drone'+str(id),
                                doTrajAction, goal = doTrajGoal(shape = 8, id = id)))

        ##############################################

        StateMachine.add('FIG8_EXECUTE', fig8_sm, transitions={'succeeded' : 'moveDrone2-2', 'aborted' : 'land_all', 'preempted' : 'land_all'})


        """HELI_EXECUTE"""  
        ###############################################
        # CONTAINER: SIMULTANEOUS SPIRALS # When working with 2 drones.
        #
        heli_sm = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
        with heli_sm:
            Concurrence.add('HELI_EXECUTE_drone2',
            SimpleActionState('heli_drone2',
                            doTrajAction, goal = doTrajGoal(shape = 0, id = ids[1])))

            Concurrence.add('HELI_EXECUTE_drone3',
            SimpleActionState('heli_drone3',
                                doTrajAction, goal = doTrajGoal(shape = 0, id = ids[2])))
        ##############################################

        # This gets executed when 'HELI_EXECUTE' is called.
        StateMachine.add('HELI_EXECUTE', heli_sm, transitions={'succeeded' : 'moveDrone2-2', 'aborted' : 'land_all', 'preempted' : 'land_all'})


        """LAND_THE_TWO"""  
        ###############################################
        # CONTAINER: LAND TWO DRONES # When working with 2 drones.
        #

        land_two = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')#,
                                        #outcome_map={'succeeded' : 'moveDrone2-2', 'preempted' : 'land_all', 'aborted' : 'land_all' })

        with land_two:
            Concurrence.add('LAND_THE_TWO_drone2',
                SimpleActionState('land_drone2',
                                    my_newAction, goal = my_newGoal(point = octogon_points[3], id = ids[1]) ))

            Concurrence.add('LAND_THE_TWO_drone3',
                SimpleActionState('land_drone3',
                                    my_newAction, goal = my_newGoal(point = octogon_points[3], id = ids[2]) ))

        ################################################

        # This gets executed when 'LAND_THE_TWO' is called.
        StateMachine.add('LAND_THE_TWO', land_two, transitions={'succeeded' : 'moveDrone2-2', 'aborted' : 'land_all', 'preempted' : 'land_all'})



        # MOVE THE DRONES INDEPENDENTLY # To the first octogon points.
        #
        StateMachine.add('moveDrone2-2',
                         SimpleActionState('drone2detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = octogon_points[2], id = ids[1] )),
                        transitions={'succeeded' : 'drone3-1', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        StateMachine.add('drone3-1',
                         SimpleActionState('drone3detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = octogon_points[0], id = ids[2] )),
                        transitions={'succeeded' : 'drone1-3', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        StateMachine.add('drone1-3',
                         SimpleActionState('drone1detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = octogon_points[4], id = ids[0] )),
                        transitions={'succeeded' : 'infinite_Octogon_3drones', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        """land_all"""        
        ###############################################
        # FULL LAND CONTAINER # Land all the drones to their respective points
        land_sm =  Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
        with land_sm:

            for id in ids:
                Concurrence.add('LAND_DRONE'+str(id),
                                SimpleActionState('land_drone'+str(id),
                                                    my_newAction, goal = my_newGoal(point = octogon_points[3], id = id)  )) 
        #############################################

        # This gets executed when 'land_all' is called.
        StateMachine.add('land_all', land_sm)


        ###############################################
        # OCTOGON_3DRONES CONTAINER # Execute Octogon with 3
        #

        def get_pointforstate(order, i):
            point_for_state = Point()

            if i <= 4 :
                point_for_state.z = octogon_points[order[i]].z + (i*0.1)
            else :
                point_for_state.z = octogon_points[order[i]].z - (i*0.1) + 0.8 

            return point_for_state

        # Concurrence of Drones 1 / 2 / 3
        sm1 = Concurrence(['succeeded', 'aborted', 'preempted'],
                'succeeded',
                )

        with sm1:
            drone1 = StateMachine(outcomes=['succeeded','aborted','preempted'])  

            test_drone = StateMachine(outcomes=['succeeded','aborted','preempted'])  

            """TEST_DRONE"""
            # TEST_DRONE CONTAINER # move DRONE 1 (id[0]) to point [0, 0, 0.5]. SIMPLE TAKEOFF
            #
            #
            #
            Concurrence.add('DRONE1', test_drone)
            with test_drone:
                
                order = (5, 6, 7, 0, 1, 2, 3, 4)
                for i in range(7):
                    point_for_state.x = self.test_point.x
                    point_for_state.y = self.test_point.y

                    if i <= 4 :
                        point_for_state.z = test_point.z + (i*0.1)
                    else :
                        point_for_state.z = test_point.z - (i*0.1) + 0.8 

                    StateMachine.add('DRONE1-' + str(order[i]),
                                     SimpleActionState('drone1detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[0])),
                                       transitions={'succeeded' : 'DRONE1-' + str(order[i+1]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})

                #make it infinit
                smach.StateMachine.add('DRONE1-' + str(order[-1]),
                               SimpleActionState('drone1detect_perimeter',
                                                    my_newAction, goal = my_newGoal(point = test_point, id = ids[0])),
                              transitions={'succeeded' : 'DRONE1-'  + str(order[0]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})
            #
            #
            #
            # END OF TEST_DRONE CONTAINER. ######################################################################################

            """OCTOGON"""
            # OCTOGON CONTAINER # move DRONE 2 (id[1]) along the octogon. Attach a LAND independent from the other drones.
            #
            Concurrence.add('DRONE1', drone1)
            with drone1:
                order = (5, 6, 7, 0, 1, 2, 3, 4)
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = octogon_points[order[i]].x
                    point_for_state.y = octogon_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = octogon_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = octogon_points[order[i]].z - (i*0.1) + 0.8 
                    StateMachine.add('DRONE1-' + str(order[i]),
                                     SimpleActionState('drone1detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[0])),
                                       transitions={'succeeded' : 'DRONE1-' + str(order[i+1]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})

                #make it infinit
                smach.StateMachine.add('DRONE1-' + str(order[-1]),
                               SimpleActionState('drone1detect_perimeter',
                                                    my_newAction, goal = my_newGoal(point = octogon_points[order[-1]], id = ids[0])),
                              transitions={'succeeded' : 'DRONE1-'  + str(order[0]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE1',
                               SimpleActionState('land_drone1',
                                                    my_newAction, goal = my_newGoal(point = octogon_points[3], id = ids[0])),
                              transitions={'succeeded' : 'LAND_DRONE1'})
            #
            # END OF OCTOGON CONTAINER. ######################################################################################


            drone2 = StateMachine(outcomes=['succeeded','aborted','preempted'])  # ['succeeded','aborted','preempted']
            Concurrence.add('DRONE2', drone2)


            # OCTOGON CONTAINER # move DRONE 2 (id[1]) along the octogon. Attach a LAND independent from the other drones.
            #
            with drone2:
                #add each state
                order = (3, 4, 5, 6, 7, 0, 1, 2 )
                for i in range(7):

                    point_for_state = Point()
                    point_for_state.x = octogon_points[order[i]].x
                    point_for_state.y = octogon_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = octogon_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = octogon_points[order[i]].z - (i*0.1) + 0.8 

                    
                    StateMachine.add('DRONE2-' + str(order[i]),
                                     SimpleActionState('drone2detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[1])),
                                       transitions={'succeeded' : 'DRONE2-' + str(order[i+1]), 'aborted' : 'LAND_DRONE2', 'preempted' : 'LAND_DRONE2'})

                #make it infinit
                smach.StateMachine.add('DRONE2-' + str(order[-1]),
                             SimpleActionState('drone2detect_perimeter',
                                                     my_newAction, goal = my_newGoal(point = octogon_points[order[-1]], id = ids[1])),
                             transitions={'succeeded' : 'DRONE2-' + str(order[0]), 'aborted' : 'LAND_DRONE2', 'preempted' : 'LAND_DRONE2'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE2',
                               SimpleActionState('land_drone2',
                                                    my_newAction, goal = my_newGoal(point = octogon_points[3], id = ids[1])),
                              transitions={'succeeded' : 'LAND_DRONE2'})
            #
            # END OF OCTOGON CONTAINER. ######################################################################################


            drone3 = StateMachine(outcomes=['succeeded','aborted','preempted'])  # ['succeeded','aborted','preempted']
            Concurrence.add('DRONE3-', drone3)



            # OCTOGON CONTAINER # move DRONE 3 (id[2]) along the octogon. Attach a LAND independent from the other drones.
            with drone3:
                #add each state
                order = (1, 2, 3, 4, 5, 6, 7, 0)
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = octogon_points[order[i]].x
                    point_for_state.y = octogon_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = octogon_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = octogon_points[order[i]].z -(i*0.1) + 0.8 
                    StateMachine.add('DRONE3-' + str(order[i]),
                                     SimpleActionState('drone3detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point =point_for_state, id = ids[2])),
                                       transitions={'succeeded' : 'DRONE3-' + str(order[i+1]), 'aborted' : 'LAND_DRONE3', 'preempted' : 'LAND_DRONE3'})

                #make it infinit
                smach.StateMachine.add('DRONE3-' + str(order[-1]),
                                   SimpleActionState('drone3detect_perimeter',
                                                   my_newAction, goal = my_newGoal(point = octogon_points[order[-1]], id = ids[2])),
                                   transitions={'succeeded' : 'DRONE3-'  + str(order[0]), 'aborted' : 'LAND_DRONE3', 'preempted' : 'LAND_DRONE3'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE3',
                               SimpleActionState('land_drone3',
                                                    my_newAction, goal = my_newGoal(point = octogon_points[3], id = ids[2])),
                              transitions={'succeeded' : 'LAND_DRONE3'})


            # END OF OCTOGON CONTAINER. ######################################################################################

        StateMachine.add('infinite_Octogon_3drones', sm1)


        sm0 = compile_sm (ids)

        start_sm_on_thread(sm0)






if __name__ == '__main__':
    rospy.init_node('smach_drone_template')
    t1 = threading.Thread(target=polygonial)
    t1.start()
    rospy.spin()
