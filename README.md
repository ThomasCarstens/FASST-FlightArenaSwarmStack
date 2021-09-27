# Service_Drones_Thesis

## Code initiated by txa on 20 November 2020
## Comments added on 27 September 2021
## Saved under https://github.com/ThomasCarstens/cfScripts/blob/master/ros_action_server.py

 OUT OF CLASS: USAGE #############################################################################
''' sm0 = self.execOctogonAndTrajOnCollision (self.ids)
 self.start_sm_on_thread(sm0)'''
 NOTE: RUNS specified command in packaged state machine.

 WITHIN STATE MACHINE: USAGE #####################################################################
 '''
 self.fig8_sm = concurrent_trajs(selected_drones = ids, traj_id = 8)
 StateMachine.add('FIG8_EXECUTE', self.fig8_sm, transitions={'succeeded' : 'land_all', 
                                                             'aborted' : 'land_all', 
                                                         'preempted' : 'land_all'}) '''
 NOTE: FUNCTION concurrent_trajs(args[]) readapts actionlib to smach.

 WITHIN FUNCTIONS:                      ########################## ##############################
    '''
 def concurrent_trajs(self, selected_drones, traj_id):
     """FIG8_EXECUTE""" 
     figure = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
     with figure:
         for drone_id in selected_drones:
             Concurrence.add('FIG8_EXECUTE_drone'+str(id),
             SimpleActionState('fig8_drone'+str(id),
                             doTrajAction, goal = doTrajGoal(shape = traj_id, id = drone_id)))
     return figure'''
 NOTE: create containers for the actionlib functions, accessible at 
 https://github.com/leonard-de-vinci/intelligent-drone-lab/blob/master/ros_ws/src/crazyswarm/scripts/ros_action_server.py