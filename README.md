Code initiated by txa on 20 November 2020
Comments added on 27 September 2021
See Thesis

This repository documents the development process of a testbed for service drones.

* Encapsulate multi-step tasks

* Encapsulate swarm instructions

* Encapsulate individual task execution

Each of these can be examined in turn.


# multi-step tasks
Multi-step tasks encapsulate the full swarm stack, from single-robot commands to dynamic management.
> Examples: https://github.com/ThomasCarstens/Service_Drones_Thesis/blob/main/sm_structs.py
## Usage
    sm0 = self.execOctogonAndTrajOnCollision (self.ids)
    self.start_sm_on_thread(sm0)

## Development
    self.fig8_sm = concurrent_trajs(selected_drones = ids, traj_id = 8)
    StateMachine.add('FIG8_EXECUTE', self.fig8_sm, transitions={'succeeded' : 'land_all', 
                                                                'aborted' : 'land_all', 
                                                            'preempted' : 'land_all'}) 


# swarm instructions
Dynamic swarm tasks are encapsulated within the multi-step tasks.
> Examples: https://github.com/ThomasCarstens/Service_Drones_Thesis/blob/main/sm_structs.py
## Usage
NOTE: FUNCTION concurrent_trajs(args[]) readapts actionlib to smach.

    concurrent_trajs(selected_drones = ids, traj_id = 8)

## Development
    def concurrent_trajs(self, selected_drones, traj_id):
        """FIG8_EXECUTE""" 
        figure = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')
        with figure:
            for drone_id in selected_drones:
                Concurrence.add('FIG8_EXECUTE_drone'+str(id),
                SimpleActionState('fig8_drone'+str(id),
                                doTrajAction, goal = doTrajGoal(shape = traj_id, id = drone_id)))
        return figure


# individual task execution
Robot-specific instructions are encapsulated within the swarm instructions.
> Code: https://github.com/ThomasCarstens/cfScripts/blob/master/ros_action_server.py
## Usage
    SimpleActionState('fig8_drone'+str(id),
                      doTrajAction, 
                      goal = doTrajGoal(shape = traj_id, id = drone_id)))

## Development
The robot instructions run as callbacks on the server side. 

    for drone in self.allcfs.crazyflies:                              
        if drone.id == goal.id: #id is a goal argument
            drone.takeoff(targetHeight=0.6, duration=3.0)
            drone.goTo(self.waypoint, yaw=0, duration=3.0)

These robot instructions are encapsulated within a server callback.
They execute with a feedback and a result component.