##### Code initiated by txa on 20 November 2020
##### Comments added on 27 September 2021
##### See Thesis

### As part of the testbed for service drones, we offer a library of **well-known swarm behaviors**, which is offered open-source to practitioners of the Crazyswarm Project. This is the first collection of ‘swarm patterns’ known to-date on this swarm framework. The library includes:

* Encapsulating multi-step tasks

* Encapsulating swarm instructions

* Encapsulating individual task execution

# multi-step tasks
Multi-step tasks encapsulate the full swarm stack, from single-robot commands to dynamic management.
|def oneDrone_ToAndFro (self, single_id): |def execOctogonAndTrajOnCollision (self, ids_involved): |
|-- | -- |
|       # move to position 1
        # move to position 2 
        # loop
        # generalized landing at abort |        
        
        # move drone 1
        # move drone 2
        # concurrent figures of eight
        # 3 drone octogon
        # generalized landing
        # monitor an external flag and preempt if detected
        # concurrent helis |


    def oneDrone_ToAndFro (self, single_id):
        # move to position 1
        # move to position 2 
        # loop
        # generalized landing at abort

    def execOctogonAndTrajOnCollision (self, ids_involved):

        # move drone 1
        # move drone 2
        # concurrent figures of eight
        # 3 drone octogon
        # generalized landing
        # monitor an external flag and preempt if detected
        # concurrent helis
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