

As part of the testbed for service drones, we offer a library of **well-known swarm behaviors**, which is offered open-source to practitioners of the Crazyswarm Project. This is the first collection of ‘swarm patterns’ known to-date on this swarm framework. The library includes:

* **Encapsulating multi-step tasks**

    *Multi-step tasks encapsulate the full swarm stack, from single-robot commands to dynamic management.*

* **Encapsulating swarm instructions**

    *Dynamic swarm tasks are encapsulated within the multi-step tasks.*

* **Encapsulating individual task execution**

    *Robot-specific instructions are encapsulated within the swarm instructions.*

*Code initiated by txa on 20 November 2020*

*Comments added on 27 September 2021*

*Contact: ThomasCarstens*

# multi-step tasks
The drone ids are selected as an argument.

| smlib.oneDrone_ToAndFro (id) | smlib.execOctogonAndTrajOnCollision (ids) |
|-- | -- |
| # move to position 1<br># move to position 2<br># loop<br># generalized landing at abort | # move drone 1<br># move drone 2<br># concurrent figures of eight<br># 3 drone octogon<br># generalized landing<br># monitor an external flag and preempt if detected<br># concurrent helis |

> More Examples: https://github.com/ThomasCarstens/Service_Drones_Thesis/blob/main/sm_structs.py


## Usage
    sm0 = self.execOctogonAndTrajOnCollision (self.ids)
    self.start_sm_on_thread(sm0)

## Development
    self.fig8_sm = concurrent_trajs(selected_drones = ids, traj_id = 8)
    StateMachine.add('FIG8_EXECUTE', self.fig8_sm, transitions={'succeeded' : 'land_all', 
                                                                'aborted' : 'land_all', 
                                                            'preempted' : 'land_all'}) 


# swarm instructions
| smlib.move_drone(drone_id, traj_waypoint) | smlib.monitor_general(monitor_topic, monitor_type, truth_function) | smlib.land_group(self, selected_drones, traj_waypoint) | smlib.concurrent_trajs(self, selected_drones, traj_id) |
|-- | -- | -- | -- |
| MOVE DRONE [id] to waypoint [index] |   | Land all the drones to their respective points | CONCURRENT Fo8s CONTAINER # Using all the ids currently running. |


> More Examples: https://github.com/ThomasCarstens/Service_Drones_Thesis/blob/main/sm_structs.py
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

| trajectory_action | fig8_ | detect_perimeter | cf4_go | cf3_follow_cf2 |
|-- | -- | -- | -- | -- |
| RUNNING HELI goal.shape on cfx (goal.id) | RUNNING FIG8 goal.shape on cfx (goal.id)   | MOVING cfx (goal.id) TO GOAL goal.point |  | MOVE DRONE 1 goal.id TO DRONE 2 POSE goal.point |

> More Examples: https://github.com/ThomasCarstens/cfScripts/blob/master/ros_action_server.py
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
The server informs the execution with **feedback** and **result** data.