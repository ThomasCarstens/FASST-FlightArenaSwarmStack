

# System Overview of Ground Station

![System Overview of Ground Station](/some_swarm_github.png "Swarm Diagram")

-------------------------------------------------------------------------------------------------------------
\textbf{Preemption}  — is the act of temporarily interrupting an executing task, with the intention of resuming it at a later time. This interrupt is done by an external scheduler with no assistance or cooperation from the task.

    # ACTION EXECUTION EXAMPLE
    while self.success == False: #While condition is not fulfilled
        ... 
        # If external preempt is executed:
        if self.actionServerInstance.is_preempt_requested():
            #Transition State Machine
            self.actionServerInstance.set_preempted() 
    
    #Once the condition is True: Transition State Machine
    self.actionServerInstance.set_succeeded() 

\captionof{minted}{\textbf{Template 1:} Example of Premption. This sets the result of the action to 'preempted' after which next states can return to the action at a later point.}

-----------------------------------------------------------------------------------------------------------

\textbf{Interruption} — is a process that tells the computer to stop running the current program so that a new one can be started or a circuit that carries such a signal. 

\begin{marginfigure}%
  \includegraphics[width=4cm]{images/testbed/microservice/client.png}
  \caption{An Action \textbf{Interruption} is executed in the Client-side State Machine.}
  \label{fig:marginfig}
\end{marginfigure}

Consider a three-step process: (1) Monitoring the agent. (2) If a condition is met, choosing the correct next action. (3) Interrupting the drone’s current action and executing the different action.

    move_monitor_cc = Concurrence(
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
    
    StateMachine.add('MOVE_AND_MONITOR',
            move_monitor_cc,
            {'succeeded':'STATE_IF_SUCCEED', 
                            'preempted':'STATE_IF_PREEMPTED'}) 
    
    with move_monitor_cc:
        Concurrence.add('MOVE_AGENT1',
                SimpleActionState('_goTo_agent1', goToAction,
                                  goal=_goTo_goal))
    
        def agent_far_away(ud, msg):
            """Returns True while agent pose 
            is at least 1 unit away from (2,5)"""
            if sqrt(pow(msg.x-9.0,2) + pow(msg.y-5.0,2)) > 2.0:
                return True
            return False
    
        Concurrence.add('MONITOR_AGENT1',
            MonitorState('/agent1/pose', geometry_msgs.Point,
                cond_cb = lambda ud,msg: not agent_far_away(ud,msg)),)
                                

\end{minted}
\captionof{minted}{\textbf{Template 2:} Example of Action Interruption. An example of interruption occurs when a drone reacts when another drone gets “too close”.}

