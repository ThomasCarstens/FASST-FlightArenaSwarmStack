# Service_Drones_Thesis

\begin{minted}
[
frame=lines,
framesep=2mm,
baselinestretch=1.2,
bgcolor=LightGray,
fontsize=\footnotesize,
linenos
]
{python}
    # ACTION EXECUTION EXAMPLE
    while self.success == False: #While condition is not fulfilled
        # Conditional check: has drone reached a point?
        self.success = self._reachedGoal(goal.id, self.initial_pose) 
        
        # If external preempt is executed:
        if self.actionServerInstance.is_preempt_requested():
            #Transition State Machine
            self.actionServerInstance.set_preempted() 
            #Landing the drone by id
            self.landDrone(goal.id) 
    
    #Once the condition is True: Transition State Machine
    self.actionServerInstance.set_succeeded() 
                                

\end{minted}
%\caption{Example of Action Concurrence Code.}
\captionof{minted}{\textbf{Template 1:} Example of Condition Testing.}