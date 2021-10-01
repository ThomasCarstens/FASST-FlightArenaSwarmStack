        #"""Enter P#meter1 ACTION SERVER [name='cf4_go', drone: goal.id, variable:'_as_cf4_go', callback:'execute_cb_cf4_go']"""
        self._feedback_cf4_go = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf4_go = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf4_go = 'cf4_go'
        print (self._action_name_cf4_go)
        self._as_cf4_go = actionlib.SimpleActionServer(self._action_name_cf4_go, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf4_go, auto_start = False)
        self._as_cf4_go.start()
        print("Ready to move _cf4.")



    def execute_cb_cf4_go(self, goal):
        #"""MOVING cf2 TO GOAL goal.point
		#detectperimeter1 ACTION SERVER [name='cf4_go', drone: cf4, variable:'_cf4_go', callback:'execute_cb_cf4_go']"""

        #speak (engine, "Moving to point.")

        self.PoseListener()

        print ("CF2 Action Server callback")
        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_cf4_go.position = Pose()
        self._feedback_cf4_go.time_elapsed = Duration(5)

        self.success_cf4 = False
        self.enable_cf4 = True
        self.waypoint = np.array([goal.point.x, goal.point.y, goal.point.z])

        #speak (engine, "YO")
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:
                print("send COMMANDS to cf...", goal.id)
                self._feedback_cf4_go.position.position.x = cf.position()[0]
                self._feedback_cf4_go.position.position.y = cf.position()[1]
                self._feedback_cf4_go.position.position.z = cf.position()[2]

        for cf in self.allcfs.crazyflies:
            if self.enable_cf4 == True:                                     #ENABLE
                if cf.id == goal.id:
                    #print("drone moves.")
                    cf.goTo(self.waypoint, yaw=0, duration=3.0)
                    #print("drone pose is", self.cf3_pose)
                    #self.enable_cfx == False

        while self.success_cf4 == False:
            self.PoseListener()

            print ("point is", goal.point)
            print("id is " + str(goal.id))

            if self._as_cf4_go.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf4_go)
                self._as_cf4_go.set_preempted()
                #for cf in self.allcfs.crazyflies:
                    #if cf.id == goal.id:
                        #print("LANDING cf...", goal.id)
                        #cf.land(0.04, 2.5)
                break

            print ("Not yet...")
            #now we test if he has reached the desired point.
            self.perimeter_monitor(goal.id, goal.point)

            try:
                # publish the feedback
                self._as_cf4_go.publish_feedback(self._feedback_cf4_go)

            except rospy.ROSInterruptException:
                rospy.loginfo("except clause opened")
                pass


        if self.success_cf4 == True:
            #for cf in self.allcfs.crazyflies:
                #print(cf.id)
                #print("press button to continue")
                #self.swarm.input.waitUntilButtonPressed()
                #cf.land(0.04, 2.5)
            print("Reached the perimeter!!")
            self.success_cf4 = False
            self._result_cf4_go.time_elapsed = Duration(5)
            self._result_cf4_go.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback_cf4_go)
            rospy.loginfo('%s: Succeeded' % self._action_name_cf4_go)
            self._as_cf4_go.set_succeeded(self._result_cf4_go)

