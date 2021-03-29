class Trainer(SnakeDQN):
    """Class for training the agent."""
    def __init__(self):
        super().__init__()

        # Constants
        self.DELTA_T = rospy.Duration.from_sec(1.0 / rospy.get_param('~rate', 30.0))
        self.MAX_T   = rospy.Duration.from_sec(rospy.get_param('~max_episode_time', 60.0))

        # Publishers
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        self.log_pub = rospy.Publisher('snake_drl/training', DiagnosticStatus, queue_size=1)

        # Services
        self.reset_srv = rospy.ServiceProxy('snake/reset', Empty)

        self.time = rospy.Time.from_sec(time.time())
        self.start_time = rospy.Time.from_sec(self.time.to_sec())

        # Main loop
        try:
            # wait for snake to initialize
            rospy.loginfo("Waiting for snake to initialize.")
            while not self.wait_for_state():
                self.time += self.DELTA_T
                self.clock_pub.publish(self.time)
            rospy.loginfo("Done.")

            # initialize variables
            self.reset()

            # train
            self.train(rospy.get_param('~num_episodes', 1000))

            # idle wait to allow saving weights, etc
            rospy.loginfo("Done Training.")
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def train(self, num_episodes):
        for episode in range(num_episodes):
            # reset episode and get initial state
            obs, reward, done, __ = self.reset_env()

            net_reward = reward
            steps = 0
            while not done:
                action = self.agent.act((obs, reward, done, {}))
                state, reward, done, __ = self.step(action)

                net_reward += reward
                steps += 1

            log_msg = DiagnosticStatus()
            log_msg.name = "DQN training"
            log_msg.values.append(KeyValue(key="episode", value=str(episode+1)))
            log_msg.values.append(KeyValue(key="score", value=str(self.score)))
            log_msg.values.append(KeyValue(key="net_reward", value=str(net_reward)))
            log_msg.values.append(KeyValue(key="steps", value=str(steps)))
            self.log_pub.publish(log_msg)

    def step(self, action):
        """Step simulation one time step and return new state."""
        # Publish action
        self.clear_state()
        self.action_pub.publish(self.get_action_msg(action))

        # Advance time
        self.time += self.DELTA_T
        self.clock_pub.publish(self.time)

        # Wait for snake
        if not self.wait_for_state():
            rospy.logwarn("Timeout waiting to get state from sim; Attempting small time advance.")
            self.time += 0.25*self.DELTA_T
            self.clock_pub.publish(self.time)
            if not self.wait_for_state():
                rospy.logwarn("Small time advance failed; Attempting large time advance.")
                self.time += self.DELTA_T
                self.clock_pub.publish(self.time)
                if not self.wait_for_state():
                    rospy.logerr("Failed to get state from sim. Exiting")
                    sys.exit(1)

        # return state
        state, reward, done, dict = self.get_env(self.time)
        if self.time - self.start_time >= self.MAX_T:
          done = True

        return (state, reward, done, dict)

    def reset_env(self):
        """Reset the environment for new episode."""
        # Clear last action
        self.action_pub.publish(Twist())

        # Reset
        self.reset_srv.call()
        self.reset()

        # Advance time (twice due to how sim's internal reset works)
        self.time += self.DELTA_T
        self.clock_pub.publish(self.time)
        self.time += self.DELTA_T
        self.clock_pub.publish(self.time)

        # Wait for snake
        if not self.wait_for_state():
            rospy.logwarn("Timeout waiting to get state from sim after reset; Attempting small time advance.")
            self.time += 0.25*self.DELTA_T
            self.clock_pub.publish(self.time)
            if not self.wait_for_state():
                rospy.logwarn("Small time advance failed; Attempting large time advance.")
                self.time += self.DELTA_T
                self.clock_pub.publish(self.time)
                if not self.wait_for_state():
                    rospy.logerr("Failed to get state from sim after reset. Exiting")
                    sys.exit(1)

        self.start_time = rospy.Time.from_sec(self.time.to_sec())

        # return initial state
        return self.get_env(self.time)