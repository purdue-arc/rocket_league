        # DRL Agent
        agent_type = rospy.get_param('~agent_type', "dqn")
        OBSERVATION_SIZE = 3 + 2*self.NUM_SEGMENTS
        if agent_type == "dqn":
            self.agent = DQN_Agent(
                OBSERVATION_SIZE,
                SnakeActions.SIZE,
                params=rospy.get_param('~dqn'))
        elif agent_type == "vac":
            self.agent = VAC_Agent(
                OBSERVATION_SIZE,
                SnakeActions.SIZE,
                params=rospy.get_param('~vac'))
        else:
            rospy.logerr("Uncompatible agent type")
            exit(1)

                    # Services
        # TODO make a service (requires custom messages)
        rospy.Subscriber('snake/dqn/load_weights', String, self.load_cb)
        rospy.Subscriber('snake/dqn/save_weights', String, self.save_cb)

            def wait_for_state(self):
        """Allow other threads to handle callbacks."""
        with self.cond:
            ret = self.cond.wait_for(self.has_state, 0.3)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()
        else:
            return ret

                def load_cb(self, load_msg):
        """Callback to load model weights."""
        self.agent.load(load_msg.data)

    def save_cb(self, save_msg):
        """Callback to save model weights."""
        self.agent.save(save_msg.data)