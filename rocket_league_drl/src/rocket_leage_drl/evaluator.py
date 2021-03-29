class Evaluator(SnakeDQN):
    """Class for running the agent live."""
    def __init__(self):
        super().__init__()

        # Services
        self.reset_srv = rospy.Service('snake_drl/reset', Empty, self.reset_cb)

        self.lock = Lock()

        try:
            # initialize variables
            self.reset()

            while not rospy.is_shutdown():
                self.lock.acquire()

                # Wait for state
                while not self.wait_for_state():
                    pass

                # update state
                state, __, done, __ = self.get_env(rospy.Time.now())

                # check for end of game and react
                if not done:
                    self.clear_state()
                    self.action_pub.publish(
                        self.get_action_msg(
                            self.agent.eval((state, 0.0, done, {}))))

                self.lock.release()

        except rospy.ROSInterruptException:
            pass

    def reset_cb(self, reset_srv):
        """Thread safe reset."""
        with self.lock:
            self.reset()
        return EmptyResponse()