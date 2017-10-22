# import rospy

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, params, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = params[0]
        self.ki = params[1]
        self.kd = params[2]
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        # rospy.loginfo('in pid, error = %s, sample_time: %s ' % (error,sample_time))

        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;

        # rospy.loginfo('in pid, y_calc = %s ' % (y))
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        # rospy.loginfo('in pid, y_out = %s ' % (val))

        return val

