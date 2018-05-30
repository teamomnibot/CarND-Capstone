
MIN_NUM = float('-inf')
MAX_NUM = float('inf')

import rospy 

class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM,k_awu = 0.50):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.

        # Anti Wind up factor
        self.k_awu = k_awu
 


    def reset(self):
        self.int_val = 0.0
        self.last_error = 0.0


    def step(self, error, sample_time):



        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * integral + self.kd * derivative;

        val = y
        windup = False

        if (y<self.min):
            val = self.min
            windup = True
        if (y>self.max): 
            val = self.max
            windup = True

        if windup==False:
            self.int_val = integral

        self.last_error = error

        #rospy.logwarn("error %f, integral %f y %f miny %f maxy %f"%(error,integral,y,self.min,self.max))

        return val
