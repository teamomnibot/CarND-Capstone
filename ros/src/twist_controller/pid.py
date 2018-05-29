
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM, k_awu = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.int_val = self.last_error = 0.0

	# Anti Wind up factor 
	self.k_awu = k_awu
	self.last_windup = 0.0


    def reset(self):
        self.int_val = 0.0
	self.last_windup = 0.0

    def step(self, error, sample_time):

        #integral = self.int_val + error * sample_time;
	error_int = self.ki * error + self.k_awu * self.last_windup
	integral = self.int_val + error_int * sample_time;

        derivative = (error - self.last_error) / sample_time;

        #ctrl_y = self.kp * error + self.ki * integral + self.kd * derivative;
	ctrl_y = self.kp * error + integral + self.kd * derivative;

        if ctrl_y > self.max:
            val = self.max
        elif ctrl_y < self.min:
            val = self.min
        else:		
            self.int_val = integral
	    val = ctrl_y

        self.last_error = error

        return val
