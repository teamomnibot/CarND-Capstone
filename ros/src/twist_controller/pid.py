
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM,k_awu = 0.50):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.
	
	# Anti Wind up factor
	self.k_awu = k_awu
	self.last_windup = 0.0
	

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0
	self.last_windup = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        #integral = self.int_val + error * sample_time;
	error_int =  error + self.k_awu * self.last_windup
	integral = self.int_val + error_int * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val = max(self.min, min(y, self.max))

        self.int_val = integral
        self.last_error = error
	self.last_windup = val-y

        return val
