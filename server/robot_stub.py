import time
import math
import random
from scipy import signal
import numpy as np

class Robot(object):
	def __init__(self):
		self.last_time = time.time()
		self.current_time = time.time()
		self.speed = 0.0
		self.rotating = False

		self.heading = 0.0
		self.x = 0.0
		self.y = 0.0

	def state(self):
		if time.time() - self.current_time < 0.02:
			time.sleep(0.02)
		self.last_time = self.current_time
		self.current_time = time.time()

		delta_t = self.current_time - self.last_time
		distance = self.speed * delta_t * (not self.rotating) * 0.1 * 255

		delta_x = distance * math.cos(math.radians(self.heading))
		delta_y = distance * math.sin(math.radians(self.heading))
		delta_heading = self.speed * delta_t * self.rotating * 0.5 * 255

		self.heading += delta_heading
		self.x += delta_x
		self.y += delta_y

		return delta_x, delta_y, delta_heading

	def move(self, speed, rotate):
		self.rotating = rotate
		self.speed = speed

	def sense(self):
		if time.time() - self.current_time < 0.02:
			time.sleep(0.02)
		t = self.current_time
		m = 180
		angle = 3*random.random()-1.5 #m - abs(t*100 % (2*m) - m) - 90
		d = 240 - self.x + 20*random.random()
		distance = abs(d/math.cos(math.radians(angle+self.heading)))

		front = distance if (distance < 255 and np.sign(math.cos(math.radians(angle + self.heading))) > 0) else 255
		rear = distance if (distance < 255 and np.sign(math.cos(math.radians(180 + angle + self.heading))) > 0) else 255

		r = []
		direction = 2*(1-int(not self.rotating))
		if front not in [-1, 255]:
			r.append((1000*self.speed, 1000*direction*self.speed, angle, front))
			print(r)
		if rear not in [-1, 255]:
			r.append((1000*self.speed, 1000*direction*self.speed, angle+180, rear))

		return r