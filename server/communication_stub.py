import robot_stub as robot

class Communication(object):
	def __init__(self):
		self.robot = robot.Robot()

	def state(self):
		return self.robot.state()

	def move(self, speed, rotate):
		self.robot.move(speed, rotate)

	def sense(self):
		return [[int(x) for x in y] for y in self.robot.sense()]

	def stop(self):
		pass

	def start(self):
		pass

	def resume(self):
		pass

	def pause(self):
		pass