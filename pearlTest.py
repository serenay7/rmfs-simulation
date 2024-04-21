
class MODEL():
    def __init__(self, env, robotList=None):
        self.env = env
        self.robotList = robotList

    def modelTest(self):
        for robot in robotlist:
            print(robot.status)

class Robot():
    def __init__(self, env, model=None, status=0):
        self.env = env
        self.model = model
        self.status = status

    def extract(self):
        self.status = 1
        self.model.modelTest()

env = 1
robot1 = Robot(env)
robot2 = Robot(env)

robotlist = [robot1, robot2]
mod = MODEL(env, robotlist)
robot1.model = mod
robot2.model = mod

robot1.extract()


import numpy as np

# Create a sample 2D NumPy array
array_2d = np.array([[1, 2, 3],
                     [4, 5, 6],
                     [7, 8, 9]])

# List of column indices to be retrieved
column_indices = [2, 0]

# Retrieve the columns
columns = array_2d[:, column_indices]

# Print the list of columns
print("List of Columns:")
print(columns)