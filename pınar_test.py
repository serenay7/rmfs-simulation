import simpy

class ChargingStation(simpy.Resource):
    def __init__(self, env, capacity, location, currentRobot=None):
        super().__init__(env=env, capacity=capacity)
        self.env = env
        self.capacity = capacity
        self.location = location
        self.currentRobot = currentRobot

class Robot:
    def __init__(self, env, batteryLevel=100):
        self.env = env
        self.batteryLevel = batteryLevel
        self.taskList = []
        self.chargingStations = []

    def dropPod(self, pod):
        pod.status = "idle"
        pod.robot = None
        self.pod = None
        yield self.env.timeout(self.dropTime)

        if self.batteryLevel =< 80:
            yield self.env.process(self.checkAndGoToChargingStation())
        else:
            if self.taskList:
                yield self.env.process(self.DoExtractTask(self.taskList[0]))
            else:
                yield self.env.process(self.goRest())

    def checkAndGoToChargingStation(self):
        closest_station = min(self.chargingStations, key=lambda station: station.location.distance_to(self.location)) #self.chargingstations==> self.model.ChargingStations

        if closest_station.capacity > 0: #station.currentrobot bakacaksın
            # There's a free spot at the closest station
            yield self.env.process(self.moveToChargingStation(closest_station))
        else:
            # Check for a robot at the station with a much higher battery level
            charging_robots = [station.currentRobot for station in self.chargingStations if station.currentRobot]
            if charging_robots:
                highest_battery_robot = max(charging_robots, key=lambda robot: robot.batteryLevel)
                if highest_battery_robot.batteryLevel - self.batteryLevel > 40:
                    # Swap the robots
                    highest_battery_robot.stopCharging()
                    highest_battery_robot.taskList = self.taskList #self.tasklist'i none'a çekeceğiz
                    yield self.env.process(self.moveToChargingStation(highest_battery_robot.station))
                    #highest battery robot, yeni task'a başlayacak
                elif self.batteryLevel < 10 :
                    yield self.env.process(self.goRest())
                elif self.taskList:
                    yield self.env.process(self.DoExtractTask(self.taskList[0]))
                else:
                    yield self.env.process(self.goRest())


    def moveToChargingStation(self, station):
        # Move to the station and start charging
        print("Moving to station at", station.location)
        yield self.env.timeout(5)  # Simulate travel time
        station.currentRobot = self
        yield self.env.timeout(120)  # Simulate charging time
        self.batteryLevel = 100  # Assume full charge for simplicity

    def stopCharging(self):
        # Simulate leaving the charging station
        print("Stopping charge and leaving station.")
        self.station.currentRobot = None
        self.station = None
        yield self.env.timeout(1)  # Time to leave the station

    # Other necessary methods...

# Set up the environment, robots, stations, etc.
env = simpy.Environment()
robot1 = Robot(env)
# Continue setting up charging stations and linking everything together...
