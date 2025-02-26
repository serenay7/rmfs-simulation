import simpy
import networkx as nx
import Layout as layout
from simpy.events import AllOf
import inspect

class Pod(simpy.Resource):

    def __init__(self, env, location, skuDict=None, robot=None, status=None, takeItemList=None):
        self.env = env
        self.skuDict = skuDict
        self.location = location #(0,0), (5,5) current location
        self.robot = robot
        self.status = status
        self.takeItemList = takeItemList 
        self.fixedLocation = location

        if skuDict is None:
            self.skuDict = {}
        else:
            self.skuDict = skuDict

    def changeSKUAmount(self, sku, amount):
        if self.skuDict.get(sku) is not None:
            self.skuDict[sku] += amount
            return amount
        return 0


class Robot():
    def __init__(self,
                 env,
                 network_corridors,
                 network,
                 robotID,
                 pod=None,
                 currentNode=None,
                 targetNode=None,
                 currentTask=None,
                 taskList=[],
                 loadedSpeed=1,
                 emptySpeed=2,
                 takeTime=3,
                 dropTime=3,
                 batteryLevel = 41.6,
                 moveLoaded = 14.53265,
                 moveEmpty = 11.9566,
                 chargingRate = 41.6,
                 chargeThreshold=35,
                 chargingStationList=[],
                 Model = None,
                 status = "rest",
                 MaxBattery = 41.6,
                 PearlRate = 0.4,
                 RestRate = 0.1,
                 ChargeFlagRate = 0.8,
                 MaxChargeRate = 0.85):

        self.env = env
        self.network = network
        self.network_corridors = network_corridors
        self.robotID = robotID
        self.pod = pod
        self.currentNode = currentNode # (0,0)
        self.targetNode = targetNode
        self.currentTask = currentTask
        self.taskList = taskList
        self.loadedSpeed = loadedSpeed
        self.emptySpeed = emptySpeed
        self.takeTime = takeTime
        self.dropTime = dropTime
        self.stepsTaken = 0  # Initialize the steps counter, for total distance calculation
        self.batteryLevel = batteryLevel
        self.chargeCycle = 0
        self.chargeCount = 0
        self.replaceCount = 0
        self.moveLoaded = moveLoaded
        self.moveEmpty = moveEmpty
        self.chargingRate = chargingRate
        self.chargeThreshold = chargeThreshold
        self.chargingStationList = chargingStationList
        self.Model = Model
        self.status = status # extract, rest, etc.
        self.MaxBattery = MaxBattery
        self.PearlRate = PearlRate
        self.RestRate = RestRate
        self.ChargeFlagRate = ChargeFlagRate
        self.MaxChargeRate = MaxChargeRate
        # self.podsCarriedCount = 0 # Counts how many pods a robot carry.

    def createPath(self, targetNode, tempGraph = None):
        self.targetNode = targetNode

        if tempGraph is not None:
            self.path = nx.shortest_path(tempGraph, source=self.currentNode, target=self.targetNode)
        elif self.pod is None:
            self.path = nx.shortest_path(self.network, source=self.currentNode, target=self.targetNode)
        else:
            self.path = nx.shortest_path(self.network_corridors, source=self.currentNode, target=self.targetNode)

    def changeCurrentNode(self, node):
        self.currentNode = node
        if self.pod is not None:
            self.pod.location = node

    def move(self):
        for next_position in self.path[1:]:
            self.stepsTaken += 1  # Increment the steps counter each time the robot moves

            if self.pod is not None: # robot loaded
                self.batteryLevel -= self.moveLoaded/(3600*self.loadedSpeed)
                event = self.env.timeout(1/self.loadedSpeed)
                event.callbacks.append(lambda event, pos=next_position: self.changeCurrentNode(pos))
                yield event

            else: # robot empty
                self.batteryLevel -= self.moveEmpty/(3600*self.emptySpeed)
                event = self.env.timeout(1/self.emptySpeed)
                event.callbacks.append(lambda event, pos=next_position: self.changeCurrentNode(pos))
                yield event


    def takePod(self, pod):
        if pod.status == "extractTaken" or pod.status == "storageTaken":
            #self.taskList.append(self.currentTask)
            #yield self.env.process(self.DoExtractTask(self.taskList[0]))
            #raise Exception("Pod is already taken")
            yield self.env.timeout(10)
            yield self.env.process(self.takePod(pod=pod))

        else:
            yield self.env.timeout(self.takeTime)
            pod.status = "extractTaken"
            pod.robot = self.robotID 
            self.pod = pod

    def dropPod(self, pod):
        yield self.env.timeout(self.dropTime)
        if not pod:
            pass
        pod.status = "idle"
        pod.robot = None
        self.pod = None
        self.currentTask = None

        if self.Model.ChargePolicy == "rawsimo":
            if self.batteryLevel < self.MaxBattery * self.ChargeFlagRate:
                yield self.env.process(self.selectChargingStationRawSIMO())
            elif self.taskList:
                yield self.env.process(self.DoExtractTask(self.taskList[0]))
            else:
                yield self.env.process(self.goRest())
        elif self.Model.ChargePolicy == "pearl":
            if self.batteryLevel <= self.MaxBattery * self.ChargeFlagRate:
                yield self.env.process(self.checkAndGoToChargingStation())
            else: #burası neden elif değil?
                if self.taskList:
                    yield self.env.process(self.DoExtractTask(self.taskList[0]))
                else:
                    yield self.env.process(self.goRest())

    def DoExtractTask(self, extractTask):
        if self.batteryLevel <= self.MaxBattery * self.RestRate and self.pod is None:
            if self.Model.ChargePolicy == "rawsimo":
                yield self.env.process(self.selectChargingStationRawSIMO())
            elif self.Model.ChargePolicy == "pearl":
                yield self.env.process(self.checkAndGoToChargingStation())
            return

        if self.currentTask != extractTask:
            self.taskList.pop(0)
        PodFixedLocation = extractTask.pod.fixedLocation
        self.currentTask = extractTask

        if self.pod is None:
            self.status = "extract"
            self.createPath(extractTask.pod.location)
            yield self.env.process(self.move())
            yield self.env.process(self.takePod(extractTask.pod))

        if self.pod is not None:
            if self.pod.status == "extractTaken" or self.targetNode == extractTask.outputstation.location:
                tempGraph = layout.create_node_added_subgraph(self.currentNode, self.network_corridors, self.network)
                self.createPath(extractTask.outputstation.location, tempGraph=tempGraph)
                del tempGraph
                yield self.env.process(self.move())
                self.pod.status = "storageTaken"

            if self.pod.status == "storageTaken": #or self.currentNode != PodFixedLocation:
                tempGraph = layout.create_node_added_subgraph(PodFixedLocation, self.network_corridors, self.network)
                self.createPath(PodFixedLocation, tempGraph=tempGraph)
                del tempGraph
                yield self.env.process(self.move())
            yield self.env.process(self.dropPod(self.pod))

    def selectChargingStationRawSIMO(self):
        def manhattan_distance(point1, point2):
            return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

        distList = []
        for station in self.chargingStationList:
            if station.currentRobot is None:
                tempList = [station, manhattan_distance(station.location, self.currentNode)]
                distList.append(tempList)

        if len(distList) == 0 and self.batteryLevel <= self.MaxBattery * self.RestRate:
            self.Model.insertChargeQueue(robot=self)
            yield self.env.process(self.goRest())

        elif len(distList) == 0:
            if self.taskList:
                yield self.env.process(self.DoExtractTask(self.taskList[0]))
            else:
                yield self.env.process(self.goRest())

        else:
            min_distance = min(distList, key=lambda x: x[1])[1]
            found_closest = False
            for item in distList:
                if item[1] == min_distance and not found_closest:
                    item[0].currentRobot = self
                    self.status = "charging"
                    yield self.env.process(self.moveToChargingStation(item[0]))
                    found_closest = True

    def getStepsTaken(self): # FOR TOTAL DISTANCE CALCULATION
        return self.stepsTaken

    def checkAndGoToChargingStation(self):
        #pearlMain
        def manhattan_distance(point1, point2):
            return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

        distList = []
        for station in self.chargingStationList:
            if station.currentRobot is None:
                tempList = [station, manhattan_distance(station.location, self.currentNode)]
                distList.append(tempList)

        # go to the nearest one among the empty ones, this part of the rawsimo code can be taken
        if distList:
            min_distance = min(distList, key=lambda x: x[1])[1]
            found_closest = False # added in case there are 2 stations at the same distance
            for item in distList:
                if item[1] == min_distance and not found_closest:
                    item[0].currentRobot = self
                    self.status = "charging"
                    yield self.env.process(self.Model.pearlVRP(simultaneousEvent=self.moveToChargingStation(item[0])))
                    yield self.env.process(self.moveToChargingStation(item[0]))
                    found_closest = True

        else:
            # Check for a robot at the station with a much higher battery level
            charging_robots = [station.currentRobot for station in self.chargingStationList if station.currentRobot]
            if charging_robots:
                highest_battery_robot = max(charging_robots, key=lambda robot: robot.batteryLevel)
                if highest_battery_robot.batteryLevel - self.batteryLevel > self.MaxBattery * self.PearlRate:
                    # Swap the robots
                    # highest_battery_robot.stopCharging()
                    highest_battery_robot.taskList = self.taskList
                    self.taskList = []

                    stationNode = highest_battery_robot.currentNode
                    for station in self.chargingStationList:
                        if station.location == stationNode:
                            currentStation = station

                    all_events = [self.env.process(self.moveToChargingStation(currentStation))]
                    self.status = "charging"
                    self.replaceCount += 1
                    if highest_battery_robot.taskList:
                        all_events.append(self.env.process(highest_battery_robot.DoExtractTask(extractTask=highest_battery_robot.taskList[0])))
                    else:
                        all_events.append(highest_battery_robot.goRest())
                    if inspect.isgenerator(self.env):
                        a = 10
                    yield AllOf(self.env, all_events)
                elif self.batteryLevel < self.MaxBattery * self.RestRate:
                    self.Model.insertChargeQueue(robot=self)
                    yield self.env.process(self.goRest())
                elif self.taskList:
                    yield self.env.process(self.DoExtractTask(self.taskList[0]))
                else:
                    yield self.env.process(self.goRest())

    def moveToChargingStation(self, chargingStation):
        self.chargeCount += 1
        chargingStation.currentRobot = self
        self.createPath(chargingStation.location)
        yield self.env.process(self.move())

    def goRest(self):
        self.status = "rest"
        yield self.env.timeout(0)

class SKU():
    def __init__(self, env, id, totalAmount):
        self.env = env
        self.id = id
        self.totalAmount = totalAmount

class InputStation(simpy.Resource):
    def __init__(self, env, location, itemList, podQueue, timeToReplenish):
        self.env = env
        self.location = location
        self.itemList = itemList
        self.podQueue = podQueue
        self.timeToReplenish = timeToReplenish

class OutputStation(simpy.Resource):
    def __init__(self, env, location, outputStationID, pickItemList=None, currentPod=None, podQueue=None, timeToPick=1):
        self.env = env
        self.location = location
        self.outputStationID = outputStationID
        self.pickItemList = pickItemList
        self.currentPod = currentPod
        self.podQueue = podQueue
        self.timeToPick = timeToPick
        self.totalPickedCount = 0

    def PickedItems(self):
        # self.pickItemsCount += 1 # UPH - stat keeper
        for row in self.pickItemList:
            pickedAmount = self.currentPod.changeSKUAmount(row[0],-row[1])
            row[1] -= pickedAmount

    # def getPickItemsCount(self): # UPH counter
    #    return self.pickItemsCount

class ChargingStation(simpy.Resource):
    def __init__(self, env, capacity, location, currentRobot=None):
        super().__init__(env=env, capacity=capacity)

        self.env = env
        self.capacity = capacity
        self.location = location
        self.currentRobot = currentRobot

    @property
    def capacity(self):
        return self._capacity

    @capacity.setter
    def capacity(self, value):
        self._capacity = value

class Task():
    pass

class ExtractTask():
    def __init__(self, env, robot=None, outputstation=None, pod=None):
        self.env = env
        self.robot = robot
        self.outputstation = outputstation
        self.pod = pod

class StorageTask(Task):
    def __init__(self, env, robot, pod, storageLocation):
        self.env = env
        self.robot = robot
        self.pod = pod
        self.storageLocation = storageLocation

if __name__ == "__main__":
    env = simpy.Environment()
    rows = 10  # 3x3
    columns = 16

    rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
    layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))

    robot1 = Robot(env, layout.create_corridor_subgraph(rectangular_network), rectangular_network,0)
    pod1 = Pod(env, (1,1))
    station1 = OutputStation(env,(0,9))
    sampleTask = ExtractTask(env,robot1,station1,pod1)
    env.process(robot1.DoExtractTask(sampleTask))
    env.run()
