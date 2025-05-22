import simpy
import networkx as nx
import Layout as layout
from simpy.events import AllOf
import inspect
import config
import logging

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
                 loadedSpeed=config.DEFAULT_LOADED_SPEED,
                 emptySpeed=config.DEFAULT_EMPTY_SPEED,
                 takeTime=config.DEFAULT_TAKE_TIME,
                 dropTime=config.DEFAULT_DROP_TIME,
                 batteryLevel=config.DEFAULT_MAX_BATTERY,  # Default to full battery
                 moveLoaded=config.DEFAULT_MOVE_LOADED_CONSUMPTION,
                 moveEmpty=config.DEFAULT_MOVE_EMPTY_CONSUMPTION,
                 chargingRate=config.DEFAULT_CHARGING_RATE,
                 chargeThreshold=config.DEFAULT_CHARGE_THRESHOLD, 
                 chargingStationList=[],
                 Model = None,
                 status = "rest", # Initial status, "rest" is a state string
                 MaxBattery=config.DEFAULT_MAX_BATTERY,
                 PearlRate=config.DEFAULT_PEARL_RATE,
                 RestRate=config.DEFAULT_REST_RATE,
                 ChargeFlagRate=config.DEFAULT_CHARGE_FLAG_RATE,
                 MaxChargeRate=config.DEFAULT_MAX_CHARGE_RATE):

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
            else: # This 'else' is appropriate. It covers the case where the battery level is above ChargeFlagRate.
                  # In this situation, the robot doesn't need to charge. It then decides its next action based on task availability.
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

    def _manhattan_distance(self, point1, point2):
        """Helper function to calculate Manhattan distance."""
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

    def _find_best_empty_charging_station(self):
        """
        Finds the nearest empty charging station.
        Returns the station object or None if no empty stations are found.
        """
        distList = []
        for station in self.chargingStationList:
            if station.currentRobot is None:
                distance = self._manhattan_distance(station.location, self.currentNode)
                distList.append({'station': station, 'distance': distance})
        
        if not distList:
            return None

        # Find the minimum distance
        min_distance = min(item['distance'] for item in distList)
        
        # Get all stations with the minimum distance
        closest_stations = [item['station'] for item in distList if item['distance'] == min_distance]
        
        # If multiple stations are at the same minimum distance, this picks the first one encountered.
        # The original code had 'found_closest' which effectively did the same.
        return closest_stations[0] if closest_stations else None

    def _attempt_pearl_swap(self):
        """
        Attempts to perform a PEARL swap with a robot currently charging if conditions are met.
        Initiates the swap process and yields events if a swap occurs.
        Returns True if a swap was initiated, False otherwise.
        """
        charging_robots = [station.currentRobot for station in self.chargingStationList if station.currentRobot]
        if not charging_robots:
            return False # No robots to swap with

        highest_battery_robot = max(charging_robots, key=lambda robot: robot.batteryLevel)

        if highest_battery_robot.batteryLevel - self.batteryLevel > self.MaxBattery * self.PearlRate:
            # Conditions for swap are met
            # highest_battery_robot.stopCharging() # This comment was in original, implies it's not strictly needed or handled otherwise
            
            # Transfer task list
            highest_battery_robot.taskList = self.taskList
            self.taskList = []

            # Identify the station of the highest_battery_robot
            currentStation = None
            for station in self.chargingStationList:
                if station.location == highest_battery_robot.currentNode: # Assuming robot is at its station
                    currentStation = station
                    break
            
            if not currentStation:
                # This case should ideally not happen if highest_battery_robot is correctly identified as charging.
                # If it does, we cannot proceed with this swap.
                # Restore task list to self if something went wrong before yielding.
                self.taskList = highest_battery_robot.taskList # Give tasks back
                highest_battery_robot.taskList = []
                return False

            all_events = [self.env.process(self.moveToChargingStation(currentStation))]
            self.status = "charging" # This robot (self) will now go to charge
            self.replaceCount += 1
            
            # The highest_battery_robot leaves the station and either does tasks or rests
            highest_battery_robot.status = "extract" # Or an appropriate status after being swapped out
            if highest_battery_robot.taskList:
                all_events.append(self.env.process(highest_battery_robot.DoExtractTask(extractTask=highest_battery_robot.taskList[0])))
            else:
                all_events.append(self.env.process(highest_battery_robot.goRest()))
            
            yield AllOf(self.env, all_events)
            return True # Swap initiated
        return False # Conditions for swap not met

    def checkAndGoToChargingStation(self):
        # This method implements the PEARL charging logic.
        # It first tries to find an empty station. If none, it attempts a PEARL swap.
        # If neither is possible, it decides based on its battery level and task list.

        best_empty_station = self._find_best_empty_charging_station()

        if best_empty_station:
            best_empty_station.currentRobot = self # Assign robot to station
            self.status = "charging"
            # FIXME: Investigate if moveToChargingStation needs to be part of simultaneousEvent AND called separately.
            # The pearlVRP method in RMFS_Model does not explicitly yield the simultaneousEvent.
            # Assuming current dual call structure is intentional for now, where pearlVRP might use the event
            # for planning, and the second call executes the move.
            yield self.env.process(self.Model.pearlVRP(simultaneousEvent=self.moveToChargingStation(best_empty_station)))
            yield self.env.process(self.moveToChargingStation(best_empty_station))
        else:
            # No empty stations, attempt a PEARL swap
            swapped = yield self.env.process(self._attempt_pearl_swap())
            
            if not swapped:
                # No swap occurred, fallback logic
                if self.batteryLevel < self.MaxBattery * self.RestRate:
                    self.Model.insertChargeQueue(robot=self)
                    yield self.env.process(self.goRest())
                elif self.taskList:
                    yield self.env.process(self.DoExtractTask(self.taskList[0]))
                else:
                    yield self.env.process(self.goRest())
            # If swapped is true, the _attempt_pearl_swap method has already handled yielding processes.

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
    def __init__(self, env, location, outputStationID, pickItemList=None, currentPod=None, podQueue=None, timeToPick=config.DEFAULT_OUTPUT_STATION_PICK_TIME):
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
        if self.pickItemList and self.currentPod: # Ensure lists and pod exist
            for row in self.pickItemList:
                # Ensure currentPod is not None before calling its method
                pickedAmount = self.currentPod.changeSKUAmount(row[0],-row[1])
                row[1] -= pickedAmount

    # def getPickItemsCount(self): # UPH counter
    #    return self.pickItemsCount

class ChargingStation(simpy.Resource):
    def __init__(self, env, capacity=config.DEFAULT_CHARGING_STATION_CAPACITY, location=None, currentRobot=None): 
        super().__init__(env=env, capacity=capacity)

        self.env = env
        # self.capacity is handled by simpy.Resource.capacity if not overridden by @property
        # If there's a @property for capacity, it should be used or this line is fine.
        # Based on the provided snippet, there is a @property, so this line is okay.
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
