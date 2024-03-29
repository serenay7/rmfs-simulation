import simpy
import networkx as nx
import koridor_deneme as layout

class Pod(simpy.Resource):
    def __init__(self, env, skuList, location, robot, status):
        self.env = env
        self.skuList = skuList
        self.location = location #(0,0), (5,5)
        self.robot = robot
        self.status = status

    def changeSKUAmount(self, sku, amount):
        for row in self.skuList:
            if row[0] == sku:
                row[1] += amount
                return amount
        return 0


class Robot():
    def __init__(self, env, network_corridors, network, robotID, pod, currentNode,
                 targetNode=None, currentTask=None, taskList=None, loadedSpeed=1, emptySpeed=2, takeTime=3, dropTime=3):
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
       # self.podsCarriedCount = 0 # Counts how many pods a robot carry.

    def completeTask(self):
        pass

    def createPath(self, targetNode, tempGraph = None):
        self.targetNode = targetNode

        if tempGraph != None:
            self.path = nx.shortest_path(tempGraph, source=self.currentNode, target=self.targetNode)
        elif self.pod == None:
            self.path = nx.shortest_path(self.network, source=self.currentNode, target=self.targetNode)
        else:
            self.path = nx.shortest_path(self.network_corridors, source=self.currentNode, target=self.targetNode)

    def changeCurrentNode(self, node):
        self.currentNode = node
        if self.pod != None: self.pod.location = node

    def move(self):
        #loaded olduğu zaman pod olmayan yerlerden gidecek
        for next_position in self.path[1:]:
            self.stepsTaken += 1  # Increment the steps counter each time the robot moves

            if self.pod != None:
                event = self.env.timeout(self.loadedSpeed)
                event.callbacks.append(lambda event, pos=next_position: self.changeCurrentNode(pos))
                yield event
            else:
                event = self.env.timeout(self.emptySpeed)
                #event.callbacks.append(self.changeCurrentNode(next_position))
                event.callbacks.append(lambda event, pos=next_position: self.changeCurrentNode(pos))
                yield event


    def takePod(self, pod):
        if pod.status == "taken":
            raise Exception("Pod is already taken")
        else:
            pod.status = "taken"
            pod.robot = self.robotID #burası direkt pod.robot = self olabilir
            self.pod = pod
            yield self.env.timeout(self.takeTime)

    def dropPod(self, pod):
        pod.status = "idle"
        pod.robot = None
        self.pod = None
        yield self.env.timeout(self.dropTime)

    def DoExtractTask(self, extractTask):
        self.createPath(extractTask.pod.location)
        yield self.env.process(self.move())
        yield self.env.process(self.takePod(extractTask.pod))
        self.createPath(extractTask.outputstation.location)
        yield self.env.process(self.move())
        #extractTask.outputstation.currentPod = self.pod
        #extractTask.outputstation.PickedItems()

    def DoStorageTask(self, storageTask):
        tempGraph = layout.create_node_added_subgraph(storageTask.storageLocation, self.network_corridors, self.network)
        self.createPath(storageTask.storageLocation, tempGraph=tempGraph)
        del tempGraph
        yield self.env.process(self.move())
        yield self.env.process(self.dropPod(self.pod))


    # def assignPod(self, pod):
    #    if self.pod != pod:  # Check if a new pod is being assigned
    #        self.pod = pod
    #        self.podsCarriedCount += 1  # Increment the pods carried counter

    def getStepsTaken(self): # FOR TOTAL DISTANCE CALCULATION
       return self.stepsTaken

    # def getPodsCarriedCount(self): # TO COUNT HOW MANY PODS A ROBOT CARRY
    #    return self.podsCarriedCount



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
    def __init__(self, env, location, pickItemList, currentPod=None, podQueue=None, timeToPick=1):
        self.env = env
        self.location = location
        self.pickItemList = pickItemList
        self.currentPod = currentPod
        self.podQueue = podQueue
        self.timeToPick = timeToPick

    def PickedItems(self):
        # self.pickItemsCount += 1 # UPH için istatistik tutucu
        # +,- değiştir
        for row in self.pickItemList:
            pickedAmount = self.currentPod.changeSKUAmount(row[0],-row[1])
            row[1] -= 0 #pickedAmount

    # def getPickItemsCount(self): # UPH counter
    #    return self.pickItemsCount


class Task():
    pass


class ExtractTask():
    def __init__(self, env, robot, outputstation, pod):
        self.env = env
        self.robot = robot
        self.outputstation = outputstation
        self.pod = pod

    # def DoExtractTask(self):
    #     self.robot.createPath(self.outputstation.location)
    #     self.robot.move()
    #     self.outputstation.currentPod = self.pod
    #     self.outputstation.PickItems()



class StorageTask(Task):
    def __init__(self, env, robot, pod, storageLocation):
        self.env = env
        self.robot = robot
        self.pod = pod
        self.storageLocation = storageLocation