import simpy
import networkx as nx

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
    def __init__(self, env, network, robotID, pod, currentNode, targetNode=None, currentTask=None, taskList=None, loadedSpeed=1, emptySpeed=2, takeTime=3):
        self.env = env
        self.network = network
        self.robotID = robotID
        self.pod = pod
        self.currentNode = currentNode # (0,0)
        self.targetNode = targetNode
        self.currentTask = currentTask
        self.taskList = taskList
        self.loadedSpeed = loadedSpeed
        self.emptySpeed = emptySpeed
        self.takeTime = takeTime

    def completeTask(self):
        pass

    def createPath(self, targetNode):
        self.targetNode = targetNode
        self.path = nx.shortest_path(self.network, source=self.currentNode, target=self.targetNode)

    def changeCurrentNode(self, node):
        self.currentNode = node
        if self.pod != None: self.pod.location = node

    def move(self):
        #loaded olduğu zaman pod olmayan yerlerden gidecek
        for next_position in self.path[1:]:
            print(f"{self.robotID} is moving from {self.currentNode} to {next_position}")
            if self.pod != None:
                event = self.env.timeout(self.loadedSpeed)
                event.callbacks.append(self.changeCurrentNode(next_position))
                yield event
            else:
                event = self.env.timeout(self.emptySpeed)
                #event.callbacks.append(self.changeCurrentNode(next_position))
                event.callbacks.append(lambda event, pos=next_position: self.changeCurrentNode(pos))
                yield event


    def takePod(self, pod):
        pod.status = "taken"
        pod.robot = self.robotID
        yield self.env.timeout(self.takeTime)

    def DoExtractTask(self,extractTask):
        self.createPath(extractTask.outputstation.location)
        yield self.env.process(self.move())
        extractTask.outputstation.currentPod = self.pod
        extractTask.outputstation.PickItems()



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

    def PickItems(self):
        for row in self.pickItemList:
            pickedAmount = self.currentPod.changeSKUAmount(row[0],-row[1])
            row[1] -= pickedAmount


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
    def __init__(self, robot, pod, storageLocation):
        self.robot = robot
        self.pod = pod
        self.storageLocation = storageLocation