import simpy
import networkx as nx

class Pod(simpy.Resource):
    def __init__(self, env, skuList, location, robot):
        self.env = env
        self.skuList = skuList
        self.location = location #(0,0), (5,5)
        self.robot = robot

    def changeSKUAmount(self, sku, amount):
        for row in self.skuList:
            if row[0] == sku:
                row[1] += amount
                return amount
        return 0


class Robot():
    def __init__(self, env, network, robotID, pod, currentNode, targetNode=None, currentTask=None, taskList=None, loadedSpeed=1, emptySpeed=2):
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

    def completeTask(self):
        pass

    def createPath(self, targetNode):
        self.targetNode = targetNode
        self.path = nx.shortest_path(self.network, source=self.currentNode, target=self.targetNode)

    def move(self):
        #burayı değiştir self.currentNode = next_position callback olarak evente ekle
        for next_position in self.path[1:]:
            yield self.env.timeout(2)  # Time taken to move from one node to another
            print(f"{self.robotID} is moving from {self.currentNode} to {next_position}")
            self.currentNode = next_position

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