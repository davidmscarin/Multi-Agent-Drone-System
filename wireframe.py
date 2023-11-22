class Node:
    def __init__(self, info):
        self.x = 50*info[0]
        self.y = 50*info[1]
        self.z = 50*info[2]
        self.real_x = info[0]
        self.real_y = info[1]
        self.real_z = info[2]
        self.type = info[3]

class Wireframe:
    def __init__(self):
        self.nodes = []

    def addNodes(self, nodeList):
        for node in nodeList:
            self.nodes.append(Node(node))
    
    def remNodes(self):
        self.nodes = []

    def outputNodes(self):
        for i, node in enumerate(self.nodes):
            print (" %d: (%.2f, %.2f, %.2f)" % (i, node.x, node.y, node.z))