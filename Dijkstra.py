import sys
from math import isinf

class Node:
    
    def __init__(self, name):
        self.name = name
        self.links = {}
        self.distanceValue = None
        self.previousNode = None

    def isNeighbor(self, node):
        if self.links[node.name] is not 9999:
            return True
        else:
            return False

class Network:

    def __init__(self):
        self.nodes = []

    def addNode(self, name):
        node = Node(name)
        self.nodes.append(node)

    def computeShortestPath(self, sourceNode):
        # Dijkstra's Algorithm
        # Examine links of the source node
        
        sourceNode.distanceValue = 0
        pathNodes = [sourceNode]

        for node in self.nodes:
            # if node isn't source and is adjacent to source
            if node.name is not sourceNode.name and node.name in sourceNode.links:
                node.distanceValue = sourceNode.links[node.name]
                node.previousNode = sourceNode
            elif node.name is not sourceNode.name:
                node.distanceValue = float("inf")

        while len(pathNodes) is not len(self.nodes):
            # find the smallest distance value of the nodes not in the path
            minCost = float("inf")
            neighbors = {}
            for node in self.nodes:
                if node not in pathNodes:
                    neighbors[node.distanceValue] = node

            minCost = min(neighbors)
            nextNode = neighbors[minCost]
            


            pathNodes.append(nextNode)

            for node in self.nodes:
                if node not in pathNodes:
                    oldValue = node.distanceValue
                    newValue = minCost + nextNode.links[node.name]
                    node.distanceValue = min(oldValue, newValue)
                    if node.distanceValue is newValue:
                        node.previousNode = nextNode

            # Generate shortest paths
            shortestPaths = []
            pathCost = ""
            for node in self.nodes:
                if node is not sourceNode:
                    path = [node.name]
                    previousNode = node.previousNode
                    while True:
                        path.append(previousNode.name)
                        if previousNode is sourceNode:
                            break
                        else:
                            previousNode = previousNode.previousNode
                    path.reverse()
                    path = ''.join(path)
                    shortestPaths.append(path)
                
                pathCost += "{}:{}, ".format(node.name, node.distanceValue)

        print
        print "Shortest path tree for node {}:".format(sourceNode.name)
        print ", ".join(shortestPaths)
        print "Costs of least-cost paths for node {}".format(sourceNode.name)
        print pathCost
        print
            
# Main
network = Network()

fileLines = []
# Accept csv file as cmnd line argument
with open(sys.argv[1], 'r') as inputFile:
    for line in inputFile:
        fileLines.append(line)
inputFile.close()

# Read csv lines
nodes = fileLines.pop(0)
nodes = nodes.rstrip()
nodes = nodes.split(",")
nodes.pop(0)

# Add nodes to network
for node in nodes:
    network.addNode(node)

# Add links to nodes
for line in fileLines:
    line = line.rstrip()
    line = line.split(",")
    sourceNode = line.pop(0)
    for node in network.nodes:
        if node.name is sourceNode:
            sourceNode = node
            break
    # Add links
    for node, link in zip(nodes, line):
        link = int(link)
        sourceNode.links[node] = link

validInput = False
while True:
    if validInput:
        break

    # Get node from user
    userInput = raw_input("Please enter the source node: ")
    
    # Error check input
    for node in nodes:
        if userInput is node:
            validInput = True
            break

# Run Dijkstra's Algorithm on the network from the input node
for node in network.nodes:
    if userInput is node.name:
        network.computeShortestPath(node)
        break