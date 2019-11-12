import sys

class Network:

    def __init__(self):
        self.nodes = []

    def addNode(self, name):
        node = Node(name)
        self.nodes.append(node)

    def computeShortestPath(self, node):
        # Dijkstra's Algorithm
        # Examine links of the source node
        pass
    
class Node:

    def __init__(self, name):
        self.name = name
        self.links = {}

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
print nodes

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
        if link is "9999":
            link = float("inf")
        else:
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