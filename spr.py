# Liam Dungan
# Nov. 25, 2017 - Rutgers CS 460/560
import sys
import math
import visualize
import matplotlib.pyplot as plt


# Takes in 3 vertices and reports the direction of turn toward next vertex
# Returns positive if clockwise, negative if counterclockwise, 0 if linear
def turn(v1, v2, v3):
    return ((v2[1] - v1[1]) * (v3[0] - v1[0])) - ((v2[0] - v1[0]) * (v3[1] - v1[1]))


# Report reflexive vertices
# Returns a list of (x,y) values as lists, i.e.
# e.g. vertices = [[x1,y1],[x2,y2],...]
def findReflexiveVertices(polygons):
    vertices = []

    # For all obstacles, identify all reflexive vertices using two lists for upper and lower
    # sides of the shape, then merge the lists
    for n in range(0, len(polygons)):
        Upper = []
        Lower = []
        obstacle = polygons[n]
        obstacle.sort()
        for p in obstacle:
            while len(Upper) > 1 and turn(Upper[-2], Upper[-1], p) <= 0:
                Upper.pop()
            Upper.append(p)
            while len(Lower) > 1 and turn(Lower[-2], Lower[-1], p) >= 0:
                Lower.pop()
            Lower.append(p)

        # For obstacles with at least 2 vertices (i.e. forming a line or polygon), remove last
        # element from upper vertices and first element from lower vertices (duplicates)
        if len(obstacle) >= 2:
            Upper.pop()
            Lower.pop(0)
        else:
            Upper.pop()

        vertices += (Upper + Lower)

    return vertices


# visiblility - true if vertices a and b are visible to each other. check against every edge
def visible(a, b):
    for p in range(0, len(polygons)):
        ob = polygons[p]
        for o in range(1, len(ob)):
            if (turn(a, ob[o - 1], ob[o]) * turn(b, ob[o - 1], ob[o])) < 0 and (turn(a, b, ob[o - 1]) * turn(a, b, ob[o])) < 0:
                return False
        if (turn(a, ob[len(ob) - 1], ob[0]) * turn(b, ob[len(ob) - 1], ob[0])) < 0 and (turn(a, b, ob[len(ob) - 1]) * turn(a, b, ob[0])) < 0:
            return False
    return True


# finds the length between two vertices
def distance(a, b):
    return round(math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2), 4)


# Compute the roadmap graph
# vertexMap should look like
# {1: [5.2,6.7], 2: [9.2,2.3], ... }
# adjacencyListMap should look like
# {1: [[2, 5.95], [3, 4.72]], 2: [[1, 5.95], [5,3.52]], ... }
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = dict()

    # create a dictionary for the vertex map
    for i in range(0, len(reflexVertices)):
        vertexMap[i + 1] = reflexVertices[i]

    # create a dictionary for the adjacency list map (graph)
    for n in range(1, len(vertexMap) + 1):
        adjacent = []
        for m in range(1, len(vertexMap) + 1):
            if n == m:
                continue
            if visible(vertexMap[n], vertexMap[m]):
                adjacent.append([m, distance(vertexMap[n], vertexMap[m])])
        adjacencyListMap[n] = adjacent

    return vertexMap, adjacencyListMap


# Perform uniform cost search
# path = [0, 15, 9, ..., -1]
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0

    distance_from_start = {start: 0}
    previous = {start: start}
    queue = [start]
    while queue:
        temp = min(distance_from_start, key=distance_from_start.get)
        nextp = queue.pop(temp)
        if nextp == goal:
            x = -1
            path.append(x)
            while x != 0:
                path.insert(0, previous[x])
                x = previous[x]
            return path, pathLength

        N = adjListMap[nextp]
        for n in N:
            new_distance = distance_from_start[nextp] + n[1]
            if (n[0] not in previous) or (new_distance < distance_from_start[n[0]]):
                queue.append(n[0])
            if ((n[0] not in distance_from_start)):
                distance_from_start[n[0]] = new_distance
                previous[n[0]] = nextp
                pathLength = new_distance

    return path, pathLength


# Agument roadmap to include start and goal
# let start and goal have vertex labels 0 and -1,
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    SG = [[x2, y2], [x1, y1]]
    updatedALMap = adjListMap.copy()
    for i in range(-1, 1):
        adjacentSG = []
        for m in range(1, len(vertexMap) + 1):
            if visible(SG[i + 1], vertexMap[m]):
                adjacentSG.append([m, distance(SG[i + 1], vertexMap[m])])
                updatedALMap[m].append([i, distance(SG[i + 1], vertexMap[m])])
        updatedALMap[i] = adjacentSG

    return startLabel, goalLabel, updatedALMap


if __name__ == "__main__":

    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append(map(float, xys[p].split(',')))
        polygons.append(polygon)

    # Print out the data
    print "Pologonal obstacles:"
    for p in range(0, len(polygons)):
        print str(polygons[p])
    print ""

    temp = list(polygons)
    fig, ax = visualize.setupPlot()
    for p in range(0, len(polygons)):
        patch = visualize.createPolygonPatch(polygons[p])
        ax.add_patch(patch)

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print "Reflexive vertices:"
    print str(reflexVertices)
    print ""

    # Compute the roadmap
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print "Vertex map:"
    print str(vertexMap)
    print ""
    print "Base roadmap:"
    print str(adjListMap)
    print ""

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print "Updated roadmap:"
    print str(updatedALMap)
    print ""

    # Search for a solution
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print "Final path:"
    print str(path)
    # print "Final path length:" + str(length)

    # Extra visualization elements goes here

    xp = []
    yp = []
    xp.append(x1 / 10.)
    yp.append(y1 / 10.)
    for j in range(1, len(path) - 1):
        xp.append(vertexMap[path[j]][0] / 10.)
        yp.append(vertexMap[path[j]][1] / 10.)
    xp.append(x2 / 10.)
    yp.append(y2 / 10.)
    plt.plot(xp, yp, '-o', color='orange')
    plt.show()

