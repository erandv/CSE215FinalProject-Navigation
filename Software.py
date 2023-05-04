
# In[2]:

from PyQt5.Qt import *
import math, copy
from main import Ui_MainWindow as MainWindow
import cv2
import haversine as hs

# python -m nuitka --standalone --follow-imports --windows-disable-console --enable-plugin=qt-plugins --enable-plugin=numpy --python-flag=no_site --mingw64 Software.py


class NavigationPath:
    def __init__(self, instructions, miles, kilometers, time):
        self.instructions = instructions
        self.miles = miles
        self.kilometers = kilometers
        self.time = time

    def GetInstructions(self):
        return self.instructions

    def GetPathLength(self):
        return (self.miles, self.kilometers, self.time)


class Node:
    def __init__(self, coords):
        # Coords is the primary distinguishing characteristic of a node, no two nodes should have the same lat/long coords.
        # F, G and H are used for the A* search algorithm used in Navigate(n).  G is the weight of all edges taken
        # from the start to the current node, and H is the heuristic (distance between node's coords and destination's coords).
        # Parent is used to trace back the exact path taken from start to goal.
        # Origin is used to find the node in the given graph, as all Nodes created in Navigate(n) are copies of nodes from the graph to ensure the graph is not effected.
        # However, these copied nodes are obviously not in the graph, so Origin is needed to find the edges of a given node.
        self.coords = coords
        self.f = 0
        self.g = 0
        self.h = 0
        self.parent = []
        self.origin = None

    def UpdateStats(self, g, h):
        self.g = g
        self.h = h
        self.f = g + h

    def GetStats(self):
        return self.f, self.g, self.h

    # EqualCoords checks if the two nodes have identical coords and if so returns true.
    def EqualCoords(self, other):
        if self.coords == other.coords:
            return True
        return False

    # EqualCoordsEqualParent checks if the two nodes have identical coords and the same parent node and if so returns true.
    def EqualCoordsEqualPath(self, other):
        if self.coords == other.coords:
            if len(self.parent) != len(other.parent):
                return False
            for i in range(0, len(self.parent)):
                if self.parent[i].origin != other.parent[i].origin:
                    return False
            return True
        return False

    def UpdateParent(self, parent):
        self.parent = parent

    def GetParent(self):
        return self.parent

    def UpdateOrigin(self, origin):
        self.origin = origin

    def GetOrigin(self):
        return self.origin

    # All nodes in a list are sorted using their F value. (there is not a __eq__ because the dictionary requires that to be left blank for a given node to be a viable key)
    def __lt__(self, other):
        if self.f < other.f:
            return True
        return False

    def __gt__(self, other):
        if self.f > other.f:
            return True
        return False

    def __repr__(self):
        return "%s" % (self.coords)


class Edge:
    # Note: weight might have to be changed to miles and mph down the line once BuildGraph is completed, so that NavigationPath's Miles, Kilometers, and Time variables actually mean something
    def __init__(self, EndNode, weight):
        self.EndNode = EndNode
        self.weight = weight

    def GetEdge(self):
        return self.EndNode, self.weight


class Graph:
    def __init__(self):
        self.graph = {}

    def AddNode(self, Node):
        for i in self.graph:
            if Node.EqualCoords(i):
                return False
        self.graph[Node] = []
        return True

    def AddEdge(self, Node, Edge):
        for i in self.graph[Node]:
            if i.EndNode == Edge.EndNode:
                return False
        NodeValue = self.graph.get(Node)
        NodeValue.append(Edge)
        self.graph[Node] = NodeValue
        return True

    def RemoveEdge(self, Node, Edge):
        self.graph[Node].remove(Edge)

    # Removes the node from the graph and all edges that point to said node.

    def RemoveNode(self, Node):
        del self.graph[Node]
        for i in self.graph:
            for j in self.graph[i]:
                if j.EndNode == Node:
                    self.graph[i].remove(j)

    def getNode(self, location):
        for i in self.graph:
            if i.coords == location.coords:
                return i
        return None

    def ResetGraph(self):
        self.graph = {}


class NavigationGraph:

    def __init__(self, CurrentLocation, Destination):
        self.currentlocation = CurrentLocation
        self.destination = Destination
        self.paths = []
        self.graph = Graph()
        # build graph
        self.BuildGraph()

    def BuildGraph(self):
        # Needs Google Maps API to work
        Brazil = Node([-47.91, -15.76])
        Bolivia = Node([-68.09, -16.46])
        Paraguay = Node([-57.57, -25.29])
        Uruguay = Node([-56.19, -34.87])
        Argentina = Node([-58.41, -34.58])
        Chile = Node([-70.68, -33.44])
        Peru = Node([-77.05, -12.05])
        Ecuador = Node([-78.55, -0.19])
        Colombia = Node([-74.13, 4.72])
        Venezuela = Node([-66.90, 10.51])
        Guyana = Node([-58.19, 6.86])
        Suriname = Node([-55.17, 5.84])
        FrenchGuiana = Node([-52.33, 4.95])

        Nodes = [Brazil, Bolivia, Paraguay, Uruguay, Argentina, Chile, Peru,
                 Ecuador, Colombia, Venezuela, Guyana, Suriname, FrenchGuiana]

        for x in Nodes:
            self.graph.AddNode(x)

        EdgeBrazilFrenchGuiana = Edge(FrenchGuiana, hs.haversine((-68.74, -11.00), (-52.33, 4.95)))
        EdgeFrenchGuianaBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-52.33, 4.95)))

        EdgeSurinameBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-55.17, 5.84)))
        EdgeBrazilSuriname = Edge(Suriname, hs.haversine((-68.74, -11.00), (-55.17, 5.84)))

        EdgeGuyanaBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-58.19, 6.81)))
        EdgeBrazilGuyana = Edge(Guyana, hs.haversine((-68.74, -11.00), (-58.19, 6.81)))

        EdgeBrazilVenezuela = Edge(Venezuela, hs.haversine((-68.74, -11.00), (-66.90, 10.51)))
        EdgeVenezuelaBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-66.90, 10.51)))

        EdgeColombiaBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-74.13, 4.72)))
        EdgeBrazilColombia = Edge(Colombia, hs.haversine((-68.74, -11.00), (-74.13, 4.72)))

        EdgeBrazilPeru = Edge(Peru, hs.haversine((-68.74, -11.00), (-77.03, -12.06)))
        EdgePeruBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-77.03, -12.06)))

        EdgeBrazilBolivia = Edge(Bolivia, hs.haversine((-68.74, -11.00), (-68.09, -16.46)))
        EdgeBoliviaBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-68.09, -16.46)))

        EdgeBrazilParaguay = Edge(Paraguay, hs.haversine((-68.74, -11.00), (-57.63, -25.28)))
        EdgeParaguayBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-57.63, -25.28)))

        EdgeBrazilArgentina = Edge(Argentina, hs.haversine((-68.74, -11.00), (-58.44, -34.61)))
        EdgeArgentinaBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-58.44, -34.61)))

        EdgeBrazilUruguay = Edge(Uruguay, hs.haversine((-68.74, -11.00), (-56.19, -34.87)))
        EdgeUruguayBrazil = Edge(Brazil, hs.haversine((-68.74, -11.00), (-56.19, -34.87)))

        EdgeFrenchGuianaSuriname = Edge(Suriname, hs.haversine((-52.33, 4.95), (-55.17, 5.84)))
        EdgeSurinameFrenchGuiana = Edge(FrenchGuiana, hs.haversine((-52.33, 4.95), (-55.17, 5.84)))

        EdgeSurinameGuyana = Edge(Guyana, hs.haversine((-55.17, 5.84), (-58.19, 6.81)))
        EdgeGuyanaSuriname = Edge(Suriname, hs.haversine((-55.17, 5.84), (-58.19, 6.81)))

        EdgeGuyanaVenezuela = Edge(Venezuela, hs.haversine((-58.19, 6.81), (-66.90, 10.51)))
        EdgeVenezuelaGuyana = Edge(Guyana, hs.haversine((-58.19, 6.81), (-66.90, 10.51)))

        EdgeVenezuelaColombia = Edge(Colombia, hs.haversine((-66.90, 10.51), (-74.13, 4.72)))
        EdgeColombiaVenezuela = Edge(Venezuela, hs.haversine((-66.90, 10.51), (-74.13, 4.72)))

        EdgeColombiaPeru = Edge(Peru, hs.haversine((-74.13, 4.72), (-77.03, -12.06)))
        EdgePeruColombia = Edge(Colombia, hs.haversine((-74.13, 4.72), (-77.03, -12.06)))

        EdgeColombiaEcuador = Edge(Ecuador, hs.haversine((-74.13, 4.72), (-78.51, -0.22)))
        EdgeEcuadorColombia = Edge(Colombia, hs.haversine((-74.13, 4.72), (-78.51, -0.22)))

        EdgeEcuadorPeru = Edge(Peru, hs.haversine((-78.51, -0.22), (-77.03, -12.06)))
        EdgePeruEcuador = Edge(Ecuador, hs.haversine((-78.51, -0.22), (-77.03, -12.06)))

        EdgePeruBolivia = Edge(Bolivia, hs.haversine((-68.09, -16.46), (-77.03, -12.06)))
        EdgeBoliviaPeru = Edge(Peru, hs.haversine((-68.09, -16.46), (-77.03, -12.06)))

        EdgeBoliviaArgentina = Edge(Argentina, hs.haversine((-68.09, -16.46), (-58.44, -34.61)))
        EdgeArgentinaBolivia = Edge(Bolivia, hs.haversine((-68.09, -16.46), (-58.44, -34.61)))

        EdgeBoliviaParaguay = Edge(Paraguay, hs.haversine((-68.09, -16.46), (-57.63, -25.28)))
        EdgeParaguayBolivia = Edge(Bolivia, hs.haversine((-68.09, -16.46), (-57.63, -25.28)))

        EdgeParaguayArgentina = Edge(Argentina, hs.haversine((-58.44, -34.61), (-57.63, -25.28)))
        EdgeArgentinaParaguay = Edge(Paraguay, hs.haversine((-58.44, -34.61), (-57.63, -25.28)))

        EdgeArgentinaUruguay = Edge(Uruguay, hs.haversine((-58.44, -34.61), (-56.19, -34.87)))
        EdgeUruguayArgentina = Edge(Argentina, hs.haversine((-58.44, -34.61), (-56.19, -34.87)))

        EdgeArgentinaChile = Edge(Chile, hs.haversine((-58.44, -34.61), (-70.65, -33.44)))
        EdgeChileArgentina = Edge(Argentina, hs.haversine((-58.44, -34.61), (-70.65, -33.44)))

        self.graph.AddEdge(Brazil, EdgeBrazilPeru)
        self.graph.AddEdge(Peru, EdgePeruBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilFrenchGuiana)
        self.graph.AddEdge(FrenchGuiana, EdgeFrenchGuianaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilSuriname)
        self.graph.AddEdge(Suriname, EdgeSurinameBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilGuyana)
        self.graph.AddEdge(Guyana, EdgeGuyanaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilVenezuela)
        self.graph.AddEdge(Venezuela, EdgeVenezuelaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilColombia)
        self.graph.AddEdge(Colombia, EdgeColombiaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilBolivia)
        self.graph.AddEdge(Bolivia, EdgeBoliviaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilParaguay)
        self.graph.AddEdge(Paraguay, EdgeParaguayBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilArgentina)
        self.graph.AddEdge(Argentina, EdgeArgentinaBrazil)

        self.graph.AddEdge(Brazil, EdgeBrazilUruguay)
        self.graph.AddEdge(Uruguay, EdgeUruguayBrazil)

        self.graph.AddEdge(FrenchGuiana, EdgeFrenchGuianaSuriname)
        self.graph.AddEdge(Suriname, EdgeSurinameFrenchGuiana)

        self.graph.AddEdge(Suriname, EdgeSurinameGuyana)
        self.graph.AddEdge(Guyana, EdgeGuyanaSuriname)

        self.graph.AddEdge(Guyana, EdgeGuyanaVenezuela)
        self.graph.AddEdge(Venezuela, EdgeVenezuelaGuyana)

        self.graph.AddEdge(Venezuela, EdgeVenezuelaColombia)
        self.graph.AddEdge(Colombia, EdgeColombiaVenezuela)

        self.graph.AddEdge(Colombia, EdgeColombiaPeru)
        self.graph.AddEdge(Peru, EdgePeruColombia)

        self.graph.AddEdge(Colombia, EdgeColombiaEcuador)
        self.graph.AddEdge(Ecuador, EdgeEcuadorColombia)

        self.graph.AddEdge(Peru, EdgePeruEcuador)
        self.graph.AddEdge(Ecuador, EdgeEcuadorPeru)

        self.graph.AddEdge(Peru, EdgePeruBolivia)
        self.graph.AddEdge(Bolivia, EdgeBoliviaPeru)

        self.graph.AddEdge(Bolivia, EdgeBoliviaArgentina)
        self.graph.AddEdge(Argentina, EdgeArgentinaBolivia)

        self.graph.AddEdge(Bolivia, EdgeBoliviaParaguay)
        self.graph.AddEdge(Paraguay, EdgeParaguayBolivia)

        self.graph.AddEdge(Argentina, EdgeArgentinaChile)
        self.graph.AddEdge(Chile, EdgeChileArgentina)

        self.graph.AddEdge(Argentina, EdgeArgentinaParaguay)
        self.graph.AddEdge(Paraguay, EdgeParaguayArgentina)

        self.graph.AddEdge(Argentina, EdgeArgentinaUruguay)
        self.graph.AddEdge(Uruguay, EdgeUruguayArgentina)

    # Returns the Elucidian Distance between the Node and NavigationGraph's destination variable.
    def Heuristic(self, Node):
        return math.sqrt(
            (Node.coords[0] - self.destination.coords[0]) ** 2 + (Node.coords[1] - self.destination.coords[1]) ** 2)

    def Navigate(self, paths):
        # Instantiates goal and start nodes based on the destination and current location, respectively
        Goal = copy.deepcopy(self.graph.getNode(self.destination))
        Goal.UpdateOrigin(self.graph.getNode(self.destination))
        Start = copy.deepcopy(self.graph.getNode(self.currentlocation))
        Start.UpdateOrigin(self.graph.getNode(self.currentlocation))
        Start.UpdateParent([])
        # Sets up the loop
        OpenList = []
        ClosedList = []
        ParentLists = []
        counter = 0
        OpenList.append(Start)
        while counter < paths:
            # If at any point the OpenList has no Nodes left, the while loop breaks.  This is to make sure if the
            # of unique paths to the goal are less than the variable paths, Navigate doesn't break
            if len(OpenList) == 0:
                break
            # Sorts the list, takes the first Node from the list and deletes said node from OpenList
            OpenList.sort()
            while OpenList[0] == None:
                del OpenList[0]
            CurrentNode = OpenList[0]
            del OpenList[0]
            # Adds deleted node to the closed list, and gets the F, G and H values from the current Node
            ClosedList.append(CurrentNode)
            CurrentF, CurrentG, CurrentH = CurrentNode.GetStats()
            CurrentParent = copy.deepcopy(CurrentNode.GetParent())
            CurrentParent.append(CurrentNode)
            # print(CurrentParent)

            # If the given node is Goal and the number of paths already found are less than the given variable paths, the entire path of nodes is added to ParentList, which in turn is added to ParentLists.
            if CurrentNode.EqualCoords(Goal):
                if counter < paths:
                    ParentLists.append((CurrentG, CurrentParent))
                    counter += 1
                    # print(counter)
            else:
                # For every unique edge CurrentNode has, it checks to see if the Node said edge has is already in OpenList or ClosedList and comes from the same parent/edge.
                for i in self.graph.graph[CurrentNode.GetOrigin()]:
                    ListCheck = True
                    E = copy.deepcopy(i.EndNode)
                    E.UpdateParent(CurrentParent)
                    E.UpdateOrigin(i.EndNode)
                    E.UpdateStats(CurrentG + i.weight, self.Heuristic(i.EndNode))
                    # If there is already a child node from that edge in ClosedList, nothing further is done.
                    coordcheck = []
                    for i in E.parent:
                        if i.coords in coordcheck:
                            ListCheck = False
                        coordcheck.append(i.coords)
                    for j in ClosedList:
                        if E.EqualCoordsEqualPath(j):
                            ListCheck = False
                            # print("e")
                            # print(E.parent)
                            # print(j.parent)
                            # print()
                    # If there is already a child node from that edge in OpenList, the node in OpenList is updated to be either its old g value, or the g value obtained currently, whichever is less.
                    # This ensures the node in question will always exemplify the fastest path to a given edge.
                    if j in OpenList:
                        if E.EqualCoordsEqualPath(j):
                            ListCheck = False
                            # print("f")
                            # print(E.parent)
                            # print(j.parent)
                            # print()
                            i.UpdateStats(min(j.g, E.g), j.h)
                    # If the given node from the edge is in neither OpenList or ClosedList, the node is appended to OpenList.
                    if ListCheck == True:
                        OpenList.append(E)

                        # print(CurrentG)
                        # print(CurrentNode.coords,E.coords,E.g)
        # Once the number of unique paths to the destination or the fastest paths number of unique paths is found,
        # whichever is less,
        # these paths stored in ParentLists are converted to NavigationPaths and appended to self.paths.
        # Before this, self.paths is set to no paths, so that the unique paths in Navigate() do not stack.
        self.paths = []
        for i in ParentLists:
            l = []
            for j in i[1]:
                l.append(j.coords)
            Path = NavigationPath(l, i[0], i[0], i[0])
            self.paths.append(Path)

    def ChangeDestination(self, Destination):
        self.destination = Destination
        # Once the destination changes, the paths in self.paths are no longer applicable, so all NavigationPaths in self.paths are deleted.
        self.paths = []

    def GetPaths(self):
        return self.paths


class Software(QMainWindow, MainWindow):
    _IMAGE = "./SouthAmerica.png"
    _TEMP = "./temp.jpg"

    def __init__(self, parent=None):
        super(Software, self).__init__(parent)
        self.setupUi(self)
        # where to store current location and destination
        self.countries = {
            "current location": None,
            "destination": None
        }
        self.nav_graph = None
        self.list_path = None
        self.path_ref = {}
        self.dict_countries = dict(
            Brazil=Node([-47.91, -15.76]),
            Bolivia=Node([-68.09, -16.46]),
            Paraguay=Node([-57.57, -25.29]),
            Uruguay=Node([-56.19, -34.87]),
            Argentina=Node([-58.41, -34.58]),
            Chile=Node([-70.68, -33.44]),
            Peru=Node([-77.05, -12.05]),
            Ecuador=Node([-78.55, -0.19]),
            Colombia=Node([-74.13, 4.72]),
            Venezuela=Node([-66.90, 10.51]),
            Guyana=Node([-58.19, 6.86]),
            Suriname=Node([-55.17, 5.84]),
            FrenchGuiana=Node([-52.33, 4.95]),
        )

        self.dict_coords = {
            "[-47.91, -15.76]": "Brazil",
            "[-68.09, -16.46]": "Bolivia",
            "[-57.57, -25.29]": "Paraguay",
            "[-56.19, -34.87]": "Uruguay",
            "[-58.41, -34.58]": "Argentina",
            "[-70.68, -33.44]": "Chile",
            "[-77.05, -12.05]": "Peru",
            "[-78.55, -0.19]": "Ecuador",
            "[-74.13, 4.72]": "Colombia",
            "[-66.9, 10.51]": "Venezuela",
            "[-58.19, 6.86]": "Guyana",
            "[-55.17, 5.84]": "Suriname",
            "[-52.33, 4.95]": "FrenchGuiana",
        }

        # available names
        self.countries_names = ["Brazil",
                     "Bolivia",
                     "Paraguay",
                     "Uruguay",
                     "Argentina",
                     "Chile",
                     "Peru",
                     "Ecuador",
                     "Colombia",
                     "Venezuela",
                     "Guyana",
                     "Suriname",
                     "FrenchGuiana"]

        # capital location in the image, relative points in pixels
        self.capital_location = {
            "Brazil":[268, 215],
            "Bolivia": [130, 225],
            "Paraguay": [201, 285],
            "Uruguay": [207, 348],
            "Argentina": [188,340],
            "Chile": [124, 345],
            "Peru": [75, 201],
            "Ecuador": [54, 121],
            "Colombia": [86, 86],
            "Venezuela": [136, 44],
            "Guyana": [194, 69],
            "Suriname": [217, 76],
            "FrenchGuiana": [236, 84]

        }

        # lat, long of countries
        self.coordinates = dict(
                Brazil = (-68.74, -11.00),
                Bolivia = (-68.09, -16.46),
                Paraguay = (-57.63, -25.28),
                Uruguay = (-56.19, -34.87),
                Argentina = (-58.44, -34.61),
                Chile = (-70.65, -33.44),
                Peru = (-77.03, -12.06),
                Ecuador = (-78.51, -0.22),
                Colombia = (-74.13, 4.72),
                Venezuela = (-66.90, 10.51),
                Guyana = (-58.19, 6.81),
                Suriname = (-55.17, 5.84),
                FrenchGuiana = (-52.33, 4.95),
                )

        # connect signals to methods
        self.destination.currentTextChanged.connect(self._get_destination)
        self.location.currentTextChanged.connect(self._get_current_location)
        self.search_path.clicked.connect(self._start_searching)
        self.available_path.currentTextChanged.connect(self.new_path_requested)
        self.description_ref = {}
        self._init_countries()
        self._load_image()
        self.map = None
        self._init_map()

    def _init_map(self):
        # init map, (get the map that is without anything drawn yet)
        # for new drawing
        self.map = cv2.imread(self._IMAGE)

    def _load_image(self):
        # load image first time
        img = cv2.imread(self._IMAGE)
        H, W, _ = img.shape
        self.image.setPixmap(QPixmap(self._IMAGE))
        self.image.resize(W, H)

    def _map_draw_capitals(self, countries):
        # draw circles (dotes)
        for key, center in self.capital_location.items():
            if key in countries:
                cv2.circle(self.map, (*center,), 2, (0,0,255), -1)

    def _draw_path_in_map(self, location, destination):
        # draw line
        c1 = self.capital_location[location]
        c2 = self.capital_location[destination]
        cv2.line(self.map, (*c1,), (*c2,), (69, 77, 50), thickness=1)

    def _draw_path(self, location, destination):
        # a path is 2 dotes and one line, dotes to present the cities
        self._map_draw_capitals([location, destination])
        self._draw_path_in_map(location, destination)

    def _draw_complete_path(self, data):
        # draw all lines and dotes, of countries in data
        self._init_map()

        for i in range(len(data) - 1):
            location = data[i]
            destination = data[i+1]
            self._draw_path(location, destination)

        self._set_picture()

    def _set_picture(self):
        # update image in interface
        cv2.imwrite(self._TEMP, self.map)
        self.image.setPixmap(QPixmap(self._TEMP))

    def _init_countries(self):
        # fill dropdown menus with available countries
        for country in self.countries_names:
            self.location.addItem(country)

        for country in self.countries_names:
            if country != self.location.currentText():
                self.destination.addItem(country)

    def _get_current_location(self, location):
        # update current location after user select new location
        # + delete the location from destination
        # user can't select same country for location and destination

        self.countries["current location"] = self.dict_countries[location]
        self.destination.clear()
        for country in self.countries_names:
            if country != location:
                self.destination.addItem(country)

    def _get_destination(self, str):
        pass

    def _get_distance(self, path):
        distance = 0
        data = [self.dict_coords[str(n)] for n in path.GetInstructions()]
        # calculate distance using lat and long,
        # distance will have decimals (meters)
        # distance can be rounded to get natural value integer.
        for i in range(len(data) - 1):
            location = data[i]
            destination = data[i+1]
            long_loc, lat_loc = self.coordinates[location]
            long_dist, lat_dist = self.coordinates[destination]
            distance += hs.haversine((lat_loc, long_loc), (lat_dist, long_dist))

        distance = round(distance)
        return distance

    def _start_searching(self):
        import traceback
        try:
            self.nav_graph = None
            self.list_path = None
            self.path_ref = {}
            self.description_ref = {}

            # get countries from dropdown menus
            current_location = self.location.currentText()
            destination = self.destination.currentText()

            self.countries["current location"] = self.dict_countries[current_location]
            self.countries["destination"] = self.dict_countries[destination]
            self.nav_graph = NavigationGraph(
                CurrentLocation=self.countries["current location"],
                Destination=self.countries["destination"]
            )

            # find all paths

            self.nav_graph.Navigate(11)
            self.list_path = self.nav_graph.GetPaths()

            # find shortest path
            # index_shortest_path = np.argmin([path.GetPathLength()[0] for path in self.list_path])

            # sort path according to length
            self.list_path.sort(key=lambda path: path.GetPathLength()[0])
            # show shortest path
            name_current_location = self.dict_coords[str(self.countries['current location'])]
            name_destination = self.dict_coords[str(self.countries['destination'])]

            # make a description + calculate distance
            result = f"Shortest path from {name_current_location} to {name_destination}: \n\n"
            result += self._show_path(self.list_path[0], 0)
            result += f"\n Distance : {self._get_distance(self.list_path[0])} Km"

            # make this distance referenced by the path string
            self.description_ref[self._show_path(self.list_path[0], 0)] = result

            self.shortest_path_desc.setPlainText(result)
            # fill available paths with all found paths after searches
            # block signals so the new path requested will not be triggered in this filling
            self.available_path.blockSignals(True)
            self.available_path.clear()
            self.available_path.addItem("-")
            for index_path, path in enumerate(self.list_path):
                res = self._show_path(path, index_path)
                if index_path > 0:
                    self.description_ref[res] = res
                    # reference descriptions to use them after each request
                    self.description_ref[res] += f"\n Distance : {self._get_distance(path)} Km"
                self.available_path.addItem(res)
                self.path_ref[res] = path

            self.available_path.setCurrentText("-")
            self.available_path.blockSignals(False)

        except:
            print(traceback.format_exc())

    """
    NEW PATH REQUESTED
    Method called when user click on new available path, 
    it takes all the countries involved in the path and draw cities and lines creating the path.
    then it changes the description into the new one corresponding the the actual path.
    description contains path + distance
    """
    def new_path_requested(self, path_name):
        path = self.path_ref[path_name]
        data = [self.dict_coords[str(n)] for n in path.GetInstructions()]
        # list of countries in the path
        self._draw_complete_path(data)
        # draw complete path (dotes + lines)
        self.shortest_path_desc.setPlainText(self.description_ref[path_name])
        # update description

    def _show_path(self, path, index_path):
        path_as_str = str(index_path) + ") " + " --> ".join([self.dict_coords[str(n)] for n in path.GetInstructions()])
        return path_as_str


app = QApplication([])
X = Software()
X.show()
app.exec()