from search import *
from utils import *
import math

class Problem:
    def __init__(self, initial, goal=None):
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def goal_test(self, state):
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def value(self, state):
        raise NotImplementedError

##########################################################################

class GraphProblem(Problem):
    def __init__(self, initial, goal, graph):
        super().__init__(initial, goal)
        self.graph = graph

    def actions(self, A):
        return list(self.graph.get(A).keys())

    def result(self, state, action):
        return action

    def path_cost(self, cost_so_far, A, action, B):
        return cost_so_far + (self.graph.get(A, B) or math.inf)

    def find_min_edge(self):
        m = math.inf
        for d in self.graph.graph_dict.values():
            local_min = min(d.values())
            m = min(m, local_min)

        return m

    def h(self, node):
        locs = getattr(self.graph, 'locations', None)
        if locs:
            if type(node) is str:
                return int(distance(locs[node], locs[self.goal]))

            return int(distance(locs[node.state], locs[self.goal]))
        else:
            return math.inf

##########################################################################

class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        return [node.action for node in self.path()[1:]]

    def path(self):
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)

##########################################################################

# Berdasarkan Jarak #

# Trip_map = UndirectedGraph(dict(
#     Semarang=dict(Lasem=138, Surakarta=105),
#     Surakarta=dict(Ngawi=83),
#     Ngawi=dict(Badad=111, Nganjuk=76),
#     Lasem=dict(Tuban=138),
#     Nganjuk=dict(Mojokerto=73),
#     Tuban=dict(Karang_Rejo=68, Badad=34),
#     Badad=dict(Gresik=58),
#     Mojokerto=dict(Surabaya=53),
#     Karang_Rejo=dict(Gresik=32),
#     Gresik=dict(Surabaya=21)))
    
    
# Trip_map.locations = dict(
#     Semarang=(-7.01126, 110.43154),
#     Surakarta=(-7.57101, 110.81311),
#     Ngawi=(-7.40544, 111.44182),
#     Lasem=(-6.69980, 111.43332),
#     Nganjuk=(-7.60235, 111.90080),
#     Tuban=(-6.89697, 112.03312),
#     Badad=(-7.11228, 112.17280),
#     Mojokerto=(-7.47085, 112.41662),
#     Karang_Rejo=(-6.95375, 112.55723),
#     Gresik=(-7.16520, 112.65199),
#     Surabaya=(-7.25854, 112.75126))

##########################################################################

# Berdasarkan Waktu #

Trip_map = UndirectedGraph(dict(
    Semarang=dict(Lasem=215, Surakarta=100),
    Surakarta=dict(Ngawi=80),
    Ngawi=dict(Badad=140, Nganjuk=55),
    Lasem=dict(Tuban=117),
    Nganjuk=dict(Mojokerto=60),
    Tuban=dict(Karang_Rejo=109, Badad=50),
    Badad=dict(Gresik=110),
    Mojokerto=dict(Surabaya=65),
    Karang_Rejo=dict(Gresik=75),
    Gresik=dict(Surabaya=44)))
    
    
Trip_map.locations = dict(
    Semarang=(-7.01126, 110.43154),
    Surakarta=(-7.57101, 110.81311),
    Ngawi=(-7.40544, 111.44182),
    Lasem=(-6.69980, 111.43332),
    Nganjuk=(-7.60235, 111.90080),
    Tuban=(-6.89697, 112.03312),
    Badad=(-7.11228, 112.17280),
    Mojokerto=(-7.47085, 112.41662),
    Karang_Rejo=(-6.95375, 112.55723),
    Gresik=(-7.16520, 112.65199),
    Surabaya=(-7.25854, 112.75126))

##########################################################################

def best_first_graph_search(problem, f, display=False):
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            if display:
                print(len(explored), "paths have been expanded and", len(frontier), "paths remain in the frontier")
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if str(f(child)) < str(frontier[child]):
                    del frontier[child]
                    frontier.append(child)
    return None

##########################################################################

def astar_search(problem, h=None):
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))

##########################################################################

Trip_problem = GraphProblem('Semarang', 'Surabaya', Trip_map)
print([node.state for node in astar_search(Trip_problem).path()])

