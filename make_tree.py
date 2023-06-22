import sys
import math
import heapq
from collections import defaultdict
from collections import deque

class Problem :
    def __init__(self, initial = None, goal = None, **kwds):
        self.__dict__.update(initial = initial, goal = goal, **kwds)

    def actions(self, state) :
        raise NotImplementedError

    def res(self, state, action) :
        raise NotImplementedError

    def is_goal(self, state) :
        if isinstance(self.goal, list) :
            return is_in(state, self.goal)
        else :
            return state == self.goal

    def action_cost(self, state1, action, state2) :
        return 1

    def h(self, node) :
        return 0
    
    def __str__(self) :
        return f'{type(self).__name__}({self.initial!r}, {self.goal!r})'

def is_in(elt, seq) :
    return any(x is elt for x in seq)

class Node :
    def __init__(self, state, parent = None, action = None, path_cost = 0):
        self.__dict__.update(state = state, parent = parent, action = action, path_cost=path_cost)

    def __repr__(self) :
        return f"<{self.state}>"

    def __len__(self) :
        return 0 if self.parent is None else (1 + len(self.parent))
    
    def __lt__(self, other) :
        return self.path_cost < other.path_cost
           
failure = Node('failure', path_cost = math.inf)
cutoff = Node('cutoff', path_cost = math.inf)

def expand(problem, node) :
    s = node.state
    for action in problem.actions(s) :
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        
def path_actions(node) :
    if node.parent is None :
        return []  
    return path_actions(node.parent) + [node.action]

def path_states(node):
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]

FIFOQueue = deque
LIFOQueue = list #stack

class PriorityQueue :
    def __init__(self, items = (), key = lambda x: x) : 
        self.key = key
        self.items = []
        for item in items :
            self.add(item)
    def add(self, item) :
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)
    def pop(self) :
        return heapq.heappop(self.items)[1]
    
    def top(self) :
        return self.items[0][1]

    def __len__(self) :
        return len(self.items)