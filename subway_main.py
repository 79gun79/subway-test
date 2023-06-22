from make_tree import *

def best_first_search(problem, f) :
    node = Node(problem.initial)
    front = PriorityQueue([node], key = f)
    reached = {problem.initial: node}
    while front :
        node = front.pop()
        if problem.is_goal(node.state) :
            return node
        for child in expand(problem, node) :
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost :
                reached[s] = child
                front.add(child)
    return failure

def is_cycle(node, k = 26) :
    "상태가 26개이므로 싸이클 노드 제한 개수를 26개로 둠."
    def find_cycle(ancestor, k) :
        return (ancestor is not None and k > 0 and
                (ancestor.state == node.state or find_cycle(ancestor.parent, k - 1)))
    return find_cycle(node.parent, k)

def g(n): return n.path_cost
"g(n) = 초기 상태 ~ 해당 노드까지의 경로비용"

def greedy_bfs(problem, h = None) : #탐욕적 최고 우선 탐색
    h = h or problem.h
    return best_first_search(problem, f = h)

def uniform_cost_search(problem) : #균일 비용 탐색
    return best_first_search(problem, f = g)

def astar_search(problem, h = None) : #A* 탐색
    h = h or problem.h
    return best_first_search(problem, f = lambda n: g(n) + h(n))

def weighted_astar_search(problem, h = None, weight = 3.0) : #가중치 A* 탐색
    "가중치는 3.0으로 설정"
    h = h or problem.h
    return best_first_search(problem, f = lambda n: g(n) + weight * h(n))

class Map :
    def __init__(self, links, locations = None, directed = False) :
        if not hasattr(links, 'items') :
            links = {link: 1 for link in links}
        if not directed :
            for (v1, v2) in list(links) :
                links[v2, v1] = links[v1, v2]
        self.distances = links
        self.neighbors = multimap(links)
        self.locations = locations or defaultdict(lambda: (0, 0))

def multimap(pairs) :
    result = defaultdict(list)
    for key, val in pairs :
        result[key].append(val)
    return result

class Route_Problem(Problem) :
    def actions(self, state) : 
        return self.map.neighbors[state]
    
    def result(self, state, action) :
        return action if action in self.map.neighbors[state] else state
    
    def action_cost(self, s, action, s1) :
        return self.map.distances[s, s1]
    
    def h(self, node) :
        locs = self.map.locations
        return straight_line_distance(locs[node.state], locs[self.goal])
    
def straight_line_distance(A, B) :
    "두 점 사이의 직선 거리를 구하는 식"
    return sum(abs(a - b) ** 2 for (a, b) in zip(A, B)) ** 0.5


"지하철 노선도 상태 공간을 표현한 맵"
subway = Map (
    {('A', 'D') : 25, ('A', 'F') : 98, ('D', 'E') : 70, ('E', 'X') : 83,
    ('E', 'F') : 98, ('E', 'J') : 41, ('E', 'U') : 81, ('U', 'O') : 72,
    ('V', 'O') : 60, ('V', 'W') : 25, ('O', 'W') : 70, ('F', 'J') : 34,
    ('J', 'B') : 66, ('F', 'G') : 20, ('O', 'P') : 16, ('B', 'P') : 57,
    ('K', 'B') : 24, ('G', 'K') : 30, ('C', 'G') : 72, ('T', 'C') : 26,
    ('T', 'Y') : 18, ('C', 'H') : 86, ('G', 'H') : 22, ('B', 'M') : 38,
    ('B', 'Q') : 60, ('P', 'Q') : 60, ('B', 'R') : 100, ('Q', 'Z') : 50,
    ('Q', 'S') : 42, ('M', 'Q') : 50, ('M', 'N') : 60, ('L', 'M') : 23,
    ('L', 'I') : 71, ('H', 'L') : 24
    },
    {'A': (20, 120), 'B': (120, 60), 'C': (40, 60), 'D': (40, 120), 'E': (60, 110),
     'F': (60, 94), 'G': (60, 60), 'H': (59, 34), 'I': (68, 12), 'J': (81, 96),
     'K': (83, 60), 'L': (83, 35), 'M': (112, 36), 'N': (115, 13), 'O': (140, 120), 
     'P': (140, 80), 'Q': (140, 40), 'R': (180, 60), 'S': (171, 19), 'T': (13, 57),
     'U': (101, 117), 'V': (144, 143), 'W': (168, 130), 'X': (29, 90), 'Y': (18, 29), 'Z': (187, 36)})


"역 J가 막힌 맵"
subway_ver2 = Map (
    {('A', 'D') : 25, ('A', 'F') : 98, ('D', 'E') : 70, ('E', 'X') : 83,
    ('E', 'F') : 98, ('E', 'U') : 81, ('U', 'O') : 72, ('V', 'O') : 60, 
    ('V', 'W') : 25, ('O', 'W') : 70, ('F', 'G') : 20, ('H', 'L') : 24,
    ('O', 'P') : 16, ('B', 'P') : 57, ('K', 'B') : 24, ('G', 'K') : 30,
    ('C', 'G') : 72, ('T', 'C') : 26, ('T', 'Y') : 18, ('C', 'H') : 86,
    ('G', 'H') : 22, ('B', 'M') : 38, ('B', 'Q') : 60, ('P', 'Q') : 60,
    ('B', 'R') : 100, ('Q', 'Z') : 50, ('Q', 'S') : 42, ('M', 'Q') : 50,
    ('M', 'N') : 60, ('L', 'M') : 23, ('L', 'I') : 71
    },
    {'A': (20, 120), 'B': (120, 60), 'C': (40, 60), 'D': (40, 120), 'E': (60, 110),
     'F': (60, 94), 'G': (60, 60), 'H': (59, 34), 'I': (68, 12),
     'K': (83, 60), 'L': (83, 35), 'M': (112, 36), 'N': (115, 13), 'O': (140, 120), 
     'P': (140, 80), 'Q': (140, 40), 'R': (180, 60), 'S': (171, 19), 'T': (13, 57),
     'U': (101, 117), 'V': (144, 143), 'W': (168, 130), 'X': (29, 90), 'Y': (18, 29), 'Z': (187, 36)})


r1 = Route_Problem('U', 'C', map = subway)
r2 = Route_Problem('F', 'Z', map = subway)
r3 = Route_Problem('D', 'C', map = subway)
#성능 평가를 위한 여러 문제의 탐색 경로들 비교
#탐욕적 최고 우선, 균일 비용, A*, 가중치 A* 탐색 순
for problem in [r1, r2, r3] :
    print(path_states(greedy_bfs(problem)))
    print(path_states(uniform_cost_search(problem)))
    print(path_states(astar_search(problem)))
    print(path_states(weighted_astar_search(problem)))
    print()


r4 = Route_Problem('A', 'B', map = subway)
r5 = Route_Problem('A', 'B', map = subway_ver2)
#평상시와 역 J가 막혔을 때 반환하는 경로의 차이
for problem2 in [r4, r5] :
    print(path_states(greedy_bfs(problem2)))