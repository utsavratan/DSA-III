# ============================================================
#  Utsav Ratan
#  2401010046
#  BTech CSE Core - Section : B
# ============================================================
# ============================================================
#  BUILDING DATA ADT
# ============================================================

class Building:
    def __init__(self, building_id, name, location):
        self.building_id = building_id
        self.name = name
        self.location = location


# ============================================================
#  BINARY SEARCH TREE IMPLEMENTATION
# ============================================================

class BSTNode:
    def __init__(self, building):
        self.building = building
        self.left = None
        self.right = None


class BinarySearchTree:
    def __init__(self):
        self.root = None

    def insert(self, building):
        self.root = self._insert(self.root, building)

    def _insert(self, node, building):
        if node is None:
            return BSTNode(building)

        if building.building_id < node.building.building_id:
            node.left = self._insert(node.left, building)
        else:
            node.right = self._insert(node.right, building)

        return node

    def search(self, building_id):
        return self._search(self.root, building_id)

    def _search(self, node, building_id):
        if node is None:
            return None
        if building_id == node.building.building_id:
            return node.building
        elif building_id < node.building.building_id:
            return self._search(node.left, building_id)
        else:
            return self._search(node.right, building_id)

    # Traversals
    def inorder(self):
        result = []
        self._inorder(self.root, result)
        return result

    def _inorder(self, node, result):
        if node:
            self._inorder(node.left, result)
            result.append(node.building.name)
            self._inorder(node.right, result)

    def preorder(self):
        result = []
        self._preorder(self.root, result)
        return result

    def _preorder(self, node, result):
        if node:
            result.append(node.building.name)
            self._preorder(node.left, result)
            self._preorder(node.right, result)

    def postorder(self):
        result = []
        self._postorder(self.root, result)
        return result

    def _postorder(self, node, result):
        if node:
            self._postorder(node.left, result)
            self._postorder(node.right, result)
            result.append(node.building.name)


# ============================================================
#  AVL TREE IMPLEMENTATION
# ============================================================

class AVLNode:
    def __init__(self, building):
        self.building = building
        self.left = None
        self.right = None
        self.height = 1


class AVLTree:
    def __init__(self):
        self.root = None

    def insert(self, building):
        self.root = self._insert(self.root, building)

    def _insert(self, node, building):
        if not node:
            return AVLNode(building)

        if building.building_id < node.building.building_id:
            node.left = self._insert(node.left, building)
        else:
            node.right = self._insert(node.right, building)

        node.height = 1 + max(self.getHeight(node.left),
                              self.getHeight(node.right))

        balance = self.getBalance(node)

        # LL Rotation
        if balance > 1 and building.building_id < node.left.building.building_id:
            return self.rightRotate(node)

        # RR Rotation
        if balance < -1 and building.building_id > node.right.building.building_id:
            return self.leftRotate(node)

        # LR Rotation
        if balance > 1 and building.building_id > node.left.building.building_id:
            node.left = self.leftRotate(node.left)
            return self.rightRotate(node)

        # RL Rotation
        if balance < -1 and building.building_id < node.right.building.building_id:
            node.right = self.rightRotate(node.right)
            return self.leftRotate(node)

        return node

    def leftRotate(self, z):
        y = z.right
        T2 = y.left
        y.left = z
        z.right = T2
        z.height = 1 + max(self.getHeight(z.left), self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))
        return y

    def rightRotate(self, z):
        y = z.left
        T3 = y.right
        y.right = z
        z.left = T3
        z.height = 1 + max(self.getHeight(z.left), self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left), self.getHeight(y.right))
        return y

    def getHeight(self, node):
        if not node:
            return 0
        return node.height

    def getBalance(self, node):
        if not node:
            return 0
        return self.getHeight(node.left) - self.getHeight(node.right)


# ============================================================
#  GRAPH IMPLEMENTATION (Adjacency List + Matrix)
# ============================================================

from collections import defaultdict
import heapq


class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.adj_list = defaultdict(list)
        self.adj_matrix = [[0]*vertices for _ in range(vertices)]

    def add_edge(self, u, v, w):
        # Adjacency List
        self.adj_list[u].append((v, w))
        self.adj_list[v].append((u, w))

        # Adjacency Matrix
        self.adj_matrix[u][v] = w
        self.adj_matrix[v][u] = w

    # BFS
    def bfs(self, start):
        visited = [False]*self.V
        queue = [start]
        visited[start] = True
        result = []

        while queue:
            u = queue.pop(0)
            result.append(u)
            for v, _ in self.adj_list[u]:
                if not visited[v]:
                    queue.append(v)
                    visited[v] = True
        return result

    # DFS
    def dfs(self, start):
        visited = [False]*self.V
        result = []

        def _dfs(v):
            visited[v] = True
            result.append(v)
            for nxt, _ in self.adj_list[v]:
                if not visited[nxt]:
                    _dfs(nxt)

        _dfs(start)
        return result

    # Dijkstra
    def dijkstra(self, start):
        dist = [float('inf')] * self.V
        dist[start] = 0
        pq = [(0, start)]

        while pq:
            d, u = heapq.heappop(pq)
            for v, w in self.adj_list[u]:
                if d + w < dist[v]:
                    dist[v] = d + w
                    heapq.heappush(pq, (dist[v], v))

        return dist

    # Kruskal MST
    def kruskal(self):
        edges = []
        for u in range(self.V):
            for v, w in self.adj_list[u]:
                if u < v:
                    edges.append((w, u, v))

        edges.sort()
        parent = list(range(self.V))

        def find(x):
            while parent[x] != x:
                x = parent[x]
            return x

        mst = []
        for w, u, v in edges:
            pu, pv = find(u), find(v)
            if pu != pv:
                mst.append((u, v, w))
                parent[pu] = pv

        return mst


# ============================================================
#  EXPRESSION TREE (Energy Bill Calculator)
# ============================================================

class ExpNode:
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None


class ExpressionTree:
    def build(self, postfix):
        stack = []
        for token in postfix:
            if token.isdigit():
                stack.append(ExpNode(token))
            else:
                node = ExpNode(token)
                node.right = stack.pop()
                node.left = stack.pop()
                stack.append(node)
        return stack.pop()

    def evaluate(self, root):
        if not root:
            return 0

        if root.value.isdigit():
            return int(root.value)

        left = self.evaluate(root.left)
        right = self.evaluate(root.right)

        if root.value == '+': return left + right
        if root.value == '-': return left - right
        if root.value == '*': return left * right
        if root.value == '/': return left / right


# ============================================================
#  CAMPUS NAVIGATION & UTILITY PLANNER SYSTEM
# ============================================================

class CampusPlanner:
    def __init__(self):
        self.bst = BinarySearchTree()
        self.avl = AVLTree()
        self.graph = None
        self.buildings = {}

    def addBuildingRecord(self, building_id, name, location):
        building = Building(building_id, name, location)
        self.bst.insert(building)
        self.avl.insert(building)
        self.buildings[building_id] = name

    def listCampusLocations(self):
        return {
            "Inorder": self.bst.inorder(),
            "Preorder": self.bst.preorder(),
            "Postorder": self.bst.postorder()
        }

    def constructCampusGraph(self, vertices, edges):
        self.graph = Graph(vertices)
        for u, v, w in edges:
            self.graph.add_edge(u, v, w)

    def findOptimalPath(self, start):
        return self.graph.dijkstra(start)

    def planUtilityLayout(self):
        return self.graph.kruskal()


# ============================================================
#  SAMPLE RUN
# ============================================================

if __name__ == "__main__":
    planner = CampusPlanner()

    # Add buildings
    planner.addBuildingRecord(1, "Admin Block", "Center")
    planner.addBuildingRecord(3, "Library", "North")
    planner.addBuildingRecord(2, "Hostel", "East")

    print("TREE TRAVERSALS:", planner.listCampusLocations())

    # Graph
    edges = [(0, 1, 10), (1, 2, 5), (0, 2, 15)]
    planner.constructCampusGraph(3, edges)

    print("BFS:", planner.graph.bfs(0))
    print("DFS:", planner.graph.dfs(0))
    print("Dijkstra:", planner.findOptimalPath(0))
    print("Kruskal MST:", planner.planUtilityLayout())

    # Expression tree
    expr = ExpressionTree()
    root = expr.build(["5", "3", "+", "2", "*"])   # (5+3)*2
    print("Energy Bill Eval:", expr.evaluate(root))
