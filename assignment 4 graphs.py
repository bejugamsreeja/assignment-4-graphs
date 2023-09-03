#!/usr/bin/env python
# coding: utf-8

# In[16]:


#implementing BFS for a graph
#BFS
graph = {
  '5' : ['3','7'],
  '3' : ['2', '4'],
  '7' : ['8'],
  '2' : [],
  '4' : ['8'],
  '8' : []
}

visited = [] # List for visited nodes.
queue = []     #Initialize a queue

def bfs(visited, graph, node): #function for BFS
  visited.append(node)
  queue.append(node)

  while queue:          # Creating loop to visit each node
    m = queue.pop(0) 
    print (m, end = " ") 

    for neighbour in graph[m]:
      if neighbour not in visited:
        visited.append(neighbour)
        queue.append(neighbour)

# Driver Code
print("Following is the Breadth-First Search")
bfs(visited, graph, '5')


# In[17]:


#DFS
graph = {
  '5' : ['3','7'],
  '3' : ['2', '4'],
  '7' : ['8'],
  '2' : [],
  '4' : ['8'],
  '8' : []
}

visited = set() # Set to keep track of visited nodes of graph.

def dfs(visited, graph, node):  #function for dfs 
    if node not in visited:
        print (node)
        visited.add(node)
        for neighbour in graph[node]:
            dfs(visited, graph, neighbour)

# Driver Code
print("Following is the Depth-First Search")
dfs(visited, graph, '5')


# In[18]:


from collections import deque
  
adj = [[] for i in range(1001)]
  
def addEdge(v, w):
    adj[v].append(w)
    adj[w].append(v)
  
def BFS(s, l):
     
    V = 100
    visited = [False] * V
    level = [0] * V
  
    for i in range(V):
        visited[i] = False
        level[i] = 0
    queue = deque()
    visited[s] = True
    queue.append(s)
    level[s] = 0
  
    while (len(queue) > 0):
        s = queue.popleft()
        
        for i in adj[s]:
            if (not visited[i]):
                level[i] = level[s] + 1
                visited[i] = True
                queue.append(i)
                
    count = 0
    for i in range(V):
        if (level[i] == l):
            count += 1
             
    return count
  
# Driver code
if __name__ == '__main__':
    addEdge(0, 1)
    addEdge(0, 2)
    addEdge(1, 3)
    addEdge(2, 4)
    addEdge(2, 5)
  
    level = 2
  
    print(BFS(0, level))
     


# In[19]:


#Counting number of trees in a forest
def addEdge(adj, u, v):
    adj[u].append(v)
    adj[v].append(u)
 
# A utility function to do DFS of graph
# recursively from a given vertex u.
def DFSUtil(u, adj, visited):
    visited[u] = True
    for i in range(len(adj[u])):
        if (visited[adj[u][i]] == False):
            DFSUtil(adj[u][i], adj, visited)
 
# Returns count of tree is the
# forest given as adjacency list.
def countTrees(adj, V):
    visited = [False] * V
    res = 0
    for u in range(V):
        if (visited[u] == False):
            DFSUtil(u, adj, visited)
            res += 1
    return res
# Driver code
if __name__ == '__main__':
 
    V = 5
    adj = [[] for i in range(V)]
    addEdge(adj, 0, 1)
    addEdge(adj, 0, 2)
    addEdge(adj, 3, 4)
    print(countTrees(adj, V))


# In[20]:


from collections import defaultdict
  
class Graph():
    def __init__(self, vertices):
        self.graph = defaultdict(list)
        self.V = vertices
  
    def addEdge(self, u, v):
        self.graph[u].append(v)
  
    def isCyclicUtil(self, v, visited, recStack):
  
        # Mark current node as visited and 
        # adds to recursion stack
        visited[v] = True
        recStack[v] = True
  
        # Recur for all neighbours
        # if any neighbour is visited and in 
        # recStack then graph is cyclic
        for neighbour in self.graph[v]:
            if visited[neighbour] == False:
                if self.isCyclicUtil(neighbour, visited, recStack) == True:
                    return True
            elif recStack[neighbour] == True:
                return True
  
        # The node needs to be popped from 
        # recursion stack before function ends
        recStack[v] = False
        return False
      # Returns true if graph is cyclic else false
    def isCyclic(self):
        visited = [False] * self.V
        recStack = [False] * self.V
        for node in range(self.V):
            if visited[node] == False:
                if self.isCyclicUtil(node, visited, recStack) == True:
                    return True
        return False
  
g = Graph(4)
g.addEdge(0, 1)
g.addEdge(0, 2)
g.addEdge(1, 2)
g.addEdge(2, 0)
g.addEdge(2, 3)
g.addEdge(3, 3)
if g.isCyclic() == 1:
    print("Graph has a cycle")
else:
    print("Graph has no cycle")
  


# In[21]:


#n-Queen's Problem
global N
N = 4
 
def printSolution(board):
    for i in range(N):
        for j in range(N):
            print (board[i][j],end=' ')
        print()

def isSafe(board, row, col):
 
    # Check this row on left side
    for i in range(col):
        if board[row][i] == 1:
            return False
 
    # Check upper diagonal on left side
    for i, j in zip(range(row, -1, -1), range(col, -1, -1)):
        if board[i][j] == 1:
            return False
 
    # Check lower diagonal on left side
    for i, j in zip(range(row, N, 1), range(col, -1, -1)):
        if board[i][j] == 1:
            return False
 
    return True
def solveNQUtil(board, col):
    if col >= N:
        return True
    for i in range(N):
 
        if isSafe(board, i, col):
            board[i][col] = 1
            if solveNQUtil(board, col + 1) == True:
                return True
 
        board[i][col] = 0
    return False
def solveNQ():
    board = [ [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]
             ]
 
    if solveNQUtil(board, 0) == False:
        print ("Solution does not exist")
        return False
 
    printSolution(board)
    return True
 
# driver program to test above function
solveNQ()


# In[ ]:





# In[ ]:




