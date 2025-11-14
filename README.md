# Campus Navigation & Utility Planner
### Assignment 3 – Data Structures (ENCS205)
### Trees & Graphs Implementation using Python

---

## 📘 Overview
The **Campus Navigation & Utility Planner** is a complete data-structure based project that models campus buildings using tree structures and campus routes using graph algorithms.

This project includes:

- Binary Search Tree (BST)
- AVL Tree with balancing (LL, RR, LR, RL rotations)
- Graph using Adjacency List & Adjacency Matrix
- BFS and DFS graph traversals
- Dijkstra’s Algorithm for shortest path
- Kruskal’s Algorithm for MST (utility/cable layout)
- Expression Tree for energy bill evaluation

---

## 🏛️ Building Data ADT

Each building contains:
- BuildingID (int)
- BuildingName (string)
- LocationDetails (string)
- Connections (used in graph paths)

Supported operations:
- Insert building record
- Search building
- Tree traversals (inorder, preorder, postorder)
- Expression tree evaluation for energy bills

---

## 🌳 Part 1 — Tree Implementations

### ✔ Binary Search Tree (BST)
Implements:
- Insert
- Search
- Inorder traversal
- Preorder traversal
- Postorder traversal

### ✔ AVL Tree
Includes:
- Automatic height balancing
- LL Rotation
- RR Rotation
- LR Rotation
- RL Rotation
- Computes AVL height vs BST height

---

## 🌐 Part 2 — Graph Implementations

### ✔ Graph Representations
- Adjacency Matrix
- Adjacency List

### ✔ Graph Traversals
- BFS (Breadth-First Search)
- DFS (Depth-First Search)

### ✔ Algorithms
- Dijkstra’s Algorithm (Shortest path navigation)
- Kruskal’s Algorithm (Minimum Spanning Tree for utility layout)

---

## 🔢 Expression Tree
Used to evaluate arithmetic expressions such as electricity/energy billing.
Supports:
- Addition (+)
- Subtraction (-)
- Multiplication (*)
- Division (/)

---

## 🧪 Sample Input Used (main.py)

Buildings:
- Admin Block
- Library
- Hostel

Campus Paths:
- Admin ↔ Library (10)
- Library ↔ Hostel (5)
- Admin ↔ Hostel (15)
