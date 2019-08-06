"""

8 puzzle game solver, using A* star.

Author: Lasse Mackeprang

"""
import numpy as np
from operator import attrgetter
import time


# Any node contains the state (board), heuristics, a reference to parent node, and last direction a tile was moved
class Node:
    h = 999
    g = 999
    parent = None
    ld = "None"

    def __init__(self, state=None, g=None, h=None, parent=None, ld=None):
        if state is not None:
            self.state = state
        if g is not None:
            self.g = g
        if h is not None:
            self.h = h
        self.f = self.g + self.h
        if parent is not None:
            self.parent = parent
        if ld is not None:
            self.ld = ld


# 8 puzzle solver using A*
def main():
    moves = 0
    open_nodes = []  # all found nodes, that haven't been active
    closed_nodes = []  # all nodes that have been active
    start = time.time()

    # If you want a random board, solvability check performed (rarely a board happens to be unsolvable??)
    b = init_board()  # Initialize a randomized 8 puzzle board

    # Random boards I timed
    # b = np.array([[5, 4, 7], [0, 1, 6], [3, 2, 8]])  # h2 = 2014 nodes 3.82 seconds. h1 = 26742 nodes 655.42 seconds.
    # b = np.array([[1, 3, 0], [7, 2, 6], [5, 4, 8]])  # h2 = 1415 nodes 1.96 seconds. h1 = 15929 nodes 244.69 seconds.
    # b = np.array([[0, 1, 5], [4, 2, 8], [3, 6, 7]])  # h2 = 33 nodes 0.0 seconds.. h1 = 56 nodes 0.01 seconds.
    # b = np.array([[7, 0, 8], [4, 3, 1], [2, 6, 5]])  # h2 = 5203 nodes 25.83 seconds, h1 = ?? (a lot)

    g_b = np.arange(9).reshape(3, 3)  # finished board, "Goal board". = [0 1 2] [3 4 5] [6 7 8]

    f = finished(b, g_b)  # check if the board is already solved (could also just check heuristic == 0)

    # Print start conditions
    print("Start board:")
    print(b)
    print("Misplaced Tiles:", h1(b, g_b))
    print("Manhattan distance:", h2(b, g_b))

    # Create initial node, put in open list
    initial_node = Node(b, 0, h2(b, g_b))
    open_nodes.append(initial_node)

    # Run loop until the game has been solved
    while f is False:
        # active_node, is the node with minimum f value
        active_node = min(open_nodes, key=attrgetter('f'))

        # Check if puzzle is done, and break out of loop
        if active_node.h == 0:
            moves = node_path_print(active_node)
            break

        # remove node from open list, and put in closed list
        open_nodes.remove(active_node)
        closed_nodes.append(active_node)

        possible_directions = pos_dir(active_node.state, active_node.ld)  # Possible directions at active node
        for direction in possible_directions:
            # Create new state
            temp_state = move(active_node.state, direction)
            # If this state has not been seen before
            if state_is_in(temp_state, open_nodes) is False:
                # find heuristic of state
                temp_heuristic = h2(temp_state, g_b)
                # Create new node
                temp_node = Node(temp_state, active_node.g + 1, temp_heuristic, active_node, ld=direction)
                # Add to list
                open_nodes.append(temp_node)

        # b = move(b, input("WASD:"))  # For manual play, also next line
        # f = finished(b, g_b) # Check if the puzzle has been solved

    # Count all nodes in both lists, and take time
    nodes = len(open_nodes) + len(closed_nodes) + 2
    end = time.time()

    print("The puzzle has been solved in", moves, "moves.")
    print("There has been created", nodes, "nodes")
    print("It took", round(end - start, 2), "seconds.")


def init_board():
    # Create numpy array from 0 to 9, 0 being the empty space
    arr = np.arange(9)
    # Shuffle the array
    np.random.shuffle(arr)
    # if board is unsolvable, shuffle until board is solvable
    while is_solvable(arr) is False:
        np.random.shuffle(arr)
    # Rearrange the array to a matrix shape
    b = arr.reshape(3, 3)
    return b


# it is important that the puzzle is actually solvable
# this can be figured out by counting how many inversions there are in the puzzle
# if this number is odd, the puzzle is unsolvable, and if even then it is solvable
# ...   Doesn't actually work every time :(    ...
def is_solvable(b):
    inversions = 0
    for i in range(len(b)):
        for j in range(1, len(b)):
            if b[j] > b[i]:
                inversions += 1
    return True if inversions % 2 == 1 else False


# Exchange two tiles, 0 and a neighbour of 0, and return new board or "state"
def move(b, d):  # b = board, d = direction
    x, y = 0, 0
    bb = np.copy(b)

    # get the coordinates of the "empty" piece, 0. Returns array with coordinates.
    z = np.where(b == 0)

    # Check what input is, and give a move direction, x is column, y is row position
    if d == "w":
        # Check if the move is valid
        if z[0] != 2:
            x, y = 0, 1
        else:
            move(b, input("Invalid move:"))
    elif d == "a":
        if z[1] != 2:
            x, y = 1, 0
        else:
            move(b, input("Invalid move:"))
    elif d == "s":
        if z[0] != 0:
            x, y = 0, -1
        else:
            move(b, input("Invalid move:"))
    elif d == "d":
        if z[1] != 0:
            x, y = -1, 0
        else:
            move(b, input("Invalid move:"))
    else:
        move(b, input("Invalid move, WASD:"))

    # Exchange the two tiles
    bb[z], bb[z[0] + y, z[1] + x] = bb[z[0] + y, z[1] + x], bb[z]
    return bb


def finished(b, g_b):  # b = board "state" to be checked, g_b = goal board
    # np.array_equal returns True or False after checking if 2 arrays are equal
    return np.array_equal(b, g_b)


# Number of misplaced tiles. "Hamming distance"
def h1(b, g_b):
    return np.count_nonzero(g_b != b)  # - 1


# Sum of distances of misplaced tiles to goal. "Manhattan distance", i exclude 0 in calculations
def h2(b, g_b):
    distance = 0
    for y in range(3):
        for x in range(3):
            if g_b[y, x] != 0:
                n = np.where(b == g_b[y, x])
                distance += abs(n[0][0] - y) + abs(n[1][0] - x)
    return distance


# Check if we have already seen the state of a node
def state_is_in(state, visited_nodes):
    for visited_node in visited_nodes:
        if np.array_equal(state, visited_node.state):
            return True
    return False


# Possible directions, checks for edges, and excludes a direction if parent came from there
def pos_dir(b, lm="0"):  # lm = last move
    z = np.where(b == 0)
    move_list = []
    if z[0][0] != 2 and lm != "s":
        move_list.append("w")

    if z[0][0] != 0 and lm != "w":
        move_list.append("s")

    if z[1][0] != 2 and lm != "d":
        move_list.append("a")

    if z[1][0] != 0 and lm != "a":
        move_list.append("d")

    return move_list


# Prints the final node and all its parent nodes in reverse, and returns how many moves it took
def node_path_print(node, moves=0):
    if node.parent is not None:
        moves = node_path_print(node.parent, moves=moves+1)
    print(node.state)
    print(node.f)
    print()
    return moves


main()
