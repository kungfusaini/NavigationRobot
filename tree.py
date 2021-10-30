#!/usr/bin/env python
from parameters import *

goal = ()


# I contributed to the building of the following helper functions
# Sets a goal
def set_goal(x, y):
    global goal
    goal = (x, y)


# Retrieves the current goal
def get_goal():
    return goal


# Calculate heuristics based on the manhattan distance to the goal
def calculate_heuristics(x, y):
    return abs(x - goal[0]) + abs(y - goal[1])


# I proposed the making of the following class
# I build the skeleton for it and I was then helped to write
# methods which allow for comparison
class Node:
    def __init__(self, x, y, g, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = calculate_heuristics(x, y)
        self.f = g + self.h
        self.parent = parent
        self.children = []

        if parent is not None:
            parent.add_children(self)

    def add_children(self, children):
        self.children.append(children)

    # '==' on two nodes compares their X and Y position
    # This also works for tuples
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        elif isinstance(other, tuple):
            return self.x == other[0] and self.y == other[1]
        return False

    # '!=' returns inverse of '==' comparison
    def __ne__(self, other):
        return not self.__eq__(other)

    # '<' check against the f value (h + g)
    def __lt__(self, other):
        return other.f > self.f

    # Overloads the printing method
    def __str__(self):
        return 'Node: {}, {}'.format(self.x, self.y)

    # Implement hashes to allow for checking if nodes are in a list
    def __hash__(self):
        return hash((self.x, self.y))
