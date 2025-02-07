import problem
from node import Node
from priority_queue import PriorityQueue
import time
import math

class SearchTimeOutError(Exception):
    pass

def compute_g(algorithm, node):
    """
    Computes the g() value based on the algorithm type.
    """
    if algorithm in {"astar", "ucs", "custom-astar"}:
        return node.get_total_action_cost() if node else 0
    elif algorithm == "bfs":
        return node.get_depth()
    elif algorithm == "gbfs":
        return 0
    raise ValueError("Unknown algorithm type")

def compute_h(algorithm, node, goal_state):
    """
    Computes the h() value based on the algorithm type.
    """
    if algorithm in {"bfs", "ucs"}:
        return 0
    elif algorithm in {"astar", "gbfs"}:
        return get_manhattan_distance(node.get_state(), goal_state)
    elif algorithm == "custom-astar":
        return get_custom_heuristic(node.get_state(), goal_state)
    raise ValueError("Unknown algorithm type")

def get_manhattan_distance(from_state, to_state):
    return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

def get_custom_heuristic(from_state, to_state):
    x_diff, y_diff = abs(from_state.x - to_state.x), abs(from_state.y - to_state.y)
    return (x_diff + y_diff) + ((math.sqrt(2) - 2) * min(x_diff, y_diff))

def get_plan(goal_node):
    """
    Constructs the action sequence from the goal node back to the start.
    """
    actions = []
    while goal_node and goal_node.get_parent():
        actions.append(goal_node.get_action())
        goal_node = goal_node.get_parent()
    return list(reversed(actions))

def run_algorithm(algorithm, helper, priority_queue, goal_state):
    """
    Executes the search algorithm.
    """
    visited_states = {}
    total_nodes_expanded = 0
    
    while not priority_queue.is_empty():
        current_node = priority_queue.pop()
        current_state = current_node.get_state()
        
        if current_state == goal_state or helper.is_goal_state(current_state):
            return get_plan(current_node), total_nodes_expanded
        
        if current_state not in visited_states:
            visited_states[current_state] = True
            successors = helper.get_successor(current_state)
            
            if successors:
                total_nodes_expanded += 1
                for action, (state, cost) in successors.items():
                    if state.get_x() >= 0.0 and state.get_y() >= 0.0:
                        state_node = Node(state, current_node, current_node.get_depth() + 1, action, cost)
                        g_value = compute_g(algorithm, state_node)
                        h_value = compute_h(algorithm, state_node, goal_state)
                        priority_queue.push(g_value + h_value, state_node)
    
    return [], total_nodes_expanded

def graph_search(algorithm, time_limit):
    """
    Performs a search using the specified algorithm.
    """
    helper = problem.Helper()
    init_state, goal_state = helper.get_initial_state(), helper.get_goal_state()[0]
    
    init_node = Node(init_state, None, 0, None, 0)
    f_score = compute_g(algorithm, init_node) + compute_h(algorithm, init_node, goal_state)
    
    priority_queue = PriorityQueue()
    priority_queue.push(f_score, init_node)
    
    end_time = time.time() + time_limit
    action_list, total_nodes_expanded = run_algorithm(algorithm, helper, priority_queue, goal_state)
    
    if time.time() >= end_time:
        raise SearchTimeOutError(f"Search timed out after {time_limit} seconds.")
    
    return action_list, total_nodes_expanded
