from tkinter import *
import sys
import os.path
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from search import *
from search import astar_search as asts
from utils import PriorityQueue
from copy import deepcopy

root = None
city_coord = {}
romania_problem = None
algo = None
start = None
goal = None
counter = -1
city_map = None
frontier = None
front = None
node = None
next_button = None
explored = None


def create_map(root):
    """This function draws out the required map."""
    global city_map, start, goal
    region_locations = map_region.locations
    width = 780
    height = 600
    margin = 5
    city_map = Canvas(root, width=width, height=height)
    city_map.pack()

    # Since lines have to be drawn between particular points, we need to list
    # them separately
    make_line(
        city_map,
        region_locations['SICEPAT'][0],
        height -
        region_locations['SICEPAT'][1],
        region_locations['Hutahaean'][0],
        height -
        region_locations['Hutahaean'][1],
        map_region.get('SICEPAT', 'Hutahaean'))
    make_line(
        city_map,
        region_locations['Hutapea'][0],
        height -
        region_locations['Hutapea'][1],
        region_locations['Tampubolon'][0],
        height -
        region_locations['Tampubolon'][1],
        map_region.get('Hutapea', 'Tampubolon'))
    make_line(
        city_map,
        region_locations['Purba'][0],
        height -
        region_locations['Purba'][1],
        region_locations['Hutapea'][0],
        height -
        region_locations['Hutapea'][1],
        map_region.get('Purba', 'Hutapea'))
    make_line(
        city_map,
        region_locations['SICEPAT'][0],
        height -
        region_locations['SICEPAT'][1],
        region_locations['Ginting'][0],
        height -
        region_locations['Ginting'][1],
        map_region.get('SICEPAT', 'Ginting'))
    make_line(
        city_map,
        region_locations['Sintong'][0],
        height -
        region_locations['Sintong'][1],
        region_locations['Hutapea'][0],
        height -
        region_locations['Hutapea'][1],
        map_region.get('Sintong', 'Hutapea'))
    make_line(
        city_map,
        region_locations['Sintong'][0],
        height -
        region_locations['Sintong'][1],
        region_locations['Manullang'][0],
        height -
        region_locations['Manullang'][1],
        map_region.get('Sintong', 'Manullang'))
    make_line(
        city_map,
        region_locations['Sintong'][0],
        height -
        region_locations['Sintong'][1],
        region_locations['Siadari'][0],
        height -
        region_locations['Siadari'][1],
        map_region.get('Sintong', 'Siadari'))
    make_line(
        city_map,
        region_locations['Hutahaean'][0],
        height -
        region_locations['Hutahaean'][1],
        region_locations['Harianja'][0],
        height -
        region_locations['Harianja'][1],
        map_region.get('Hutahaean', 'Harianja'))
    make_line(
        city_map,
        region_locations['Hutahaean'][0],
        height -
        region_locations['Hutahaean'][1],
        region_locations['Pasaribu'][0],
        height -
        region_locations['Pasaribu'][1],
        map_region.get('Hutahaean', 'Pasaribu'))
    make_line(
        city_map,
        region_locations['Pasaribu'][0],
        height -
        region_locations['Pasaribu'][1],
        region_locations['Simamora'][0],
        height -
        region_locations['Simamora'][1],
        map_region.get('Pasaribu', 'Simamora'))
    make_line(
        city_map,
        region_locations['Pasaribu'][0],
        height -
        region_locations['Pasaribu'][1],
        region_locations['Purba'][0],
        height -
        region_locations['Purba'][1],
        map_region.get('Pasaribu', 'Purba'))
    make_line(
        city_map,
        region_locations['Pasaribu'][0],
        height -
        region_locations['Pasaribu'][1],
        region_locations['Sibarani'][0],
        height -
        region_locations['Sibarani'][1],
        map_region.get('Pasaribu', 'Sibarani'))
    make_line(
        city_map,
        region_locations['Sibarani'][0],
        height -
        region_locations['Sibarani'][1],
        region_locations['Sidabutar'][0],
        height -
        region_locations['Sidabutar'][1],
        map_region.get('Sibarani', 'Sidabutar'))
    make_line(
        city_map,
        region_locations['Sidabutar'][0],
        height -
        region_locations['Sidabutar'][1],
        region_locations['Pangaribuan'][0],
        height -
        region_locations['Pangaribuan'][1],
        map_region.get('Sidabutar', 'Pangaribuan'))
    make_line(
        city_map,
        region_locations['Pangaribuan'][0],
        height -
        region_locations['Pangaribuan'][1],
        region_locations['Saragih'][0],
        height -
        region_locations['Saragih'][1],
        map_region.get('Pangaribuan', 'Saragih'))
    make_line(
        city_map,
        region_locations['Saragih'][0],
        height -
        region_locations['Saragih'][1],
        region_locations['Girsang'][0],
        height -
        region_locations['Girsang'][1],
        map_region.get('Saragih', 'Girsang'))
    make_line(
        city_map,
        region_locations['Harianja'][0],
        height -
        region_locations['Harianja'][1],
        region_locations['Purba'][0],
        height -
        region_locations['Purba'][1],
        map_region.get('Harianja', 'Purba'))
    make_line(
        city_map,
        region_locations['Harianja'][0],
        height -
        region_locations['Harianja'][1],
        region_locations['Sitorus'][0],
        height -
        region_locations['Sitorus'][1],
        map_region.get('Harianja', 'Sitorus'))
    make_line(
        city_map,
        region_locations['Simamora'][0],
        height -
        region_locations['Simamora'][1],
        region_locations['Hutapea'][0],
        height -
        region_locations['Hutapea'][1],
        map_region.get('Simamora', 'Hutapea'))
    make_line(
        city_map,
        region_locations['Simamora'][0],
        height -
        region_locations['Simamora'][1],
        region_locations['Situmorang'][0],
        height -
        region_locations['Situmorang'][1],
        map_region.get('Simamora', 'Situmorang'))
    make_line(
        city_map,
        region_locations['Situmorang'][0],
        height -
        region_locations['Situmorang'][1],
        region_locations['Silaen'][0],
        height -
        region_locations['Silaen'][1],
        map_region.get('Situmorang', 'Silaen'))
    make_line(
        city_map,
        region_locations['Silaen'][0],
        height -
        region_locations['Silaen'][1],
        region_locations['Damanik'][0],
        height -
        region_locations['Damanik'][1],
        map_region.get('Silaen', 'Damanik'))
    make_line(
        city_map,
        region_locations['Silaen'][0],
        height -
        region_locations['Silaen'][1],
        region_locations['Sinaga'][0],
        height -
        region_locations['Sinaga'][1],
        map_region.get('Silaen', 'Sinaga'))
    make_line(
        city_map,
        region_locations['Damanik'][0],
        height -
        region_locations['Damanik'][1],
        region_locations['Siagian'][0],
        height -
        region_locations['Siagian'][1],
        map_region.get('Damanik', 'Siagian'))
    make_line(
        city_map,
        region_locations['Siagian'][0],
        height -
        region_locations['Siagian'][1],
        region_locations['Panjaitan'][0],
        height -
        region_locations['Panjaitan'][1],
        map_region.get('Siagian', 'Panjaitan'))
    make_line(
        city_map,
        region_locations['Panjaitan'][0],
        height -
        region_locations['Panjaitan'][1],
        region_locations['Pakpahan'][0],
        height -
        region_locations['Pakpahan'][1],
        map_region.get('Panjaitan', 'Pakpahan'))
    make_line(
        city_map,
        region_locations['Sinaga'][0],
        height -
        region_locations['Sinaga'][1],
        region_locations['Silaban'][0],
        height -
        region_locations['Silaban'][1],
        map_region.get('Sinaga', 'Silaban'))
    make_line(
        city_map,
        region_locations['Sinaga'][0],
        height -
        region_locations['Sinaga'][1],
        region_locations['Sitepu'][0],
        height -
        region_locations['Sitepu'][1],
        map_region.get('Sinaga', 'Sitepu'))
    make_line(
        city_map,
        region_locations['Silaban'][0],
        height -
        region_locations['Silaban'][1],
        region_locations['Napitupulu'][0],
        height -
        region_locations['Napitupulu'][1],
        map_region.get('Silaban', 'Napitupulu'))
    make_line(
        city_map,
        region_locations['Sintong'][0],
        height -
        region_locations['Sintong'][1],
        region_locations['Tampubolon'][0],
        height -
        region_locations['Tampubolon'][1],
        map_region.get('Sintong', 'Tampubolon'))
    make_line(
        city_map,
        region_locations['Tampubolon'][0],
        height -
        region_locations['Tampubolon'][1],
        region_locations['Manurung'][0],
        height -
        region_locations['Manurung'][1],
        map_region.get('Tampubolon', 'Manurung'))
    make_line(
        city_map,
        region_locations['Sitepu'][0],
        height -
        region_locations['Sitepu'][1],
        region_locations['Manullang'][0],
        height -
        region_locations['Manullang'][1],
        map_region.get('Sitepu', 'Manullang'))
    make_line(
        city_map,
        region_locations['Manullang'][0],
        height -
        region_locations['Manullang'][1],
        region_locations['Manurung'][0],
        height -
        region_locations['Manurung'][1],
        map_region.get('Manullang', 'Manurung'))
    make_line(
        city_map,
        region_locations['Manullang'][0],
        height -
        region_locations['Manullang'][1],
        region_locations['Hutajulu'][0],
        height -
        region_locations['Hutajulu'][1],
        map_region.get('Manullang', 'Hutajulu'))
    make_line(
        city_map,
        region_locations['Hutajulu'][0],
        height -
        region_locations['Hutajulu'][1],
        region_locations['Simarmata'][0],
        height -
        region_locations['Simarmata'][1],
        map_region.get('Hutajulu', 'Simarmata'))
    make_line(
        city_map,
        region_locations['Girsang'][0],
        height -
        region_locations['Girsang'][1],
        region_locations['Panjaitan'][0],
        height -
        region_locations['Panjaitan'][1],
        map_region.get('Girsang', 'Panjaitan'))
    make_line(
        city_map,
        region_locations['Manurung'][0],
        height -
        region_locations['Manurung'][1],
        region_locations['Simarmata'][0],
        height -
        region_locations['Simarmata'][1],
        map_region.get('Manurung', 'Simarmata'))
    make_line(
        city_map,
        region_locations['Pakpahan'][0],
        height -
        region_locations['Pakpahan'][1],
        region_locations['Pandingan'][0],
        height -
        region_locations['Pandingan'][1],
        map_region.get('Pakpahan', 'Pandingan'))
    make_line(
        city_map,
        region_locations['Pandingan'][0],
        height -
        region_locations['Pandingan'][1],
        region_locations['Lumbantoruan'][0],
        height -
        region_locations['Lumbantoruan'][1],
        map_region.get('Pandingan', 'Lumbantoruan'))
    make_line(
        city_map,
        region_locations['Napitupulu'][0],
        height -
        region_locations['Napitupulu'][1],
        region_locations['Sitinjak'][0],
        height -
        region_locations['Sitinjak'][1],
        map_region.get('Napitupulu', 'Sitinjak'))
    make_line(
        city_map,
        region_locations['Lumbantoruan'][0],
        height -
        region_locations['Lumbantoruan'][1],
        region_locations['Sitinjak'][0],
        height -
        region_locations['Sitinjak'][1],
        map_region.get('Lumbantoruan', 'Sitinjak'))
    make_line(
        city_map,
        region_locations['Sitinjak'][0],
        height -
        region_locations['Sitinjak'][1],
        region_locations['Hutajulu'][0],
        height -
        region_locations['Hutajulu'][1],
        map_region.get('Sitinjak', 'Hutajulu'))
    make_line(
        city_map,
        region_locations['Siadari'][0],
        height -
        region_locations['Siadari'][1],
        region_locations['Manurung'][0],
        height -
        region_locations['Manurung'][1],
        map_region.get('Siadari', 'Manurung'))
    make_line(
        city_map,
        region_locations['Simamora'][0],
        height -
        region_locations['Simamora'][1],
        region_locations['Sintong'][0],
        height -
        region_locations['Sintong'][1],
        map_region.get('Simamora', 'Sintong'))
    make_line(
        city_map,
        region_locations['Ginting'][0],
        height -
        region_locations['Ginting'][1],
        region_locations['Sitorus'][0],
        height -
        region_locations['Sitorus'][1],
        map_region.get('Ginting', 'Sitorus'))
    make_line(
        city_map,
        region_locations['Ginting'][0],
        height -
        region_locations['Ginting'][1],
        region_locations['Sianturi'][0],
        height -
        region_locations['Sianturi'][1],
        map_region.get('Ginting', 'Sianturi'))
    make_line(
        city_map,
        region_locations['Sianturi'][0],
        height -
        region_locations['Sianturi'][1],
        region_locations['Aruan'][0],
        height -
        region_locations['Aruan'][1],
        map_region.get('Sianturi', 'Aruan'))
    make_line(
        city_map,
        region_locations['Aruan'][0],
        height -
        region_locations['Aruan'][1],
        region_locations['Sitohang'][0],
        height -
        region_locations['Sitohang'][1],
        map_region.get('Aruan', 'Sitohang'))
    make_line(
        city_map,
        region_locations['Sitohang'][0],
        height -
        region_locations['Sitohang'][1],
        region_locations['Lubis'][0],
        height -
        region_locations['Lubis'][1],
        map_region.get('Sitohang', 'Lubis'))
    make_line(
        city_map,
        region_locations['Lubis'][0],
        height -
        region_locations['Lubis'][1],
        region_locations['Saragih'][0],
        height -
        region_locations['Saragih'][1],
        map_region.get('Lubis', 'Saragih'))




    for city in region_locations.keys():
        make_rectangle(
            city_map,
            region_locations[city][0],
            height -
            region_locations[city][1],
            margin,
            city)

    make_legend(city_map)


def make_line(map, x0, y0, x1, y1, distance):
    """This function draws out the lines joining various points."""
    map.create_line(x0, y0, x1, y1)
    map.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=distance)


def make_rectangle(map, x0, y0, margin, city_name):
    """This function draws out rectangles for various points."""
    global city_coord
    rect = map.create_rectangle(
        x0 - margin,
        y0 - margin,
        x0 + margin,
        y0 + margin,
        fill="white")
    if "Ginting" in city_name or "Hutajulu" in city_name or "Silaen" in city_name \
            or "Damanik" in city_name or "Siagian" in city_name:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=E)
    else:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=SE)
    city_coord.update({city_name: rect})


def make_legend(map):

    rect1 = map.create_rectangle(600, 100, 610, 110, fill="white")
    text1 = map.create_text(615, 105, anchor=W, text="Un-explored")

    rect2 = map.create_rectangle(600, 115, 610, 125, fill="orange")
    text2 = map.create_text(615, 120, anchor=W, text="Frontier")

    rect3 = map.create_rectangle(600, 130, 610, 140, fill="red")
    text3 = map.create_text(615, 135, anchor=W, text="Currently Exploring")

    rect4 = map.create_rectangle(600, 145, 610, 155, fill="grey")
    text4 = map.create_text(615, 150, anchor=W, text="Explored")

    rect5 = map.create_rectangle(600, 160, 610, 170, fill="dark green")
    text5 = map.create_text(615, 165, anchor=W, text="Final Solution")


def tree_search(problem):
    """
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Don't worry about repeated paths to a state. [Figure 3.7]
    This function has been changed to make it suitable for the Tkinter GUI.
    """
    global counter, frontier, node

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):

            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:

        display_explored(node)
    return None


def graph_search(problem):
    """
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Figure 3.7]
    This function has been changed to make it suitable for the Tkinter GUI.
    """
    global counter, frontier, node, explored
    if counter == -1:
        frontier.append(Node(problem.initial))
        explored = set()

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def display_frontier(queue):
    """This function marks the frontier nodes (orange) on the map."""
    global city_map, city_coord
    qu = deepcopy(queue)
    while qu:
        node = qu.pop()
        for city in city_coord.keys():
            if node.state == city:
                city_map.itemconfig(city_coord[city], fill="orange")


def display_current(node):
    """This function marks the currently exploring node (red) on the map."""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="red")


def display_explored(node):
    """This function marks the already explored node (gray) on the map."""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="gray")


def display_final(cities):
    """This function marks the final solution nodes (green) on the map."""
    global city_map, city_coord
    for city in cities:
        city_map.itemconfig(city_coord[city], fill="green")


def breadth_first_tree_search(problem):
    """Search the shallowest nodes in the search tree first."""
    global frontier, counter, node
    if counter == -1:
        frontier = deque()

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.popleft()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def depth_first_tree_search(problem):
    """Search the deepest nodes in the search tree first."""
    # This search algorithm might not work in case of repeated paths.
    global frontier, counter, node
    if counter == -1:
        frontier = []  # stack

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def breadth_first_graph_search(problem):
    """[Figure 3.11]"""
    global frontier, node, explored, counter
    if counter == -1:
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node

        frontier = deque([node])  # FIFO queue

        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.popleft()
        display_current(node)
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def depth_first_graph_search(problem):
    """Search the deepest nodes in the search tree first."""
    global counter, frontier, node, explored
    if counter == -1:
        frontier = []  # stack
    if counter == -1:
        frontier.append(Node(problem.initial))
        explored = set()

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def best_first_graph_search(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    global frontier, node, explored, counter

    if counter == -1:
        f = memoize(f, 'f')
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node
        frontier = PriorityQueue('min', f)
        frontier.append(node)
        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        display_current(node)
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None

def astar_search(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))


# TODO:
# Remove redundant code.
# Make the interchangbility work between various algorithms at each step.
def on_click():
    """
    This function defines the action of the 'Next' button.
    """
    global algo, counter, next_button, region_problem, start, goal
    region_problem = GraphProblem(start.get(), goal.get(), map_region)
    if "A* - Search" == algo.get():
        node = astar_search(region_problem)
        if node is not None:
            final_path = asts(region_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1


def reset_map():
    global counter, city_coord, city_map, next_button
    counter = -1
    for city in city_coord.keys():
        city_map.itemconfig(city_coord[city], fill="white")
    next_button.config(state="normal")

# TODO: Add more search algorithms in the OptionMenu


def main():
    global algo, start, goal, next_button
    root = Tk()
    root.title("Route Finding Kurir SICEPAT")
    root.geometry("1080x1950")
    algo = StringVar(root)
    start = StringVar(root)
    goal = StringVar(root)
    algo.set("A* - Search")
    start.set('SICEPAT')
    goal.set('Hutahaean')
    cities = sorted(map_region.locations.keys())
    algorithm_menu = OptionMenu(
        root,
        algo, "A* - Search")
    Label(root, text="\n Search Algorithm").pack()
    algorithm_menu.pack()
    Label(root, text="\n Start City").pack()
    start_menu = OptionMenu(root, start, "SICEPAT")
    start_menu.pack()
    Label(root, text="\n Goal City").pack()
    goal_menu = OptionMenu(root, goal, *cities)
    goal_menu.pack()
    frame1 = Frame(root)
    next_button = Button(
        frame1,
        width=4,
        height=1,
        text="Next",
        command=on_click,
        padx=2,
        pady=2,
        relief=GROOVE)
    next_button.pack(side=RIGHT)
    reset_button = Button(
        frame1,
        width=4,
        height=1,
        text="Reset",
        command=reset_map,
        padx=2,
        pady=2,
        relief=GROOVE)
    reset_button.pack(side=RIGHT)
    frame1.pack(side=BOTTOM)
    create_map(root)
    root.mainloop()


if __name__ == "__main__":
    main()
