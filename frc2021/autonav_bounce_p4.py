from casadi.casadi import sqrt
from omgtools import *
import os
import csv
import json
import sys

import numpy as np


def default(obj):
    if type(obj).__module__ == np.__name__:
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return obj.item()
    raise TypeError("Unknown type:", type(obj))


def parse_y(coord):
    return {"Z": 15, "A": 12.5, "B": 10, "C": 7.5, "D": 5, "E": 2.5, "F": 0}[coord]


def parse_x(coord):
    return coord * 2.5


def coord_to_pair(coord):
    if type(coord) == str:
        return [parse_x(float(coord[1:])), parse_y(coord[0])]
    else:
        return coord


def coords_to_bounding_box(p1, p2, draw=True):
    (x1, y1) = coord_to_pair(p1)
    (x2, y2) = coord_to_pair(p2)
    width = abs(x2 - x1)
    height = abs(y2 - y1)
    if width + height == 0:
        width = 2.5 / 12.0
        height = 2.5 / 12.0
    pos_x = (x1 + x2) / 2
    pos_y = (y1 + y2) / 2
    return {
        "shape": Rectangle(width=width, height=height),
        "position": [pos_x, pos_y],
        "draw": draw,
    }


def basic_rectangle(p1, p2):
    (x1, y1) = coord_to_pair(p1)
    (x2, y2) = coord_to_pair(p2)
    print(x1, y1, x2, y2)
    return ((min(x1, x2), max(y1, y2)), (max(x1, x2), min(y1, y2)))


def check_overlap(first, second):
    (l1, r1) = basic_rectangle(*first)
    (l2, r2) = basic_rectangle(*second)

    if l1[0] >= r2[0] or l2[0] >= r1[0]:
        return False

    if l1[1] <= r2[1] or l2[1] <= r1[1]:
        return False

    return True


def save(trajectories, signals, options):
    # save results for check in c++
    testdir = os.path.join(options["directory"], "test")
    if not os.path.isdir(testdir):
        os.makedirs(os.path.join(options["directory"], "test"))

    with open(os.path.join(testdir, options["name"] + "_trajectories.json"), "w") as f:
        json.dump(trajectories, f, default=default)
    with open(os.path.join(testdir, options["name"] + "_signals.json"), "w") as f:
        json.dump(signals, f, default=default)

    # jump = int(simulator.update_time / simulator.sample_time)
    # size = len(trajectories[vehicle.label]["state"])
    # with open(os.path.join(testdir, "data_state_" + options["name"] + "_signals.csv"), "w") as f:
    #     w = csv.writer(f)
    #     for i in range(0, size, jump):
    #         for k in range(trajectories["state"][i].shape[0]):
    #             w.writerow(trajectories["state"][i][k, :])
    # with open(os.path.join(testdir, "data_input_" + options["name"] + ".csv"), "w") as f:
    #     w = csv.writer(f)
    #     for i in range(0, size, jump):
    #         for k in range(trajectories[vehicle.label]["input"][i].shape[0]):
    #             w.writerow(trajectories[vehicle.label]["input"][i][k, :])


configs = {
    "Barrel Racing": {
        "path": ["C1", "C1"],
        "markers": ["B1", "B2", "B8", "D1", "D2", "D5", "D10"],
        "rooms": [
            ["B0", "D8"],
            ["B5", "F8"],
            ["D2", "F8"],
            ["B2", "F5"],
            ["B2", "D10"],
            ["Z8", "D10"],
            ["Z5", "B10"],
            ["Z5", "D8"],
            ["B5", "F10"],
            ["D8", "F12"],
            ["B10", "F12"],
            ["B0", "D12"],
        ],
    },
    "Slalom": {
        "path": ["E1", "C1"],
        "markers": ["B1", "B2", "D1", "D2", "D4", "D5", "D6", "D7", "D8", "D10"],
        "rooms": [
            ["D0", "F4"],
            ["B2", "F4"],
            ["B2", "D10"],
            ["B8", "F10"],
            ["D8", "F12"],
            ["B10", "F12"],
            ["B8", "D12"],
            ["B8", "F10"],
            ["D2", "F10"],
            ["B2", "F4"],
            ["B0", "D4"],
        ],
    },
    "Bounce": {
        "path": ["A9", "C11"],
        "markers": [
            Obstacle(
                {"position": coord_to_pair("A1")},
                shape=Rectangle(width=5, height=5),
            ),
            Obstacle(
                {"position": coord_to_pair("A4.5")},
                shape=Rectangle(width=2.5, height=5),
            ),
            Obstacle(
                {"position": coord_to_pair("B7.5")},
                shape=Rectangle(width=2.5, height=10),
            ),
            Obstacle(
                {"position": coord_to_pair("A11")},
                shape=Rectangle(width=5, height=5),
            ),
            Obstacle(
                {"position": coord_to_pair("C5")},
                shape=Rectangle(width=2.5 / 12, height=5),
            ),
            Obstacle(
                {"position": coord_to_pair("E11")},
                shape=Rectangle(width=5, height=5),
            ),
            Obstacle(
                {"position": coord_to_pair("E1.5")},
                shape=Rectangle(width=7.5, height=5),
            ),
        ],
        "rooms": [
            ["Z8", "F12"],
        ],
    },
}

# Create the vehicle instance
vehicle = Holonomic(
    shapes=Square(2),
    options={
        "syslimit": "norm_2",
        "safety_distance": 0.2,
        "stop_tol": 1.0e-2,
        "safety_weight": 10.0,
        "room_constraints": True,
        "ideal_prediction": False,
        "ideal_update": False,
        "1storder_delay": False,
        "time_constant": 0.01,
        "input_disturbance": None,
    },
    bounds={"vmax": 18, "vmin": -18, "amax": 54, "amin": -54},
)

vehicle = Dubins(
    shapes=Square(2),
    options={
        "safety_distance": 0.0,
        "safety_weight": 10.0,
        "room_constraints": True,
        "stop_tol": 1.0e-3,
        "ideal_prediction": False,
        "ideal_update": False,
        "1storder_delay": False,
        "time_constant": 0.1,
        "input_disturbance": None,
        "stop_tol": 1.0e-2,
        "substitution": False,
        "exact_substitution": False,
    },
    bounds={"vmax": 18, "amax": 54, "amin": -54, "wmin": -np.pi, "wmax": np.pi},
)

# We provide our vehicle with a desired initial and terminal position:

# this specific impl is for Barrel Racing

path = "Bounce"

rooms = [coords_to_bounding_box(*pair) for pair in configs[path]["rooms"]]

# for i in range(len(configs[path]["rooms"]) - 1):
#     print(i, configs[path]["rooms"][i], '->', configs[path]["rooms"][i+1], '@', rooms[i]['position'], rooms[i+1]['position'])
#     assert(check_overlap(configs[path]["rooms"][i], configs[path]["rooms"][i+1]))


# Now, we create an environment
# An environment is determined by a room with certain shape
environment = Environment(room=rooms)

for marker in configs[path]["markers"]:
    if type(marker) == str:
        square = Square(2.5 / 12.0)
        environment.add_obstacle(
            Obstacle({"position": coord_to_pair(marker)}, shape=square)
        )
    else:
        environment.add_obstacle(marker)
environment.init()

end = 1
vehicle.set_initial_conditions([*coord_to_pair(configs[path]["path"][end - 1]), 0])
vehicle.set_terminal_conditions([*coord_to_pair(configs[path]["path"][end]), np.pi/2])

print("Environment Setup.")

# do our special bounce shit
problem = Point2point(
    vehicle,
    environment,
    options={
        "verbose": 2,
        "solver": "ipopt",
        "solver_options": {
            "ipopt": {
                "ipopt.tol": 1e-3,
                "ipopt.warm_start_init_point": "yes",
                "ipopt.print_level": 0,
                "print_time": 0,
                "ipopt.fixed_variable_treatment": "make_constraint",
            }
        },
        "codegen": {"build": "jit", "flags": "-O2"},
    },
    freeT=True,
)
problem.init()

print("Problem Initialized.")

# simulate the problem
simulator = Simulator(problem)

# define what you want to plot
problem.plot("scene", knots=True, prediction=True)
vehicle.plot("state", knots=True, prediction=True, labels=["x (m)", "y (m)", "t (rad)"])
vehicle.plot("input", knots=True, prediction=True, labels=["v (m/s)", "w (rad/s)"])
# vehicle.plot(
#     "dinput", knots=True, prediction=True, labels=["a_s (m/s/s)", "a_r (rad/s/s)"]
# )

print("Simulator Setup.")

options = {}
options["directory"] = os.path.join(os.getcwd(), "export_f/")
# path to object files of your exported optimization problem
options["casadiobj"] = os.path.join(options["directory"], "bin/")
options["namespace"] = "omgf"
options["name"] = path + "p4"

simulator.run_once()

save(vehicle.traj_storage, vehicle.signals, options)
# 
# problem.plot_movie("scene", number_of_frames=200, repeat=False)
# problem.save_movie(
#     "scene",
#     format="gif",
#     name="bounce_p1",
#     number_of_frames=200,
#     movie_time=10,
#     axis=False,
# )
# 