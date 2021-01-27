import math
import pprint


def dist(poseA, poseB):
    x = poseA[1] - poseB[1]
    y = poseA[2] - poseB[2]
    return math.sqrt(x ** 2 + y ** 2)


def curv(poseA, poseB, poseC):
    area = (
        0.5 * poseA[1] * (poseB[2] - poseC[2])
        + poseB[1] * (poseC[2] - poseA[2])
        + poseC[1] * (poseA[2] - poseB[2])
    )
    dists = dist(poseA, poseB) * dist(poseB, poseC) * dist(poseA, poseC)
    if dists == 0:
        return 0
    return 4.0 * area / dists
    pass


def limitAccel(rev, constraints, constrainedState):
    factor = -1 if rev else 1
    for constraint in constraints:
        mma = constraint["mma"](
            constrainedState["pose"],
            constrainedState["curv"],
            constrainedState["maxVelocity"] * factor,
        )
        if mma[0] > mma[1]:
            raise Exception(
                "This constraint's min accel is greater than its max accel!"
            )

        constrainedState["minAcceleration"] = max(
            constrainedState["minAcceleration"], -mma[1] if rev else mma[0]
        )
        constrainedState["maxAcceleration"] = max(
            constrainedState["maxAcceleration"], -mma[0] if rev else mma[1]
        )


"""
a constraint is a dictionary of lambdas
{
    "mma": lambda pose, curv, maxVel: {},
    "mv": lambda pose, curv, maxVel: {}
}
"""


def timeParameterizeTrajectory(poses, constraints, sv, ev, mv, ma, rev):
    constrainedStates = [
        {
            "pose": pose,
            "curv": 0,
            "distance": 0,
            "maxVelocity": 0,
            "minAcceleration": 0,
            "maxAcceleration": 0,
        }
        for pose in poses
    ]

    constrainedStates[0]["maxVelocity"] = sv
    constrainedStates[0]["maxAcceleration"] = ma
    constrainedStates[0]["minAcceleration"] = -ma

    # first, do a pass to determine curvature at each point
    # it's imperfect because we just have points to work with but it'll do for now.
    # note: bounce needs to be 4 separate trajectories because of this
    for i in range(len(constrainedStates)):
        if i == 0 or i == len(constrainedStates) - 1:
            continue
        constrainedStates[i]["curv"] = curv(
            constrainedStates[i - 1]["pose"],
            constrainedStates[i]["pose"],
            constrainedStates[i + 1]["pose"],
        )

    predecessor = constrainedStates[0]

    for i in range(len(constrainedStates)):
        constrainedState = constrainedStates[i]
        distance = dist(constrainedState["pose"], predecessor["pose"])
        constrainedState["distance"] = distance + predecessor["distance"]

        while True:
            constrainedState["maxVelocity"] = min(
                mv,
                math.sqrt(
                    predecessor["maxVelocity"] ** 2
                    + 2.0 * predecessor["maxAcceleration"] * distance
                ),
            )
            constrainedState["minAcceleration"] = -ma
            constrainedState["maxAcceleration"] = ma

            for constraint in constraints:
                constrainedState["maxVelocity"] = min(
                    constrainedState["maxVelocity"],
                    constraint["mv"](
                        constrainedState["pose"],
                        constrainedState["curv"],
                        constrainedState["maxVelocity"],
                    ),
                )

            limitAccel(rev, constraints, constrainedState)

            if distance < 1e-6:
                break

            actualAccel = (
                constrainedState["maxVelocity"] ** 2 - predecessor["maxVelocity"] ** 2
            ) / (2.0 * distance)

            if constrainedState["maxAcceleration"] < actualAccel - 1e-6:
                predecessor["maxAcceleration"] = constrainedState["maxAcceleration"]
            else:
                if actualAccel > predecessor["minAcceleration"] + 1e-6:
                    predecessor["maxAcceleration"] = actualAccel
                break
        predecessor = constrainedState

    print("finished forward pass")

    successor = {
        "pose": constrainedStates[-1]["pose"],
        "distance": constrainedStates[-1]["distance"],
        "maxVelocity": ev,
        "maxAcceleration": ma,
        "minAcceleration": -ma,
    }

    for i in range(len(constrainedStates)):
        i = len(constrainedStates) - 1 - i
        constrainedState = constrainedStates[i]
        distance = constrainedState["distance"] - successor["distance"]

        while True:
            newMax = math.sqrt(
                successor["maxVelocity"] ** 2
                + successor["minAcceleration"] * distance * 2.0
            )
            if newMax > constrainedState["maxVelocity"]:
                break

            constrainedState["maxVelocity"] = newMax

            limitAccel(rev, constraints, constrainedState)

            if distance > -1e-6:
                break

            actualAccel = (
                constrainedState["maxVelocity"] ** 2 - successor["maxVelocity"] ** 2
            ) / (2.0 * distance)

            if constrainedState["minAcceleration"] > actualAccel + 1e-6:
                successor["minAcceleration"] = constrainedState["minAcceleration"]
            else:
                successor["minAcceleration"] = actualAccel
                break
        successor = constrainedState

    print("finished backward pass")

    states = []
    time = 0
    distance = 0
    velocity = 0
    for i in range(len(constrainedStates)):
        state = constrainedStates[i]
        ds = state["distance"] - distance

        accel = (state["maxVelocity"] ** 2 - velocity ** 2) / (ds * 2) if ds > 0 else 0

        dt = 0
        if i > 0:
            states[i - 1]["accel"] = -accel if rev else accel
            if abs(accel) > 1e-6:
                dt = (state["maxVelocity"] - velocity) / accel
            elif abs(state["maxVelocity"]) > 1e-6 and ds > 0:
                dt = ds / velocity
            else:
                print("we've stopped at pose", i)
                break

                # raise Exception("Something went wrong! fuck")

        velocity = state["maxVelocity"]
        distance = state["distance"]

        time += dt

        states.append(
            {
                "time": time,
                "distance": distance,
                "velocity": -velocity if rev else velocity,
                "accel": -accel if rev else accel,
                "pose_time": state["pose"][0],
                "pose_state_x": state["pose"][1],
                "pose_state_y": state["pose"][2],
                "pose_state_w": state["pose"][3],
                "pose_input_v": state["pose"][4],
                "pose_input_w": state["pose"][5],
                "pose_acc": state["pose"][6],
                "pose_pose_x": state["pose"][7],
                "pose_pose_y": state["pose"][8],
                "pose_pose_z": state["pose"][9],
                "pose_splines_vtil": state["pose"][10],
                "pose_splines_tg_ha": state["pose"][11],
                "curv": state["curv"],
            }
        )
    print("finished integration")

    return states


if __name__ == "__main__":
    import sys
    import csv

    with open(sys.argv[1]) as f:
        rdr = csv.reader(f)
        rdr.__next__()
        lines = [[float(i) for i in line] for line in rdr]

    traj = timeParameterizeTrajectory(
        lines, [], 0, 0, float(sys.argv[3]), float(sys.argv[4]), False
    )

    with open(sys.argv[2], "w") as f:
        wrtr = csv.DictWriter(
            f,
            [
                "time",
                "distance",
                "velocity",
                "accel",
                "pose_time",
                "pose_state_x",
                "pose_state_y",
                "pose_state_w",
                "pose_input_v",
                "pose_input_w",
                "pose_acc",
                "pose_pose_x",
                "pose_pose_y",
                "pose_pose_z",
                "pose_splines_vtil",
                "pose_splines_tg_ha",
                "curv",
            ],
        )
        wrtr.writeheader()
        wrtr.writerows(traj)
