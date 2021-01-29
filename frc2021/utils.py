DUBINS_HEADER = [
    "time",
    "state_x",
    "state_y",
    "state_t",
    "input_v",
    "input_w",
    "acc",
    "pose_x",
    "pose_y",
    "pose_z",
    "splines_vtil",
    "splines_tg_ha",
]
HOLONOMIC_HEADER = [
    "time",
    "state_x",
    "state_y",
    "input_x",
    "input_y",
    "dinput_x",
    "dinput_y",
    "v_tot",
    "pose_x",
    "pose_y",
    "pose_z",
    "splines_x",
    "splines_y",
]


def json_to_csv(js, dubins):
    return (
        [
            js["time"][0],
            js["state"][0],
            js["state"][1],
            js["state"][2],
            js["input"][0],
            js["input"][1],
            js["acc"][0],
            js["pose"][0],
            js["pose"][1],
            js["pose"][2],
            js["splines"][0],
            js["splines"][1],
        ]
        if dubins
        else [
            js["time"][0],
            js["state"][0],
            js["state"][1],
            js["input"][0],
            js["input"][1],
            js["dinput"][0],
            js["dinput"][1],
            js["v_tot"][0],
            js["pose"][0],
            js["pose"][1],
            js["pose"][2],
            js["splines"][0],
            js["splines"][1],
        ]
    )


def transpose(csv):
    return [[csv[j][i] for j in range(len(csv))] for i in range(len(csv[0]))]


def convert_json_to_csv(ifile, ofile, dubins=True):
    import json
    import csv

    with open(ifile) as f:
        js = json.load(f)
    rows = transpose(json_to_csv(js, dubins))
    with open(ofile, "w") as f:
        writer = csv.writer(f)
        if dubins:
            writer.writerow(DUBINS_HEADER)
        else:
            writer.writerow(HOLONOMIC_HEADER)

        writer.writerows(rows)


# unused - setup for holonomic
# def append_csvs(ifiles, ofile):
#     import csv
#     csvs = []
#     for ifile in ifiles:
#         with open(ifile) as f:
#             reader = csv.reader(f)
#             header = reader.__next__()
#             lines = [[float(i) for i in line] for line in reader]
#             csvs.append(lines)
#
#     # one quick pass to fix time
#     base_time = 0
#     for i in range(len(csvs)):
#         for line in csvs[i]:
#             line[0] += base_time
#         base_time = csvs[i][-1][0]
#
#     total = []
#     for lines in csvs:
#         total.extend(lines)
#
#     with open(ofile, 'w') as f:
#         writer = csv.writer(f)
#         writer.writerow(["time", "state_x", "state_y", "state_t", "input_v", "input_w", "acc", "pose_x", "pose_y", "pose_z", "splines_x", "splines_y"])
#         writer.writerows(total)

if __name__ == "__main__":
    import sys

    if sys.argv[1].lower() in ("dubins", "holonomic"):
        convert_json_to_csv(sys.argv[2], sys.argv[3], sys.argv[1].lower() == "dubins")
    else:
        print("format: utils.py [dubins|holonomic] <input> <output>")
