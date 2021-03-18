"""Pulls path info out of the points as opposed to the true spline and then prints out the code needed to add the waypoints to the list"""

# Based from pwd of your terminal (or absolute)
filepath = "src\\main\\auxiliary\\paths\\SlalomPath.path"
with open(filepath, 'r') as f:
    for i, line in enumerate(f):
        if i != 0:
            split_line = line.split(",")
            print("waypoints.add(new Translation2d({0}, {1}));".format(split_line[0], split_line[1]))