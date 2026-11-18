# CEE201

import numpy as np
from scipy.optimize import linprog

# Number of cars in each parking lot
x1 = 1988
x2 = 2534
x3 = 487
x4 = 504
x5 = 515
x6 = 527
x7 = 795
x8 = 225
x9 = 671
x10 = 0
x11 = 153


# Total number of cars
C = x1 + x2 + x3 + x4 + x5 + x6 + x7 + x8 + x9 + x10 + x11

print("Total cars:", C)


k = x1 #change which lot you close for x1 through x11)
factor = C / (C - k)

x1_new = 0          #put 0 for the lot you close
x2_new = x2 * factor
x3_new = x3 * factor
x4_new = x4 * factor
x5_new = x5 * factor
x6_new = x6 * factor
x7_new = x7 * factor
x8_new = x8 * factor
x9_new = x9 * factor
x10_new = 0
x11_new = x11 * factor


# Defining link distances (d_ij)
d_E1_O1 = 782
d_O1_O2 = 532
d_O2_OK = 389
d_OK_E8 = 567
d_OK_O3 = 438
d_O3_O4 = 437
d_O4_OM = 429
d_OM_E7 = 345
d_F1_F2 = 361
d_F2_F3 = 121
d_F3_F4 = 447
d_F4_FK = 239
d_FK_OK = 800
d_FK_F5 = 430
d_F5_F6 = 438
d_F6_FM = 428
d_FM_OM = 812
d_FM_E6 = 384
d_FM_RM = 1467
d_RM_M1 = 368
d_M1_M2 = 161
d_M2_E5 = 777
d_RM_R3 = 265
d_R3_R2 = 213
d_R2_R1 = 361
d_R1_RK = 470
d_RK_K1 = 722
d_K1_E4 = 580
d_RK_RP = 890
d_RP_P1 = 340
d_P1_E3 = 930
d_RP_E2 = 822

# Defining the links
links = [
    ("E1", "O1", d_E1_O1),
    ("O1", "O2", d_O1_O2),
    ("O2", "OK", d_O2_OK),
    ("OK", "E8", d_OK_E8),
    ("OK", "O3", d_OK_O3),
    ("O3", "O4", d_O3_O4),
    ("O4", "OM", d_O4_OM),
    ("OM", "E7", d_OM_E7),
    ("F1", "F2", d_F1_F2),
    ("F2", "F3", d_F2_F3),
    ("F3", "F4", d_F3_F4),
    ("F4", "FK", d_F4_FK),
    ("FK", "OK", d_FK_OK),
    ("FK", "F5", d_FK_F5),
    ("F5", "F6", d_F5_F6),
    ("F6", "FM", d_F6_FM),
    ("FM", "OM", d_FM_OM),
    ("FM", "E6", d_FM_E6),
    ("FM", "RM", d_FM_RM),
    ("RM", "M1", d_RM_M1),
    ("M1", "M2", d_M1_M2),
    ("M2", "E5", d_M2_E5),
    ("RM", "R3", d_RM_R3),
    ("R3", "R2", d_R3_R2),
    ("R2", "R1", d_R2_R1),
    ("R1", "RK", d_R1_RK),
    ("RK", "K1", d_RK_K1),
    ("K1", "E4", d_K1_E4),
    ("RK", "RP", d_RK_RP),
    ("RP", "P1", d_RP_P1),
    ("P1", "E3", d_P1_E3),
    ("RP", "E2", d_RP_E2)
]

# Creating dij for both directions
d = {}

for i, j, distance in links:
    d[(i, j)] = distance
    d[(j, i)] = distance

# Flow can go in both directions
arcs = []

for i, j, distance in links:
    arcs.append((i, j))
    arcs.append((j, i))

# Assign flow an index so it is easy to find later
arc_index = {}

for k, (i, j) in enumerate(arcs):
    arc_index[(i, j)] = k

# Parking lot gate supply of cars
supplies = {
    "O1": x1_new/5,
    "O2": x1_new/5,
    "F1": x1_new/5,
    "F3": x1_new/5,
    "F4": x1_new/5,
    "O3": x2_new/4,
    "O4": x2_new/4,
    "F5": x2_new/4 + x3_new,
    "F6": x2_new/4 + x5_new,
    "R1": x4_new,
    "R2": x6_new,
    "F2": x7_new,
    "P1": x8_new,
    "K1": x9_new,
    "R3": x10_new/2,
    "M1": x10_new/2,
    "M2": x11_new

}

# Defining intersections
intersections = [
    "OK", "OM", "FK", "FM", "RM", "RK", "RP"
]

# Defining exits
exits = {
    "E1": "O1",
    "E2": "RP",
    "E3": "P1",
    "E4": "K1",
    "E5": "M2",
    "E6": "FM",
    "E7": "OM",
    "E8": "OK"
}

# Equal exit constraint
cars_per_exit = C / 8

print("Cars per exit:", cars_per_exit)

# Objective Function: minimize total car miles
c = np.array([
    d[(i, j)] / 5280    #converting feet into miles
    for i, j in arcs
])

# Flow conservation contraint at nodes (Used AI to make these because I didn't know how)
nodes = list(supplies.keys()) + intersections + list(exits.keys())

A_eq = []
b_eq = []

for node in nodes:

    row = np.zeros(len(arcs))

    for k, (i, j) in enumerate(arcs):

        # Outgoing flow
        if i == node:
            row[k] += 1

        # Incoming flow
        if j == node:
            row[k] -= 1

    A_eq.append(row)

    # Gate constraints
    if node in supplies:
        b_eq.append(supplies[node])

    # Exit constraints
    if node in exits:
        b_eq.append(-cars_per_exit)

    # Intersection constraints
    if node in intersections:
        b_eq.append(0)


A_eq = np.array(A_eq)
b_eq = np.array(b_eq)

# No returning from exit constraints and Non Negativity
bounds = []

for i, j in arcs:

    # No cars can travel out of an exit
    if i in exits:
        bounds.append((0, 0))

    # All other flows must be nonnegative
    else:
        bounds.append((0, None))

# Solve optimization model
result = linprog(
    c,
    A_eq=A_eq,
    b_eq=b_eq,
    bounds=bounds,
    method="highs"
)

# Report findings
if result.success:

    print("\nOptimal solution found!")

    print("Minimum total car-miles:", round(result.fun, 3))

    print("\nFlow on Every Physical Link:")

    for i, j, distance in links:

        # Directional flows
        forward = result.x[arc_index[(i, j)]]
        backward = result.x[arc_index[(j, i)]]

        print(
            f"{i} to {j}: {forward:.3f} cars | "
            f"{j} to {i}: {backward:.3f} cars"
        )
