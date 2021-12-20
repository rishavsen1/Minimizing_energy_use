from os import name
from docplex.mp.model import Model
import numpy as np


def move_cost(t1, t2, type):
    energy = 0
    cost = E_cost(t1, t2)
    if type == 'electric':
        energy = cost * 100 * 0.01 * 0.1
    elif type == 'gas':
        energy = cost * 3.5
    return energy

def E_cost(t1, t2):
    finish = t1
    start = t2
    if (t2.start_time< t1.start_time):
        finish = t2
        start = t1
    start_pos = finish.coords
    end_pos = start.coords
    avg_speed = 30 
    cost = (start_pos - end_pos) * avg_speed #using avg speed to set a time frame, instead of actual calculations
    return cost

m = Model(name="MILP")

V = np.array([]) #set of buses, each with its 'type'
T = np.array([]) # set of trips, with location and start and end times
# CP = np.array([]) # set of C x (slots)
C = np.array([]) # set of charging points
S = np.array([]) # set of charging slots

costs = np.array() #moving cost of buses
# A = np.array([[]])

service_trip_pairs = [] # trips between service points
non_service_trip_pairs = [] # trips between non-service points

K_elec = 1 # cost of electric bus
K_gas = 1 # cost of diesel bus

A = m.binary_var_matrix(V.shape[0], (T.shape[0] + C.shape[0]), name='a_v_t')
# a_v_cp_s = m.binary_var_matrix(V.shape[0], C.shape[0], name='a_v_cp_s')
M = m.binary_var_matrix(name='m_v_x1_x2')

C_M_v = 100

e_v_s = m.continuous_var(name='e_v_s', lb = 0, ub = C_M_v)
c_v_s = m.continuous_var(name='c_v_s', lb = 0, ub = C_M_v)

# the objective function consists of the cost of movement for service and non-service trips
for v in V:
    for t1, t2 in service_trip_pairs:
        costs.append(move_cost(t1, t2, type))

for v in V:
    for t1, t2 in non_service_trip_pairs:
        costs.append(move_cost(t1, t2, type))

const_nos = 0
avg_spd = 30
'''
constraints : sum (a_{v,t}) >= 1
conditions: t | T, v | V
'''
for v in V:
    for t in T:
        m.add_constraint((sum(A[v][t]) for i in range(V.shape[0])) == 1)
        const_nos+=1

#m.add_constraint((sum(a_v_cp_s) for i in range(V.shape[0])) == 1)

'''
constraints : a_{v,t1} + a_{v,t2} <= 1
conditions: if ~Feasible, t1,t2 | T, v | V
'''
for v in V:
    for t1, t2 in non_service_trip_pairs:
        if t1.dest + (t1.end - t2.start) *avg_spd < t2.end:
            m.add_constraint((sum(A[v][t1], A[v][t2])) <= 1 )
            const_nos+=1

'''
constraints : m_{v,t1,t2} - a_{v,t1} - a_{v,t2} + sum (a_{v,t}) >= -1
conditions: if Feasible, t1,t2 | T; v | V; t | T[t1:t2];
'''
for v in V:
    for t1, t2 in service_trip_pairs:
        for t in T[t1:t2]:
            sum_trip = 0
            if (A[v][t]):
                sum_trip = 1                
        m.add_constraint(mov(v, t1, t2) - (sum(A[v][t], A[v][t2])) + sum_trip >= -1 )
        const_nos+=1


'''
constraints : sum (a_{v,c}) <= 1
conditions : c | C, v | V, is_electric(v)
'''
for v in V:
    for c in C:
        if v.type=='electric':
            m.add_constraint(sum(A[v][c]) <=1)

'''
constraints : sum (a_{v,c} · P) - c_{v,s} => 0
conditions : c | C, s | S is_in_slot(c, s); v | V, is_electric(v);
'''
for c in C:
    for s in S:
        for v in V:
            if v.type=='electric':
                power = 5 * 0.5 # power charged in 1 time slot (assuming linear charging curve)
                m.add_constraint(sum(A[v][c]*power) - c[v][s] >= 0)


# m.add_constraint((sum(C[i]) for i in range(V.shape[0]))*(charge_per_slot) >= c_v_s)
# m.add_constraint((sum(a_v_cp_s) for i in range(V.shape[0])) == 1)

