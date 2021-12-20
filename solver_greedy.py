from datetime import datetime
import numpy as np
import os
import random

class Greedy:

    def __init__(self):
        self.V = [] #list of buses
        self.T = [] #list of trips
        self.A = []# assignment matrix

    def assign(self):
   
        num = len(self.T)

        cost_matrix = CostMatrix(self.A, self.T, self.V)
        while num > 0:
            assigned = False
            v_i = -1
            t_i = -1
            np_cost_matrix = np.array(cost_matrix.matrix)
            min_cost = np.min(np_cost_matrix)
            if min_cost == np.inf:
                print("No more feasible solutions")
                break
            
            final_assigns = np.argwhere(np_cost_matrix == min_cost)
            t_i, v_i = final_assigns[0]
            trip = self.T[t_i]
            vehicle= self.V[v_i]
            
            if (trip, vehicle) in cost_matrix.req_charging.keys():
                charging = cost_matrix.req_charging[trip, vehicle]
                if self.A.assign_charge_force(charging, vehicle):
                    if self.A.assign(trip, vehicle):
                        assigned = True

            elif self.A.assign(trip, vehicle):
                assigned = True

            if assigned:
                num -= 1
                cost_matrix.update(self.A, v_i, t_i)
            else:
                
                print("Assignemts remaining ", num)
                break
        return self.A
        



class SimulatedAnnealing:
    def __init__(self, dump_structure, args):
        self.min_assign = None
        self.min_cost = None
        self.summary_file_name = ""
        self.run(dump_structure, args)

    def nearest_neighbour(assignment, swap_prob):
    
        swap_count = max(1, int(swap_prob * len(assignment.get_trips())))
        at_least_one_swapped = False
        start_time = datetime.now()
        child = None
        while swap_count > 0:
            if (datetime.now() - start_time).total_seconds() > 4:
                if not at_least_one_swapped:
                    s_print("Nothing modified breaking due to time limitation")
                break
            child, swapped = assignment.swap()
            if swapped:
                at_least_one_swapped = True
                swap_count -= 1

        if child is None:
            child = assignment
        return child


    def run(self):
        cycle_count = 50000
        start_prob = 0.7
        end_prob = 0.4
        swap_prob = 0.5

        swap_cond = 0 < swap_prob < 1
        start_end_cond = 0 < end_prob < start_prob < 1

        if not swap_cond or not start_end_cond:
            raise ValueError("inconsistent parameters")

        assignment = Greedy.assign()
        energy_cost = assignment.total_energy_cost()
        self.min_assign = assignment
        self.min_cost = energy_cost

        temp_start = -1.0 / np.log(start_prob)
        temp_end = -1.0 / np.log(end_prob)
        rate_of_temp = (temp_end / temp_start) ** (1.0 / (cycle_count - 1.0))

        selected_temp = temp_start
        delta_e_avg = 0.0
        number_of_accepted = 1
        prefix = "{}_{}_{}_".format(args.start_prob, args.end_prob, args.swap_prob)
        prefix = prefix.replace(".", "_")
        
        dir_name = os.getcwd()
        self.summary_file_name = dir_name + prefix + "simulated_annealing.csv"
        if "." in dir_name:
            pwd = dir_name.rfind("/")
            if pwd != -1:
                dir_name = dir_name[:pwd + 1]
        if not os.path.exists(dir_name):
            if "/" not in dir_name:
                os.mkdir(dir_name)
            else:
                try:
                    os.makedirs(dir_name)
                except FileExistsError:
                    print(dir_name + " already exists")
        
        
        for i in range(cycle_count):
            print('Temperature: 'selected_temp)
            nn_sel = SimulatedAnnealing.nearest_neighbour(assignment, swap_prob)
            nn_energy_cost = nn_sel.total_energy_cost()
            delta_e = abs(nn_energy_cost - energy_cost)
            if nn_energy_cost > energy_cost:
                if i == 0:
                    delta_e_avg = delta_e
                denominator = (delta_e_avg * selected_temp)
                p = np.exp(-1 * np.inf) if denominator == 0 else np.exp(-delta_e / denominator)
                accept = True if random.random() < p else False
            else:
                accept = True
            # save current minimum to avoid losing details due to crash
            if self.min_cost > nn_energy_cost:
                self.min_assign = nn_sel.copy()
                self.min_cost = nn_energy_cost
                nn_sel.write("current_min")
                nn_sel.write_bus_stat("current_min")

            if accept:
                assignment = nn_sel
                energy_cost = nn_energy_cost
                print(energy_cost)
                delta_e_avg = delta_e_avg + (delta_e - delta_e_avg) / number_of_accepted
                number_of_accepted += 1
            selected_temp = rate_of_temp * selected_temp
        improve_perc = round(100.0 * (energy_cost - self.min_cost) / energy_cost, 3)
        print("Improvement in energy cost = ",improve_perc)




class CostMatrix:

    def __init__(self, assignment, operating_self.trips, assign_buses):
        """
        Args:
            assignment: the structure that store the assignments of self.trips to buses
            operating_self.trips: list of all operating self.trips
            assign_buses: list of all buses
        Returns:
            two dimensional matrix containing the cost of serving each trip t in operating_self.trips
            by each bus v in assign_buses.
        """
        self.operating_self.trips = operating_self.trips
        self.assign_buses = assign_buses
        self.matrix = [[0 for v_i, _ in enumerate(assign_buses)] for t_i, _ in enumerate(operating_self.trips)]
        self.req_charging = {}
        self.init(assignment)

    def init(self, assignment):
        """
        Args:
            assignment: the structure that store the assignments of self.trips to buses
        """
        for _t_i, _trip in enumerate(self.operating_self.trips):
            for v_i, _bus in enumerate(self.assign_buses):
                self.matrix[_t_i][v_i] = assignment.energy_cost(_trip, _bus)

    def update(self, assignment, bus_pos, trip_pos):
        """
        Args:
            assignment: the structure that store the assignments of self.trips to buses
            trip_pos: last assigned trip's position
            bus_pos: last assigned bus's position
        Returns:
            two dimensional matrix containing the cost of serving each trip t in operating_self.trips
            by each bus v in assign_buses.
        """
        import math
        self.matrix.pop(trip_pos)
        prev_trip = self.operating_self.trips.pop(trip_pos)
        clean_keys = []
        for trip, bus in self.req_charging.keys():
            if trip == prev_trip:
                clean_keys.append((trip, bus))
        for key in clean_keys:
            self.req_charging.pop(key)
        for _t_i, _trip in enumerate(self.operating_self.trips):
            _bus = self.assign_buses[bus_pos]
            copy_assign = assignment.copy()
            _info = copy_assign.check(_trip, _bus)
            energy_cost = math.inf
            if _info.feasible():
                energy_cost = copy_assign.energy_cost(_trip, _bus)
            else:
                if is_electric(_bus) and _info.time_feasible() and not _info.energy_feasible():
                    _charge_info = copy_assign.assign_charge(_info, _bus, _trip)
                    if _charge_info.feasible():
                        _info_new = copy_assign.check(_trip, _bus)
                        if _info_new.feasible():
                            energy_cost = copy_assign.energy_cost(_trip, _bus)
                            self.req_charging[_trip, _bus] = _charge_info.entity()
            self.matrix[_t_i][bus_pos] = energy_cost
            del copy_assign


class UpdateCostMatrix(CostMatrix):
    def init(self, assignment):
        """
        Args:
            assignment: the structure that store the assignments of self.trips to buses
        """
        import math
        for _t_i, _trip in enumerate(self.operating_self.trips):
            for v_i, _bus in enumerate(self.assign_buses):
                copy_assign = assignment.copy()
                energy_cost = math.inf
                _info = copy_assign.check(_trip, _bus)
                if _info.feasible():
                    energy_cost = copy_assign.energy_cost(_trip, _bus)
                self.matrix[_t_i][v_i] = energy_cost