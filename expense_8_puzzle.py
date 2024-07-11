import sys
import time
from operator import itemgetter

class StateGenerator:
    def __init__(self, start, goal, method):
        self.possible_next_states = []
        self.start = start
        self.goal = goal
        self.method = method
        self.state = start[0]
        self.hori_trav, self.verti_trav = self.find_zero_position(self.state)

    def find_zero_position(self, state):
        for i, object in enumerate(state):
            for j, value in enumerate(object):
                if value == 0:
                    return i, j
        return -1, -1 

    def add_next_state(self, i_offset, j_offset, action):
        depth = self.start[3] + 1
        next_state = [list(object) for object in self.state]
        cost = next_state[self.hori_trav + i_offset][self.verti_trav + j_offset]
        next_state[self.hori_trav][self.verti_trav] = next_state[self.hori_trav + i_offset][self.verti_trav + j_offset]
        next_state[self.hori_trav + i_offset][self.verti_trav + j_offset] = 0
        pair = [
            next_state,
            [action],
            self.start,
            depth,
            self.start[4] + cost,
        ]
        #if self.method != 'astar':
        pair.append(self.estimation_function(next_state, self.goal))
        #else:
            #pair.append(self.estimation_function(next_state, self.goal) + cost)
        self.possible_next_states.append(pair)

    def estimation_function(self, start, goal):
        total_distance = 0
        for i, object in enumerate(start):
            for j, value in enumerate(object):
                if i < len(goal) and j < len(goal[0]):  
                    goal_value = goal[i][j]
                    if value != 0 and value != goal_value:
                        x_goal, y_goal = target_configuration(goal_value, goal)
                        x_start, y_start = i, j
                        distance = sum(abs(a - b) for a, b in zip((x_start, y_start), (x_goal, y_goal)))
                        total_distance += distance
        return total_distance

    def find_child(self, start, goal, method):
        if self.hori_trav > 0:
            self.add_next_state(-1, 0, "Move {} down".format(self.state[self.hori_trav - 1][self.verti_trav]))
        if self.hori_trav < 2:
            self.add_next_state(1, 0, "Move {} up".format(self.state[self.hori_trav + 1][self.verti_trav]))
        if self.verti_trav > 0:
            self.add_next_state(0, -1, "Move {} right".format(self.state[self.hori_trav][self.verti_trav - 1]))
        if self.verti_trav < 2:
            self.add_next_state(0, 1, "Move {} left".format(self.state[self.hori_trav][self.verti_trav + 1]))

        return self.possible_next_states

def target_configuration(num, goal):
    i = 0
    while i < len(goal):
        for j, value in enumerate(goal[i]):
            if value == num:
                return i, j
        i += 1
    return None, None 


class SearchState:
    def __init__(self):
        self.pn_exp, self.hue_num, self.cst_exp, self.gn_exp = 0, 0, 0, 0
        self.closed_states, self.dp_exp, self.largest_len, self.en_exp, self.cus_fg = set(), 0, 0, 0, []


def generate_search_trace(method):
    current_time = str(int(time.time()))  
    temp=f"search_trace_{current_time}.txt"
    search_trace = open(temp, 'w')
    search_trace.write("Command-Line Arguments: ['start.txt', 'goal.txt', '{}', 'true']\n".format(method))
    search_trace.write("Method Selected: {}\n".format(method))
    search_trace.write("Running: {}\n".format(method))
    return search_trace


def dfs_cal(start, goal, dump_flag):
    search_state = SearchState()
    search_state.cus_fg.append([start, None, None, search_state.dp_exp, search_state.cst_exp, search_state.gn_exp])

    search_trace = None
    if dump_flag:
        search_trace = generate_search_trace(method="bfs")
        
    while search_state.cus_fg:
        search_state.largest_len = max(search_state.largest_len, len(search_state.cus_fg))
        start_state, search_state.cus_fg, search_state.pn_exp = search_state.cus_fg[-1], search_state.cus_fg[:-1], search_state.pn_exp + 1

        if dump_flag:
            search_trace.write(f"Generating next_states to < state = {start_state[0]} >, action = {search_state.cus_fg}, cost = {search_state.cst_exp}, depth = {search_state.dp_exp} ")
            search_trace.write(f"\nFRINGE:{search_state.en_exp}{search_state.cus_fg}\nClosed:{search_state.en_exp}{search_state.closed_states}\n\n")
 
        if start_state[0] == goal:
            search_state.dp_exp = start_state[3]
            search_state.cst_exp = start_state[4]
            return start_state, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp
        

        state_generator = StateGenerator(start_state, goal_state, method="bfs")
        encountered = sum(1 for i in search_state.closed_states if tuple(map(tuple, start_state[0])) != i)
        if len(search_state.closed_states) == encountered:
            new_element = tuple(map(tuple, start_state[0]))
            search_state.closed_states |= {new_element}
            possible_next_states = state_generator.find_child(start_state, goal_state, method="bfs")
            search_state.gn_exp += len(possible_next_states)
            search_state.en_exp+=1
            search_state.cus_fg.extend(possible_next_states)


def dfs_print(start_state, goal_state, dump_flag):
    try:
        result = dfs_cal(start_state, goal_state, dump_flag)
        if result is not None:
            approach_desired, nodes, gen, exp, max_fringe, d, c = result
            print("Nodes Popped:", nodes)
            print("Nodes Expanded:", exp)
            print("Nodes Generated", gen)
            print("Max_fringe_size", max_fringe)
            print("Depth is:", d)        
            print("Cost is", c)
            print("Steps:")
            for i in range(len(approach_desired) - 1):
                if i == 1 and approach_desired[i] is not None:
                    steps(approach_desired[i + 1])
                    print(approach_desired[i][0], " ")
        else:
            print("Depth limit exceeded")
    except Exception as e:
        print(str(e))


def bfs(start, goal, dump_flag):
    search_state = SearchState()
    search_state.cus_fg.append([start, None, None, search_state.dp_exp, search_state.cst_exp, search_state.gn_exp])

    search_trace = None
    if dump_flag:
        search_trace = generate_search_trace(method="bfs")
        
    while search_state.cus_fg:
        search_state.largest_len = max(search_state.largest_len, len(search_state.cus_fg))
        start_state, search_state.cus_fg, search_state.pn_exp = search_state.cus_fg[0], search_state.cus_fg[1:], search_state.pn_exp + 1

        if dump_flag:
            search_trace.write(f"Generating next_states to < state = {start_state[0]} >, action = {search_state.cus_fg}, cost = {search_state.cst_exp}, depth = {search_state.dp_exp} ")
            search_trace.write(f"\nFRINGE:{search_state.en_exp}{search_state.cus_fg}\nClosed:{search_state.en_exp}{search_state.closed_states}\n\n")
 
        if start_state[0] == goal:
            search_state.dp_exp = start_state[3]
            search_state.cst_exp = start_state[4]
            return start_state, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp
        

        state_generator = StateGenerator(start_state, goal_state, method="bfs")
        encountered = sum(1 for i in search_state.closed_states if tuple(map(tuple, start_state[0])) != i)
        if len(search_state.closed_states) == encountered:
            new_element = tuple(map(tuple, start_state[0]))
            search_state.closed_states |= {new_element}
            possible_next_states = state_generator.find_child(start_state, goal_state, method="bfs")
            search_state.gn_exp += len(possible_next_states)
            search_state.en_exp+=1
            search_state.cus_fg.extend(possible_next_states)
            

def bfs_print(start_state, goal_state, dump_flag):
    approach_desired, nodes, gen, exp, max_fringe, d, c  = bfs(start_state, goal_state, dump_flag)
    result_list = [approach_desired, nodes, exp, gen, max_fringe, d, c]
    return result_list
                

def dls(start, goal, dump_flag, limit, method):
    search_state = SearchState()
    search_state.cus_fg.append([start, None, None, search_state.dp_exp, search_state.cst_exp, search_state.gn_exp])

    search_trace = None
    if dump_flag:
        search_trace = generate_search_trace(method)

    while search_state.cus_fg:
        search_state.largest_len = max(search_state.largest_len, len(search_state.cus_fg))
        
        start_state = search_state.cus_fg.pop()
        search_state.pn_exp = search_state.pn_exp + 1

        state_generator = StateGenerator(start_state, goal_state, method="dls")
        if dump_flag:
            search_trace.write(f"Generating next_states to < state = {start_state[0]} >, action = {search_state.cus_fg}, cost = {search_state.cst_exp}, depth = {search_state.dp_exp}")
            search_trace.write(f"\nFRINGE:{search_state.en_exp}{search_state.cus_fg}\nClosed:{search_state.en_exp}{search_state.closed_states}\n\n")
 
        '''if dump_flag and start_state[0] == goal:
                search_trace.close()
                search_state.dp_exp = start_state[3]
                search_state.cst_exp = start_state[4]
                return start_state, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp
        '''
        if start_state[0] == goal:
                #search_trace.close()
                search_state.dp_exp = start_state[3]
                search_state.cst_exp = start_state[4]
                return start_state, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp

        else:
            encountered = sum(1 for i in search_state.closed_states if tuple(map(tuple, start_state[0])) != i)
            if len(search_state.closed_states) == encountered:
                possible_next_states = state_generator.find_child(start_state, goal_state, method="ids")

                search_state.gn_exp += len(possible_next_states)
                search_state.en_exp+=1
                search_state.dp_exp = start_state[3] + 1

                filtered_children = [child for child in possible_next_states if child[3] < limit]

                for each in filtered_children:
                    if each[0] == goal or each[3] >= limit:  
                        if dump_flag:
                            search_trace.close()

                        search_state.dp_exp = each[3]
                        search_state.cst_exp = each[4]
                        return each, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp

                updated_children = [(child[0], child[1], child[2], search_state.dp_exp, child[4]) for child in filtered_children]
                search_state.cus_fg.extend(updated_children)

    ids_result = "Limit Exceeded"
    if dump_flag:
        search_trace.close()
    return ids_result, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp


def ids_print(start_state, goal_state, dump_flag):
    limit = 0
    while True:
        try:
            approach_desired, nodes, gen, exp, max_fringe, d, c = dls(start_state, goal_state, dump_flag, limit, method="ids")

            if approach_desired != "Limit Exceeded":
                result_list = [approach_desired, nodes, exp, gen, max_fringe, d, c]
                return result_list
            limit += 1
        except Exception as e:
            print(e)
            pass


def ucs(start, goal, dump_flag):
    search_trace = None
    if dump_flag:
        search_trace = generate_search_trace(method="ucs")

    search_state = SearchState()
    search_state.cus_fg.append([start, None, None, search_state.dp_exp, search_state.cst_exp, search_state.hue_num])

    while search_state.cus_fg:
        search_state.largest_len = max(search_state.largest_len, len(search_state.cus_fg))
        
        search_state.cus_fg = sorted(search_state.cus_fg, key=itemgetter(4))
        start_state, search_state.cus_fg, search_state.pn_exp= search_state.cus_fg[0], search_state.cus_fg[1:], search_state.pn_exp + 1

        if dump_flag:
            search_trace.write(f"Generating next_states to < state = {start_state[0]} >, action = {search_state.cus_fg}, cost = {search_state.cst_exp}, depth = {search_state.dp_exp}")
            search_trace.write(f"\nFRINGE:{search_state.en_exp}{search_state.cus_fg}\nClosed:{search_state.en_exp}{search_state.closed_states}\n\n")

        state_generator = StateGenerator(start_state, goal_state, method="ucs")
        
    
        if start_state[0] == goal:
            search_state.dp_exp = start_state[3]
            search_state.cst_exp = start_state[4]
            return start_state, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp

        encountered = sum(1 for i in search_state.closed_states if tuple(map(tuple, start_state[0])) != i)
        if len(search_state.closed_states) == encountered:
            new_element = tuple(map(tuple, start_state[0]))
            search_state.closed_states |= {new_element}
            possible_next_states = state_generator.find_child(start_state, goal_state, method="ucs")
            search_state.gn_exp += len(possible_next_states)
            search_state.en_exp+=1
            search_state.cus_fg.extend(possible_next_states)

def ucs_print(start_state, goal_state, dump_flag):
    approach_desired, nodes, gen, exp, max_fringe, d, c= ucs(start_state, goal_state, dump_flag)
    result_list = [approach_desired, nodes, exp, gen, max_fringe, d, c]
    return result_list
    

def greedy(start, goal, dump_flag):
    search_state = SearchState()
    search_state.cus_fg.append([start, None, None, search_state.dp_exp, search_state.cst_exp, search_state.hue_num])

    search_trace = None
    if dump_flag:
        search_trace = generate_search_trace(method="greedy")

    while search_state.cus_fg:
        search_state.largest_len = max(search_state.largest_len, len(search_state.cus_fg))

        search_state.cus_fg = sorted(search_state.cus_fg, key=itemgetter(5))
        start_state, search_state.cus_fg, search_state.pn_exp = search_state.cus_fg[0], search_state.cus_fg[1:], search_state.pn_exp + 1

        if dump_flag:
            search_trace.write(f"Generating next_states to < state = {start_state[0]} >, action = {search_state.cus_fg}, cost = {search_state.cst_exp}, depth = {search_state.dp_exp}")
            search_trace.write(f"\nFRINGE:{search_state.en_exp}{search_state.cus_fg}\nClosed:{search_state.en_exp}{search_state.closed_states}\n\n")

        state_generator = StateGenerator(start_state, goal_state, method="greedy")

        if start_state[0] == goal:
            search_state.dp_exp = start_state[3]
            search_state.cst_exp = start_state[4]
            return start_state, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp

        encountered = sum(1 for i in search_state.closed_states if tuple(map(tuple, start_state[0])) != i)
        if len(search_state.closed_states) == encountered:
            new_element = tuple(map(tuple, start_state[0]))
            search_state.closed_states |= {new_element}
            possible_next_states = state_generator.find_child(start_state, goal_state, method="greedy")
            search_state.gn_exp += len(possible_next_states)
            search_state.en_exp+=1
            search_state.cus_fg.extend(possible_next_states)
            

def greedy_print(start_state, goal_state, dump_flag):
    approach_desired, nodes, gen, exp, max_fringe, d, c = greedy(start_state, goal_state, dump_flag)
    result_list = [approach_desired, nodes, exp, gen, max_fringe, d, c]
    return result_list


def astar(start, goal, dump_flag):
    search_state = SearchState()
    search_state.cus_fg.append([start, None, None, search_state.dp_exp, search_state.cst_exp, search_state.hue_num])

    search_trace = None
    if dump_flag:
        search_trace = generate_search_trace(method="a*")

    while search_state.cus_fg:
        search_state.largest_len = max(search_state.largest_len, len(search_state.cus_fg))

        start_state, search_state.cus_fg, search_state.pn_exp = search_state.cus_fg[0], search_state.cus_fg[1:], search_state.pn_exp + 1

        if dump_flag:
            search_trace.write(f"Generating next_states to < state = {start_state[0]} >, action = {search_state.cus_fg}, cost = {search_state.cst_exp}, depth = {search_state.dp_exp}")
            search_trace.write(f"\nFRINGE:{search_state.en_exp}{search_state.cus_fg}\nClosed:{search_state.en_exp}{search_state.closed_states}\n\n")

        state_generator = StateGenerator(start_state, goal_state, method="astar")
        
        if start_state[0] == goal:
            search_state.dp_exp = start_state[3]
            search_state.cst_exp = start_state[4]
            return start_state, search_state.pn_exp, search_state.gn_exp, search_state.en_exp, search_state.largest_len, search_state.dp_exp, search_state.cst_exp

        encountered = sum(1 for i in search_state.closed_states if tuple(map(tuple, start_state[0])) != i)
        if len(search_state.closed_states) == encountered:
            new_element = tuple(map(tuple, start_state[0]))
            search_state.closed_states |= {new_element}
            possible_next_states = state_generator.find_child(start_state, goal_state, method="astar")
            search_state.cus_fg = sorted(search_state.cus_fg, key=itemgetter(5))
            search_state.gn_exp += len(possible_next_states)
            search_state.en_exp+=1
            search_state.cus_fg.extend(possible_next_states)

def astar_print(start_state, goal_state, dump_flag):
    approach_desired, nodes, gen, exp, max_fringe, d, c = astar(start_state, goal_state, dump_flag)
    result_list = [approach_desired, nodes, exp, gen, max_fringe, d, c]
    return result_list

def steps(approach_desired):                                         
    for i in range(len(approach_desired)-1):
        if i == 1 and approach_desired[i]!= None:   
            steps(approach_desired[i+1])
            print(approach_desired[i][0], " ")

def read_state_file(file_name):
    state_data = []
    with open(file_name, "r") as file:
        for line in file:
            if line.strip() == "END OF FILE":
                break
            elements = [int(ele) for ele in line.split()]
            state_data.append(elements)
    return state_data


def main():
    start_file = sys.argv[1]
    goal_file = sys.argv[2]
    
    start_state = read_state_file(start_file)
    goal_state = read_state_file(goal_file)

    print(start_state, goal_state)
    if len(sys.argv) >= 4:
        method = sys.argv[3]
    else:
        method = "astar"
    
    if len(sys.argv) >= 5:
        dump_flag = sys.argv[4].lower() == "true"
    else:
        dump_flag = False

    
    if(method=='astar'):
        result_list=astar_print(start_state, goal_state, dump_flag)
    elif(method=='dfs'):
        result_list=dfs_print(start_state, goal_state, dump_flag)
        return
    elif(method=='bfs'):
        result_list=bfs_print(start_state, goal_state, dump_flag)
    elif(method=='ucs'):
        result_list=ucs_print(start_state, goal_state, dump_flag)
    elif(method=='greedy'):
        result_list=greedy_print(start_state, goal_state, dump_flag)
    if(method=='ids'):
        result_list=ids_print(start_state, goal_state, dump_flag)
    elif method=='dls':
        print("Enter Limit\n")
        depth_limit = int(input())

        approach_desired, nodes, popped, exp, max_fringe, d, c = dls(start_state,goal_state, dump_flag, depth_limit, method="dls")
        print("reached at limit ",depth_limit)
        result_list = [approach_desired, nodes, exp, popped, max_fringe, d, c]
        
        if approach_desired == "Limit Exceeded":
            print("Depth limit of", depth_limit, "reached. Goal not found.")
        else:
            #print("Results:")
            labels = ["Results:", "Nodes Popped:",  "Nodes Expanded", "Nodes Generated:", "Max Fringe Size:", "Depth:", "Cost:"]
            for label, value in zip(labels, result_list):
                if isinstance(value, list):
                    print(label)
                else:
                    print(f"{label} {value}")
            approach_desired=result_list[0]
            print("Steps:")
            for i in range(len(approach_desired) - 1):
                if i == 1 and approach_desired[i] is not None:
                    steps(approach_desired[i + 1])
                    print(approach_desired[i][0], " ")
            return
    '''else: 
        print("Method does not exist, printing a*\n")
        result_list=astar_print(start_state, goal_state, dump_flag)
'''

    #print("Results:")
    labels = ["Results:", "Nodes Popped:", "Nodes Expanded", "Nodes Generated:", "Max Fringe Size:", "Depth:", "Cost:"]
    for label, value in zip(labels, result_list):
        if isinstance(value, list):
            print(label)
        else:
            print(f"{label} {value}")
    approach_desired=result_list[0]
    print("Steps:")
    for i in range(len(approach_desired) - 1):
        if i == 1 and approach_desired[i] is not None:
            steps(approach_desired[i + 1])
            print(approach_desired[i][0], " ")

if __name__ == "__main__":
    method='astar'
    start_state = []
    goal_state = []
    main()