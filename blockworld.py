import random
import time
import heapq


class BlockWorld:
    def __init__(self, blocks, table=None):
        self.blocks = blocks
        
        if table is None:
            self.table = self.create_random_table()
        else:
            self.table = table
        
        # costs for A* search
        self.g_cost = 0 #moves it took to get to state
        self.h_cost = 0 #estimated # of moves to reach goal state
        self.f_cost = 0 # g + h 
    
    def create_random_table(self):
        blocks_to_place = list(self.blocks)
        random.shuffle(blocks_to_place)
        
        table = [[] for _ in range(len(self.blocks))]
        
        while blocks_to_place:
            block = blocks_to_place.pop()
            stack_index = random.randint(0, len(table) - 1)
            table[stack_index].append(block)
        
        return table
    
    def __lt__(self, other):
        #A* cost comparison for priority queue
        return self.f_cost < other.f_cost
    
    def get_state(self):
        #converts the table into a form that can be used as a dict key
        return tuple(tuple(stack) for stack in self.table)
    
    def display(self):
        max_height = max(len(stack) for stack in self.table)
        table_width = len(self.table) * 4 + 1
        
        print("-" * table_width)
        
        for level in range(max_height - 1, -1, -1):
            line = ""
            for stack in self.table:
                if len(stack) > level:
                    line += f"| {stack[level]} "
                else:
                    line += "|   "
            line += "|"
            print(line)
        
        print("-" * table_width)
        
        line = ""
        for i in range(len(self.table)):
            line += f"| {i} "
        line += "|"
        print(line)
        print("-" * table_width)
    
    def is_goal_state(self):
        goal_stack = sorted(self.blocks)
        return any(stack == goal_stack for stack in self.table)
    
    def get_possible_moves(self):
        possible_moves = []
        
        for source_stack_index in range(len(self.table)):
            source_stack = self.table[source_stack_index]
            
            if source_stack:
                block_to_move = source_stack[-1] # top element of column
                
                for dest_stack_index in range(len(self.table)):
                    if source_stack_index != dest_stack_index:
                        new_table = [stack[:] for stack in self.table] # old table copy
                        new_table[source_stack_index].pop() # remove moved block
                        new_table[dest_stack_index].append(block_to_move) #appened moved block to new position
                        
                        possible_moves.append(BlockWorld(self.blocks, new_table))
        
        return possible_moves


def calculate_heuristic(world):
    """
    Logic:
      If a stack's bottom block is part of the goal, check each block in the stack
      If any block is in the wrong position, add 1 to the cost
      If a stack's bottom block isn't part of the goal, ALL blocks in that stack need to move
    """
    h_cost = 0
    goal_stack = sorted(world.blocks)
    
    for stack in world.table:
        if len(stack) == 0:
            continue
        # count misplaced blocks within the stack
        if stack[0] in goal_stack:
            for i, block in enumerate(stack):
                if i >= len(goal_stack) or block != goal_stack[i]:
                    h_cost += 1
        else:
            # all misplaced blocks add to cost
            h_cost += len(stack)
    
    return h_cost


def a_star_search(initial_world):
    initial_world.g_cost = 0
    initial_world.h_cost = calculate_heuristic(initial_world)
    initial_world.f_cost = initial_world.g_cost + initial_world.h_cost

    open_set = [(initial_world.f_cost, initial_world)]
    came_from = {} # track how we reached each state for path reconstruction
    g_costs = {initial_world.get_state(): 0}
    
    states_explored = 0
    
    while open_set:
        # get state with lowest f_cost
        current_f_cost, current_world = heapq.heappop(open_set)
        states_explored += 1
        
        if current_world.is_goal_state():
            print(f"Goal reached! Explored {states_explored} states.")
            return reconstruct_solution_path(came_from, current_world)
        
        for neighbor_world in current_world.get_possible_moves():
            neighbor_state = neighbor_world.get_state()
            tentative_g_cost = current_world.g_cost + 1
            
            if tentative_g_cost < g_costs.get(neighbor_state, float('inf')):
                came_from[neighbor_state] = current_world
                
                neighbor_world.g_cost = tentative_g_cost
                neighbor_world.h_cost = calculate_heuristic(neighbor_world)
                neighbor_world.f_cost = neighbor_world.g_cost + neighbor_world.h_cost
                
                g_costs[neighbor_state] = tentative_g_cost
                
                heapq.heappush(open_set, (neighbor_world.f_cost, neighbor_world))
    
    print(f"No solution found after exploring {states_explored} states.")
    return None


def reconstruct_solution_path(came_from, goal_world):
    path = []
    current_world = goal_world
    
    while current_world.get_state() in came_from:
        path.append(current_world)
        current_world = came_from[current_world.get_state()]
    
    path.append(current_world)
    return path[::-1]


def main():
    print("=== Block World A* Solution ===")
    blocks = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J' ]
    initial_world = BlockWorld(blocks)
    
    print("Initial State:")
    initial_world.display()
    
    if initial_world.is_goal_state():
        print("Already at goal state! No moves needed.")
        return
    
    print("\nStarting A* Search...")
    start_time = time.time()
    
    solution_path = a_star_search(initial_world)
    
    search_time = time.time() - start_time
    
    if solution_path:
        num_moves = len(solution_path) - 1
        print(f"\nSolution found in {num_moves} moves!")
        print(f"Search completed in {search_time:.2f} seconds")
        
        print("\n=== Solution Path ===")
        for step, world_state in enumerate(solution_path):
            print(f"\nStep {step}:")
            world_state.display()
            
            if step < len(solution_path) - 1:
                print("Move a block")
    else:
        print(f"\nNo solution found in {search_time:.2f} seconds")


if __name__ == "__main__":
    main()