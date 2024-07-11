Structure:
expense_8_puzzle.py will give the solution to the 8 puzzle problem where a desired goal state must be reached from a start state.
It provides actions to be take at each step, total cost of operations to reach the goal and other information using bfs, dfs, ids, dls, ucs, greedy and astar methods.
The code is divided into different functions that will execute the above methods. All the functions will execute a common class function that calculates the actions and cost.
Other information related to the states and nodes at each level are maintained in another class which is declared in the very beginning of the code.

Instructions to execute:
1. Install python in the system
2. Include start.txt and goal.txt in the same folder as that of the code for input values
3. Open Terminal/command line prompt 
4. Navigate to the folder that contains the code file and execute the following code 
    - Command: python3 <filename> start.txt goal.txt <method> <dump flag>
    - Example: python3 expense_8_puzzle.py start.txt goal.txt bfs true
    - Default method is a* and default dump flag value is false
    - Methods available: [bfs, dfs, ids, dls, ucs, greedy, astar]
    - Dump flag values: [True, False]

Note:
1. All method names must be given in lower case i.e. [bfs, dfs, ids, dls, ucs, greedy, astar]
2. To explicitly call the a* search, you must type 'astar'
