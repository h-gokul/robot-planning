
- To run individual test cases, run this command:

```python3 numberpuzzle.py --init_state "1 4 7 2 0 8 3 5 6" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test2"```

###### Note: the initial and goal states are in columnwise order.

- To run all ten test cases, run command in terminal.
`sudo ./run.sh`

###### Note: Please ensure that `run.sh` is an executable, if not run command
`chmod +x run.sh`


Refer the results in `./results/test<number>` folders where \<number\> denotes the test case.

`moves.txt` contains the moves to be taken to reach the goal
  
`Nodes.txt` contains all the nodes that were examined during the breadth first search
  
`nodePath.txt` contains the set of state that are to be reached while reaching the goal
  
`NodeInfo.txt` contains the `Node_index`, `Parent_Node_index` and corresponding `Cost`


Repository Tree:
```
bash
├── numberpuzzle.py
├── plot_path.py
├── Readme.txt
├── results
          ├── test
```
