#!/bin/bash

echo "test case 1:" 
python3 numberpuzzle.py --init_state "1 4 7 0 2 8 3 5 6" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test1"

echo "test case 2:" 
python3 numberpuzzle.py --init_state "1 4 7 2 0 8 3 5 6" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test2"

echo "test case 3:" 
python3 numberpuzzle.py --init_state "0 4 7 1 5 8 2 3 6" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test3"

echo "test case 4:" 
python3 numberpuzzle.py --init_state "1 0 7 2 4 6 3 8 5" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test4"

echo "test case 5:" 
python3 numberpuzzle.py --init_state "1 7 6 4 0 5 3 8 2" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test5"

echo "test case 6:"  
python3 numberpuzzle.py --init_state "3 4 6 1 5 0 2 8 7" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test6"

echo "test case 7:" 
python3 numberpuzzle.py --init_state "1 4 7 2 8 6 0 3 5" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test7"

echo "test case 8:" 
python3 numberpuzzle.py --init_state "0 2 7 4 1 5 3 6 8" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test8"

echo "test case 9:" 
python3 numberpuzzle.py --init_state "1 5 4 3 2 0 6 8 7" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test9"

echo "test case 10:" 
python3 numberpuzzle.py --init_state "0 5 7 4 3 8 1 2 6" --goal_state "1 4 7 2 5 8 3 6 0" --savefolder "./results/test10"

echo "Done all test cases"
