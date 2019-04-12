# Discrete-Planner


Follow the instructions to run the code on linux system.

1. extract the content of .zip file Gupta_Manas.zip into any working directory.
2. Open a terminal and go to the working directory where the files are extracted.
3. cd Discrete_Planner 
4. mkdir build
5. cd build
6. cmake ..
7. make
8. ./planner -- to run the planner implementation of random and optima planners.
9. ./test    -- to run the test cases for random and optimal planner.



Results ---

output of ./planner

Path Found by Random Planner->[(2 0) (3 0) (3 1) (2 1) (2 2) (2 3) (1 3) (1 4) (1 5) (0 5) (0 4) (0 3) (0 4) (0 5) (1 5) (2 5) (3 5) (4 5) (5 5) ]
Path Found by Optimal Planner->[(2 0) (2 1) (3 1) (4 1) (5 1) (5 2) (5 3) (5 4) (5 5) ]


output of ./test

Test1::move_up() method passed successfully\
Test2::move_down() method passed successfully
Test3::move_right() method passed successfully
Test4::move_left() method passed successfully
Test5::random_state method passed successfully
Test6::collision_check method passed successfully for non collision
Test7::collision_check method passed successfully for collision
Test8::check_visited method passed successfully for visited state
Test9::check_visited method passed successfully for non-visited state
Test10::goal_check method passed successfully for goal state
Test11::goal_check method passed successfully for non-goal state
Test12::h_value method passed successfully
test13::search random planner method path found passed successfully
Unable to reach goal
test14::search random planner method no path found passed successfully
test15::search optimal planner method path found passed successfully
Goal not found
test16::search optimal planner method no path found passed successfully
