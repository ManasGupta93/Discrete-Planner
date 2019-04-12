
#ifndef PLANNER_CLASS_HPP_
#define PLANNER_CLASS_HPP_


#include <iostream>
#include<vector>
#include<cstdlib>
#include<ctime>
#include<stdio.h>
#include<cmath>
#include<list>
#include<set>

struct pose 														// structure to store the state and various attributes..
{																	//.. associated with a state for random and optimal planners.			
	int x,y;
	int parent_x,parent_y;
	double f,g,h;
};


class Planner
{
private:

	pose new_state;                                                 // For a new explored state
	std::vector<pose> children;									    // Stores the children of a node
	std::list<pose> path;											// Stores the final policy or trajectory or path
	std::vector<pose> visited;										// Stores the states that are explored
	std::vector<pose> open_list;									// Stores the states that are yet to be explored
	pose new_pose;													// To store new state
	int max_step;													// Maximum number of steps planner can take to reach goal
	bool collision_status;											// Flag for collision checking
	bool visited_status;											// Flag to check the visited status of a state
	bool plan_status=true;											// Flag to check if the planner finds the path successfully or not
	bool no_way_out=false;											// Flag to check if there is no way out from a state
	int count;														// Flag ot keep a count
public:
	Planner(int);													//Constructor 
	pose move_right(pose& curr_pose);								// Method to move right from a present state
	pose move_left(pose& curr_pose);								// Method to move left from a present state
	pose move_up(pose& curr_pose); 									// Method to move up from a present state
	pose move_down(pose& curr_pose);								// Method to move down from a present state
	pose random_state(int world_state[6][6],pose& curr_pose);		// Method to generate a random state
	bool collision_check(int world_state[6][6], pose& curr_pose );	// Method for collision check
	bool check_visited(std::vector<pose> visited, pose& curr_pose);	// Method to check if a state is visited already
	std::list<pose> search_random(int world_state[6][6], pose& robot_pose, pose& goal_pose); 	//Planning method for the randomized search 
	bool goal_check(pose& goal, pose& curr_pose );												// Method to check if the goal is reached
	double h_value(pose& goal, pose& curr_pose);												// Method to calculate the h value for A* algorithm
	std::list <pose> trace_path(std::vector<pose> visited, pose& start, pose& goal);			//Method to trace back the trajectory from goal to start state
	std::list<pose> search_optimal(int world_state[6][6],pose& robot_pose,pose& goal_pose);		// Planning method for the optimal algorithm (A*)
};

#endif