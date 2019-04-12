#include <iostream>
#include "include/planner_class.hpp"


int main()
{
	srand (time(NULL));																		// Seeding to time 0
	int world_state[6][6]={{0, 0, 1, 0, 0, 0},												// Defining map (world state) as an occupancy gid
							{0, 0, 1, 0, 0, 0},
							{0, 0, 0, 0, 1, 0},
							{0, 0, 0, 0, 1, 0},
							{0, 0, 1, 1, 1, 0},
							{0, 0, 0, 0, 0, 0}};


	Planner search(150);																	// Instantiation of class Planner with max step size 150
	pose robot_pose, goal_pose;																
	robot_pose.x=2;																			// Initializing start x position
	robot_pose.y=0;																			// Initializing start y position
	goal_pose.x=5;																			// Initializing goal x position
	goal_pose.y=5;																			// Initializing goal y position
	auto final_path=search.search_random(world_state,robot_pose,goal_pose);					// computing a path with randomized planner
	if(final_path.size()>0)
	{
		std::cout<<"Path Found by Random Planner->[";
		for(auto itr =final_path.begin(); itr!=final_path.end(); itr++)						// Displaying the path
		{
			std::cout<<"("<<itr->x<<" "<<itr->y<<") ";
		}
		std::cout<<"]"<<std::endl;
	}	

	final_path=search.search_optimal(world_state,robot_pose,goal_pose);						// Computing a path with optimal planner

	if(final_path.size()>0)
	{
		std::cout<<"Path Found by Optimal Planner->[";
		for(auto itr =final_path.rbegin(); itr !=final_path.rend(); itr++)					// Displaying the path
		{
			std::cout<<"("<<itr->x<<" "<<itr->y<<") ";
		}
		std::cout<<"]"<<std::endl;
	}	
	return 0;

}


