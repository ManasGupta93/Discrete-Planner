#include "include/planner_class.hpp"														// Including the class definitions

Planner::Planner(int max_step_number)												// Constructor
	{ 
		max_step=max_step_number;
	} 

pose Planner::move_right(pose& curr_pose)											// Method to move to a right state
	{
		 this->new_pose.x=curr_pose.x;
		 this->new_pose.y=curr_pose.y+1;

		return this->new_pose;
	}

pose Planner::move_left(pose& curr_pose)											// Method to move to a left state
	{
		 this->new_pose.x=curr_pose.x;
		 this->new_pose.y=curr_pose.y-1;

		return this->new_pose;
	}
pose Planner::move_up(pose& curr_pose)												//Method to move to a above state
	{
		 this->new_pose.x=curr_pose.x-1;
		 this->new_pose.y=curr_pose.y;

		return this->new_pose;
	}
pose Planner::move_down(pose& curr_pose)											// Method to move to a below state
	{

		 this->new_pose.x=curr_pose.x+1;
		 this->new_pose.y=curr_pose.y;

		return this->new_pose;
	}

pose Planner::random_state(int world_state[6][6],pose& curr_pose)
	{
		this->children.clear();
		this->no_way_out=false;
		pose state = move_down(curr_pose);
		this->children.push_back(state);
		state = move_right(curr_pose);
		this->children.push_back(state);
		state = move_up(curr_pose);
		this->children.push_back(state);
		state = move_left(curr_pose);
		this->children.push_back(state);

	  	int random= rand() % 4 + 0;													// Randomly choosing a state
	  	bool status=false;
	  	int count=0;
	  	for (auto itr : children)													// Checking the collision and cisited status of each child
	  	{
	  		this->collision_status=collision_check(world_state,itr);
			this->visited_status= check_visited(visited,itr);

			status=(collision_status || visited_status);
			if(status)
			{
				count++;
			}


	  	}

	  	if(count==children.size())													// If there is no valid state to move, the agent is stuck loacally
	  	{
	  		this->no_way_out=true;													// Flag is updated to denote that the agent is stuck locally

	  	}
	  	

	return children[random];														// returning a next random state

	}


bool Planner::collision_check(int world_state[6][6], pose& curr_pose )				// Method to check the collision and validity
	{
		bool check=false;
		// int world_x=curr_pose.x;
		// int world_y= curr_pose.y;
		if(world_state[curr_pose.x][curr_pose.y]==1)								// Using occupancy grid checking the collision with the obstackle
			check=true;
		else if(curr_pose.x>5 || curr_pose.x<0)										// Checking for boundary conditions in x-direction
			check=true;
		else if(curr_pose.y>5 || curr_pose.y<0)										// Checking for boundary conditions in y-direction
			check=true;

	return check;
	}

bool Planner::check_visited(std::vector<pose> visited, pose& curr_pose)				//Method to check if the state is already explored
	{
		if(no_way_out)																// if the agent is stuck locally
		{
			for(int i=0; i<visited.size(); i++)
			{
				for (auto itr : this->children)
				{
					if((visited[i].x==itr.x)&&(visited[i].y==itr.y))					// only in such scenarios it can visit already visited states
					{
						visited.erase(visited.begin()+i);								// A loop to facilitate such behaviour
					}
				}
			}
	}

		for (auto itr :visited)														// To check if the current state has already been visited before
		{
			if((itr.x==curr_pose.x)&&(itr.y==curr_pose.y))
			{
				return true;
	
			}
		}
	}


bool Planner::goal_check(pose& goal, pose& curr_pose )								// goal cheking 
{
	if((goal.x==curr_pose.x)&&(goal.y==curr_pose.y))								// returns true of the current state is the goal state
		return true;
	else
		return false;
}

double Planner::h_value(pose& goal, pose& curr_pose)								// Calulating the h value for A* algorithm
{
	return abs(curr_pose.x-goal.x)+abs(curr_pose.y-goal.y);							// Using Manhattan distace since it has orthogonal moves
}

std::list <pose> Planner::trace_path(std::vector<pose> visited, pose& start, pose& goal)  
 {																					// Method to trace back the path from goal to initial state
 	std::list<pose> trace_path;
 	pose parent_trace;
 	parent_trace.x=goal.x;
 	parent_trace.y=goal.y;
 	
 	while(!((parent_trace.x==start.x)&&(parent_trace.y==start.y)))
 	{
 		trace_path.push_back(parent_trace);
 		for (auto itr : visited)
 		{
 			if((itr.x==parent_trace.x)&&(itr.y==parent_trace.y))					// Path is traced back by tracing the parent of respective nodes
 			{
 				parent_trace.x=itr.parent_x;
 				parent_trace.y=itr.parent_y;			
 			}
 		} 		
 		
 	}

 	trace_path.push_back(start);
 	return trace_path;
 }



std::list<pose> Planner::search_random(int world_state[6][6], pose& robot_pose, pose& goal_pose)
{																				// Method to implement randomized planner
	this->path.push_back(robot_pose);
	this->visited.push_back(robot_pose);										// Initiallization of visited vector 
	pose init_state=robot_pose;													// Intialization of current state
	int steps=0;																// Initialization of number of steps taken

	while(!(goal_check(init_state,goal_pose)))									// Run unill goal is reached
	{
		bool status=true;
		while (status==true)													// Run untill valid state is discovered
		{
			this->new_state=random_state(world_state,init_state);				// Retrive next new random state
			this->collision_status=collision_check(world_state,new_state);		// Check for the collision of a new state
			this->visited_status= check_visited(visited,new_state);				// Check if the new state is already visited before

			status=(collision_status || visited_status);						// get the staus of both the above mentioned conditions (true/fasle)
			
			
			steps++;
			if(steps>this->max_step)											// if steps taken are more than maximum steps permitted
			{
				
				break;															// Abort
				
			}
		}

		if(steps>this->max_step)
			{
				std::cout<<"Unable to reach goal"<<std::endl;				// Abort and conclude that the goal is not reachable 
				plan_status=false;
				break;
				
			}
		this->visited.push_back(new_state);									// storing the explored state in visited vector
		this->path.push_back(new_state);									// storing the vaid state in the path
		init_state=new_state;
		
			if(this->visited.size()>sqrt(this->max_step))					// Due to limited memory, states older than sqrt(max_steps_requried)..
			{																//.. are forgotten
				this->visited.erase(this->visited.begin());
				
		 	}
	}


	if(this->plan_status==true)												// if path is found to goal... 
	{
		return this->path;													// path (policy/trajectory) is returned 
	}
	else 
		path.clear();
		return this->path;													// if no path found, empty array is returned.
	
}


 


std::list<pose> Planner::search_optimal(int world_state[6][6],pose& robot_pose,pose& goal_pose)
{																			// Method  to implement an optimal planner (A*)
	this->open_list.push_back(robot_pose);									// Initialization of a open list to store the states yet to be explored
	this->visited.clear();													// Same vector is used hence clearing the vector 
	bool found_goal=false;													// Initializing the found_goal flag
	pose parent=robot_pose;													// initializing the current state
	while(!(open_list.empty()))												// Run untill the open list becomes empty
	{	
		// Move right														// Move to a right state
		pose curr_node=move_right(parent);
		curr_node.parent_x=parent.x;										// finding the state to the right side
		curr_node.parent_y=parent.y;

		if(check_visited(this->visited,curr_node)==false)					// check if the new state is already been visited
		{
			if(collision_check(world_state,curr_node)==false)				// check if the new state is collision free
			{
				if(goal_check(goal_pose, curr_node))						// check if the new state is the goal state
				{
					found_goal=true;										// if the new state is goal state... goal is found
					visited.push_back(curr_node);							// updating the visited vecotr
					this->path=trace_path(this->visited,robot_pose,goal_pose);		// path is traced back to the initial state

				}

				curr_node.g=parent.g +1;									// updating the g value for the new state
				curr_node.h =h_value(goal_pose,curr_node);					// updating the h value for the new state (Manhattan distance)
				curr_node.f=curr_node.g+curr_node.h;						// updating the f value for the new state

				if(check_visited(this->open_list,curr_node))				// checking if the new node is in the open list
				{
					for(auto itr: this->open_list)							// if it is already present in the open list
					{
						if((itr.x==curr_node.x)&&(itr.y==curr_node.y)&&(itr.f>curr_node.f))			// and the value of the already present state has...
						{																			//.. greater f value, then update the already present...
							itr.f=curr_node.f;														//.. state attributes with the new state attributes
							itr.g=curr_node.g;
							itr.h=curr_node.h;
							itr.parent_x=curr_node.parent_x;										// upadting the parent as well
							itr.parent_y=curr_node.parent_y;
						}
					}
				}
				else
				{
					open_list.push_back	 (curr_node);					// otherwise insert the new state in open state
				}
			}
		}
		// Move left													// Doing exact same for the left state
		curr_node=move_left(parent);
		curr_node.parent_x=parent.x;
		curr_node.parent_y=parent.y;

		if(check_visited(this->visited,curr_node)==false)
		{
			if(collision_check(world_state,curr_node)==false)
			{
				if(goal_check(goal_pose,curr_node))
				{
					found_goal=true;
					visited.push_back(curr_node);
					this->path=trace_path(this->visited,robot_pose,goal_pose);
				}

				curr_node.g=parent.g +1;
				curr_node.h =h_value(goal_pose,curr_node);
				curr_node.f=curr_node.g+curr_node.h;

				if(check_visited(this->open_list,curr_node))
				{
					for(auto itr: this->open_list)
					{
						if((itr.x==curr_node.x)&&(itr.y==curr_node.y)&&(itr.f>curr_node.f))
						{
							itr.f=curr_node.f;
							itr.g=curr_node.g;
							itr.h=curr_node.h;
							itr.parent_x=curr_node.parent_x;
							itr.parent_y=curr_node.parent_y;
						}
					}
				}
				else
				{
					open_list.push_back(curr_node);
				}
			}
		}

		// Move up 															// DOing exact same with above state

		curr_node=move_up(parent);
		curr_node.parent_x=parent.x;
		curr_node.parent_y=parent.y;

		if(check_visited(this->visited,curr_node)==false)
		{
			if(collision_check(world_state,curr_node)==false)
			{
				if(goal_check(goal_pose, curr_node))
				{
					found_goal=true;
					visited.push_back(curr_node);
					this->path=trace_path(this->visited,robot_pose,goal_pose);
				}

				curr_node.g=parent.g +1;
				curr_node.h =h_value(goal_pose,curr_node);
				curr_node.f=curr_node.g+curr_node.h;

				if(check_visited(this->open_list,curr_node))
				{
					for(auto itr: this->open_list)
					{
						if((itr.x==curr_node.x)&&(itr.y==curr_node.y)&&(itr.f>curr_node.f))
						{
							itr.f=curr_node.f;
							itr.g=curr_node.g;
							itr.h=curr_node.h;
							itr.parent_x=curr_node.parent_x;
							itr.parent_y=curr_node.parent_y;
						}
					}
				}
				else
				{
					open_list.push_back(curr_node);
				}
			}
		}


		// move down 														// Doing exact same with the below state

		curr_node=move_down(parent);
		curr_node.parent_x=parent.x;
		curr_node.parent_y=parent.y;

		if(check_visited(this->visited,curr_node)==false)
		{
			if(collision_check(world_state,curr_node)==false)
			{
				if(goal_check(goal_pose,curr_node))
				{
					found_goal=true;
					visited.push_back(curr_node);
					this->path=trace_path(this->visited,robot_pose,goal_pose);
				}

				curr_node.g=parent.g +1;
				curr_node.h =h_value(goal_pose,curr_node);
				curr_node.f=curr_node.g+curr_node.h;

				if(check_visited(this->open_list,curr_node))
				{
					for(auto itr: this->open_list)
					{
						if((itr.x==curr_node.x)&&(itr.y==curr_node.y)&&(itr.f>curr_node.f))
						{
							itr.f=curr_node.f;
							itr.g=curr_node.g;
							itr.h=curr_node.h;
							itr.parent_x=curr_node.parent_x;
							itr.parent_y=curr_node.parent_y;
						}
					}
				}
				else
				{
					open_list.push_back(curr_node);
				}
			}
		}

		parent=open_list.front();														// exploring the first state of open list
		visited.push_back(parent);														// updating the visited vector
		open_list.erase(open_list.begin());												// deleting the first state from the open list

			
				
		}

		if(found_goal==false)															// if the goal is not reached
		{
			std::cout<<"Goal not found"<<std::endl;										// Conculding goal is nor found
			return this->path;															// returning an empty path
		}	




	return this->path;																   // else returning the optimal path
}
	




