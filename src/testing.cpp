#include "include/planner_class.hpp"
#include<iostream>
#include<cassert>


int main()
{

	Planner plan_test(150);																// Insatantiation of the Planner class
	pose state, goal;
	state.x=2;																			// Initial state x value
	state.y=2;																			// Initial state y value
	int world_state[6][6]={{0, 0, 1, 0, 0, 0},											// Defining map (world state) as an occupancy gid
							{0, 0, 1, 0, 0, 0},
							{0, 0, 0, 0, 1, 0},
							{0, 0, 0, 0, 1, 0},
							{0, 0, 1, 1, 1, 0},
							{0, 0, 0, 0, 0, 0}};																			


	pose state_new=plan_test.move_up(state);											// testing move_up method
	assert(state_new.x==1 && state_new.y==2);
    std::cout << "Test1::move_up() method passed successfully"<<std::endl;

	state_new=plan_test.move_down(state);												// testing move_down method
	assert(state_new.x==3 && state_new.y==2);
    std::cout << "Test2::move_down() method passed successfully"<<std::endl;

    state_new=plan_test.move_right(state);												// testing move_right method
	assert(state_new.x==2 && state_new.y==3);
    std::cout << "Test3::move_right() method passed successfully"<<std::endl;

    state_new=plan_test.move_left(state);												// testing move_left method
	assert(state_new.x==2 && state_new.y==1);
    std::cout << "Test4::move_left() method passed successfully"<<std::endl;
    

    state_new=plan_test.random_state(world_state,state);								// testing random state generater method
    assert((state_new.x==2 && state_new.y==3 ||
    		(state_new.x==2 && state_new.y==1) ||
    		(state_new.x==1 && state_new.y==2) ||
    		(state_new.x==3 && state_new.y==2)));
    std::cout << "Test5::random_state method passed successfully"<<std::endl;

    pose state_collide;
    state_collide.x=1;
    state_collide.y=2;
    bool status=plan_test.collision_check(world_state,state);							// testing collision_check method for non colliding state
    assert(status==false);
    std::cout << "Test6::collision_check method passed successfully for non collision"<<std::endl;

    status=plan_test.collision_check(world_state,state_collide);						// testing collision_check method for colliding state
    assert(status);
    std::cout << "Test7::collision_check method passed successfully for collision"<<std::endl;


    std::vector<pose> visited_states;
    visited_states.push_back(state);

    status=plan_test.check_visited(visited_states,state);								// testing check_visited method for visited state
    assert(status);
    std::cout << "Test8::check_visited method passed successfully for visited state"<<std::endl;


    status=plan_test.check_visited(visited_states,state_new);							// testing check_visited method for non-visited state
    assert(status==false);
    std::cout << "Test9::check_visited method passed successfully for non-visited state"<<std::endl;

    goal.x=2;
    goal.y=2;

    status=plan_test.goal_check(goal,state);											// testing goal_check method for goal state
    assert(status);
    std::cout << "Test10::goal_check method passed successfully for goal state"<<std::endl;


    status=plan_test.goal_check(goal,state_new);										// testing goal_check method for non-goal state
    assert(status==false);
    std::cout << "Test11::goal_check method passed successfully for non-goal state"<<std::endl;


    assert(plan_test.h_value(goal,state_collide)==1);									// testing h_value method for correctness
    std::cout << "Test12::h_value method passed successfully"<<std::endl;

    goal.x=5;
    goal.y=5;
    std::list<pose> path;
    
    for(int i=150; i>19; i-=130)														// testing for the random planner search
    {

		Planner plan_test(i);	    		
	    path=plan_test.search_random(world_state,state,goal);
	    if(path.size()>0)
	    {
	    	pose first=path.front();
	    	pose last=path.back();
	    	assert((first.x==2&&first.y==2)&&(last.x==5&&last.y==5));					// testing for finding path successfully
	    	std::cout<<"test13::search random planner method path found passed successfully"<<std::endl;	
	    }
	    else
	    {																				// testing  for no path found
	    	std::cout<<"test14::search random planner method no path found passed successfully"<<std::endl;
	    }
	}

	path=plan_test.search_optimal(world_state,state,goal);								// testing for the optimal planner
	pose first=path.front();
	pose last=path.back();
	assert((first.x==5&&first.y==5)&&(last.x==2&&last.y==2));							// testing for finding path successfully
	std::cout<<"test15::search optimal planner method path found passed successfully"<<std::endl;	

	path.clear();
	goal.x=4;
	goal.y=4;
	path=plan_test.search_optimal(world_state,state,goal);
	assert(path.size()>0);																 // testing for no path found
	std::cout<<"test16::search optimal planner method no path found passed successfully"<<std::endl;





    return 0;
}