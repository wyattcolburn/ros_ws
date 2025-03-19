// Goal is to implement current work in python into C++
// Raytracing
// Obstacle creation
// MPC calculations





#include "raylib.h"
#include <iostream>
#include <obstack.h>
#include <raylib.h>

#include <cmath>
#include <limits>

#include "mpc.hpp"
#include "raytracing.hpp"
#include "obstacles.hpp"
using namespace std;
//What would my c++ program do for thesis, 
// What we have odom_x, odom_y, and local_goals_x, local_goals_y
// Want to create obstacles, to create lidar readings





int main() {
    
	const int screenHeight = 800;
	const int screenWidth = 800;
	
	InitWindow(screenWidth, screenHeight, "Init window");
	SetTargetFPS(60);
	
	cout << "hello world" << endl;
    	
    int num_lidar_readings = 1080;  // LIDAR resolution

    // Setup obstacles array
    int num_obstacles = 2;  // Using the two obstacles you've defined
    
    float distances[num_lidar_readings];
    
	//obstacle manager
	ObstacleManager obstacle_manager;

	auto [newOb1, newOb2] = create_obstacle(400.0, 400.0, 450.0, 410.0); //simulated local goals, will be Astar values
	newOb1.radius = 10;
	newOb2.radius = 10;
    obstacle_manager.add_obstacle(newOb1); 
    obstacle_manager.add_obstacle(newOb2); 
	Obstacle TBOT;
	TBOT.center_x = 480;
	TBOT.center_y = 400;
	TBOT.radius = TURTLEBOT_RADIUS;
	

	bool VAL;
	VAL = circles_intersect(obstacle_manager.obstacle_array[0], TBOT);
	cout << "CIRCLE INTERESECT : " << VAL << endl;
	//want to return all the estimate future poitns to draw
	Obstacle future_points[10];

	modulation(TBOT.center_x,TBOT.center_y, 1.0, PI, obstacle_manager, future_points);

	//new odom then new cmd velocity, logic 


	//read the publishing topic ( does not have local goals data)
	//put that info into compute lidar
	//how are we gonna update obstackes,
	//run NN
	//modulate
	//publish
	// input values-->output values
	while (!WindowShouldClose()) {

        BeginDrawing();
        ClearBackground(WHITE);
        
        // Draw obstacles
        for (int i = 0; i < num_obstacles; i++) {
            DrawCircle(obstacle_manager.obstacle_array[i].center_x, obstacle_manager.obstacle_array[i].center_y, 
                       obstacle_manager.obstacle_array[i].radius, RED);
        }
		for (int i = 0; i < 10; i++) {

			DrawLine(TBOT.center_x, TBOT.center_y,future_points[i].center_x, future_points[i].center_y, BLACK);
		}
        // Draw LIDAR origin point
        //DrawCircle(newOb1.center_x, newOb1.center_y, newOb1.radius, PURPLE);  
        //DrawCircle(newOb2.center_x, newOb2.center_y, newOb2.radius, PURPLE);  

        DrawCircle(TBOT.center_x,TBOT.center_y, TURTLEBOT_RADIUS, BLACK);  
		//Draw local goal points
		DrawCircle(400,400, 10, BLUE);
		DrawCircle(450,410, 10, BLUE);
        
		// Compute LIDAR distances considering all obstacles
        compute_lidar_distances(TBOT.center_x, TBOT.center_y, 
                              num_lidar_readings, obstacle_manager, &distances[0]);
        
        // Draw LIDAR rays
		draw_lidar(TBOT.center_x, TBOT.center_y, &distances[0], num_lidar_readings);        
        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}
