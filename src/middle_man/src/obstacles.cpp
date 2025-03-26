#include "raytracing.hpp"
#include "obstacles.hpp"
#include <iostream>
#include <cmath>
#include <utility>
using namespace std;
//okay lets assume we get the local goals from A star, 
//	we can assume that it is like an array of points, Ox, Oy
//	Still want to use the perpidicular approach with requires a offset value

std::pair<Obstacle,Obstacle> create_obstacle(double current_lg_x, double current_lg_y, double next_lg_x, double next_lg_y){
	
	//verify slope not zero
	//calculate inverse slope
    //calculate offset
	//create obstacles
	
	float offset_x;
	float offset_y; 
	float slope;
	float perp_slope;
    float mag; 
	float mx, my;

	mx = (current_lg_x + next_lg_x) / 2;
	my = (current_lg_y + next_lg_y) / 2;
	cout << "pont values "  << current_lg_x << "   " << current_lg_y << "    " <<  next_lg_x << "     "<< next_lg_y << "    "<< endl;	
	if ((current_lg_x - next_lg_x) == 0){
        cout << "vertical slope" << endl;
		offset_x = 0;
		offset_y = OFFSET;
	}

	else if ((current_lg_y - next_lg_y) == 0) {
		cout << "horizontal slope" << endl;
	}

	else {

		slope = ((next_lg_y - current_lg_y) / float(next_lg_x -  current_lg_x));
		perp_slope = -1 / slope;
		mag = sqrt(1 + perp_slope * perp_slope);
		offset_x = (OFFSET / mag);
		offset_y = ((OFFSET * perp_slope) / (mag));

	}
	cout << "mx, my "  << mx << "  " << my << endl;
	
	cout << "slope and neg slope " << slope << "    " << perp_slope << endl;
	cout << "mag    " << mag << endl;
	cout << "offsetx, y  " << offset_x << "   " << offset_y << endl;
	float ob1_x, ob1_y, ob2_x, ob2_y;
	ob1_x = mx+ offset_x;
	ob1_y = my  + offset_y;

	ob2_x = mx - offset_x;
	ob2_y = my - offset_y;
	Obstacle ob1;
	ob1.center_x = ob1_x;
	ob1.center_y = ob1_y;
	Obstacle ob2;
	ob2.center_x = ob2_x;
	ob2.center_y = ob2_y;
    cout << "center x " << ob1.center_x << "center y" << ob1.center_y << endl;	
	return {ob1,ob2};
}
