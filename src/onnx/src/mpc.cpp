#include "mpc.hpp"
#include "obstacles.hpp"
/* This function will take in a the output of NN (cmd_v, cmd_w)
 * and calculate the safety factor
 * 
 * We need to do two things in this function: 
 *		1) calculate p safety
 *			Calculate the trad of output cmd_v/w
 *			record how many collisions (want to apply noise on each one)
*				gaussin noise is zero mean with 10% standard eviation 
 *		2) module the output according to :
 *			
 *			e^ (w1-w2)(1-P(safety))* {v,w}
 *			where w1 = .4 and w2 = 1 (correalting to 50%-150%)
 *
 *			x_future = x_current + linear_velocity * cos(theta_current) * dt
			y_future = y_current + linear_velocity * sin(theta_current) * dt
			theta_future = theta_current + angular_velocity * dt
 *
 *		Need current odom_x/y, need to know where obstacles exist, output
 *		of neural net, how many times to check
*/
float gaussian(float input_val, float mean, float std_dev){

	static std::random_device rd; //random seed
	static std::mt19937 generator(rd()); //static so only once
	std::normal_distribution<float>distribution(mean, std_dev);
	input_val = input_val * (1 + distribution(generator));  //small value - negative value = -value, 
	return input_val;
}
bool circles_intersect(const Obstacle& t_bot, const Obstacle& obstacle) {
    float dx = t_bot.center_x - obstacle.center_x;
    float dy = t_bot.center_y - obstacle.center_y;
    float distanceSquared = dx * dx + dy * dy;  // Avoids unnecessary sqrt
    float radiusSum = t_bot.radius + obstacle.radius;

    return distanceSquared <= (radiusSum * radiusSum);
}

std::pair<float, float> modulation_onnx(float odom_x, float odom_y,
		float input_cmd_v, float input_cmd_w, std::vector<Obstacle> obstacle_data) {

	/* Function takes current localation, outs of neural network and calculates 
	 * the modulation factor then applies
	 *
	 *
	 *
	 *
	 *
	*/

	//num of obstacles
	//obstacles
    
		
	//modulate low angular velocity
	if (std::abs(input_cmd_w) < .04) {
		input_cmd_w = 0;
	}

	Obstacle T_BOT;
	T_BOT.radius = TURTLEBOT_RADIUS;

	int attempt_lim = 10;
	int p_safety_counter = 0;

	float x_future, y_future;
	float noisy_cmd_v, noisy_cmd_w;
	// add noise to cmd v and restrict w < .04 to 0
    for (int attempts = 0; attempts < attempt_lim; attempts++) {
	    
		noisy_cmd_v = gaussian(input_cmd_v, 0, .1);
		noisy_cmd_w = gaussian(input_cmd_w, 0, .1);
		
		x_future = odom_x + noisy_cmd_v* cos(noisy_cmd_w) * TIMESTEP*1000;
		y_future = odom_y + noisy_cmd_v* sin(noisy_cmd_w) * TIMESTEP*1000;

		T_BOT.center_x = x_future;
		T_BOT.center_y = y_future;

		std::cout << "x future, y future" << x_future << "  " << y_future << std::endl;
		for (int obs_counter = 0; obs_counter < 20 ; obs_counter++){
			if (circles_intersect(T_BOT, obstacle_data[obs_counter])){
				p_safety_counter++;
				std::cout << "collision" << std::endl;
				break;
			}
			else {
				std::cout << "no collision, next obstacles" << std::endl;
			}
		}
	}

	std::cout << "num of collisions" << p_safety_counter << std::endl;
	float p_safety = float(p_safety_counter)/ float(attempt_lim);
	float exponent = (W1 - W2*(1-p_safety));
	float mod_factor = std::exp(exponent);

	float mod_cmd_v = input_cmd_v * mod_factor;
	float mod_cmd_w = input_cmd_w * mod_factor;


	return {mod_cmd_v, mod_cmd_w};
}
