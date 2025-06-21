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
float gaussian(float input_val, float mean, float std_dev) {

    static std::random_device rd;        // random seed
    static std::mt19937 generator(rd()); // static so only once
    std::normal_distribution<float> distribution(mean, std_dev);
    input_val = input_val * (1 + distribution(generator)); // small value - negative value = -value,
    return input_val;
}
double normalize_angle(double angle_rad) {
    while (angle_rad > M_PI)
        angle_rad -= 2.0 * M_PI;
    while (angle_rad < -M_PI)
        angle_rad += 2.0 * M_PI;
    return angle_rad;
}

bool circles_intersect(const Obstacle &t_bot, const Obstacle &obstacle) {
    float dx = t_bot.center_x - obstacle.center_x;
    float dy = t_bot.center_y - obstacle.center_y;
    float distanceSquared = dx * dx + dy * dy; // Avoids unnecessary sqrt
    float radiusSum = t_bot.radius + obstacle.radius;

    return distanceSquared <= (radiusSum * radiusSum);
}

std::pair<float, float> modulation_onnx_lidar(float odom_x, float odom_y, float input_cmd_v, float input_cmd_w,
                                              const double *lidar_ranges, const size_t num_lidars) {

    /* Function takes current localation, outs of neural network and calculates
     * the modulation factor then applies
     *
     *
     *
     *
     *
     */
    if (lidar_ranges == nullptr) {
        return {input_cmd_v, input_cmd_w};
    }
    // modulate low angular velocity
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

    float angle_step = 2.0f * M_PI / num_lidars;
    for (int attempts = 0; attempts < attempt_lim; attempts++) {

        noisy_cmd_v = gaussian(input_cmd_v, 0, .1);
        noisy_cmd_w = gaussian(input_cmd_w, 0, .1);

        // do i need to propegate in the future
        x_future = odom_x + noisy_cmd_v * cos(noisy_cmd_w) * TIMESTEP *
                                10; // does it matter, how much accuracy does it mater to factor in each odom_initial
        y_future = odom_y + noisy_cmd_v * sin(noisy_cmd_w) * TIMESTEP * 10;

        T_BOT.center_x = x_future;
        T_BOT.center_y = y_future;

        std::cout << "x future, y future" << x_future << "  " << y_future << std::endl;

        for (size_t lidar_counter = 0; lidar_counter < num_lidars; lidar_counter++) {

            // float theta_prev = map_yaw - M_PI / 2 + angle_step * index;
            double theta = angle_step * lidar_counter;

            float proj_x = odom_x + lidar_ranges[lidar_counter] * static_cast<float>(cos(theta));
            float proj_y = odom_y + lidar_ranges[lidar_counter] * static_cast<float>(sin(theta));
            Obstacle currentObs;
            currentObs.center_x = proj_x;
            currentObs.center_y = proj_y;
            currentObs.radius = .1;
            if (circles_intersect(T_BOT, currentObs)) {
                p_safety_counter++;
                break;
            }
        }
    }
    // Equation in the paper is e^(w1-w2(1-P(safety)), where no collisions equals 150 speed up
    // versus 50 percent slow down
    std::cout << "num of collisions" << p_safety_counter << std::endl;
    float p_safety = float(p_safety_counter) / float(attempt_lim);
    float exponent = (W1 - W2 * (p_safety));
    float mod_factor = std::exp(exponent);

    std::cout << "mod factor is " << mod_factor << std::endl;

    float mod_cmd_v = input_cmd_v * mod_factor;
    float mod_cmd_w = input_cmd_w * mod_factor;

    return {mod_cmd_v, mod_cmd_w};
}
std::pair<float, float> modulation_onnx(float odom_x, float odom_y, float input_cmd_v, float input_cmd_w,
                                        std::vector<Obstacle> obstacle_data) {

    /* Function takes current localation, outs of neural network and calculates
     * the modulation factor then applies
     *
     *
     *
     *
     *
     */
    if (obstacle_data.empty()) {
        return {input_cmd_v, input_cmd_w};
    }
    // modulate low angular velocity
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

        x_future = odom_x + noisy_cmd_v * cos(noisy_cmd_w) * TIMESTEP *
                                10; // does it matter, how much accuracy does it mater to factor in each odom_initial
        y_future = odom_y + noisy_cmd_v * sin(noisy_cmd_w) * TIMESTEP * 10;

        T_BOT.center_x = x_future;
        T_BOT.center_y = y_future;

        std::cout << "x future, y future" << x_future << "  " << y_future << std::endl;

        for (size_t obs_counter = 0; obs_counter < obstacle_data.size(); obs_counter++) {
            if (circles_intersect(T_BOT, obstacle_data[obs_counter])) {
                p_safety_counter++;
                break;
            }
        }
    }
    // Equation in the paper is e^(w1-w2(1-P(safety)), where no collisions equals 150 speed up
    // versus 50 percent slow down
    std::cout << "num of collisions" << p_safety_counter << std::endl;
    float p_safety = float(p_safety_counter) / float(attempt_lim);
    float exponent = (W1 - W2 * (p_safety));
    float mod_factor = std::exp(exponent);

    std::cout << "mod factor is " << mod_factor << std::endl;

    float mod_cmd_v = input_cmd_v * mod_factor;
    float mod_cmd_w = input_cmd_w * mod_factor;

    return {mod_cmd_v, mod_cmd_w};
}
