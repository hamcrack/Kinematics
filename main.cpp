#include "challenge.h"

#include <iostream>
#include <cstdlib> 
#include <random>
#include <ctime>

int main()
{
	Kinematics k;
	k.set_debug(false);
	k.set_error(0.001);

    // Part 1 - Forward kinematics
        std::cout << "Part 1 - Forward kinematics" << std::endl;

	// Joint values set
	double a1 = 10;
	double a2 = 20;
	double a3 = 30;
	double a4 = 0;
	double d5 = 0;
	unsigned int a1_index = 0;
	unsigned int a2_index = 1;
	unsigned int a3_index = 2;
	unsigned int a4_index = 3;
	k.set_joint_angle(a1_index, a1);
	k.set_joint_angle(a2_index, a2);
	k.set_joint_angle(a3_index, a3);
	k.set_joint_angle(a4_index, a4);
	k.set_extention_dist(d5);

	// Calculate end effector position
	std::cout << "a1 set to: " << a1 << std::endl;
	std::cout << "a2 set to: " << a2 << std::endl;
	std::cout << "a3 set to: " << a3 << std::endl;
	std::cout << "a4 set to: " << a4 << std::endl;
	std::cout << "d5 set to: " << d5 << std::endl;
	std::cout << "End effector pos = " << std::endl;
	std::cout << k.my_frame_to_challenge_frame(k.get_end_pos()) << std::endl;
	std::cout << std::endl;

    // Part 2 - Analytical inverse kinematics
        std::cout << "Part 2 - Analytical inverse kinematics" << std::endl;

	// The result from part one is given as the target position for part two to check solution
	Eigen::Vector3d target_pos;
	target_pos = k.get_end_pos();
	std::cout << "Target pos: " << std::endl;
	std::cout << k.my_frame_to_challenge_frame(target_pos) << std::endl;

	// All joints set to zero
	k.set_joint_angle(a1_index, 0);
	k.set_joint_angle(a2_index, 0);
	k.set_joint_angle(a3_index, 0);
	k.set_joint_angle(a4_index, 0);
	k.set_extention_dist(0);

	// Re-calculate joint values
	if (k.set_end_pos(target_pos)){
		std::cout << "a1 = " << k.get_joint_angle(a1_index) << std::endl;
		std::cout << "a2 = " << k.get_joint_angle(a2_index) << std::endl;
		std::cout << "a3 = " << k.get_joint_angle(a3_index) << std::endl;
		std::cout << "a4 = " << k.get_joint_angle(a4_index) << std::endl;
		std::cout << "d5 = " << k.get_extention_dist() << std::endl;
		std::cout << std::endl;
	} else {
		std::cout << "Target position out of range!" << std::endl;
		std::cout << std::endl;
	}
    
    // Part 3 - Numerical inverse kinematics
        std::cout << "Part 3 - Numerical inverse kinematics" << std::endl;

	// Random start position (could be in the floor but should be comply with joint limits)
	std::srand(std::time(nullptr));
	a1 = ((double)rand()/(double)RAND_MAX) * 180 - 90;
	a2 = ((double)rand()/(double)RAND_MAX) * 180 - 90;
	a3 = ((double)rand()/(double)RAND_MAX) * 270 - 135;
	a4 = ((double)rand()/(double)RAND_MAX) * 270 - 135;
	d5 = ((double)rand()/(double)RAND_MAX) * 0.05;
	k.set_joint_angle(a1_index, a1);
	k.set_joint_angle(a2_index, a2);
	k.set_joint_angle(a3_index, a3);
	k.set_joint_angle(a4_index, a4);
	k.set_extention_dist(d5);
	std::cout << "a1 set to: " << a1 << std::endl;
	std::cout << "a2 set to: " << a2 << std::endl;
	std::cout << "a3 set to: " << a3 << std::endl;
	std::cout << "a4 set to: " << a4 << std::endl;
	std::cout << "d5 set to: " << d5 << std::endl;
	std::cout << "Initial end effector pos: " << std::endl;
	std::cout << k.my_frame_to_challenge_frame(k.get_end_pos()) << std::endl;

	// Moves to target position if the target is valid (in reach and not in the floor)
	std::cout << "Target pos: " << std::endl;
	std::cout << k.my_frame_to_challenge_frame(target_pos) << std::endl;
	if (k.move_end_to(target_pos)){
		std::cout << "a1 = " << k.get_joint_angle(a1_index) << std::endl;
		std::cout << "a2 = " << k.get_joint_angle(a2_index) << std::endl;
		std::cout << "a3 = " << k.get_joint_angle(a3_index) << std::endl;
		std::cout << "a4 = " << k.get_joint_angle(a4_index) << std::endl;
		std::cout << "d5 = " << k.get_extention_dist() << std::endl;
	} else {
		std::cout << "Target position could not be reached!" << std::endl;
		std::cout << std::endl;
	}
	
	return 0;	
}
