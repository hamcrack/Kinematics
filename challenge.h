#ifndef challenge_h
#define challenge_h

#include <Eigen/Dense>
#include <math.h>
#include <iostream>

#define PI 3.14159265

class Kinematics
{
public:
	Kinematics();
	~Kinematics();

	void set_debug(bool flag);
	void set_error(double error);
	bool set_end_pos(Eigen::Vector3d pos);
	void set_joint_angle(unsigned int joint, double delta);
	void set_extention_dist(double dist);

	Eigen::Vector3d get_end_pos();
	double get_joint_angle(unsigned int joint);
	double get_extention_dist();

	bool move_end_to(Eigen::Vector3d pos);

	Eigen::Vector3d my_frame_to_challenge_frame(Eigen::Vector3d my_frame);
	Eigen::Vector3d challenge_frame_to_my_frame(Eigen::Vector3d challenge_frame);	

private:
    // Debug Flag
        bool m_debug;

    // Error 
        double m_error;

    // Dimentions (m)
	double m_a1;
	double m_a2;
	double m_a3;
	double m_a4;
	double m_a5;

    // Joint Angles (deg)
	Eigen::Vector4d m_deltas;
    
    // Joint Extentions (m)
        double m_d5;

    // Rotation Matrices
	Eigen::Matrix3d m_R0_1;
	Eigen::Matrix3d m_R1_2;
	Eigen::Matrix3d m_R2_3;
	Eigen::Matrix3d m_R3_4;
	Eigen::Matrix3d m_R4_5;

    // Displacement Vectors
	Eigen::Vector3d m_D0_1;
	Eigen::Vector3d m_D1_2;
	Eigen::Vector3d m_D2_3;
	Eigen::Vector3d m_D3_4;
	Eigen::Vector3d m_D4_5;

    // Homogeneous Transformation Matrix
        Eigen::Matrix4d m_H0_1;
	Eigen::Matrix4d m_H1_2;
	Eigen::Matrix4d m_H2_3;
	Eigen::Matrix4d m_H3_4;
	Eigen::Matrix4d m_H4_5;

    // Jacobian Matrix
    	Eigen::MatrixXd m_J{6, 5};

    // End Effector Position
	Eigen::Vector3d m_end_pos;

	void set_R0_1();
	void set_R1_2();
	void set_R2_3();
	void set_R3_4();
	void set_R4_5();

	void set_D0_1();
	void set_D1_2();
	void set_D2_3();
	void set_D3_4();
	void set_D4_5();

	void set_H0_1();
	void set_H1_2();
	void set_H2_3();
	void set_H3_4();
	void set_H4_5();

	bool set_jacobian_mat();
	
	bool pos_valid(Eigen::Vector3d pos);
};
#endif
