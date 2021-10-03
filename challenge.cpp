#include "challenge.h"

Kinematics::Kinematics()
{
	m_debug = false;
	
	m_error = 0.001;

	m_a1 = 0.30;
	m_a2 = 0.00;
	m_a3 = 0.40;
	m_a4 = 0.35;
	m_a5 = 0.05;

	m_deltas << 0, 0, 0, 0;

	m_d5 = 0;
}

Kinematics::~Kinematics()
{

}

void Kinematics::set_debug(bool flag)
{
	m_debug = flag;
}

void Kinematics::set_error(double error)
{
	m_error = error;
}

bool Kinematics::set_end_pos(Eigen::Vector3d pos)
{
	bool in_range = true;

	if(pos_valid(pos)){
		m_end_pos = pos;

		double d1 = sqrt(pow(m_end_pos(0), 2) + pow(m_end_pos(1), 2));
		double delta_rad1 = atan2(m_end_pos(1), m_end_pos(0));
		if (delta_rad1 < (-90 * PI) / 180 || delta_rad1 > (90 * PI) / 180){
			std::cout << "WARNING: a1 out of range: " << (delta_rad1 * 180) / PI << std::endl;
			in_range = false;
		}
		m_deltas(0) = delta_rad1;

		double r1 = sqrt(pow((d1 - m_a2), 2) + pow((m_end_pos(2) - m_a1),2));
		double sigma1 = acos((pow((m_a4+m_a5), 2) - pow(m_a3, 2) - pow(r1, 2))/(-2*m_a3*r1));
		double sigma2 = atan((m_end_pos(2) - m_a1)/(d1 - m_a2));
		double delta_rad2 = sigma2 - sigma1;
		if (delta_rad2 < (-90 * PI) / 180 || delta_rad2 > (90 * PI) / 180){
			std::cout << "WARNING: a2 out of range: " << (delta_rad2 * 180) / PI << std::endl;
			in_range = false;
		}
		m_deltas(1) = delta_rad2;

		double c_rule_val = (pow(r1, 2) - pow(m_a3, 2) - pow((m_a4 + m_a5), 2))/(-2*m_a3*(m_a4+m_a5));
		double sigma3 = 0;
		if (c_rule_val<=-1.0) {
			sigma3 = PI;
		} else if (c_rule_val<1.0) {
			sigma3 = acos(c_rule_val);
		}
		double delta_rad3 = PI - sigma3;
		if (delta_rad3 < (-135 * PI) / 180 || delta_rad3 > (135 * PI) / 180){
			std::cout << "WARNING: a3 out of range: " << (delta_rad3 * 180) / PI << std::endl;
			in_range = false;
		}
		m_deltas(2) = delta_rad3;

		if (m_debug){
			std::cout << "pos = " << pos << std::endl;

			std::cout << "d1 = " << d1 << std::endl;
			std::cout << "delta_rad1 = " <<delta_rad1 << std::endl;

			std::cout << "r1 = " << r1 << std::endl;
			std::cout << "delta_rad2 = " << delta_rad2 << std::endl;
			
			std::cout << "sigma3 = " << sigma3 << std::endl;
			std::cout << "delta_rad3 = " << delta_rad3 << std::endl;
		}
	} else {
		in_range = false;
	}
	return in_range;
}

void Kinematics::set_joint_angle(unsigned int joint, double delta)
{
	m_deltas(joint) = (delta * PI) / 180;
}

void Kinematics::set_extention_dist(double dist)
{
	m_d5 = dist;
}

Eigen::Vector3d Kinematics::get_end_pos()
{
	set_R0_1();
	set_R1_2();
	set_R2_3();
	set_R3_4();
	set_R4_5();

	set_D0_1();
	set_D1_2();
	set_D2_3();
	set_D3_4();
	set_D4_5();

	set_H0_1();
	set_H1_2();
	set_H2_3();
	set_H3_4();
	set_H4_5();

	Eigen::Matrix4d H0_2 = m_H0_1 * m_H1_2;
	Eigen::Matrix4d H0_3 = H0_2 * m_H2_3;
	Eigen::Matrix4d H0_4 = H0_3 * m_H3_4;
	Eigen::Matrix4d H0_5 = H0_4 * m_H4_5;

	if (H0_2(2, 3) < 0){
		std::cout << "WARNING: Floor collition detected at joint 2! " << std::endl;	
	}
	if (H0_3(2, 3) < 0){
		std::cout << "WARNING: Floor collition detected at joint 3! " << std::endl;	
	}
	if (H0_4(2, 3) < 0){
		std::cout << "WARNING: Floor collition detected at joint 4! " << std::endl;	
	}
	if (H0_5(2, 3) < 0){
		std::cout << "WARNING: Floor collition detected at joint 5! " << std::endl;	
	}

	if (m_debug){
		std::cout << "H0_2: " << std::endl;
		std::cout << H0_2 << std::endl;
	
		std::cout << "H0_3: " << std::endl;
		std::cout << H0_3 << std::endl;

		std::cout << "H0_4: " << std::endl;
		std::cout << H0_4 << std::endl;
	
		std::cout << "H0_5: " << std::endl;
		std::cout << H0_5 << std::endl;
	}
	m_end_pos << H0_5(0, 3), H0_5(1, 3), H0_5(2, 3);

	return m_end_pos;
}

double Kinematics::get_joint_angle(unsigned int joint)
{
	return (m_deltas(joint) * 180) / PI;
}

double Kinematics::get_extention_dist()
{
	return (m_d5);
}

Eigen::Vector3d Kinematics::my_frame_to_challenge_frame(Eigen::Vector3d my_frame)
{
	Eigen::Vector3d challenge_frame;
        challenge_frame	<< my_frame(0), my_frame(2), -my_frame(1);
	return challenge_frame;
}

Eigen::Vector3d Kinematics::challenge_frame_to_my_frame(Eigen::Vector3d challenge_frame)
{
	Eigen::Vector3d my_frame;
	my_frame << challenge_frame(0), -challenge_frame(2), challenge_frame(1);
	return my_frame;
}


void Kinematics::set_R0_1()
{
	m_R0_1(0, 0) = cos(m_deltas(0));
	m_R0_1(0, 1) = 0;
	m_R0_1(0, 2) = sin(m_deltas(0));
	m_R0_1(1, 0) = sin(m_deltas(0));
	m_R0_1(1, 1) = 0;
	m_R0_1(1, 2) = -cos(m_deltas(0));
	m_R0_1(2, 0) = 0;
	m_R0_1(2, 1) = 1;
	m_R0_1(2, 2) = 0;
  
	if (m_debug){
		std::cout << "m_R0_1: " << std::endl;
		std::cout << m_R0_1 << std::endl;
	}
}

void Kinematics::set_R1_2()
{
	m_R1_2(0, 0) = cos(m_deltas(1));
	m_R1_2(0, 1) = -sin(m_deltas(1));
	m_R1_2(0, 2) = 0;
	m_R1_2(1, 0) = sin(m_deltas(1));
	m_R1_2(1, 1) = cos(m_deltas(1));
	m_R1_2(1, 2) = 0;
	m_R1_2(2, 0) = 0;
	m_R1_2(2, 1) = 0;
	m_R1_2(2, 2) = 1;
        
	if (m_debug){
		std::cout << "m_R1_2: " << std::endl;
		std::cout << m_R1_2 << std::endl;
	}
}

void Kinematics::set_R2_3()
{
	m_R2_3(0, 0) = cos(m_deltas(2));
	m_R2_3(0, 1) = -sin(m_deltas(2));
	m_R2_3(0, 2) = 0;
	m_R2_3(1, 0) = sin(m_deltas(2));
	m_R2_3(1, 1) = cos(m_deltas(2));
	m_R2_3(1, 2) = 0;
	m_R2_3(2, 0) = 0;
	m_R2_3(2, 1) = 0;
	m_R2_3(2, 2) = 1;

	if (m_debug){	
		std::cout << "m_R2_3: " << std::endl;
		std::cout << m_R2_3 << std::endl;
	}
}

void Kinematics::set_R3_4()
{
	m_R3_4(0, 0) = -sin(m_deltas(3));
	m_R3_4(0, 1) = 0;
	m_R3_4(0, 2) = cos(m_deltas(3));
	m_R3_4(1, 0) = cos(m_deltas(3));
	m_R3_4(1, 1) = 0;
	m_R3_4(1, 2) = sin(m_deltas(3));
	m_R3_4(2, 0) = 0;
	m_R3_4(2, 1) = 1;
	m_R3_4(2, 2) = 0;

	if (m_debug){	
		std::cout << "m_R3_4: " << std::endl;
		std::cout << m_R3_4 << std::endl;
	}
}

void Kinematics::set_R4_5()
{
	m_R4_5(0, 0) = 1;
	m_R4_5(0, 1) = 0;
	m_R4_5(0, 2) = 0;
	m_R4_5(1, 0) = 0;
	m_R4_5(1, 1) = 1;
	m_R4_5(1, 2) = 0;
	m_R4_5(2, 0) = 0;
	m_R4_5(2, 1) = 0;
	m_R4_5(2, 2) = 1;
	
	if (m_debug){
		std::cout << "m_R4_5: " << std::endl;
		std::cout << m_R4_5 << std::endl;
	}
}

void Kinematics::set_D0_1()
{
	m_D0_1 << m_a2 * cos(m_deltas(0)), m_a2 * sin(m_deltas(0)), m_a1;
	
	if (m_debug){	
		std::cout << "m_D0_1: " << std::endl;
		std::cout << m_D0_1 << std::endl;
	}
}

void Kinematics::set_D1_2()
{
	m_D1_2 << m_a3 * cos(m_deltas(1)), m_a3 * sin(m_deltas(1)), 0;
	
	if (m_debug){
		std::cout << "m_D1_2: " << std::endl;
		std::cout << m_D1_2 << std::endl;
	}
}

void Kinematics::set_D2_3()
{
	m_D2_3 << m_a4 * cos(m_deltas(2)), m_a4 * sin(m_deltas(2)), 0;
	
	if (m_debug){
		std::cout << "m_D2_3: " << std::endl;
		std::cout << m_D2_3 << std::endl;
	}
}

void Kinematics::set_D3_4()
{
	m_D3_4 << 0, 0, 0;
	
	if (m_debug){
		std::cout << "m_D3_4: " << std::endl;
		std::cout << m_D3_4 << std::endl;
	}
}

void Kinematics::set_D4_5()
{
	m_D4_5 << 0, 0, m_a5 + m_d5;

	if (m_debug){
		std::cout << "m_D4_5: " << std::endl;
		std::cout << m_D4_5 << std::endl;
	}
}

void Kinematics::set_H0_1()
{
	m_H0_1 << m_R0_1(0, 0), m_R0_1(0, 1), m_R0_1(0, 2), m_D0_1(0),
	          m_R0_1(1, 0), m_R0_1(1, 1), m_R0_1(1, 2), m_D0_1(1),
		  m_R0_1(2, 0), m_R0_1(2, 1), m_R0_1(2, 2), m_D0_1(2), 
		  0, 0, 0, 1;
	
	if (m_debug){
		std::cout << "m_H0_1: " << std::endl;
		std::cout << m_H0_1 << std::endl;
	}
}

void Kinematics::set_H1_2()
{
	m_H1_2 << m_R1_2(0, 0), m_R1_2(0, 1), m_R1_2(0, 2), m_D1_2(0),
	          m_R1_2(1, 0), m_R1_2(1, 1), m_R1_2(1, 2), m_D1_2(1),
		  m_R1_2(2, 0), m_R1_2(2, 1), m_R1_2(2, 2), m_D1_2(2), 
		  0, 0, 0, 1;
	
	if (m_debug){
		std::cout << "m_H1_2: " << std::endl;
		std::cout << m_H1_2 << std::endl;
	}
}

void Kinematics::set_H2_3()
{
	m_H2_3 << m_R2_3(0, 0), m_R2_3(0, 1), m_R2_3(0, 2), m_D2_3(0),
	          m_R2_3(1, 0), m_R2_3(1, 1), m_R2_3(1, 2), m_D2_3(1),
		  m_R2_3(2, 0), m_R2_3(2, 1), m_R2_3(2, 2), m_D2_3(2), 
		  0, 0, 0, 1;
	
	if (m_debug){
		std::cout << "m_H2_3: " << std::endl;
		std::cout << m_H2_3 << std::endl;
	}
}

void Kinematics::set_H3_4()
{
	m_H3_4 << m_R3_4(0, 0), m_R3_4(0, 1), m_R3_4(0, 2), m_D3_4(0),
	          m_R3_4(1, 0), m_R3_4(1, 1), m_R3_4(1, 2), m_D3_4(1),
		  m_R3_4(2, 0), m_R3_4(2, 1), m_R3_4(2, 2), m_D3_4(2), 
		  0, 0, 0, 1;

	if (m_debug){
		std::cout << "m_H3_4: " << std::endl;
		std::cout << m_H3_4 << std::endl;
	}
}

void Kinematics::set_H4_5()
{
	m_H4_5 << m_R4_5(0, 0), m_R4_5(0, 1), m_R4_5(0, 2), m_D4_5(0),
	          m_R4_5(1, 0), m_R4_5(1, 1), m_R4_5(1, 2), m_D4_5(1),
		  m_R4_5(2, 0), m_R4_5(2, 1), m_R4_5(2, 2), m_D4_5(2), 
		  0, 0, 0, 1;

	if (m_debug){
		std::cout << "m_H4_5: " << std::endl;
		std::cout << m_H4_5 << std::endl;
	}
}

bool Kinematics::set_jacobian_mat()
{
	bool singularity = false;

	Eigen::Matrix4d H0_2 = m_H0_1 * m_H1_2;
	Eigen::Matrix4d H0_3 = H0_2 * m_H2_3;
	Eigen::Matrix4d H0_4 = H0_3 * m_H3_4;
	Eigen::Vector3d D0_2(H0_2(0, 3), H0_2(1, 3), H0_2(2, 3));
	Eigen::Vector3d D0_3(H0_3(0, 3), H0_3(1, 3), H0_3(2, 3));
	Eigen::Vector3d D0_4(H0_4(0, 3), H0_4(1, 3), H0_4(2, 3));
	Eigen::Matrix3d R0_2 {
		{H0_2(0, 0), H0_2(0, 1), H0_2(0, 2)},
		{H0_2(1, 0), H0_2(1, 1), H0_2(1, 2)},
		{H0_2(2, 0), H0_2(2, 1), H0_2(2, 2)}
	};
	Eigen::Matrix3d R0_3 {
		{H0_3(0, 0), H0_3(0, 1), H0_3(0, 2)},
		{H0_3(1, 0), H0_3(1, 1), H0_3(1, 2)},
		{H0_3(2, 0), H0_3(2, 1), H0_3(2, 2)}
	};
	Eigen::Matrix3d R0_4 {
		{H0_4(0, 0), H0_4(0, 1), H0_4(0, 2)},
		{H0_4(1, 0), H0_4(1, 1), H0_4(1, 2)},
		{H0_4(2, 0), H0_4(2, 1), H0_4(2, 2)}
	};

	Eigen::Vector3d Vec_z(0, 0, 1);
	Eigen::Vector3d J0_0 = Vec_z.cross(D0_2);
	m_J(0, 0) = J0_0(0);
	m_J(1, 0) = J0_0(1);
	m_J(2, 0) = J0_0(2);
	m_J(3, 0) = Vec_z(0);
	m_J(4, 0) = Vec_z(1);
	m_J(5, 0) = Vec_z(2);

	Eigen::Vector3d Vz_R1 = m_R0_1 * Vec_z;
	Eigen::Vector3d J0_1 = (Vz_R1).cross(D0_2 - m_D0_1);
	m_J(0, 1) = J0_1(0);
	m_J(1, 1) = J0_1(1);
	m_J(2, 1) = J0_1(2);
	m_J(3, 1) = Vz_R1(0);
	m_J(4, 1) = Vz_R1(1);
	m_J(5, 1) = Vz_R1(2);

	Eigen::Vector3d Vz_R2 = R0_2 * Vec_z;
	Eigen::Vector3d J0_2 = (Vz_R2).cross(D0_3 - D0_2);
	m_J(0, 2) = J0_2(0);
	m_J(1, 2) = J0_2(1);
	m_J(2, 2) = J0_2(2);
	m_J(3, 2) = Vz_R2(0);
	m_J(4, 2) = Vz_R2(1);
	m_J(5, 2) = Vz_R2(2);

	Eigen::Vector3d Vz_R3 = R0_3 * Vec_z;
	Eigen::Vector3d J0_3 = (Vz_R3).cross(D0_4 - D0_3);
	m_J(0, 3) = J0_3(0);
	m_J(1, 3) = J0_3(1);
	m_J(2, 3) = J0_3(2);
	m_J(3, 3) = Vz_R3(0);
	m_J(4, 3) = Vz_R3(1);
	m_J(5, 3) = Vz_R3(2);

	Eigen::Vector3d Vz_R4 = R0_4 * Vec_z;
	m_J(0, 4) = Vz_R4(0);
	m_J(1, 4) = Vz_R4(1);
	m_J(2, 4) = Vz_R4(2);
	m_J(3, 4) = 0;
	m_J(4, 4) = 0;
	m_J(5, 4) = 0;

	if ((m_J*m_J.transpose()).determinant() == 0){
		std::cout << "WARNING: Singularity detected!" << std::endl;
		singularity = true;
	}
	
	if (m_debug){
		std::cout << "Jacobian Matrix = " << std::endl;
		std::cout << m_J << std::endl;
	}
	return !singularity;
}

bool Kinematics::move_end_to(Eigen::Vector3d target_pos)
{
	if (pos_valid(target_pos)){
		Eigen::Vector3d pos_dif = target_pos - m_end_pos;
		unsigned int singularity_cnt = 0;
		unsigned int loop_cnt = 0;

		while (pos_dif.norm() > m_error){
			if (!set_jacobian_mat()){
				singularity_cnt += 1;
				if (singularity_cnt > 9){
					return false;
				}
			}
			Eigen::MatrixXd sudo_inv_J = m_J.completeOrthogonalDecomposition().pseudoInverse();
			if (m_debug){
				std::cout << "Sudo inverse J: " << std::endl;
				std::cout << sudo_inv_J << std::endl;
				std :: cout << std::endl;
			}

			Eigen::VectorXd delta_v(6);
			delta_v(0) = pos_dif(0);
			delta_v(1) = pos_dif(1);
			delta_v(2) = pos_dif(2);
			delta_v(3) =  0;
			delta_v(4) =  0;
			delta_v(5) =  0;
			if (m_debug){
				std::cout << "Pose change: " << std::endl;
				std::cout << delta_v << std::endl;
				std :: cout << std::endl;
			}

			Eigen::VectorXd delta_a(5); 
			delta_a = sudo_inv_J * delta_v;
			if (m_debug){
				std::cout << "Joint changes: " << std::endl;
				std::cout << delta_a << std::endl;
				std::cout << std::endl;
			}

			m_deltas(0) += delta_a(0);
			if (m_deltas(0) < (-90 * PI) / 180){
				m_deltas(0) = (-90 * PI) / 180;
			}
			if (m_deltas(0) > (90 * PI) / 180){
				m_deltas(0) = (90 * PI) / 180;
			}

			m_deltas(1) += delta_a(1);
			if (m_deltas(1) < (-90 * PI) / 180){
				m_deltas(1) = (-90 * PI) / 180;
			}
			if (m_deltas(1) > (90 * PI) / 180){
				m_deltas(1) = (90 * PI) / 180;
			}

			m_deltas(2) += delta_a(2);
			if (m_deltas(2) < (-135 * PI) / 180){
				m_deltas(2) = (-135 * PI) / 180;
			}
			if (m_deltas(2) > (135 * PI) / 180){
				m_deltas(2) = (135 * PI) / 180;
			}

			m_deltas(3) += delta_a(3);
			if (m_deltas(3) < (-135 * PI) / 180){
				m_deltas(3) = (-135 * PI) / 180;
			}
			if (m_deltas(3) > (135 * PI) / 180){
				m_deltas(3) = (135 * PI) / 180;
			}

			m_d5 += delta_a(4);
			if (m_d5 < 0.0){
				m_d5 = 0.0;
			}
			if (m_d5 > 0.05){
				m_d5 = 0.05;
			}

			pos_dif = target_pos - get_end_pos();

			loop_cnt += 1;
			if (loop_cnt > 500){
				return false;
			}

			if (loop_cnt % 10 == 0){
				std::cout << "Pos Error = " << pos_dif.norm() << std::endl;
				if (m_debug){
					std::cout << "Joint positions = " << get_joint_angle(0) << ", " << get_joint_angle(1)  << ", " << get_joint_angle(2)  << ", " << get_joint_angle(3) << ", " << m_d5 << std::endl;
				}
			}
		}
		return true;
	} else {
		return false;
	}

}

bool Kinematics::pos_valid(Eigen::Vector3d pos)
{
	bool valid = true;

	if (pos(2) < 0){
		valid = false;	
		if (true){
			std::cout << "Out of range: in ground plane" << std::endl;
		}
	}
	
	double z_rot = atan(pos(1)/pos(0));
	
	Eigen::Matrix3d rot_mat_z;
	rot_mat_z << cos(-z_rot), -sin(-z_rot), 0,
		     sin(-z_rot), cos(-z_rot), 0, 
		     0, 0, 1;
		
	Eigen::Vector3d pos_2d = rot_mat_z * pos;
	if (m_debug){
		std::cout << "Rotated pos on y plane: " << std::endl;
		std::cout << pos_2d << std::endl;
	}

	if (sqrt(pow(pos_2d(0), 2) + pow((pos_2d(2) - m_a1), 2)) < sqrt(pow(m_a3, 2) + pow(m_a4, 2) - (2*m_a3*m_a4*cos(PI/4))) - (m_a5 + 0.05)){
		valid = false;
		if (true){
			std::cout << "Out of range: inner radius" << std::endl;
		}
	}
	if (pos_2d(0) >= 0){
		if (sqrt(pow(pos_2d(0), 2) + pow((pos_2d(2) - m_a1), 2)) > (m_a1 + m_a3 + m_a4 + m_a5 + 0.05)){
		        valid = false;
			if (true){
				std::cout << "Out of range: positive x" << std::endl;
			}
		}
	} else if (atan2(-pos_2d(0), pos_2d(2) - (m_a1 + m_a3)) <= (135 * PI) / 180){
		if (sqrt(pow(pos_2d(0), 2) + pow((pos_2d(2) - (m_a1 + m_a3)), 2)) > (m_a4 + m_a5 + 0.05)){
	        	valid = false;
			if (true){
				std::cout << "Out of range: negative x, top" << std::endl;
			}
		}
	} else if (sqrt(pow(pos_2d(0) + (sin(PI/4) * m_a4), 2) + pow(pos_2d(2) - (m_a1 + m_a3) - (cos(PI/4) * m_a4), 2)) > (m_a5 + 0.05)){
		valid = false;
		if (true){
			std::cout << "Out of range: negative x, bottom" << std::endl;
		}
	}
	return valid;
}
