// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

// added force sensor
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/joint_robot.urdf";

// State Machine Definition
#define INITIAL_POS				1 // Starting position
#define MOVING 						2	// Moving with nozzle up (joint space)
#define POSITION_HOLD			3	// Pause for 100 controller loop cycles
// #define NOZZLE_DOWN				4	// Move nozzle down until force felt exceeds threshold (task space)
// #define POURING_RIGHT			5	// Track the groove and move right w/nozzle down until force felt changes (task space?) -- or just move to next spot
// #define NOZZLE_UP					6	// Move nozzle up back to initial position

int state = INITIAL_POS;


// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// ORIENTATIONS (robot facing directions)
double WEST = 0.0;
double EAST = 3.1415;
double NORTH = -1.5708;
double SOUTH = 1.5708;


unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::robot::mmp_panda::actuators::fgc";

	// added force sensor
	const std::string EE_FORCE_KEY = "cs225a::sensor::force";
	const std::string EE_MOMENT_KEY = "cs225a::sensor::moment";

	// Add state tracking for simulation
	const std::string ACTIVE_STATE_KEY = "active_state";
	const std::string NOZZLE_POS_KEY = "nozzle_pos";
	Eigen::Vector3d nozzle_pos_vec;

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// output force sensor values
	ofstream force_outputs;
	force_outputs.open("../../force_outputs.txt");
	ofstream moment_outputs;
	moment_outputs.open("../../moment_outputs.txt");

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// load force sensor
	Eigen::Vector3d sensed_force;
	Eigen::Vector3d sensed_moment;
	sensed_force = redis_client.getEigenMatrixJSON(EE_FORCE_KEY);
	sensed_moment = redis_client.getEigenMatrixJSON(EE_MOMENT_KEY);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// Read positions from path file
	uint32_t num_pos = 500;
	MatrixXd positions(num_pos, 4);
	fstream pathfile;
	pathfile.open("/Users/ruta/stanford/cs225a/sai/core/sai2-planning/pytest/rrt_path.txt");
	if (pathfile.is_open()) {
		uint32_t i = 0;
		for(std::string line; std::getline(pathfile, line); ) {
			std::istringstream in(line);
			float x, y;
			in >> x;
			in.ignore();
			in >> y;

			positions(i, 0) = x;
			positions(i, 1) = y;
			positions(i, 2) = NORTH;
			positions(i, 3) = 0;
			i++;
		}
		pathfile.close();
	}
	cout << "positions = " << positions << "\n";
	cout << "positions.row(0) = " << positions.row(0) << "\n";


	// controller desired positions
	double tolerance = 0.01;
	Vector3d x_des = Vector3d::Zero(3);
	MatrixXd ori_des = Matrix3d::Zero();

	/*** SET UP JOINT TASK ***/
	auto joint_task = new Sai2Primitives::JointTask(robot);

	#ifdef USING_OTG
		joint_task->_use_interpolation_flag = false;
	#else
		joint_task->_use_velocity_saturation_flag = true;
	#endif
  	joint_task->_saturation_velocity << M_PI/3, M_PI/3, M_PI/3; //M_PI/12, M_PI/12, M_PI/12; // set new slower velocity (to help visualize)


	// controller gains
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 50.0; //15.0;
	joint_task->setDynamicDecouplingFull();

	// controller desired angles
	VectorXd q_des = VectorXd::Zero(dof);

	/*** BEGIN LOOP ***/
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	double lastStopped = controller_counter; // Update this when you stop
	double pauseCounter = 0;
	double pc = 0;
	int nozzle_pos = 0; // 0 is up, 1 is down
	// int nozzle_ori = 0; // 0 is horizontal, 1 is vertical
	redis_client.set(ACTIVE_STATE_KEY, "INITIAL");

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();

		if (state == INITIAL_POS) {
			VectorXd newPos = positions.row(pc);
			q_des << newPos;
			cout << "qdes = " << q_des << "\n";
			state = MOVING;
		}

		else if (state == MOVING) {
			// cout << "Made it to moving state\n";
			if((robot->_q - q_des).norm() < tolerance){ // check if goal position reached
				pc++;
				state = POSITION_HOLD;
				cout << "Reached position: " << q_des << "\n";
			}
		}

		else if (state == POSITION_HOLD) {
			// cout << "Made it to position hold\n";
			if (pc < num_pos) {
				// joint_task->reInitializeTask();
				VectorXd newPos = positions.row(pc);
				q_des << newPos;
				cout << "qdes = " << q_des << "\n";
				state = MOVING;
			}
		}

		if (state == MOVING) {
			/* Primary Joint Task Control */
			/* Invoked when we want the base to move and the nozzle to maintain position. */
			joint_task->_desired_position = q_des;

			// update task model and set hierarchy
			// N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;
		}
		else if (state == POSITION_HOLD) {
			// Maintain all joint angles at the current position
			joint_task->_desired_position = robot->_q;

			// update task model and set hierarchy
			// N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	// close files
	force_outputs.close();
	moment_outputs.close();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
