#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <iostream>

static const double Pos[][3] = {{0.6, 0.3, 0.4},{0.3, 0.3, 0.4},{0.0, 0.3, 0.4},{-0.3, 0.3, 0.4},{-0.6, 0.3, 0.4}};

void go_to_position(int position, moveit::planning_interface::MoveGroupInterface *interface_arm)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Pose target_pose;

	//Get the current pose and copy its orientation
	current_pose = interface_arm->getCurrentPose("ee_link");
	target_pose.orientation = current_pose.pose.orientation;

	//Paste the position from the presets
	target_pose.position.x = Pos[position][0];
	target_pose.position.y = Pos[position][1];
	target_pose.position.z = Pos[position][2];
	interface_arm->setPoseTarget(target_pose);

	//Start planning
	auto planningResult = interface_arm->plan(my_plan_arm);

	//Check if the planning was successful
	if (planningResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Planning failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Planning success!");

	auto movementResult = interface_arm->move();

	//Check if the movement was successful
	if (movementResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Movement failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Movement success!");
}

void go_home(moveit::planning_interface::MoveGroupInterface *interface_arm, moveit::planning_interface::MoveGroupInterface *interface_gripper)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

	//Find the home position preset and set it as the target
	auto homePosition = interface_arm->getNamedTargetValues("home");
	interface_arm->setJointValueTarget(homePosition);

	//Start planning
	auto planningResult = interface_arm->plan(my_plan_arm);

	//Check if the planning was successful
	if (planningResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Planning failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Planning success!");

	auto movementResult = interface_arm->move();

	//Check if the movement was successful
	if (movementResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Movement failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Movement success!");
}

void go_down(moveit::planning_interface::MoveGroupInterface *interface_arm, moveit::planning_interface::MoveGroupInterface *interface_gripper)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Pose target_pose;

	//Get the current pose and copy its orientation
	current_pose = interface_arm->getCurrentPose("ee_link");
	target_pose.orientation = current_pose.pose.orientation;

	//Set the position to the current pose
	target_pose.position = current_pose.pose.position;
	//Move the target Z to 0.2 (down)
	target_pose.position.z = 0.2;

	interface_arm->setPoseTarget(target_pose);

	//Start planning
	auto planningResult = interface_arm->plan(my_plan_arm);

	//Check if the planning was successful
	if (planningResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_WARN_NAMED("movement_test", "Planning failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Planning success!");

	auto movementResult = interface_arm->move();

	//Check if the movement was successful
	if (movementResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_WARN_NAMED("movement_test", "Movement failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Movement success!");
}

void go_up(moveit::planning_interface::MoveGroupInterface *interface_arm, moveit::planning_interface::MoveGroupInterface *interface_gripper)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Pose target_pose;

	//Get the current pose and copy its orientation
	current_pose = interface_arm->getCurrentPose("ee_link");
	target_pose.orientation = current_pose.pose.orientation;

	//Set the position to the current pose
	target_pose.position = current_pose.pose.position;
	//Move the target Z to 0.4 (normal height)
	target_pose.position.z = 0.4;

	interface_arm->setPoseTarget(target_pose);

	//Start planning
	auto planningResult = interface_arm->plan(my_plan_arm);

	//Check if the planning was successful
	if (planningResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Planning failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Planning success!");

	auto movementResult = interface_arm->move();

	//Check if the movement was successful
	if (movementResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Movement failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Movement success!");
}

void go_close(moveit::planning_interface::MoveGroupInterface *interface_arm, moveit::planning_interface::MoveGroupInterface *interface_gripper)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

	//Get the joint values for the closed preset
	interface_gripper->setJointValueTarget(interface_gripper->getNamedTargetValues("closed"));
	//Start planning
	auto planningResult = interface_gripper->plan(my_plan_gripper);

	//Check if the planning was successful
	if (planningResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Planning failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Planning success!");

	auto movementResult = interface_gripper->move();

	//Check if the movement was successful
	if (movementResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Movement failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Movement success!");
}

void go_open(moveit::planning_interface::MoveGroupInterface *interface_arm, moveit::planning_interface::MoveGroupInterface *interface_gripper)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

	//Get the joint values for the open preset
	auto openPosition = interface_gripper->getNamedTargetValues("open");
	interface_gripper->setJointValueTarget(openPosition);
	//Start planning
	auto planningResult = interface_gripper->plan(my_plan_gripper);

	//Check if the planning was successful
	if (planningResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Planning failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Planning success!");

	auto movementResult = interface_gripper->move();

	//Check if the movement was successful
	if (movementResult != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR_NAMED("movement_test", "Movement failed!");
		return;
	}

	ROS_INFO_NAMED("movement_test", "Movement success!");

}

int main(int argc, char **argv)
{
	//Init ros connection
	ros::init(argc, argv, "movement_test");

	//Start the update spinner
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//Define constants for the planning group names
	static const std::string PLANNING_GROUP_ARM = "ur5_arm";
	static const std::string PLANNING_GROUP_GRIPPER = "gripper";

	//Create interfaces for these planning groups
	moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
	moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

	//Go to home
	go_home(&move_group_interface_arm, &move_group_interface_gripper);

	//create the input loop
	bool running = true;
	while (running)
	{
		//get command
		auto command = getchar();

		//We ignore the \n character
		if (command == '\n') continue;

		ROS_INFO_NAMED("movement_test", "got char: %c", command);

		switch (command)
		{
			case '1':	//go to position 1
				go_to_position(0, &move_group_interface_arm);
				break;
			case '2':	//go to position 2
				go_to_position(1, &move_group_interface_arm);
				break;
			case '3':	//go to posistion 3
				go_to_position(2, &move_group_interface_arm);
				break;
			case '4':	//go to position 4
				go_to_position(3, &move_group_interface_arm);
				break;
			case '5':	//go to position 5
				go_to_position(4, &move_group_interface_arm);
				break;
			case 's':	//go down and grab
				go_down(&move_group_interface_arm, &move_group_interface_gripper);
				go_close(&move_group_interface_arm, &move_group_interface_gripper);
				go_up(&move_group_interface_arm, &move_group_interface_gripper);
				break;
			case 'w':	//go up and release
				go_down(&move_group_interface_arm, &move_group_interface_gripper);
				go_open(&move_group_interface_arm, &move_group_interface_gripper);
				go_up(&move_group_interface_arm, &move_group_interface_gripper);
				break;
			case 'e':	//go home
				go_home(&move_group_interface_arm, &move_group_interface_gripper);
			case 'q':	//exit
				running = false;	//set running to false, to break out of the loop
				break;
			default:
				ROS_ERROR_NAMED("movement_test", "Invalid command: %c", command);
				break;
		}
	}

	//Shut down ros connection and exit
	ros::shutdown();
	return 0;
}