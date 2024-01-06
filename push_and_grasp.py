import random
import numpy as np
from helper import World
import pdb

np.set_printoptions(precision=3, suppress=True)

GRIPPER_WIDTH = 0.2
BLOCK_WIDTH = 0.05
BLOCK_HEIGHT = 0.1
DROP_POSE = np.array([0.2, -0.2, 0.05, 0. * np.pi / 180., 0.05])
ROBOT_HOME = np.array([0.25, 0.25, 0.25, 0., 0.])

def interpolate_poses(initial_pose, final_pose, n):
    """
    Create a series of intermediate poses to smoothly transition the robot from the initial pose to the final pose.
    """
    x_initial, y_initial, z_initial, _, _ = initial_pose
    x_final, y_final, z_final = final_pose

    x_interp = np.linspace(x_initial, x_final, n)
    y_interp = np.linspace(y_initial, y_final, n)
    z_interp = np.linspace(z_initial, z_final, n)

    ang, gripper_width = initial_pose[3], initial_pose[4]

    interpolated_poses = [
        np.array([x, y, z, ang, gripper_width])
        for x, y, z in zip(x_interp, y_interp, z_interp)
    ]

    return interpolated_poses
def open_gripper():
    pass

def close_gripper():
    pass
def pick_up(env, obj_idx):
    """
    Perform a grasp in the simulation with obj_idx from obj_state.
    """
    obj_states = env.get_obj_state()
    obj_pos = obj_states[obj_idx, :3]
    z_offset = 0.15  # Adjust as needed based on the object height and gripper clearance.

    # Move to a safe distance above the object before grasping.
    pre_grasp_pose = np.array([obj_pos[0], obj_pos[1], obj_pos[2] + z_offset, 0., GRIPPER_WIDTH])
    env.robot_command([pre_grasp_pose])

    # Move down to grasp the object.
    grasp_pose = np.array([obj_pos[0], obj_pos[1], obj_pos[2] - 0.025, 0., GRIPPER_WIDTH])  # Z-offset for grasp.
    waypoints = interpolate_poses(pre_grasp_pose, grasp_pose[:3], 5)
    env.robot_command(waypoints)

    # Close the gripper to grasp the object.
    pre_grasp_pose[-1] = 0.0  # Assumes 0.0 means closed gripper.
    env.robot_command([pre_grasp_pose])

    # Lift the object up to the pre-grasp height after grasping.
    pre_grasp_pose[2] += z_offset  # Move back to the pre-grasp height.
    robot_cmd = [pre_grasp_pose]
    env.robot_command([pre_grasp_pose])

    return robot_cmd[-1]


def push(env, obj_idx, direction, distance):
    """
    Perform a push on the specified object in the given direction by the specified distance.
    """

    obj_states = env.get_obj_state()

    # Get the position of the object to be pushed
    obj_pos = obj_states[obj_idx, :3]

    target_pos = obj_pos + np.array(direction) * distance
    #print(target_pos)
    
    # Pre-push-1 pose : Change angle of gripper
    rob_state = env.get_robot_state()
    gripper_pose = np.array([rob_state[0], rob_state[1], rob_state[2], 90. * np.pi / 180., 0.])
    # Command the robot to move to the Pre-push-1 push pose
    env.robot_command([gripper_pose])
    

    # Pre-push-2 pose : Stay above object's pre-push pose
    rob_state = env.get_robot_state()
    gripper_pose = np.array([obj_pos[0], obj_pos[1]+0.15, obj_pos[2]+0.3, rob_state[3], 0.])
    #print("Pre-push-2 push pose: ",gripper_pose)
    env.robot_command([gripper_pose])
    

    # Pre-push pose : Go to pre-push pose
    gripper_pose = np.array([obj_pos[0], obj_pos[1]+0.15, obj_pos[2], gripper_pose[3], 0.])
    #print("Pre-push pose: ",gripper_pose)
    # Command the robot to move to the pre-push pose
    env.robot_command([gripper_pose])
    
    # Interpolate between current pre-push pose to target
    waypoints = interpolate_poses(gripper_pose, target_pos, 5)

    env.robot_command(waypoints)


def stack():
    """
    function to stack objects
    :return: average height of objects
    """
    # DO NOT CHANGE: initialize world
    env = World()

    # ============================================
    # YOUR CODE HERE:

    obj_state = env.get_obj_state()
    print(obj_state)
    #pdb.set_trace()
    # TODO: Put stack plan primitives in loop for every object
    NEW_DROP_POSE = DROP_POSE
    for obj_idx in range(0,len(obj_state)):

        if obj_idx<len(obj_state)-1:
            push(env, obj_idx, [0.,-1.,0.], 0.2) 

        # MOVE: Up after push
        rob_state = env.get_robot_state()
        rob_state[2] = rob_state[2] + 0.2
        robot_command = [np.array([rob_state[0], rob_state[1], rob_state[2], 0. * np.pi / 180., 0.])]
        env.robot_command(robot_command)
        pick_up(env, obj_idx)

        # MOVE: Drop-pose align
        gripper_pose = NEW_DROP_POSE
        gripper_pose[2] += 0.15
        robot_command = [gripper_pose]
        env.robot_command(robot_command)

        # MOVE: Drop-pose
        waypoints = interpolate_poses(gripper_pose, NEW_DROP_POSE[:3], 5)
        #pdb.set_trace()
        env.robot_command(waypoints) # FIXME: Get current gripper width
        # robot_command = [NEW_DROP_POSE]
        # env.robot_command(robot_command)

        # GRIPPER: Open to drop object
        drop_pose =  waypoints[-1] #NEW_DROP_POSE
        drop_pose[-1] = 0.2   # Gripper fully open
        robot_command = [drop_pose]
        env.robot_command(robot_command)

        # Update next-drop pose
        rob_state = env.get_robot_state()
        obj_state = env.get_obj_state()
        NEW_DROP_POSE = np.array([rob_state[0], rob_state[1], obj_state[obj_idx][2]+0.07, rob_state[3], 0.]) # FIXME: Can also use obj_idx

        # MOVE: Move to pre-drop pose
        gripper_pose = np.array([rob_state[0], rob_state[1], rob_state[2]+0.25, rob_state[3], 0.2])
        robot_command = [gripper_pose]
        env.robot_command(robot_command)

        # MOVE: Go Home position
        env.robot_command([ROBOT_HOME])

    # ============================================
    # DO NOT CHANGE: getting average height of objects:
    obj_state = env.get_obj_state()
    avg_height = np.mean(obj_state[:, 2])
    print("Average stacked Height: {:4.3f}".format(avg_height))
    return env, avg_height

if __name__ == "__main__":
    np.random.seed(7)
    random.seed(7)
    
    stack()
