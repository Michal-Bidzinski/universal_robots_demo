import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInteface(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id("RRTstarkConfigDefault")
    move_group.set_planning_time(10.0)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    ## 
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state ==========="
    print robot.get_current_state()
    print ""
    print "============ Printing robot pose ============"
    print move_group.get_current_pose().pose

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.2
    p.pose.position.y = 0.25
    p.pose.position.z = 0.025
    #scene.add_box("table", p, (1, 0.05, 0.05))


  def go_to_joint_state(self, j0, j1, j2, j3, j4, j5):
    move_group = self.move_group

    ## Planning to a Joint Goal
    ## 
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = j0 * pi/180
    joint_goal[1] = j1 * pi/180
    joint_goal[2] = j2 * pi/180
    joint_goal[3] = j3 * pi/180
    joint_goal[4] = j4 * pi/180
    joint_goal[5] = j5 * pi/180


    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, x, y, z, xo, yo, zo, wo):
    move_group = self.move_group

    ## Planning to a Pose Goal
    ## 
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = wo
    pose_goal.orientation.x = xo
    pose_goal.orientation.y = yo
    pose_goal.orientation.z = zo

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    move_group = self.move_group

    ## Cartesian Paths
    ## 
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## Displaying a Trajectory
    ## 
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):
    move_group = self.move_group

    ## Executing a Plan
    ## 
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    ## Ensuring Collision Updates Are Receieved
    ## 
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  def add_box(self, name, x, y, z, ox, oy, oz, ow, frame, width, height, depth, timeout=4):
    box_name = name
    scene = self.scene

    ## Adding Objects to the Planning Scene
    ## 
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame
    box_pose.pose.orientation.x = ox
    box_pose.pose.orientation.y = oy
    box_pose.pose.orientation.z = oz
    box_pose.pose.orientation.w = ow
    box_pose.pose.position.x = x 
    box_pose.pose.position.y = y 
    box_pose.pose.position.z = z 
    box_name = name
    scene.add_box(box_name, box_pose, size=(width, height, depth))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, name, timeout=4):
    box_name = name
    robot = self.robot
    scene = self.scene
    group_names = self.group_names

    ## Attaching Objects to the Robot
    ## 
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'manipulator'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box("world", box_name, touch_links=touch_links)

    #scene.set_color(name, 0, 0, 1, a=0.9)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## Detaching Objects from the Robot
    ## 
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)


    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    ## Removing Objects from the Planning Scene
    ## 
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def euler_to_quaternion(self, roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

  def quaternion_to_euler(self, x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

  def move_to_point(self, x, y, z, xo, yo, zo, wo, cur_orientation):
    current_pose = self.move_group.get_current_pose().pose
    current_position = current_pose.position
    current_orientation = current_pose.orientation

    orient = self.move_group.get_current_pose().pose.orientation

    #print "orient:  x:", orient.x, " y: ", orient.y, " z: ", orient.z, " w: ", orient.w 
    #euler = self.quaternion_to_euler(orient.x, orient.y, orient.z, orient.w)
    #print "euler: ", euler[0] * 180.0 / np.pi , euler[1] * 180.0 / np.pi, euler[2] * 180.0 / np.pi
    #q = self.euler_to_quaternion(euler[0], euler[1], euler[2])
    #print "orient:  x:", q[0], " y: ", q[1], " z: ", q[2], " w: ", q[3] 

    no_samples = 50

    x_linspace = np.linspace(current_position.x, x, num=no_samples)
    y_linspace = np.linspace(current_position.y, y, num=no_samples)
    z_linspace = np.linspace(current_position.z, z, num=no_samples)

    if cur_orientation == False:
      x_orientation = np.linspace(current_orientation.x, xo, num=no_samples)
      y_orientation = np.linspace(current_orientation.y, yo, num=no_samples)
      z_orientation = np.linspace(current_orientation.z, zo, num=no_samples)
      w_orientation = np.linspace(current_orientation.w, wo,num=no_samples)
    else:
      new_pose = current_pose
      new_pose.orientation.x = 0
      new_pose.orientation.y = 0
      new_pose.orientation.z = 0
      new_pose.orientation.w = 1

    waypoints = []

    for i in range(1, no_samples):
      new_pose.position.x = x_linspace[i]
      new_pose.position.y = y_linspace[i]
      new_pose.position.z = z_linspace[i]
      if cur_orientation == False:
        new_pose.orientation.x = x_orientation[i]
        new_pose.orientation.y = y_orientation[i]
        new_pose.orientation.z = z_orientation[i]
        new_pose.orientation.w = w_orientation[i]

      print(x_linspace[i],y_linspace[i],z_linspace[i])
      waypoints.append(copy.deepcopy(new_pose))

    (plan, fraction) = self.move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    self.display_trajectory(plan)
    rospy.sleep(4)
    self.move_group.execute(plan,wait=True)

  def execute_path(self, array):
    current_pose = self.move_group.get_current_pose().pose
    current_position = current_pose.position
    current_orientation = current_pose.orientation

    # orient = self.move_group.get_current_pose().pose.orientation

    waypoints = np.insert(array, 0, [current_position.x, current_position.y, current_position.z, current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w, False], axis=0)

    way = []
    no_samples = 4
    print len(waypoints)

    for i in range(len(waypoints)-1): 
      print "i: ",i
      x_linspace = np.linspace(waypoints[i][0], waypoints[i+1][0], num=no_samples)
      y_linspace = np.linspace(waypoints[i][1], waypoints[i+1][1], num=no_samples)
      z_linspace = np.linspace(waypoints[i][2], waypoints[i+1][2], num=no_samples)

      print "i: ",i
      print waypoints[i+1][7]

      new_pose = current_pose
      if waypoints[i+1][7]:
        x_orientation = np.linspace(waypoints[i][3], waypoints[i+1][3], num=no_samples)
        y_orientation = np.linspace(waypoints[i][4], waypoints[i+1][4], num=no_samples)
        z_orientation = np.linspace(waypoints[i][5], waypoints[i+1][5], num=no_samples)
        w_orientation = np.linspace(waypoints[i][6], waypoints[i+1][6],num=no_samples)
      else:
        new_pose.orientation.x = 0
        new_pose.orientation.y = 0
        new_pose.orientation.z = 0
        new_pose.orientation.w = 1

      for j in range(1, no_samples):
        new_pose.position.x = x_linspace[j]
        new_pose.position.y = y_linspace[j]
        new_pose.position.z = z_linspace[j]
        if waypoints[i+1][7]:
          new_pose.orientation.x = x_orientation[j]
          new_pose.orientation.y = y_orientation[j]
          new_pose.orientation.z = z_orientation[j]
          new_pose.orientation.w = w_orientation[j]

        print(x_linspace[j],y_linspace[j],z_linspace[j])
        way.append(copy.deepcopy(new_pose))

    (plan, fraction) = self.move_group.compute_cartesian_path(
                                   way,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0, # jump_threshold
                                   avoid_collisions=True)       

    self.move_group.execute(plan,wait=True)
