from pynput.keyboard import Key, Listener
import rospy

def on_press(key):
    print('{0} pressed'.format(
        key))

def on_release(key):
    print('{0} release'.format(
        key))
    if key == Key.esc:
        # Stop listener
        return False

# Collect events until released
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()


class OffboardControl:
    """ PX4 UAV controller works in offboard mode"""
    def __init__(self):
        self.curr_pose = PoseStamped()
        self.isReadyToFly = False
        self.hover_loc = [0, 0, 7, 0, 0, 0, 0]
        self.mode = "HOVER"

        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # state machine code defined in controller function
        self.controller()

    def pose_callback(self, msg):
        self.curr_pose = msg

    def state_callback(self, msg):
        if msg.mode == 'OFFBOARD':
            self.isReadyToFly = True
        else:
            print(msg.mode)


    def hover(self):
        """ hover at height mentioned in location
            set mode as HOVER to make it work
        """
        location = self.hover_loc
        loc = [location,
               location,
               location,
               location,
               location,
               location,
               location,
               location,
               location]

        rate = rospy.Rate(10)
        rate.sleep()
        shape = len(loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode == "HOVER" and not rospy.is_shutdown():
            if waypoint_index == shape:
                waypoint_index = 0
                sim_ctr += 1
                print("HOVER STOP COUNTER: " + str(sim_ctr))
            if self.isReadyToFly:

                des_x = loc[waypoint_index][0]
                des_y = loc[waypoint_index][1]
                des_z = loc[waypoint_index][2]
                des_pose.pose.position.x = des_x
                des_pose.pose.position.y = des_y
                des_pose.pose.position.z = des_z
                des_pose.pose.orientation.x = loc[waypoint_index][3]
                des_pose.pose.orientation.y = loc[waypoint_index][4]
                des_pose.pose.orientation.z = loc[waypoint_index][5]
                des_pose.pose.orientation.w = loc[waypoint_index][6]

                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z

                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) +
                                 (curr_z - des_z)*(curr_z - des_z))
                if dist < self.distThreshold:
                    waypoint_index += 1

            pose_pub.publish(des_pose)
            rate.sleep()


    def controller(self):
        while not rospy.is_shutdown():
            if self.mode == "HOVER":
                self.hover()