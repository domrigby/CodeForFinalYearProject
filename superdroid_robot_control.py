#!/usr/bin/env python3
from cmath import inf
import tf
import rospy
import serial

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry

import numpy as np
from numpy import linalg as la
import time 
import tf2_ros
import tf_conversions

import scipy.optimize

class ObstacleAvoid():

    def __init__(self):

        rospy.init_node("lidarobstacle")

        self.pointgiven=False
        self.calibrate = False
        self.UWBPoint=PointStamped()

        print(self.rotate([0,5],np.pi/2))

        self.cubecount=0

        self.UWB_file = open("/home/dom/robot_data/UWB_data.txt","a")
        self.UWB_file.write("Actual position: roughly -1.8,0 \n")

        self.x_pos = 0
        self.y_pos = 0

        self.theta_to_goal = None

        self.first_time = True
        self.pose_list = []

        #self.angle_ranges = [[658,670],[546,558],[444,454],[332,342],[220,230]]
        self.angle_ranges = [[115,125],[235,245],[355,365],[475,485],[595,605]] #east , north checked,
        self.assosciated_angle = [(np.pi/2),(np.pi/4),0,-(np.pi/4),-(np.pi/2)] # starts from left... right hand rule therefore that is positive pi/2

        self.max_ang_vel = 0.5
        self.max_speed = 0.5

        self.max_obs_dist = 2.5
        self.obstacle_weight = 0.8
        self.bias = 3
        self.direction_weights = [0.75*self.max_obs_dist,1.0*self.max_obs_dist,self.max_obs_dist,1.0*self.max_obs_dist,0.75*self.max_obs_dist]
        self.original_weights = self.direction_weights

        self.direction_distance_values = []

        for i in range(len(self.assosciated_angle)):
            self.direction_distance_values.append([self.max_obs_dist * np.cos(self.assosciated_angle[i]) , self.max_obs_dist * np.sin(self.assosciated_angle[i])])


        self.goal_weight = 0.4

        self.UWB_vec = [0,0]
        self.UWB_dists = []
        self.start_collecting = False

        self.direction_list = ["West","North-West","North","North-East","East"]
        #self.direction_list = ["East","North-East","North","North-West","West"]


        # All ros subscribing and publishing:

        self.laser_sub = rospy.Subscriber("/scan",LaserScan,callback=self.LaserScanCallback,queue_size=1)

        self.move_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.pose_sub =  rospy.Subscriber("/odometry/filtered",Odometry,callback = self.posecallback,queue_size=1)

        self.click_sub = rospy.Subscriber("/clicked_point",PointStamped,callback = self.clicked_callback,queue_size=1)

        self.rviz_pub = rospy.Publisher("visualise_arrow",Marker,queue_size=3)

        self.map_saver = rospy.Subscriber("/map",OccupancyGrid,callback=self.get_map,queue_size=1)

        self.UWB_sub = rospy.Subscriber("/UWB_topic",String,callback=self.measure_UWB_dist,queue_size=1)

        self.cubepubber = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        ## make a node that will publish to the arduino!

        # at start until it gets updates from pose
        self.x_pos - 0
        self.y_pos = 0
        self.theta_pos = 0

        self.epoch = rospy.get_time()
        self.time_between_UWB = 30
        self.UWB_count = 0

        self.rand_angle = np.random.uniform(-np.pi/4,np.pi/4)

        self.turn_to_angle(self.theta_pos + self.rand_angle)

        while not rospy.is_shutdown():  
            if not hasattr(self, 'distances'): # waits for laser scan data to start the robot
                print("no laser data")
                rospy.sleep(1)
            else:
                print("run robot")
                self.run_robot()
            #try:
                #self.run_robot() 
            #except Exception as e:
                #print(e)
                #rospy.sleep(2)
            
        rospy.on_shutdown(self.at_exit)

    def LaserScanCallback(self,data):
        self.distances = data.ranges
        
    def clicked_callback(self,msg):
        self.x_click = msg.point.x
        self.y_click = msg.point.y
        self.pointgiven = True

        print("point published")

        self.plot_cube(self.x_click,self.y_click,0.2,colour=[0,0,1])

    def run_robot(self):  

        #duration = rospy.Duration(20,0)
        #rospy.Timer(duration,self.wait_for_point,oneshot=True)

        while not rospy.is_shutdown():
            if True : # commment out this code to activate trilaterating behaviour
                break
            if not self.calibrate: # the timer function sets this to true evert 20 seconds, so if it is true and it hasn not calibrated, function will run
                break
            while not rospy.is_shutdown():
                if self.pointgiven:
                    self.UWB_calibrate() # now it calibrates
                    self.calibrate = False
                    self.pointgiven = False
                    self.first_time = False
                print(self.calibrate)
                if not self.calibrate:
                    break
                else:
                    print("Please click point")
                    rospy.sleep(1)
            break

        self.reset_vectors()

        # extracts the obstacle avoidance vector
        angles = self.angle_ranges

        vector_list = []
        tries = 0

        while len(vector_list) < 5 and tries < 10 :
            vector_list = self.GetVectors(self.distances,angles)
            tries += 1

        # the obstacle avoidance vector is constantly recalculate to search for the obstacle free path

        self.obs_vec =self.ObstacleAvoidVec(vector_list) # sums the obstacle vectors

        # this will return an angle of the vecotr, relative to the x-axis of the robot (straight on)
        # ie if the robot is facing in pi/2 , and the angle of the vector is pi/4 it will want to turn till the robot angle is 3pi/4
        # therefore must sum angle of robot and angle of vector: did not show at first due to angle of robot being 0
        # ALSO! Gives benefit that the error of the robots angle is always just the angle between the vector and the x-axis
            
        #rospy.Timer(rospy.Duration(50), self.UWB_calibrate())

        self.Blend() # blend UWB vector into this

        goal_angle = np.arctan2((self.blend_vector[1]),(self.blend_vector[0]))

        #goal_angle is relative to your front so no need to include your theta...
        # its in robot frame

        self.error = goal_angle

        print("goal angle " , goal_angle)
        print("blend vector: ",self.blend_vector)

        if abs(self.error) > np.pi :
            if self.error > 0 :
                self.error += - 2*np.pi
            else:
                self.error += 2*np.pi

        print("error ", self.error)

        if abs(self.error) > 0.25 :
            self.turn_direction(self.error)
            if self.error > 0:
                print("Turn counter clockwise")
            else:
                print("Turn clockwise")        
        else:
            print("go forward") # if the obstacle or blend vector is within 0.25 radian of the x-axis of the robot, it will go forward until this changes
            self.go_forward(0.6)

        self.PlotArrow(self.obs_vec, "base_link" , size = 1)
        self.plot_cube(self.x_pos,self.y_pos,0.05,[1,0,0])

        print(self.pointgiven)

        rospy.sleep(0.1)

    def wait_for_point(self,time):
        print("setting calibrate true ")
        self.calibrate = True
        

    def go_to_point(self,point,in_frame): #  go to goal behavior: turns to towards the point and moves toward it until the angle to the poin goes over 0.05
        if in_frame :
            goal = self.rotate(point,self.theta_pos)  
            goal = [goal[0]+self.x_pos,goal[1]+self.y_pos]
        else:
            goal = [point[0]-self.x_pos,point[1]-self.y_pos]
        print("goal = ",goal)
        goal_angle = np.arctan2((goal[1]),(goal[0]))
        if goal_angle > np.pi :
            goal_angle= goal_angle - 2*np.pi
        print("turning to :",goal_angle)
        self.turn_to_angle(goal_angle)
        
        while not rospy.is_shutdown():
            
            goal_angle = np.arctan2((point[1]-self.y_pos),(point[0]-self.x_pos))
            dist_to_goal = ((goal[0]-self.x_pos)**2+(goal[1]-self.y_pos)**2)**0.5
            print("dist ",dist_to_goal)

            error =   goal_angle - self.theta_pos

            if abs(self.error) > np.pi :
                if self.error > 0:   
                    self.error += - 2*np.pi
                else:
                    self.error += 2*np.pi

            error = np.arctan2(np.sin(error),np.cos(error))

            if abs(error) > 0.05 :
                self.turn_direction(error)
            else:
                if dist_to_goal > self.max_speed:
                    speed = self.max_speed
                else:
                    speed = dist_to_goal  
                self.go_forward(speed)

            if dist_to_goal < 0.10 :
                self.stop()
                break

            rospy.sleep(0.1)

    def turn_to_angle(self,angle):

        while not rospy.is_shutdown():

            error = angle - self.theta_pos

            if abs(error) > np.pi : # these bit of code are to prevent errors when cross from -pi to pi or vice versa
                if error > 0:
                    error += - 2*np.pi
                else:
                    error += 2*np.pi

            print(error)
            
            error = np.arctan2(np.sin(error),np.cos(error))

            print("error calculated: ",error)

            self.turn_direction(angle)

            if abs(error) < 0.05 : # turns until the angle error is less than 0.05
                break

            rospy.sleep(0.1)

    def turn_direction(self,angle): # potentially for just turn a direction
        error = np.arctan2(np.sin(angle),np.cos(angle))
        print("error in turn direction",error)
        pose_message = Twist()
        pose_message.linear.x = 0

        if abs(error) > self.max_ang_vel:
            if error < 0 :
                error = -self.max_ang_vel
            if error > 0 :
                error = self.max_ang_vel

        print("error published" , error)

        pose_message.angular.z = error
        self.move_pub.publish(pose_message)
        rospy.sleep(0.1)

    def go_forward(self,speed):
        msg = Twist()
        msg.angular.z = 0
        msg.linear.x = speed

        self.move_pub.publish(msg)

    def stop(self):
        msg = Twist()
        msg.angular.z = 0
        msg.linear.x = 0
    
        self.move_pub.publish(msg)

    def GetVectors(self,distances,angles):
        dist_array = np.array(distances) # distances returned by the lidar

        dist_array = np.clip(dist_array,0,self.max_obs_dist) # limits the distance that obstacles will be detected

        vector_list = []

        for i in range(len(angles)):
            self.direction_dists = np.array(dist_array[angles[i][0]:angles[i][1]]) # Extracts an array of distances , changed just now from distance -> dist_array
            inf_filter = [x for x in self.direction_dists if x != inf] # filter out the infinities
            self.direction_dists = np.clip(self.direction_dists,0,self.direction_weights[i])
            # above line is checke

            if len(inf_filter) > 0:
                min_dist = min(inf_filter) # filters out the 0 values which occur when lidar fails to measure a distance
                vector = [min_dist * np.cos(self.assosciated_angle[i]) , min_dist * np.sin(self.assosciated_angle[i])]
                vector_list.append(vector) # first vector is to the left of the lidar
                print(self.direction_list[i], " : " , vector_list[i] )

            else: 
                min_dist = self.max_obs_dist # filters out the 0 values which occur when lidar fails to measure a distance
                vector = [min_dist * np.cos(self.assosciated_angle[i]) , min_dist * np.sin(self.assosciated_angle[i])]
                vector = self.rotate(vector,np.pi)
                vector_list.append(vector)
                pass

        # avoid immediate obstacles

        if vector_list[2][0] < 1.7 :
            if abs(vector_list[0][1]) > abs(vector_list[4][1]):
                vector_list.append([0,self.bias])
            elif abs(vector_list[0][1]) < abs(vector_list[4][1]):
                vector_list.append([0,-self.bias])
            else:
                if np.linalg.norm(vector_list[1]) > np.linalg.norm(vector_list[3]):
                    vector_list.append([0,self.bias])
                else:
                    vector_list.append([0,-self.bias])

        return vector_list

    def reset_vectors(self):
        self.blend_vector = [0,0]

    def ObstacleAvoidVec(self,vector_list):
        # sums the roomba style obstacle vectors
        x_travel = 0
        y_travel = 0
        for i in range(len(vector_list)):
            x_travel +=  vector_list[i][0]
            y_travel += vector_list[i][1]

        return [x_travel,y_travel]

    def posecallback(self, pose_message):
        self.quaternion=[pose_message.pose.pose.orientation.x, pose_message.pose.pose.orientation.y, pose_message.pose.pose.orientation.z, pose_message.pose.pose.orientation.w]
        self.euler=tf.transformations.euler_from_quaternion(self.quaternion)
        self.theta_pos = self.euler[2] 
        
        self.x_pos= pose_message.pose.pose.position.x 
        self.y_pos= pose_message.pose.pose.position.y

        self.plot_cube(self.x_pos,self.y_pos,0.05,[1,0,0])

    def Blend(self):

        UWB_vec_away = self.rotate(self.UWB_vec,np.pi) # makes a vector to point away from the other robot

        UWB_vec_away = np.linalg.norm(UWB_vec_away)
        
        self.blend_vector = np.add(np.multiply(self.obs_vec,1),UWB_vec_away)

        # remember to rotate vectors as we move... as robots reference frame constantly rotate

    # here we add code for the robot to go forwards and backwards, left and right to get 

    def rotate(self,vector,angle): # rotation matrix for 2d vectors
        rotation_matrix = [[np.cos(angle),-np.sin(angle)],
                           [np.sin(angle),np.cos(angle)]]

        return np.matmul(vector,rotation_matrix)
        

    def UWB_calibrate(self): # the code to perform the trilaterion routine

        point_list = [[1,0],[1,0],[0,-1],[-1,0]] # List of commands, translates to : forward 1m , forward 1m, right 1m, forward 1m

        self.pos_list = []
        self.D_matrix = []

        for point in point_list:
            self.go_to_point(point,True)
            dist = self.take_dists()
            self.D_matrix.append(dist)
            self.pos_list.append([self.x_pos,self.y_pos])
            rospy.sleep(3)


        b = self.take_dists()
        self.D_matrix.append(b)
        self.pos_list.append([self.x_pos,self.y_pos])

        self.UWB_vector = self.trilaterate_dists(self.UWB_dists,self.pos_list)

    def measure_UWB_dist(self,msg):

        self.UWB_dist = msg.data

        if self.start_collecting :
            self.UWB_av.append(float(self.UWB_dist))

    def take_dists(self):
        self.UWB_av = []
        while len(self.UWB_av) < 30:
            self.start_collecting = True # flips a switch for the UWB callback to start collecting data
            rospy.sleep(0.1)
        
        self.start_collecting = False

        average_dist = np.average(np.array(self.UWB_av))

        return average_dist


    def trilaterate_dists(self,UWB_dists,pos_list):

        # we have the vectors from the starting position of where the robot is when the dists are taken... now must triangulate them to get the vector.
        # must plot circles around the point and see where they overlap
        # solve via matrix method?

        # using technique called trilateration

        result = scipy.optimize.least_squares(self.equations,np.array([4,4]))
        self.UWB_file.write(str(result.x) + "\n")
        self.other_pos = result.x

        UWB_vector = [self.other_pos[0] - self.x_pos , self.other_pos[1] - self.y_pos] # calculates vector in ma frame

        log_msg = 'UWB vector: x: ' + str(UWB_vector[0]) + 'y: ' + str(UWB_vector[1])

        rospy.loginfo(log_msg)
        print(UWB_vector)

        return UWB_vector 

    def equations(self,var):
        equations = np.array([])
        for i in range(len(self.pos_list)):
            eq = (var[0]-self.pos_list[i][0])**(2) + (var[1]-self.pos_list[i][1])**(2) - self.D_matrix[i]**2
            equations = np.append(equations,eq)

        return np.asarray(equations)

    def at_exit(self):
        print("exit")


    def get_map(self,msg):
        self.map_occ_grid = msg.data
        self.map_meta_data = msg.info

    def PlotArrow(self,vector_move, frame_name , size): # publish arrows to rviz

        marker = Marker()

        marker.header.frame_id = frame_name
        marker.header.stamp = rospy.Time.from_sec(0)
        marker.ns = "direction_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        euler_angle = np.arctan2(vector_move[0],vector_move[1])
        quat = tf.transformations.quaternion_from_euler(0,0,euler_angle-np.pi/2)
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = (vector_move[0]**2 + vector_move[1]**2)**0.5 * size
        marker.scale.y = 0.2 * size
        marker.scale.z = 0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)


        print("Marker published")


        self.rviz_pub.publish(marker)

    def plot_cube(self,x,y,size,colour): # publish cubes to rviz in the map frame
            markermessage=Marker()
            markermessage.header.stamp=rospy.Time()
            markermessage.header.frame_id="map"
            markermessage.ns="cubes"
            markermessage.id=self.cubecount
            self.cubecount+=1
            markermessage.type=1
            markermessage.action=0
            
            markermessage.pose.position.x= x #data.pose.pose.position.x
            #data.pose.pose.position.x
            markermessage.pose.position.y= y #data.pose.pose.position.y
            #data.pose.pose.position.y
            markermessage.pose.position.z=0.0 
            markermessage.pose.orientation.x=0.0
            markermessage.pose.orientation.y=0.0
            markermessage.pose.orientation.z=0.0
            markermessage.pose.orientation.w=1.0
            
            markermessage.scale.x=size
            markermessage.scale.y=size
            markermessage.scale.z=size
            markermessage.color.a=1.0
            markermessage.color.g=colour[1]
            markermessage.color.b=colour[2]
            markermessage.color.r=colour[0]
            markermessage.frame_locked=False
            self.cubepubber.publish(markermessage)

def main():
    obstacle  = ObstacleAvoid()

if __name__ == "__main__":
    main()