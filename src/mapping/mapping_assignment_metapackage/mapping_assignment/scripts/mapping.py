#!/usr/bin/env python3

"""
    # {Sebastian Barbas Laina}
    # {961005-0230}
    # {ssbl@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs, pi

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def Translation_matrix(self,robot, map_origin,yaw):
  
        """T01=[[cos(yaw),-sin(yaw),0,robot.pose.position.x-map_origin.pose.position.x],
            [sin(yaw),cos(yaw),0,robot.pose.position.y-map_origin.pose.position.y],
            [0,0,1,(robot.pose.position.z-map_origin.pose.position.z)],
            [0,0,0,1]]"""
        T01=[[cos(yaw),-sin(yaw),0,robot.x-map_origin.x],
            [sin(yaw),cos(yaw),0,robot.y-map_origin.y],
            [0,0,1,(robot.z-map_origin.z)],
            [0,0,0,1]]
        return T01
    
    def max_vals(self,vector):
        xmax=vector[0][0]
        xmin=vector[0][0]
        ymax=vector[0][1]
        ymin=vector[0][1]
        for point in vector:
            if point[0]>xmax:
                xmax=point[0]
            if point[0]<xmin:
                xmin=point[0]
            if point[1]<ymin:
                ymin=point[1]
            if point[1]>ymax:
                ymax=point[1]
            
        
        return xmax, ymax, xmin, ymin


    def Laser2Coordenates(self,scan):
        endposition=np.array([])
        angle=scan.angle_min
        for ranges in scan.ranges:

            if ranges>scan.range_min and ranges<scan.range_max:
               
                xpos=ranges*cos(angle)
                ypos=ranges*sin(angle)
                if len(endposition)==0:
                    endposition=[[xpos,ypos,0,1]]
                else:
                    endposition.append([xpos,ypos,0,1])
                #endposition.append([xpos,ypos,0,1])
            angle=angle+scan.angle_increment
        return endposition

    
    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        matrix=self.Translation_matrix(pose.pose.position,origin.position,robot_yaw)
        traversed=np.array([])
        positions=self.Laser2Coordenates(scan)
        #print(positions[0],'x: ', positions[0][0],'y: ',positions[0][1])
        #print(positions)
        origin_positions=[]
        i=0
        for pos in positions:
            vector_or=np.dot(matrix,pos)
            if len(origin_positions)==0:
                origin_positions=[[vector_or[0],vector_or[1]]]
            else:
                origin_positions.append([vector_or[0],vector_or[1]])
        origin_robot=[pose.pose.position.x-origin.position.x,pose.pose.position.y-origin.position.y]
        
        for i in origin_positions:
            if len(traversed)==0:
                traversed=self.raytrace([int(origin_robot[0]/resolution),int(origin_robot[1]/resolution)],[int(i[0]/resolution),int(i[1]/resolution)])
            else:
                traversed.extend(self.raytrace([int(origin_robot[0]/resolution),int(origin_robot[1]/resolution)],[int(i[0]/resolution),int(i[1]/resolution)]))
        

        data_array=traversed
    

        for points in data_array:
            if self.is_in_bounds(grid_map,points[0],points[1]):
                self.add_to_map(grid_map,points[0],points[1],self.free_space)
        
        for opos in origin_positions:
            if self.is_in_bounds(grid_map,int(opos[0]/resolution),int(opos[1]/resolution)):
                self.add_to_map(grid_map,int(opos[0]/resolution),int(opos[1]/resolution),self.occupied_space)
                data_array.append((int(opos[0]/resolution),int(opos[1]/resolution)))




        """
        Fill in your solution here
        """


        """
        For C only!
        Fill in the update correctly below.
        """ 
        
        xmax, ymax, xmin, ymin=self.max_vals(data_array)

        data_to_send=[]

        for yval in range(ymin,ymax+1):
            for xval in range(xmin,xmax+1):
                if len(data_to_send)==0:
                    data_to_send=[grid_map.__getitem__([xval,yval])]
                else:
                    data_to_send.append(grid_map.__getitem__([xval,yval]))
            


        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = xmin
        # The minimum y index in 'grid_map' that has been updated
        update.y = ymin
        # Maximum x index - minimum x index + 1
        update.width = xmax-xmin+1
        # Maximum y index - minimum y index + 1
        update.height = ymax-ymin+1
        # The map data inside the rectangle, in row-major order.
        update.data = data_to_send

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        xorg=0
        yorg=0
        xmaxmap=xorg+grid_map.get_width()
        ymaxmap=yorg+grid_map.get_height()


        for i in range(xorg,xmaxmap):
            for j in range(yorg,ymaxmap):
                if grid_map.__getitem__([i,j])==self.occupied_space:
                    ang=0
                    while ang<2*pi:
                        points=self.raytrace([i,j],[int(i+self.radius*cos(ang)),int(j+self.radius*sin(ang))])
                        for point in points:
                            if self.is_in_bounds(grid_map,point[0],point[1]) and grid_map.__getitem__([point[0],point[1]])!=self.occupied_space:
                                self.add_to_map(grid_map,point[0],point[1],self.c_space)
                        ang=ang+pi/18






        
        # Return the inflated map"""
        return grid_map
