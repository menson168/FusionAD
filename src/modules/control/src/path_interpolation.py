#!/usr/bin/env python

"""Publishes relative path ECEF poordinates from given geodetic coordinates

Subscribes to:
    None

Publishes to:
    /planning/trajectory

Sends nav_msg/Path to "core_control" node (control_node.cpp)

Convenience degrees-minute-second to decimal converter can be found here: https://repl.it/@DRmoto/DMStoDec
"""

from __future__ import print_function
from __future__ import division

import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path

import gps_parser
import geodesy

def get_point_density(x1, y1, z1, x2, y2, z2, centimetersPerPoint):
    pointDensity = geodesy.euclidian_distance_3d(x1, y1, z1, x2, y2, z2) / (centimetersPerPoint / 100.0)

    return int(pointDensity)

# Instead of different functions for positive and negative
def interpolate(i, relativeX, relativeY, relativeZ, numberOfCoarsePoints):
    """Interpolate between two points, given index of one of the points."""
    
    finePointsX = []
    finePointsY = []
    finePointsZ = []

    # Vanilla case: for all points except final point
    if i < numberOfCoarsePoints-1:
        # Number of points between each interpolated point
        pointDensity = get_point_density(relativeX[i], relativeX[i], relativeZ[i], relativeX[i+1], relativeY[i+1], relativeZ[i+1], 25)

        # Declare the first and second positions for interpolation
        x0 = relativeX[i]     
        x1 = relativeX[i+1]
        y0 = relativeY[i]     
        y1 = relativeY[i+1]   
        z0 = relativeZ[i]     
        z1 = relativeZ[i+1]

        for n in range(pointDensity):
            finePointsX.append( x0 + (x1-x0)*(n/pointDensity) )
            finePointsY.append( y0 + (y1-y0)*(n/pointDensity) ) # was previously: finePointsY.append( ((y1-y0) / (x1-x0)) * (finePointsX[n]) + y0*((x1-x0) / (y1-y0)) )
            finePointsZ.append( z0 + (z1-z0)*(n/pointDensity) )

    # Corner case: for final point    
    if i == numberOfCoarsePoints-1:
        pointDensity = get_point_density(relativeX[i-1], relativeX[i-1], relativeZ[i-1], relativeX[i], relativeY[i], relativeZ[i], 25)

        x0 = relativeX[i-1]
        x1 = relativeX[i]
        y0 = relativeY[i-1]
        y1 = relativeY[i]
        z0 = relativeZ[i-1]     
        z1 = relativeZ[i]

        for n in range(pointDensity):
            finePointsX.append( x0 + (x1-x0)*(n/pointDensity) )
            finePointsY.append( y0 + (y1-y0)*(n/pointDensity) )
            finePointsZ.append( z0 + (z1-z0)*(n/pointDensity) )

    # print("pointDensity used on iteration {} was {}".format(i, pointDensity))
    return finePointsX, finePointsY, finePointsZ

def interpolation_publish(relativeX, relativeY, relativeZ, chosenHeight):
    """Interpolates between all ECEF coordinates and publishes them as a Path.

    Subscribes
    ----------
    None
    
    Publishes
    ---------
    std_msgs/Float64
        /interpolation_x -- List of interpolated points in the x direction in ECEF coordinates
    
    std_msgs/Float64
        /interpolation_x -- List of interpolated points in the x direction in ECEF coordinates
    """

    path_publisher = rospy.Publisher('/planning/trajectory', Path, queue_size=1000)
    rospy.init_node('interpolation_node', anonymous = True)
    rate = rospy.Rate(.1)

    while not rospy.is_shutdown():
        path = Path()

        # Contains lists of fine points, including coarse points
        xInterpolatedPositions = []
        yInterpolatedPositions = []
        zInterpolatedPositions = []

        numberOfCoarsePoints = len(relativeX)

        for i in range(numberOfCoarsePoints):
            finePointsX, finePointsY, finePointsZ = interpolate(i, relativeX, relativeY, relativeZ, numberOfCoarsePoints)

            xInterpolatedPositions.extend(finePointsX)
            yInterpolatedPositions.extend(finePointsY)
            zInterpolatedPositions.extend(finePointsZ)
        
        ################################
        ##### Publish Path message #####
        ################################

        for i in range(len(yInterpolatedPositions)):
            # # Attempting to add points directly in one line without creating point object first
            # path.poses.append(path.poses[i].pose.position.x = 0.0) # finePointsX[i]
            # path.poses[i].pose.position.y = 0.0 # finePointsY[i]
            # path.poses[i].pose.position.z = 0.0

            currentPoseStampMsg = PoseStamped()
            h = Header()

            h.stamp = rospy.Time.now()
            h.seq = i
            currentPoseStampMsg.header.seq = h.seq
            currentPoseStampMsg.header.stamp = h.stamp

            currentPoseStampMsg.pose.position.x = xInterpolatedPositions[i] 
            currentPoseStampMsg.pose.position.y = yInterpolatedPositions[i] 
            currentPoseStampMsg.pose.position.z = zInterpolatedPositions[i]

            path.poses.append(currentPoseStampMsg)
        
        path_publisher.publish(path)
        print("Published Path with {} steps".format(i))
        rate.sleep()

def main():

    # From https://www.maps.ie/coordinates.html at SJSU
    chosenHeight = 60.0

    inputLatitudes, inputLongitudes, inputHeights = gps_parser.read_file_coarse_points("gps_coarse_points/testCoordinates1.txt", chosenHeight)
    # print("\ninputLatitudes: {}\ninputLongitudes: {}\ninputHeights: {}".format(inputLatitudes, inputLongitudes, inputHeights))
    
    xPosition, yPosition, zPosition = geodesy.geodetic_data_to_ECEF_data(inputLatitudes, inputLongitudes, inputHeights)
    # print("\nxPosition =", xPosition, "\nyPosition =", yPosition, "\nzPosition =", zPosition)

    relativeX, relativeY, relativeZ = geodesy.global_to_relative(xPosition, yPosition, zPosition)
    # print("\nrelativeX =", relativeX, "\nrelativeY =", relativeY, "\nrelativeZ =", relativeZ, "\n")
    
    try:
        interpolation_publish(relativeX, relativeY, relativeZ, chosenHeight)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

