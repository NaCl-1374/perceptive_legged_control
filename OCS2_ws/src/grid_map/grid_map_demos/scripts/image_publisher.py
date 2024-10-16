'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2024-03-05 21:50:22
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2024-03-24 15:37:16
FilePath: /leg_perceptive_ws/src/grid_map/grid_map_demos/scripts/image_publisher.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python
# simple script to publish a image from a file.
import rospy
import rospkg
import time
import cv2
import sensor_msgs.msg

def callback(self):
    """ Convert a image to a ROS compatible message
        (sensor_msgs.Image).
    """
    global publisher, imagePath

    img = cv2.imread(imagePath, cv2.IMREAD_UNCHANGED)

#    print img.shape
#    print img.size
#    print img.dtype.itemsize

    rosimage = sensor_msgs.msg.Image()
    if img.dtype.itemsize == 2:
       if len(img.shape) == 3:
           if img.shape[2] == 3:
               rosimage.encoding = 'bgr16'
           if img.shape[2] == 4:
           	   rosimage.encoding = 'bgra16'
       else:
           rosimage.encoding = 'mono16'
    if img.dtype.itemsize == 1:
       if len(img.shape) == 3:
           if img.shape[2] == 3:
               rosimage.encoding = 'bgr8'
           if img.shape[2] == 4:
           	   rosimage.encoding = 'bgra8'
       else:
           rosimage.encoding = 'mono8'
#    print "Encoding: ", rosimage.encoding

    rosimage.width = img.shape[1]
    rosimage.height = img.shape[0]
    rosimage.step = img.strides[0]
    rosimage.data = img.tostring()
    rosimage.header.stamp = rospy.Time.now()
    rosimage.header.frame_id = 'map'

    publisher.publish(rosimage)


#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publisher, imagePath
    rospack = rospkg.RosPack()
    rospy.init_node('image_publisher')
    imagePath = rospy.get_param('~image_path')
    topicName = rospy.get_param('~topic')
    publisher = rospy.Publisher(topicName, sensor_msgs.msg.Image, queue_size=10)
    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
