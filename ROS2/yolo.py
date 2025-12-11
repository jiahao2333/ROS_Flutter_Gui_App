import ros_numpy
import rospy
from sensor_msgs.msg import Image

from ultralytics import YOLO

detection_model = YOLO("yolo11m.pt")
rospy.init_node("ultralytics")
det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)


def callback(data):
    array = ros_numpy.numpify(data)
    det_result = detection_model(array)
    det_annotated = det_result[0].plot(show=False)
    det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))


rospy.Subscriber("/ascamera/camera_publisher/rgb0/image", Image, callback)
rospy.spin()