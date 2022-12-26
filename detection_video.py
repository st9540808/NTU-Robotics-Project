import cv2
import numpy as np
from cv2 import aruco
import rospy
from std_msgs.msg import Int32
import math
######################
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
########################

ID = 0

bridge = CvBridge()

# heart：紅桃 spade：黑桃 diamond：方塊  club：梅花
A = ["heart A", "spade A", "heart 2","spade 2", "heart 3", "spade 3", "heart 4", "spade 4", "heart 5","spade 5","heart 6","spade 6",
"heart 7","spade 7", "heart 8","spade 8","heart 9","spade 9","heart 10","spade 10"]

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_100) #建立字典，字典由100个marker组成，每个marker大小为4x4bits
MARKER_SIZE = 400

#generate 20 aruco markers
for id in range(20):
    marker_image = aruco.drawMarker(marker_dict, id, MARKER_SIZE)
    #cv2.imwrite(f"markers/marker_{id}.png", marker_image)

param_markers = aruco.DetectorParameters_create()

# cap = cv2.VideoCapture(0) 


def ArucoDetection(image):

    data = []

    # ret, frame = .read()   #ret的值为ture or false, 代表有没有读到照片；frame代表当前截取一帧的照片
    cv2.imshow("result",image)
    gray_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  #frame为要进行处理的图片，后者为色彩转换方式，如BGR2GRAY

        # corner
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters = param_markers)
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv2.polylines(image, [corners.astype(np.int32)],True, (0,255,0), 2, cv2.LINE_AA)  #画框线
            corners = (corners.reshape(4,2))  #将数据变为4X2的矩阵
            corners = corners.astype(int)     #将数据类型转化为int(整数型)
            # top_right = corners[0].ravel()    #corners[0]表示取第0行
            center = (corners[0].ravel() + corners[1].ravel() + corners[2].ravel() + corners[3].ravel())/4  #将每组坐标拉成1维数组并平均
            center = center.astype(int)     #再次转换数据类型
            angle =  -(math.atan((corners[1].ravel()[0] - corners[2].ravel()[0])/(corners[1].ravel()[1] - corners[2].ravel()[1])))*180/math.pi    
            data.append([ids[0], center[0], center[1], angle])
    return data
      
def output_msg(input_img): 

    data = ArucoDetection(input_img)
    all = np.array(data)
    all = all.transpose()[0]
    all_list = list(all)
    id =ID

    pub_all = rospy.Publisher("aruco_id", Float32MultiArray)
    pub_all.publish(Float32MultiArray(data = all_list))

    try:
        for i in data:
            if i[0] == id:
                info = [i[1], i[2], i[3]]
                pub = rospy.Publisher("aruco_xya", Float32MultiArray)
                pub.publish(Float32MultiArray(data = info))

    except:
        print('error')


def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, msg.encoding)
        output_msg(cv2_img)
        
    except CvBridgeError:
        print('error')

def get_ID(data):
    global ID
    ID = data.data

if __name__ == "__main__":

    rospy.init_node('imgaruco')
    image_topic = '/usb_cam/image_raw' # "/camera/color/image_raw"
    rospy.Subscriber('move_group_id', Int32, get_ID)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()