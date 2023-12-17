#!/usr/bin/env python
# -*- coding:utf-8 -*-
# limo_application/scripts/lane_detection/control.py
# WeGo LIMO Pro를 이용한 주행 코드
# mk2-1

import rospy
import os
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from object_msgs.msg import ObjectArray
from limo_base.msg import LimoStatus
from dynamic_reconfigure.server import Server
from limo_application.cfg import controlConfig
from ar_track_alvar_msgs.msg import AlvarMarkers

import math


class LimoController:
    '''
        차선 인식, 횡단보도 인식, LiDAR 기반 장애물 인식, YOLO 기반 신호등 및 표지판 인식
        위 기능을 통합한 전체 주행 코드
        Private Params --> control_topic_name
        < Subscriber >
        limo_status (LimoStatus) --> LIMO의 Motion Model 확인용
        /limo/lane_x (Int32) --> 인식된 차선 (카메라 좌표계 x)
        /limo/crosswalk_y (Int32) --> 인식된 횡단보도 (카메라 좌표계 y)
        /limo/traffic_light (String) --> YOLO 기반 인식 결과
        /limo/lidar_warning (String) --> LiDAR 기반 장애물 검출 결과
        < Publisher >
        /cmd_vel (Twist) --> Default 출력, LIMO 제어를 위한 Topic
    '''
    def __init__(self):
        rospy.init_node('limo_control', anonymous=True)
        self.LIMO_WHEELBASE = 0.2
        self.distance_to_ref = 0
        self.right_distance_to_ref = 0
        self.true_distance_to_ref = 0
        #self.crosswalk_detected = False
        #self.yolo_object = "green"
        self.e_stop = "Safe"
        self.parking = "nono"
        #self.is_pedestrian_stop_available = True
        #self.pedestrian_stop_time = 5.0
        #self.pedestrian_stop_last_time = rospy.Time.now().to_sec()
        #self.yolo_object_last_time = rospy.Time.now().to_sec()
        #self.bbox_size = [0, 0]
        self.limo_mode = "ackermann"

        self.dkdkdkdk = 0
        
        self.stay = 0
        self.rihgt_stay = 0
        self.right = 0
        self.left = 0
        self.lane_time_ok = 0

        self.marker_0 = 0
        self.marker_1 = 0
        self.marker_2 = 0
        self.marker_3 = 0
        self.marker_00 = 0
        self.marker_11 = 0
        self.marker_22 = 0
        self.marker_33 = 0
        self.marker_000 = 0
        self.marker_111 = 0
        self.marker_222 = 0
        self.marker_333 = 0
        self.markertime_count = 0
        self.marker_distance = 0

        self.rrr = 0
        self.lll = 0
        self.lll2 = 0

        self.bump = "not_bump"

        self.ppp = 0
        #self.bool = False
        #self.current_time = rospy.get_time()

        # /ar_pose_marker 토픽으로부터 AlvarMarkers 메시지를 수신하는 Subscriber 생성
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        srv = Server(controlConfig, self.reconfigure_callback)
        rospy.Subscriber("limo_status", LimoStatus, self.limo_status_callback)
        rospy.Subscriber("/limo/lane_x", Int32, self.lane_x_callback)
        rospy.Subscriber("/limo/right_lane_x", Int32, self.right_lane_x_callback)
        #rospy.Subscriber("/limo/crosswalk_y", Int32, self.crosswalk_y_callback)
        #rospy.Subscriber("/limo/yolo_object", ObjectArray, self.yolo_object_callback)
        rospy.Subscriber("/limo/lidar_warning", String, self.lidar_warning_callback)
        rospy.Subscriber("/limo/imu_pitch", String, self.bump_detect_callback)
        rospy.Subscriber("/limo/parking", String, self.parking_callback)
        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel"), Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.drive_callback)

    # return float
   
    
    # ==============================================
    #               Callback Functions
    # ==============================================

    def limo_status_callback(self, _data):
        '''
            LIMO의 상태가 Ackermann인지, Differential인지 확인하여, 모드 변경
            최종 출력의 angular.z 값이 달라지므로, 이와 같은 처리가 필요
        '''
        if _data.motion_mode == 1:
            if self.limo_mode == "ackermann":
                pass
            else:
                self.limo_mode = "ackermann"
                rospy.loginfo("Mode Changed --> Ackermann")
        else:
            if self.limo_mode == "diff":
                pass
            else:
                self.limo_mode = "diff"
                rospy.loginfo("Mode Changed --> Differential Drive")

    def marker_CB(self, data):
        self.markertime = rospy.get_time()
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            for marker in data.markers: 
            # data.markers 에 있는 마커 정보를 처리
                # id가 0번일 경우
                self.marker_distance = marker.pose.pose.position.x
                rospy.loginfo(self.marker_distance)
                if marker.id == 0 and self.marker_distance >= 0.1:  # 정지 마커
                    if self.marker_00 == 0:
                        self.marker_0 = 1
                        self.marker_00 = 1    
                    else:
                        if (self.marker_1 == 0):
                            self.marker_11 = 0
                        if (self.marker_2 == 0):
                            self.marker_22 = 0          
                # id가 1번일 경우
                if marker.id == 1 and self.marker_distance >= 0.1: #오른쪽
                    if self.marker_11 == 0:
                        self.marker_1 = 1
                        self.marker_11 = 1
                    else:
                        if (self.marker_1 == 0):
                            self.marker_11 = 0
                        if (self.marker_2 == 0):
                            self.marker_22 = 0     
                # id가 2번일 경우
                if marker.id == 2 and self.marker_distance >= 0.1: #왼쪽
                    if self.marker_22 == 0:
                        self.marker_2 = 1
                        self.marker_22 = 1
                    else:
                        if (self.marker_1 == 0):
                            self.marker_11 = 0
                        if (self.marker_2 == 0):
                            self.marker_22 = 0     
                # id가 3일 경우 
                if marker.id == 3 and self.marker_distance >= 0.1: #주차
                    if self.marker_33 == 0:
                        self.marker_3 = 1
                        self.marker_33 = 1
                    else:
                        if (self.marker_1 == 0):
                            self.marker_11 = 0
                        if (self.marker_2 == 0):
                            self.marker_22 = 0     
                
                self.markertime_count = rospy.get_time()

              
        else:
            if (self.markertime - self.markertime_count >= 1):
                if (self.marker_0 == 0):
                    self.marker_00 = 0
                if (self.marker_1 == 0):
                    self.marker_11 = 0
                if (self.marker_2 == 0):
                    self.marker_22 = 0
               
                    

            
            
    def lidar_warning_callback(self, _data):
        '''
            장애물 유무 저장
        '''
        self.e_stop = _data.data

    def bump_detect_callback(self, _data):
        '''
            방지 턱 여부 결정
        '''
        self.bump = _data.data
    
    def parking_callback(self, _data):
        '''
            주차시 벽과 거리 측정
        '''
        self.parking = _data.data


    def lane_x_callback(self, _data):
        '''
            실제 기준 좌표와 검출된 차선과의 거리 저장
        '''
        if _data.data == -1:
            self.left = 1
            #rospy.logwarn("=============================")
        else:
            self.distance_to_ref = self.REF_X - _data.data
            self.left = 0
            

    def right_lane_x_callback(self, _data):
        '''
            실제 기준 좌표와 검출된 차선과의 거리 저장
        '''
        if _data.data == 1:
            self.right = 1
            #rospy.logwarn("------------------------------")
        else:
            self.right_distance_to_ref = self.right_REF_X - _data.data
            self.right = 0   


    def reconfigure_callback(self, _config, _level):
        '''
            Dynamic_Reconfigure를 활용
            차량 제어 속도 (BASE_SPEED)
            횡방향 제어 Gain (LATERAL_GAIN)
            차선 기준 좌표 (카메라 픽셀 좌표계 기준) (REF_X)
        '''
        self.BASE_SPEED = _config.base_speed
        self.LATERAL_GAIN = float(_config.lateral_gain * 0.0015)
        self.REF_X = _config.reference_lane_x
        self.right_REF_X = self.REF_X + 175
        self.PEDE_STOP_WIDTH = _config.pedestrian_width_min
        return _config

    def drive_callback(self, _event):
        '''
            입력된 데이터를 종합하여,
            속도 및 조향을 조절하여 최종 cmd_vel에 Publish
        '''
        
        

        if (self.left == 0 and self.right == 0):
            self.true_distance_to_ref = self.distance_to_ref
            self.stay = self.distance_to_ref
            #rospy.logwarn("both")
            

        if (self.left == 0 and self.right == 1):
            self.true_distance_to_ref = self.distance_to_ref
            self.stay = self.distance_to_ref
            #rospy.logwarn("left")
            
            
        if (self.left == 1 and self.right == 0):
            if self.e_stop == "ahhhhhhhh":
                self.true_distance_to_ref = self.stay
            else:
                self.true_distance_to_ref = self.right_distance_to_ref
                self.stay = self.right_distance_to_ref
            #rospy.logwarn("right")

        if (self.left == 1 and self.right == 1):
            self.true_distance_to_ref = self.stay
            #rospy.logwarn("none")
        

        # print(current_time)
        drive_data = Twist()
        drive_data.angular.z = self.true_distance_to_ref * self.LATERAL_GAIN
        #rospy.loginfo("OFF_CENTER, Lateral_Gain = {}, {}".format(self.distance_to_ref, self.LATERAL_GAIN))
        #rospy.loginfo("Bbox Size = {}, Bbox_width_min = {}".format(self.bbox_size, self.PEDE_STOP_WIDTH))

        try:
            if self.e_stop == "Warning":
                drive_data.linear.x = 0.0
                drive_data.angular.z = 0.0
            
            elif (self.bump == "bump"):
                drive_data.linear.x = self.BASE_SPEED / 2
                drive_data.angular.z = 0.0
                
            else:
                if (self.marker_0 == 1):
                    self.loop_time = rospy.get_time()
                    self.wait_time = rospy.get_time()
                    while (self.wait_time - self.loop_time <= 3):
                        drive_data.linear.x = 0.0
                        drive_data.angular.z = 0.0
                        self.wait_time = rospy.get_time()
                    self.marker_0 = 0
                    #rospy.logwarn("marker 0 is there , Stop!")                   
                
                elif (self.marker_1 == 1):
                    #rospy.logwarn("Marker 1 dedect!")
                    self.wait_time = rospy.get_time()
                    drive_data.linear.x = self.BASE_SPEED
                    if (self.right == 1):
                        if self.wait_time - self.loop_time >= 0.65:
                            self.wait_time = rospy.get_time()
                            self.rrr = 1
                    else:
                        self.loop_time = rospy.get_time()

                    if self.rrr == 1:
                        if (self.right == 0 and self.left == 0):
                            if self.wait_time - self.lloop_time >= 1:
                                self.marker_1 = 0
                                self.rrr = 0
                        else:
                            self.lloop_time = rospy.get_time()
                            drive_data.angular.z = -1.4
                            



                elif (self.marker_2 == 1):
                    #rospy.logwarn("Marker 1 dedect!")
                    self.wait_time = rospy.get_time()
                    drive_data.linear.x = self.BASE_SPEED
                    if (self.left == 1):
                        if self.wait_time - self.loop_time >= 0.7:
                            self.wait_time = rospy.get_time()
                            self.lll = 1
                    else:
                        self.loop_time = rospy.get_time()

                    if self.lll == 1:
                        if (self.right == 0 and self.left == 0):
                            if self.lll2 == 0:
                                self.lll2 = 1
                                drive_data.angular.z = 1.4
                            if self.lll2 == 2:
                                if self.wait_time - self.lloop_time >= 1:
                                    self.marker_2 = 0
                                    self.lll = 0
                                    self.lll2 = 0
                        elif self.right == 1 and self.left == 1:
                            if self.lll2 == 2:
                                drive_data.angular.z = 0
                                self.marker_2 = 0
                                self.lll = 0
                                self.lll2 = 0
                            elif self.lll2 == 0:
                                self.lll2 = 2
                            else:
                                self.lloop_time = rospy.get_time()
                                drive_data.angular.z = 1.4
                        else:
                            self.lloop_time = rospy.get_time()
                            drive_data.angular.z = 1.4
                            if self.lll2 == 1:
                                self.lll2 = 2
                    


                elif (self.marker_3 == 1):
                    if self.parking == "back" :
                        rospy.logwarn("------------------------")
                        if self.right == 1 and self.left == 1:
                            if self.ppp == 0:
                                self.ppp = 1
                                rospy.logwarn("===================")
                            if self.ppp == 2:
                                rospy.logwarn("++++++++++++++++++++")
                                self.loop_time = rospy.get_time()
                                self.wait_time = rospy.get_time()
                                while (self.wait_time - self.loop_time <= 1):
                                    self.wait_time = rospy.get_time()
                                    drive_data.linear.x = -(self.BASE_SPEED / 2)
                                    drive_data.angular.z = 0.0
                                    self.wait_time = rospy.get_time()
                                    self.drive_pub.publish(drive_data)
                                self.marker_3 = 0
                                self.ppp = 0

                        else:
                            rospy.logwarn("lllllllllllllllllllllllllll")
                            if self.ppp == 1:
                                rospy.logwarn("////////////////////////")
                                self.ppp =2
                            drive_data.linear.x = -(self.BASE_SPEED / 2)
                            drive_data.angular.z = -1.4
                    else:
                        drive_data.linear.x = self.BASE_SPEED 

                    #rospy.logwarn("marker 2 is there , Left!")
                    
                    
                else:
                    drive_data.linear.x = self.BASE_SPEED
                    #rospy.loginfo("All Clear, Just Drive!")

            if self.limo_mode == "diff":
                self.drive_pub.publish(drive_data)
            elif self.limo_mode == "ackermann":
                if drive_data.linear.x == 0:
                    drive_data.angular.z = 0

                else:
                    drive_data.angular.z = \
                        math.tan(drive_data.angular.z / 2) * drive_data.linear.x / self.LIMO_WHEELBASE
                    # 2를 나눈 것은 Differential과 GAIN비율을 맞추기 위함
                    self.drive_pub.publish(drive_data)
                    #rospy.loginfo(drive_data.angular.z)


        except Exception as e:
            rospy.logwarn(e)


def run():
    new_class = LimoController()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down") 
