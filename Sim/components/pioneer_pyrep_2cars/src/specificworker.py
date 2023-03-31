#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from genericworker import *
import time
from pyrep import PyRep
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
import numpy as np
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
import cv2
from threading import Lock
import itertools as it
import numpy_indexed as npi


class TimeControl:
    def __init__(self, period_):
        self.counter = 0
        self.start = time.time()  # it doesn't exist yet, so initialize it
        self.start_print = time.time()  # it doesn't exist yet, so initialize it
        self.period = period_

    def wait(self):
        elapsed = time.time() - self.start
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        self.start = time.time()
        self.counter += 1
        if time.time() - self.start_print > 1:
            print("Freq -> ", self.counter, " Hz")
            self.counter = 0
            self.start_print = time.time()


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.tm = TransformManager()
        self.tm.add_transform("origin", "world",
                              pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0.0, 0.0, 0.0]), [0.0, 0.0, 0.0])
                              )

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        #SCENE_FILE = '../../etc/informatica.ttt'
        SCENE_FILE = '/home/robocomp/robocomp/components/MelexCar_private/CONTROL_LOCAL/Sim_Car/etc/digitaltwin_prueba.ttt'

        self.pr = PyRep()
        self.melex_cars = {"Melex_1":{}}#, "Melex_2":{}}
        # self.melex_cars = {"Melex_1": {}}
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        for i in [1, 1]:
            robot_name = "/Melex_" + str(i)
            self.melex_cars["Melex_%s" % i]["robot_object"] = Shape(robot_name) # TODO: DUPLICATE IN ORDER TO HAVE TWO CARS
            self.melex_cars["Melex_%s" % i]["back_left_wheel"] = Joint(robot_name+"/p3at_back_left_wheel_%s_joint" % i)
            self.melex_cars["Melex_%s" % i]["back_right_wheel"] = Joint(robot_name+"/p3at_back_right_wheel_%s_joint" % i)
            self.melex_cars["Melex_%s" % i]["front_left_wheel"] = Joint(robot_name+"/p3at_front_left_wheel_%s_joint"% i)
            self.melex_cars["Melex_%s" % i]["front_right_wheel"] = Joint(robot_name+"/p3at_front_right_wheel_%s_joint"% i)
            self.melex_cars["Melex_%s" % i]["radius"] = 105 #110  # mm
            self.melex_cars["Melex_%s" % i]["semi_width"] = 200 #140  # mm

        # cameras
            self.cameras_write = {}
            self.cameras_read = {}

            self.melex_cars["Melex_%s" % i]["front_left_camera_name"] = "left_camera_%s"% i
            cam = VisionSensor(robot_name+"/"+self.melex_cars["Melex_%s" % i]["front_left_camera_name"])
            self.cameras_write[self.melex_cars["Melex_%s" % i]["front_left_camera_name"]] = {"handle": cam,
                                                         "id": 0,
                                                         "angle": np.radians(cam.get_perspective_angle()),
                                                         "width": cam.get_resolution()[0],
                                                         "height": cam.get_resolution()[1],
                                                         "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                             np.radians(cam.get_perspective_angle() / 2)),
                                                         "rgb": np.array(0),
                                                         "depth": np.ndarray(0)}

            self.melex_cars["Melex_%s" % i]["front_center_camera_name"] = "three_frontal_camera_%s" % i
            cam = VisionSensor(robot_name+"/"+self.melex_cars["Melex_%s" % i]["front_center_camera_name"])
            self.cameras_write[self.melex_cars["Melex_%s" % i]["front_center_camera_name"]] = {"handle": cam,
                                                                "id": 1,
                                                                "angle": np.radians(cam.get_perspective_angle()),
                                                                "width": cam.get_resolution()[0],
                                                                "height": cam.get_resolution()[1],
                                                                "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                                    np.radians(cam.get_perspective_angle() / 2)),
                                                                "rgb": np.array(0),
                                                                "depth": np.ndarray(0)}

            self.melex_cars["Melex_%s" % i]["front_right_camera_name"] = "right_camera_%s" % i
            cam = VisionSensor(robot_name+"/"+self.melex_cars["Melex_%s" % i]["front_right_camera_name"])
            self.cameras_write[self.melex_cars["Melex_%s" % i]["front_right_camera_name"]] = {"handle": cam,
                                                                "id": 2,
                                                                "angle": np.radians(cam.get_perspective_angle()),
                                                                "width": cam.get_resolution()[0],
                                                                "height": cam.get_resolution()[1],
                                                                "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                                    np.radians(cam.get_perspective_angle() / 2)),
                                                                "rgb": np.array(0),
                                                                "depth": np.ndarray(0)}

            self.melex_cars["Melex_%s" % i]["virtual_camera_name"] = "melex_virtual_camera_%s" % i
            self.cameras_write[self.melex_cars["Melex_%s" % i]["virtual_camera_name"]] = {"handle": None,
                                                            "id": 3,
                                                            "angle": np.pi,
                                                            "width": self.cameras_write["three_frontal_camera_%s" % i]["height"]*3,
                                                            "height": self.cameras_write["three_frontal_camera_%s" % i]["width"],
                                                            "focal": self.cameras_write["three_frontal_camera_%s" % i]["focal"],
                                                            "rgb": np.array(0),
                                                            "depth": np.ndarray(0)}
            self.melex_cars["Melex_%s" % i]["cameras_write"] = self.cameras_write


            #self.cameras_read = self.cameras_write.copy()
            self.melex_cars["Melex_%s" % i]["cameras_read"] = self.cameras_write.copy()

            self.mutex_c = Lock()

            # laser
            # self.ldata_write = []
            # self.ldata_read = []
            self.melex_cars["Melex_%s" % i]["ldata_read"] = []
            self.melex_cars["Melex_%s" % i]["ldata_write"] = []



            # PoseEstimation
            # self.robot_full_pose_write = RoboCompFullPoseEstimation.FullPoseEuler()
            # self.robot_full_pose_read = RoboCompFullPoseEstimation.FullPoseEuler()
            self.melex_cars["Melex_%s" % i]["robot_full_pose_write"] = RoboCompFullPoseEstimationMulti.FullPoseEuler()
            self.melex_cars["Melex_%s" % i]["robot_full_pose_read"] = RoboCompFullPoseEstimationMulti.FullPoseEuler()
            self.mutex = Lock()


            # self.ldata = []
            # self.joystick_newdata = []
            # self.speed_robot = []
            # self.speed_robot_ant = []
            # self.last_received_data_time = 0
            self.melex_cars["Melex_%s" % i]["ldata"] = []
            self.melex_cars["Melex_%s" % i]["joystick_newdata"] = []
            self.melex_cars["Melex_%s" % i]["speed_robot"] = []
            self.melex_cars["Melex_%s" % i]["speed_robot_ant"] = []
            self.melex_cars["Melex_%s" % i]["last_received_data_time"] = 0
            self.melex_cars["Melex_%s" % i]["slam_0"] = Shape(robot_name+"/"+"slam_0_%s" % i)

            self.people = {}
            if Dummy.exists("Bill_base"):
                self.people["Bill"] = Dummy("Bill_base")
            elif Dummy.exists("Bill"):
                self.people["Bill"] = Dummy("Bill")

            for i in range(0, 2):
                name = "Bill#" + str(i)
                name_base = "Bill_base#" + str(i)
                if Dummy.exists(name_base):
                    self.people[name] = Dummy(name_base)
                elif Dummy.exists(name):
                    self.people[name] = Dummy(name)
    def compute(self):
        tc = TimeControl(0.001)
        while True:
            self.pr.step()

            for key in self.melex_cars.keys():

                self.read_laser(key)
                self.read_cameras(key)
                # self.read_joystick()
                self.read_robot_pose(key)
                self.move_robot(key)
                self.read_people()

            tc.wait()

    # def compute(self):
    #     tc = TimeControl(0.05)
    #     while True:
    #         self.pr.step()
    #         self.read_laser()
    #         self.read_cameras([self.front_left_camera_name, self.front_right_camera_name, self.front_center_camera_name])
    #         self.read_joystick()
    #         self.read_robot_pose()
    #         self.move_robot()
    #
    #         tc.wait()

    ###########################################
    ### LASER get and publish laser data
    ###########################################
    def read_people(self):
        self.people = {}
        if Dummy.exists("Bill_base"):
            self.people["Bill"] = Dummy("Bill_base")
        elif Dummy.exists("Bill"):
            self.people["Bill"] = Dummy("Bill")

        for i in range(0, 2):
            name = "Bill#" + str(i)
            name_base = "Bill_base#" + str(i)
            if Dummy.exists(name_base):
                self.people[name] = Dummy(name_base)
            elif Dummy.exists(name):
                self.people[name] = Dummy(name)
        people_data = RoboCompHumanToDSRPub.PeopleData()
        people_data.timestamp = time.time()
        people = []  # RoboCompHumanToDSRPub.People()
        for name, handle in self.people.items():
            pos = handle.get_position()
            rot = handle.get_orientation()
            person = RoboCompHumanToDSRPub.Person(len(people), pos[0] * 1000, pos[1] * 1000, pos[2] * 1000,
                                                  rot[2] - np.pi / 2,
                                                  {})
            people.append(person)
        try:
            people_data.peoplelist = people
            self.humantodsrpub_proxy.newPeopleData(people_data)
        except Ice.Exception as e:
            print(e)
    def read_laser(self,key):
        data = self.pr.script_call("get_depth_data@laser_front_1", 1)
        if len(data[1]) > 0:
            polar = np.zeros(shape=(int(len(data[1]) / 3), 2))
            i = 0
            for x, y, z in self.grouper(data[1], 3):  # extract non-intersecting groups of 3
                polar[i] = [-np.arctan2(y, x), np.linalg.norm([x, y])]  # add to list in polar coordinates
                i += 1

            angles = np.linspace(-np.radians(120), np.radians(120), 360)  # create regular angular values
            positions = np.searchsorted(angles,
                                        polar[:, 0])  # list of closest position in polar for each laser measurement
            self.melex_cars[key]["ldata_write"] = [RoboCompLaser.TData(a, 0) for a in angles]  # create empty 360 angle array
            pos, medians = npi.group_by(positions).median(polar[:, 1])  # group by repeated positions
            for p, m in it.zip_longest(pos, medians):  # fill the angles with measures
                if p < len(self.melex_cars[key]["ldata_write"]):
                    self.melex_cars[key]["ldata_write"][p].dist = int(m * 1000)  # to millimeters
            if self.melex_cars[key]["ldata_write"][0] == 0:
                self.melex_cars[key]["ldata_write"][0] = 200  # half robot width
            for i in range(0, len(self.melex_cars[key]["ldata_write"])):
                if self.melex_cars[key]["ldata_write"][i].dist == 0:
                    self.melex_cars[key]["ldata_write"][i].dist = self.melex_cars[key]["ldata_write"][i - 1].dist

            self.melex_cars[key]["ldata_read"], self.melex_cars[key]["ldata_write"] = self.melex_cars[key]["ldata_write"], self.melex_cars[key]["ldata_read"]

            try:
                self.laserpub_proxy.pushLaserData(self.melex_cars[key]["ldata_read"])
            except Ice.Exception as e:
                print(e)

    def grouper(self, inputs, n, fillvalue=None):
        iters = [iter(inputs)] * n
        return it.zip_longest(*iters, fillvalue=fillvalue)

    ###########################################
    ### CAMERAS get and publish cameras data
    ###########################################
    def read_cameras(self, key):
        # virtual frame
        virtual_width = self.melex_cars[key]["cameras_write"][self.melex_cars[key]["virtual_camera_name"]]['width']
        virtual_height = self.melex_cars[key]["cameras_write"][self.melex_cars[key]["virtual_camera_name"]]["height"]
        height = self.melex_cars[key]["cameras_write"][self.melex_cars[key]["front_left_camera_name"]]['width']
        width = self.melex_cars[key]["cameras_write"][self.melex_cars[key]["front_left_camera_name"]]['height']

        frame_virtual = np.zeros((width, height * 3, 3), dtype=np.uint8)
        winLeft = frame_virtual[0:width, 0:height]
        winCenter = frame_virtual[0:width, height:height * 2]
        winRight = frame_virtual[0:width, height * 2:height * 3]

        wins = [winLeft, winCenter, winRight]
        for w, cam_name in zip(wins,  self.melex_cars[key]["cameras_read"].keys()):
            cam = self.melex_cars[key]["cameras_write"][cam_name]
            image_float = cam["handle"].capture_rgb()
            depth = cam["handle"].capture_depth(True)
            image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                  dtype=cv2.CV_8U)
            buffer = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            w[:, :, :] = buffer[:, :, :]

        # cv2.imshow("virtual", frame_virtual)
        # print(frame_virtual.shape)

        #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cam = self.melex_cars[key]["cameras_write"][self.melex_cars[key]["virtual_camera_name"]]
        buffer = cv2.imencode(".jpg", frame_virtual)[1]
        cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"], width=cam["width"], height=cam["height"],
                                                     depth=3, focalx=cam["focal"], focaly=cam["focal"],
                                                     alivetime=time.time(), image=buffer)

        # cam["depth"] = RoboCompCameraRGBDSimple.TDepth(cameraID=cam["id"], width=cam["handle"].get_resolution()[0],
        #                                                height=cam["handle"].get_resolution()[1],
        #                                                focalx=cam["focal"], focaly=cam["focal"],
        #                                                alivetime=time.time(), depthFactor=1.0,
        #                                                depth=depth.tobytes())

        self.mutex_c.acquire()
        self.melex_cars[key]["cameras_write"], self.melex_cars[key]["cameras_read"] = self.melex_cars[key]["cameras_read"], self.melex_cars[key]["cameras_write"]
        self.mutex_c.release()

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata:  # and (time.time() - self.joystick_newdata[1]) > 0.1:
            datos = self.joystick_newdata[0]
            adv = 0.0
            rot = 0.0
            for x in datos.axes:
                if x.name == "advance":
                    adv = x.value if np.abs(x.value) > 10 else 0
                if x.name == "rotate" or x.name == "turn":
                    rot = x.value if np.abs(x.value) > 0.01 else 0

            converted = self.convert_base_speed_to_motors_speed(adv, rot)
            #print("Joystick ", [adv, rot], converted)
            self.joystick_newdata = None
            self.last_received_data_time = time.time()
        else:
            elapsed = time.time() - self.last_received_data_time
            if elapsed > 2 and elapsed < 3:
                self.convert_base_speed_to_motors_speed(0, 0)

    def convert_base_speed_to_motors_speed(self, adv, rot, key):
        #  adv = r*(Wl + Wr)/2
        #  rot = r*(-Wl + Wr)/2c
        #  isolating Wl,Wr
        #  Wl = ( adv - c*rot ) / r
        #  Wr = ( adv + c*rot ) / r
        left_vel = (adv - self.melex_cars[key]["semi_width"] * 1.5*rot) / self.melex_cars[key]["radius"]
        right_vel = (adv + self.melex_cars[key]["semi_width"] * 1.5*rot) / self.melex_cars[key]["radius"]
        
        #left_vel = (adv + self.semi_width * rot) / self.radius
        #right_vel = (adv - self.semi_width * rot) / self.radius
        
        self.melex_cars[key]["back_left_wheel"].set_joint_target_velocity(left_vel)
        self.melex_cars[key]["back_right_wheel"].set_joint_target_velocity(right_vel)
        self.melex_cars[key]["front_left_wheel"].set_joint_target_velocity(left_vel)
        self.melex_cars[key]["front_right_wheel"].set_joint_target_velocity(right_vel)
        #print("VELOCIDADES RUEDA: ", left_vel , right_vel)
        return left_vel, right_vel

    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    def read_robot_pose(self, key):
        # slam_0 = Shape("slam_0")
        pose = self.melex_cars[key]["slam_0"].get_position()
        rot = self.melex_cars[key]["slam_0"].get_orientation()
        linear_vel, ang_vel = self.melex_cars[key]["slam_0"].get_velocity()

        isMoving = np.abs(linear_vel[0]) > 0.01 or np.abs(linear_vel[1]) > 0.01 or np.abs(ang_vel[2]) > 0.01
        self.melex_cars[key]["bState"] = RoboCompGenericBase.TBaseState(x=pose[0] * 1000,
                                                     z=pose[1] * 1000,
                                                     alpha=rot[2]- np.pi,
                                                     advVx=linear_vel[0] * 1000,
                                                     advVz=linear_vel[1] * 1000,
                                                     rotV=ang_vel[2],
                                                     isMoving=isMoving)

        # self.tm.add_transform("world", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                             ([rot[0], rot[1], rot[2]-np.pi]),
        #                                                             [pose[0]*1000.0, pose[1]*1000.0, pose[2]*1000.0]
        #                                                             ))
        #
        # t = self.tm.get_transform("origin", "robot")
        # angles = pyrot.extrinsic_euler_xyz_from_active_matrix(t[0:3, 0:3])

        self.melex_cars[key]["robot_full_pose_write"].x = pose[0] * 1000
        self.melex_cars[key]["robot_full_pose_write"].y = pose[1] * 1000
        self.melex_cars[key]["robot_full_pose_write"].z = pose[2] * 1000
        self.melex_cars[key]["robot_full_pose_write"].rx = rot[0]
        self.melex_cars[key]["robot_full_pose_write"].ry = rot[1]
        self.melex_cars[key]["robot_full_pose_write"].rz = rot[2] - np.pi
        self.melex_cars[key]["robot_full_pose_write"].vx = linear_vel[0] * 1000.0
        self.melex_cars[key]["robot_full_pose_write"].vy = linear_vel[1] * 1000.0
        self.melex_cars[key]["robot_full_pose_write"].vz = linear_vel[2] * 1000.0
        self.melex_cars[key]["robot_full_pose_write"].vrx = ang_vel[0]
        self.melex_cars[key]["robot_full_pose_write"].vry = ang_vel[1]
        self.melex_cars[key]["robot_full_pose_write"].vrz = ang_vel[2]

        # swap
        self.mutex.acquire()
        self.melex_cars[key]["robot_full_pose_write"], self.melex_cars[key]["robot_full_pose_read"] = self.melex_cars[key]["robot_full_pose_read"], self.melex_cars[key]["robot_full_pose_write"]
        self.mutex.release()

    ###########################################
    ### MOVE ROBOT from Omnirobot interface
    ###########################################
    def move_robot(self, key):

        if self.melex_cars[key]["speed_robot"]: #!= self.speed_robot_ant:  # or (isMoving and self.speed_robot == [0,0,0]):
            self.convert_base_speed_to_motors_speed(self.melex_cars[key]["speed_robot"][0], self.melex_cars[key]["speed_robot"][1],key)
            # print("Velocities sent to robot:", self.speed_robot)
            #self.speed_robot_ant = self.speed_robot
            self.melex_cars[key]["speed_robot"] = None

    ##################################################################################
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    ###################################################################################
    def JoystickAdapter_sendData(self, data):
        self.joystick_newdata = [data, time.time()]

    ##################################################################################
    #                       Methods for CameraRGBDSimple
    # ===============================================================================
    #
    # getAll  ID 3 for VIRTUAL CAMERA
    #
    def CameraRGBDSimpleMulti_getAll(self, robotid, camera):
        if camera in self.melex_cars["Melex_%s" % str(robotid)]["cameras_read"].keys():
            return RoboCompCameraRGBDSimpleMulti.TRGBD(self.melex_cars["Melex_%s" % str(robotid)]["cameras_read"][camera]["rgb"], self.melex_cars["Melex_%s" % str(robotid)]["cameras_read"][camera]["depth"])
        else:
            e = RoboCompCameraRGBDSimpleMulti.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    #
    # getDepth
    #
    def CameraRGBDSimpleMulti_getDepth(self, robotid, camera):
        if camera in self.melex_cars["Melex_%s" % str(robotid)]["cameras_read"].keys():
            return self.melex_cars["Melex_%s" % str(robotid)]["cameras_read"][camera]["depth"]
        else:
            e = RoboCompCameraRGBDSimpleMulti.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    #
    # getImage  ID 3 for VIRTUAL CAMERA
    #
    def CameraRGBDSimpleMulti_getImage(self, robotid, camera):
        if camera in self.melex_cars["Melex_%s" % str(robotid)]["cameras_read"].keys():
            return self.melex_cars["Melex_%s" % str(robotid)]["cameras_read"][camera]["rgb"]
        else:
            e = RoboCompCameraRGBDSimpleMulti.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    ##############################################
    ## Omnibase
    #############################################
    #
    # correctOdometer
    #
    # def DifferentialRobot_correctOdometer(self, x, z, alpha):
    #     pass
    #
    # #
    # # getBasePose
    # #
    # def DifferentialRobot_getBasePose(self, id):
    #     #
    #     # implementCODE
    #     #
    #     x = self.melex_cars[id]["bState"].x
    #     z = self.melex_cars[id]["bState"].z
    #     alpha = self.melex_cars[id]["bState"].alpha
    #     return [x, z, alpha]

    #
    # getBaseState
    #
    def DifferentialRobotMulti_getBaseState(self, id):
        return self.melex_cars["Melex_%s" % str(id)]['bState']

    #
    # resetOdometer
    #
    # def DifferentialRobot_resetOdometer(self):
    #     pass
    #
    # #
    # # setOdometer
    # #
    # def DifferentialRobot_setOdometer(self, state):
    #     pass
    #
    # #
    # # setOdometerPose
    # #
    # def DifferentialRobot_setOdometerPose(self, x, z, alpha):
    #     pass

    #
    # setSpeedBase
    #
    def DifferentialRobotMulti_setSpeedBase(self, id, advz, rot):
        #print("Speed", advz , rot)
        self.melex_cars["Melex_"+str(id)]["speed_robot"] = [advz, rot]

    #
    # stopBase
    #
    # def DifferentialRobot_stopBase(self):
    #     pass

    # ===================================================================
    # CoppeliaUtils
    # ===================================================================
    def CoppeliaUtils_addOrModifyDummy(self, type, name, pose):
        if not Dummy.exists(name):
            dummy = Dummy.create(0.1)
            # one color for each type of dummy
            if type == RoboCompCoppeliaUtils.TargetTypes.Info:
                pass
            if type == RoboCompCoppeliaUtils.TargetTypes.Hand:
                pass
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                pass
            dummy.set_name(name)
        else:
            dummy = Dummy(name)
            parent_frame_object = None
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                parent_frame_object = Dummy("viriato_head_camera_pan_tilt")
            # print("Coppelia ", name, pose.x/1000, pose.y/1000, pose.z/1000)
            dummy.set_position([pose.x / 1000., pose.y / 1000., pose.z / 1000.], parent_frame_object)
            dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getFullPose method from FullPoseEstimation interface
    #
    def FullPoseEstimationMulti_getFullPoseEuler(self, robotid):
        #print("FullPoseEstimationMulti_getFullPoseEuler", self.melex_cars["Melex_%s" % str(robotid)]["robot_full_pose_read"])
        return self.melex_cars["Melex_%s" % str(robotid)]["robot_full_pose_read"]
        # return RoboCompFullPoseEstimationMulti.FullPoseEuler()

    def FullPoseEstimationMulti_getFullPoseMatrix(self, robotid):
        t = self.tm.get_transform("origin", "robot")
        m = RoboCompFullPoseEstimationMulti.FullPoseMatrix()
        m.m00 = t[0][0]
        m.m01 = t[0][1]
        m.m02 = t[0][2]
        m.m03 = t[0][3]
        m.m10 = t[1][0]
        m.m11 = t[1][1]
        m.m12 = t[1][2]
        m.m13 = t[1][3]
        m.m20 = t[2][0]
        m.m21 = t[2][1]
        m.m22 = t[2][2]
        m.m23 = t[2][3]
        m.m30 = t[3][0]
        m.m31 = t[3][1]
        m.m32 = t[3][2]
        m.m33 = t[3][3]
        return m

    #
    # IMPLEMENTATION of setInitialPose method from FullPoseEstimation interface
    #
    def FullPoseEstimationMulti_setInitialPose(self, robotid, x, y, z, rx, ry, rz):

        # should move robot in Coppelia to designated pose
        self.tm.add_transform("origin", "world",
                               pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]), [x, y, z])
        )

    #
    # IMPLEMENTATION of getAllSensorDistances method from Ultrasound interface
    #
    def Ultrasound_getAllSensorDistances(self):
        ret = RoboCompUltrasound.SensorsState()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getAllSensorParams method from Ultrasound interface
    #
    def Ultrasound_getAllSensorParams(self):
        ret = RoboCompUltrasound.SensorParamsList()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getBusParams method from Ultrasound interface
    #
    def Ultrasound_getBusParams(self):
        ret = RoboCompUltrasound.BusParams()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getSensorDistance method from Ultrasound interface
    #
    def Ultrasound_getSensorDistance(self, sensor):
        ret = int()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getSensorParams method from Ultrasound interface
    #
    def Ultrasound_getSensorParams(self, sensor):
        ret = RoboCompUltrasound.SensorParams()
        #
        # write your CODE here
        #
        return ret

    # ===================================================================
    # ===================================================================
    #
    # IMPLEMENTATION of getRSSIState method from RSSIStatus interface
    #
    def RSSIStatus_getRSSIState(self):
        ret = RoboCompRSSIStatus.TRSSI()
        ret.percentage = 100;
        return ret

    #
    # IMPLEMENTATION of getBatteryState method from BatteryStatus interface
    #
    def BatteryStatus_getBatteryState(self):
        ret = RoboCompBatteryStatus.TBattery()
        ret.percentage = 100
        return ret
    #

# =============== Methods for Component Implements ==================
    # ===================================================================
    #
    # IMPLEMENTATION of getLaserAndBStateData method from Laser interface
    #
    # def Laser_getLaserAndBStateData(self):
    #     ret = RoboCompLaser.TLaserData()
    #     bState = RoboCompGenericBase.TBaseState()
    #     return [ret, bState]
    #
    # IMPLEMENTATION of getLaserConfData method from Laser interface
    #
    def LaserMulti_getLaserConfData(self, robotid):
        ret = RoboCompLaserMulti.LaserConfData()
        return ret
    #
    # IMPLEMENTATION of getLaserData method from Laser interface
    #
    def LaserMulti_getLaserData(self, robotid):
        return  self.melex_cars["Melex_%s" % str(robotid)]["ldata_read"]
    #
    # IMPLEMENTATION of getData method from GpsUblox interface
    #
    def GpsUbloxMulti_getData(self, robotid):
        #print("GpsUbloxMulti_getData",
              #self.melex_cars["Melex_%s" % str(robotid)]["robot_full_pose_read"])
        ret = RoboCompGpsUbloxMulti.DatosGPS()
        ret.latitude = 0
        ret.longitude = 0
        ret.altitude = 0
        ret.UYMx = 0
        ret.UTMy = 0
        ret.mapx = self.melex_cars["Melex_%s" % str(robotid)]["robot_full_pose_read"].x
        ret.mapy = self.melex_cars["Melex_%s" % str(robotid)]["robot_full_pose_read"].y
        #print(ret)
        return ret

    #
    # IMPLEMENTATION of setInitialPose method from GpsUblox interface
    #
    def GpsUbloxMulti_setInitialPose(self, robotid, x, y):
        pass

    def GpsUblox_getData(self):
        ret = RoboCompGpsUblox.DatosGPS()
        ret.latitude = 0
        ret.longitude = 0
        ret.altitude = 0
        ret.UYMx = 0
        ret.UTMy = 0
        ret.mapx = self.robot_full_pose_read.x
        ret.mapy = self.robot_full_pose_read.y
        return ret

    #
    # IMPLEMENTATION of setInitialPose method from GpsUblox interface
    #
    def GpsUblox_setInitialPose(self, x, y):
        pass


