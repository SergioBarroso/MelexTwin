#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
from APIManager import *

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)
import time
from pydsr import *
import csv
import numpy as np
from scipy import spatial
import utm
from pyproj import Proj, transform, Transformer, Geod


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000

        self.agent_id = 666
        self.g = DSRGraph(0, "api_communicator", self.agent_id)
        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)
        QObject.connect(self.ui.busy_melex_1, QtCore.SIGNAL('clicked()'), lambda id=1: self.set_busy(id))
        QObject.connect(self.ui.busy_melex_2, QtCore.SIGNAL('clicked()'), lambda id=2: self.set_busy(id))
        QObject.connect(self.ui.start_melex_1, QtCore.SIGNAL('clicked()'), lambda id=1: self.start_mission(id))
        QObject.connect(self.ui.start_melex_2, QtCore.SIGNAL('clicked()'), lambda id=2: self.start_mission(id))
        QObject.connect(self.ui.path_melex_1, QtCore.SIGNAL('clicked()'), lambda id=1: self.insert_path(id))
        QObject.connect(self.ui.path_melex_2, QtCore.SIGNAL('clicked()'), lambda id=2: self.insert_path(id))
        QObject.connect(self.ui.intention_melex_1, QtCore.SIGNAL('clicked()'), lambda id=1: self.insert_intention_node(id))
        QObject.connect(self.ui.intention_melex_2, QtCore.SIGNAL('clicked()'), lambda id=2: self.insert_intention_node(id))
        QObject.connect(self.ui.emergency_melex_1, QtCore.SIGNAL('clicked()'), lambda id=1: self.stop_mission(id))
        QObject.connect(self.ui.emergency_melex_2, QtCore.SIGNAL('clicked()'), lambda id=2: self.stop_mission(id))
        self.ui.progress_list.itemDoubleClicked.connect(self.remove_item_progress)
        self.ui.requests_list.itemDoubleClicked.connect(self.remove_item_requests)
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        self.paradas = {
            '1': [4747.0, 19000.0],
            '2': [2810.0, 19100.0],
            '3': [-789.0, -37713.0],
            '4': [10596.0, -38619.0]
        }

        self.theta = 0.24906021553765
        self.phi = 0
        # Build rotation matrix
        rot = np.array([
            [np.cos(self.theta), -np.sin(self.theta), 0.0, 0.0],
            [np.sin(self.theta), np.cos(self.theta), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0], ])

        # Build shear/skew matrix
        m = np.tan(self.phi)
        skew = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [m, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0], ])

        # get affine transform
        self.a = rot @ skew
        print(self.a)

        # Build pipeline
        self.pt_transform = Transformer.from_pipeline(
            f"+proj=pipeline "
            # f"+step +proj=affine +xoff={origin_x} +yoff={origin_y} "
            f"+step +proj=affine +xoff={0} +yoff={0} "
            f"+s11={self.a[0, 0]} +s12={self.a[0, 1]} +s13={self.a[0, 2]} "
            f"+s21={self.a[1, 0]} +s22={self.a[1, 1]} +s23={self.a[1, 2]} "
            f"+s31={self.a[2, 0]} +s32={self.a[2, 1]} +s33={self.a[2, 2]} ")

        self.fixedX = -371895.97221695725  # -371886.67816326916
        self.fixedY = 4417962.616456918  # 4417967.8278194275

        self.x_path = []
        self.y_path = []
        with open('etc/rutacompleta.csv', mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    print(f'Column names are {", ".join(row)}')
                    line_count += 1
                #print(row["x"], row["y"])
                self.x_path.append(int(float(row["x"])))
                self.y_path.append(int(float(row["y"])))
                # print("PATH", self.x_path, self.y_path)
                line_count += 1
            self.path_xy = np.column_stack((self.x_path, self.y_path))
            print("NUMPY PATH", self.path_xy)


        ###############API LOGIC#############
        self.task_inprogress = []
        self.task_received = []
        self.task_data_in_progress = {
            'state': "progress",
            'car': None
        }
        self.n_cars = 2
        self.cars = {}
        # self.free_cars = [APIMng('Melex' + str(i + 1)) for i in range(self.n_cars)]
        self.cars = {}
        for i in range(self.n_cars):
            self.cars['Melex'+str(i+1)] = {"Estado": "waitingRequest", "Manager": APIMng('Melex'+str(i+1))}
            self.cars['Melex'+str(i+1)]["Manager"].create_ParametersCCAA_json({"busy": False})
            #self.cars['Melex'+str(i+1)]["Manager"].put_ParametersCCAA_api()

        self.free_cars = []

        #Comprobación de tareas activas y borrado de las mismas
        #list_task, ret = self.cars["Melex1"]["Manager"].get_all_tasks() #Comprueba si hay tarea en progreso
        task_data = {
            'state': "complete",
            'car': None
        }

        #BORRADO DE REQUESTS, NO USAR POR PETICIÓN DE GAMMA

        #if ret:
         #   for t in list_task:
          #      self.cars["Melex1"]["Manager"].requests_put_json(task_data, t['id'])
           #     self.cars["Melex1"]["Manager"].delete_tasks()
        #else:
         #   print("NO HAY TAREAS EN COLA")


        #for key in self.cars.keys():
         #   self.cars[key]["Manager"].create_ParametersCCAA_json({"busy": False})
          #  self.cars[key]["Manager"].put_ParametersCCAA_api()
        #time.sleep(1)




    def __del__(self):
        """Destructor"""
        pass

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):

        #TODO: actualizar estado coche al llegar al destino

       # self.update_cars_info()  # ACTUALIZAMOS INFORMACION MELEX EN API

        for key in self.cars.keys():
            print(key, self.cars[key]["Estado"])

        #self.task_inprogress, self.task_received = self.get_server_tasks()
        #print("TAREAS EN PROGRESO", len(self.task_inprogress))
        #print("TAREAS EN COLA", len(self.task_received))
        self.ui.progress_list.clear()
        if len(self.task_inprogress) > 0:
            for progress in self.task_inprogress:
                self.insert_stop_coords(progress)
                self.create_itemwidget(progress)
                # new_item = QTreeWidgetItem()
                #
                # new_item.setText(0, str(progress["id"]))
                # new_item.setText(1, str(progress["car"]))
                # new_item.setText(2, str(progress["pickupStop"]))
                # new_item.setText(5, str(progress["arrivalStop"]))
                # self.ui.progress_list.addTopLevelItem(new_item)

                print(progress)
                # print("coche", progress["car"], progress["pickupStop"], progress["arrivalStop"])
        self.ui.requests_list.clear()
        if len(self.task_received) > 0:
            self.tasks_distribution()
            # for received in self.task_received:
            #     for key in self.cars.keys():
            #         if self.cars[key]["Estado"] == "waitingRequest":
            #             self.insert_pickup_coords(received, key[5])
            #             task_data = {
            #                         'state': "progress",
            #                         'car': key
            #                     }
            #             self.cars[key]["Manager"].requests_put_json(task_data, received['id'])
            #             self.cars[key]["Manager"].put_requests_api()
            #             self.cars[key]["Estado"] = "goPickupStop"
            #             self.update_cars_info()
            #             break

        self.set_waiting_state()
        self.check_tasks_completed()

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    def tasks_distribution(self):

        for received in self.task_received:
            self.create_itemwidget(received)
            # new_item = QTreeWidgetItem()
            # new_item.setText(0, str(received["id"]))
            # new_item.setText(1, str(received["car"]))
            # new_item.setText(2, str(received["pickupStop"]))
            # new_item.setText(5, str(received["arrivalStop"]))
            # self.ui.requests_list.addTopLevelItem(new_item)
            for key in self.cars.keys():
                if self.cars[key]["Estado"] == "waitingRequest":
                    self.insert_pickup_coords(received, key[5])
                    self.task_data_in_progress["car"] = key
                    self.cars[key]["Manager"].requests_put_json(self.task_data_in_progress, received['id'])
                    self.cars[key]["Manager"].put_requests_api()
                    self.cars[key]["Estado"] = "goPickupStop"
                    self.update_cars_info()
                    break

    def create_itemwidget(self, received):
        new_item = QTreeWidgetItem()
        new_item.setText(0, str(received["id"]))
        new_item.setText(1, str(received["car"]))
        new_item.setText(2, str(received["pickupStop"]))
        new_item.setText(5, str(received["arrivalStop"]))
        self.ui.progress_list.addTopLevelItem(new_item) if received["state"] == "progress" else self.ui.requests_list.addTopLevelItem(new_item)

    def read_G(self, key):
        name = "robot_"+key[5]
        robot = self.g.get_node(name)
        task = self.g.get_node('task_'+key[5])
        bateria = self.g.get_node('battery_'+key[5])
        gps = self.g.get_node('gps_'+key[5])
        odometry = self.g.get_node('odometry_'+key[5])
        data = {
            "lat": gps.attrs["gps_latitude"].value if gps else 39.478987,
            "lng": gps.attrs["gps_longitude"].value if gps else -6.342290,
            # "speed": odometry.attrs['odometry_vel'].value if odometry else 10,
            "speed": None,
            "battery": int(bateria.attrs['battery_load'].value) if bateria else -1,
            "state": self.cars[key]["Estado"]
        }
        # nombre = "Melex1"
        try:
            data["speed"] = robot.attrs['robot_ref_adv_speed'].value*0.0036
        except:
            data["speed"] = -1
        return data

    def insert_pickup_coords(self, task_new, car):
        # if int(task_new['pickupStop']) < 1 or int(task_new['pickupStop']) > 4:
        print(self.paradas)
        if task_new['pickupStop'] not in self.paradas:
            # PROCESO PARA CONVERSIÓN LAT LONG TO UTM #################################
            print("LATITUD", task_new)
            u = utm.from_latlon(float(task_new['lat']), float(task_new['lng']))
            mapX, mapY = self.pt_transform.transform(u[0], u[1])
            mapX = mapX * 1000 - (self.fixedX * 1000)
            mapY = mapY * 1000 - (self.fixedY * 1000)
            # punto = np.array([[task_new['lat'], task_new['lng']]])
            punto = np.array([[mapX, mapY]])
            coordenadas = self.get_closest_point_to_user(punto)
            print("COORDENADAS", coordenadas)
            pickup_coords = coordenadas
        else:
            pickup_coords = self.paradas[task_new['pickupStop']]
        mind =self.g.get_node('mind_'+car)
        if mind != None:
            task = self.g.get_node('task_'+car)
            if task is None:
                task = Node(self.agent_id, "task", "task_"+car)
                task.attrs['task_assigned'] = Attribute(True, self.agent_id)
                task.attrs['task_car'] = Attribute(int(car), self.agent_id)
                task.attrs['task_pickup_values'] = Attribute(list(pickup_coords), self.agent_id)
                task.attrs['task_completed'] = Attribute(False, self.agent_id)
                task.attrs['task_movement'] = Attribute(True, self.agent_id)
                self.g.insert_node(task)
                task_edge = Edge(task.id, mind.id,  'has', self.agent_id)
                self.g.insert_or_assign_edge(task_edge)
                print("NODO TASK CREADOOOOOOOOOOO------------OOOOOOOOOOOOO-----------------------OOOOOOOOOOOOOOOOOOOOO")
            else:
                print("NODE TASK EXISTS")
        else:
            print("No node Mind")

    def insert_stop_coords(self, task_new):
        # print(task_new)
        arrival_coords = self.paradas[task_new['arrivalStop']]
        print("ROBOT NODE", str(task_new['car'][5]))
        if robot := self.g.get_node('robot_'+str(task_new['car'][5])):
            task_to_change = self.g.get_node('task_' + str(task_new['car'][5]))
            if task_to_change is not None:
                if not task_to_change.attrs['task_movement'].value and not task_to_change.attrs['task_completed'].value and robot.attrs['robot_occupied'].value:
                    task_to_change.attrs['task_destination_values'] = Attribute(list(arrival_coords), self.agent_id)
                    self.g.update_node(task_to_change)
                    self.cars["Melex"+str(task_new['car'][5])]["Estado"] = "goArrivalStop"
                elif not task_to_change.attrs['task_movement'].value and not task_to_change.attrs['task_completed'].value and not robot.attrs['robot_occupied'].value:
                    self.cars["Melex" + str(task_new['car'][5])]["Estado"] = "waitingUser"

            else:
                # task_data = {
                #     'state': "complete",
                #     'car': "Melex"+str(task_new['car'][5])
                # }
                # self.cars["Melex"+str(task_new['car'][5])]["Manager"].requests_put_json(task_data, task_new['id'])
                # self.cars["Melex"+str(task_new['car'][5])]["Manager"].put_requests_api()
                self.cars["Melex"+str(task_new['car'][5])]["Estado"] = "finishTravel"
                print("Task Node does not exist")
        else:
            print("Robot Node does not exist")

    def task_state(self, name = 'robot'): # TODO: Create JSON for set task state
        pass

    def task_manager(self, tasks):
        pass

    def set_waiting_state(self):
        for key in self.cars.keys():
            if robot := self.g.get_node('robot_' + str(key[5])):
                self.cars[key]["Estado"] = "waitingRequest" if (not robot.attrs['robot_occupied'].value and self.cars[key]["Estado"] == "finishTravel") else self.cars[key]["Estado"]
                # self.cars[key]["Estado"] = "waitingRequest" if (not robot.attrs['robot_occupied'].value == False and self.cars[key]["Estado"] == "finishTravel") else self.cars[key]["Estado"]
                    #self.cars[key]["Estado"] = "waitingRequest"

    def update_cars_info(self):
        for key in self.cars.keys():
            self.cars[key]["Manager"].create_ParametersCCAA_json(self.read_G(key))
            self.cars[key]["Manager"].put_ParametersCCAA_api()
            robot = self.g.get_node("robot_" + key[5])
            print("CAR STATUS", self.cars[key]["Manager"].get_cars_status(key))
            if key[5] == "1":
                self.ui.melex_state_1.setText(self.cars[key]["Manager"].get_cars_status(key)['state'])
            else:
                self.ui.melex_state_2.setText(self.cars[key]["Manager"].get_cars_status(key)['state'])
            robot.attrs["robot_occupied"].value = True if int(self.cars[key]["Manager"].get_cars_status(key)['busy']) == 1 else False
            self.g.update_node(robot)

    def get_server_tasks(self):
        undo_tasks = self.cars["Melex1"]["Manager"].get_requests_api()
        tasks_progress = self.cars["Melex1"]["Manager"].task_progress()

        return tasks_progress, undo_tasks

    ###################### FOR TESTING ######################

    def set_busy(self, id):
        data = self.read_G("Melex"+str(id))
        print(id, self.cars["Melex"+str(id)]["Manager"].get_cars_status("Melex"+str(id)))
        data["busy"] = 0.0 if int(self.cars["Melex"+str(id)]["Manager"].get_cars_status("Melex"+str(id))['busy']) == 1 else 0.0
        self.cars["Melex" + str(id)]["Manager"].create_ParametersCCAA_json(data)
        self.cars["Melex" + str(id)]["Manager"].put_ParametersCCAA_api()
    ###################### NODE CREATORS ######################
    def insert_intention_node(self, id):
        local_robot_mind_name = "mind_" + str(id)
        local_current_intention_name = "current_intention_" + str(id)
        mind = self.g.get_node(local_robot_mind_name)
        if mind != None:
            if self.g.get_node(local_current_intention_name) == None:
                intention = Node(self.agent_id, "intention", local_current_intention_name)
                print("NODE ID", mind.id)
                # intention.attrs['parent'] = Attribute(True, mind.id)
                # intention.attrs['level'] = Attribute(True, self.g.get_node_level(local_robot_mind_name).value + 1)
                intention.attrs['pos_x'] = Attribute(float(-290+int(id)*100),  self.agent_id)
                intention.attrs['pos_y'] = Attribute( float(-474+int(id)*100), self.agent_id)
                self.g.insert_node(intention)
                print("NODE local_current_path_name ------------------------------------------",
                self.g.get_node(local_current_intention_name))
                # time.sleep(1)
                if self.g.get_node(local_current_intention_name) != None:
                    edge = Edge(intention.id, mind.id, 'has', self.agent_id)
                    self.g.insert_or_assign_edge(edge)
                    print("EDGE intention.id, mind.id ------------------------------------------", self.g.get_edge(mind.id, intention.id, 'has'))
            else:
                print("Current intention node exists")
        else:
            print("No node Mind")

    def insert_path(self, id):
        local_robot_mind_name = "mind_" + str(id)
        local_current_intention_name = "current_intention_" + str(id)
        local_current_path_name = "current_path_" + str(id)
        path = self.g.get_node(local_current_path_name)
        if path != None:
            path.attrs["path_x_values"].value = self.x_path
            path.attrs["path_y_values"].value = self.y_path
            path.attrs["path_is_cyclic"].value = True
            self.g.update_node(path)
        else:
            intention = self.g.get_node(local_current_intention_name)
            if intention != None:
                path_to_target = Node(self.agent_id, "path_to_target", local_current_path_name)
                # print("X PATH type", self.x_path)
                path_to_target.attrs['path_x_values'] = Attribute(self.x_path, self.agent_id)
                path_to_target.attrs['path_y_values'] = Attribute(self.y_path, self.agent_id)
                path_to_target.attrs['path_is_cyclic'] = Attribute(True, self.agent_id)
                path_to_target.attrs['pos_x'] = Attribute(float(-542), self.agent_id)
                path_to_target.attrs['pos_y'] = Attribute(float(106), self.agent_id)
                path_to_target.attrs['parent'] = Attribute(intention.id, self.agent_id)
                path_to_target.attrs['level'] = Attribute(3, self.agent_id)
                self.g.insert_node(path_to_target)
                # time.sleep(1)
                print("NODE local_current_path_name ------------------------------------------", self.g.get_node(local_current_path_name))
                #time.sleep(1)
                if self.g.get_node(local_current_path_name) != None:
                    edge = Edge(path_to_target.id, intention.id,'thinks', self.agent_id)
                    self.g.insert_or_assign_edge(edge)
                    # print("INSERCION", self.g.insert_or_assign_edge(edge))
                    print("EDGE path_to_target.id, intention.id ------------------------------------------", self.g.get_edge(intention.id, path_to_target.id,'thinks'))

    def start_mission(self, id):
        self.insert_intention_node(id)
        self.insert_path(id)

    def stop_mission(self, id):
        mind = self.g.get_node("mind_" + str(id))
        task_node = self.g.get_node("task_" + str(id))
        if task_node != None:
            try:
                self.g.delete_edge(mind.id, task_node.id, 'has')
            except:
                print("HAS EDGE HAS BEEN DELETED")
            try:
                self.g.delete_node("task_" + str(id))
            except:
                print("HAS EDGE HAS BEEN DELETED")

        path_node = self.g.get_node("current_path_" + str(id))
        intention_node = self.g.get_node("current_intention_" + str(id))
        try:
            self.g.delete_edge(mind.id, intention_node.id, 'has')
        except:
            print("HAS EDGE HAS BEEN DELETED")
        try:
            self.g.delete_edge(intention_node.id, path_node.id, 'thinks')
        except:
            print("HAS EDGE HAS BEEN DELETED")
        try:
            self.g.delete_node("current_path_" + str(id))
        except:
            print("HAS EDGE HAS BEEN DELETED")
        try:
            self.g.delete_node("current_intention_"+ str(id))
        except:
            print("HAS EDGE HAS BEEN DELETED")

    def check_tasks_completed(self):
        task_nodes = self.g.get_nodes_by_type('task')
        if len(task_nodes) > 0:
            for node in task_nodes:
                if node.attrs['task_completed'].value:
                    self.g.delete_node(node.name)
                    print("----------------BORRADO NODO TAREA -------------")

    def remove_item_progress(self):
        task_data = {
            'state': "complete",
            'car': "Melex" + str(self.ui.progress_list.currentItem().text(1)[5])
        }
        self.cars["Melex" + str(self.ui.progress_list.currentItem().text(1)[5])]["Manager"].requests_put_json(task_data, int(self.ui.progress_list.currentItem().text(0)))
        self.cars["Melex" + str(self.ui.progress_list.currentItem().text(1)[5])]["Manager"].put_requests_api()
        print("BORRANDO PROGRESS", task_data)
        self.cars["Melex" + str(self.ui.progress_list.currentItem().text(1)[5])]["Estado"] = "waitingRequest"
        self.g.delete_node("task_"+ str(self.ui.progress_list.currentItem().text(1)[5]))
        self.ui.progress_list.currentItem().removeChild(self.ui.progress_list.currentItem())
    def remove_item_requests(self):
        task_data = {
            'state': "complete",
            'car': None
        }
        self.cars["Melex1"]["Manager"].requests_put_json(task_data, int(self.ui.progress_list.currentItem().text(0)))
        self.cars["Melex1"]["Manager"].delete_tasks()
        self.ui.requests_list.currentItem().removeChild(self.ui.requests_list.currentItem())

    def mousePressEvent(self, event):
        if event.pos().x() > 111 and event.pos().x() < 633 and event.pos().y() > 342 and event.pos().y() < 892:
            # x: 111 - 633 ; -20656 : 43217
            # y : 342 - 892 ; 25429 : -44547
            x_pos = -20656 + (event.pos().x()-111)*(63873/522)
            y_pos = 25429 - (event.pos().y() - 342) * (69976/550)
            punto = np.array([[x_pos, y_pos]])
            datapru = {
                "arrivalStop": "4",
                "car": "",
                "id": None,
                "lat": x_pos,
                "lng": y_pos,
                "pickupStop": "",
                "state": "recived",
                "timestamp": "2022-11-11T12:46:49",
                "username": "sergiobarrosoram"
            }
            self.cars["Melex1"]["Manager"].request_create_post_request(datapru)
            self.cars["Melex1"]["Manager"].post_request_api()
            data = {
                "lat": x_pos,
                "lng": y_pos,
                "name": datapru["username"],
                "num": 0
            }
            self.cars["Melex1"]["Manager"].create_stop_json(data)
            self.cars["Melex1"]["Manager"].stop_post()



    def get_closest_point_to_user(self, point):
        closest_point = self.norm_method(self.path_xy, point)
        return(self.path_xy[np.argmin(closest_point)])

    def norm_method(self, arr, point):
        dist = spatial.distance.cdist(arr, point)
        return dist
    # DSR SLOTS
    def update_node_att(self, id: int, attribute_names: [str]):
        # console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')
        pass

    def update_node(self, id: int, type: str):
        # console.print(f"UPDATE NODE: {id} {type}", style='green')
        pass

    def delete_node(self, id: int):
        # console.print(f"DELETE NODE:: {id} ", style='green')
        pass

    def update_edge(self, fr: int, to: int, type: str):
        # console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')
        pass

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        # console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')
        pass

    def delete_edge(self, fr: int, to: int, type: str):
        # console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
        pass
