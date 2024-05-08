import os
import time
import datetime
import pathlib
import json
import random
import shapely
import math
from collections import deque
from itertools import chain

import numpy as np
import cv2
import carla
from carla_birdeye_view import BirdViewProducer, BirdViewCropType, PixelDimensions
from PIL import Image

import sys
sys.path.append('/home/zc/LMDrive/leaderboard')
from team_code.map_agent import MapAgent
from team_code.pid_controller import PIDController
from agents.navigation.local_planner import RoadOption
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


WEATHERS = {
    "ClearNoon": carla.WeatherParameters.ClearNoon,
    "ClearSunset": carla.WeatherParameters.ClearSunset,
    "CloudyNoon": carla.WeatherParameters.CloudyNoon,
    "CloudySunset": carla.WeatherParameters.CloudySunset,
    "WetNoon": carla.WeatherParameters.WetNoon,
    "WetSunset": carla.WeatherParameters.WetSunset,
    "MidRainyNoon": carla.WeatherParameters.MidRainyNoon,
    "MidRainSunset": carla.WeatherParameters.MidRainSunset,
    "WetCloudyNoon": carla.WeatherParameters.WetCloudyNoon,
    "WetCloudySunset": carla.WeatherParameters.WetCloudySunset,
    "HardRainNoon": carla.WeatherParameters.HardRainNoon,
    "HardRainSunset": carla.WeatherParameters.HardRainSunset,
    "SoftRainNoon": carla.WeatherParameters.SoftRainNoon,
    "SoftRainSunset": carla.WeatherParameters.SoftRainSunset,
    "ClearNight": carla.WeatherParameters(5.0,0.0,0.0,10.0,-1.0,-90.0,60.0,75.0,1.0,0.0),
    "CloudyNight": carla.WeatherParameters(60.0,0.0,0.0,10.0,-1.0,-90.0,60.0,0.75,0.1,0.0),
    "WetNight": carla.WeatherParameters(5.0,0.0,50.0,10.0,-1.0,-90.0,60.0,75.0,1.0,60.0),
    "WetCloudyNight": carla.WeatherParameters(60.0,0.0,50.0,10.0,-1.0,-90.0,60.0,0.75,0.1,60.0),
    "SoftRainNight": carla.WeatherParameters(60.0,30.0,50.0,30.0,-1.0,-90.0,60.0,0.75,0.1,60.0),
    "MidRainyNight": carla.WeatherParameters(80.0,60.0,60.0,60.0,-1.0,-90.0,60.0,0.75,0.1,80.0),
    "HardRainNight": carla.WeatherParameters(100.0,100.0,90.0,100.0,-1.0,-90.0,100.0,0.75,0.1,100.0),
}
WEATHERS_IDS = list(WEATHERS)


def get_entry_point():
    return "DataRecorder"


def _numpy(carla_vector, normalize=False):
    result = np.float32([carla_vector.x, carla_vector.y])

    if normalize:
        return result / (np.linalg.norm(result) + 1e-4)

    return result


def _location(x, y, z):
    return carla.Location(x=float(x), y=float(y), z=float(z))


def _orientation(yaw):
    return np.float32([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])


def get_collision(p1, v1, p2, v2):
    A = np.stack([v1, -v2], 1)
    b = p2 - p1

    if abs(np.linalg.det(A)) < 1e-3:
        return False, None

    x = np.linalg.solve(A, b)
    collides = all(x >= 0) and all(x <= 1)  # how many seconds until collision

    return collides, p1 + x[0] * v1


def check_episode_has_noise(lat_noise_percent, long_noise_percent):
    lat_noise = False
    long_noise = False
    if random.randint(0, 101) < lat_noise_percent:
        lat_noise = True

    if random.randint(0, 101) < long_noise_percent:
        long_noise = True

    return lat_noise, long_noise


class DataRecorder():
    
    def __init__(self, world, ego_vehicle) -> None:
        self.reserve = 0
        self._world = world
        self._vehicle = ego_vehicle
        self._affected_by_stop = (
            False  # if the ego vehicle is influenced by a stop sign
        )
        self._traffic_lights = list()

    def get_matrix(self, transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


    def _find_obstacle_3dbb(self, obstacle_type, max_distance=50):
        """Returns a list of 3d bounding boxes of type obstacle_type.
        If the object does have a bounding box, this is returned. Otherwise a bb
        of size 0.5,0.5,2 is returned at the origin of the object.

        Args:
            obstacle_type (String): Regular expression
            max_distance (int, optional): max search distance. Returns all bbs in this radius. Defaults to 50.

        Returns:
            List: List of Boundingboxes
        """
        obst = list()

        _actors = self._world.get_actors()
        _obstacles = _actors.filter(obstacle_type)

        for _obstacle in _obstacles:
            distance_to_car = _obstacle.get_transform().location.distance(
                self._vehicle.get_location()
            )

            if 0 < distance_to_car <= max_distance:

                if hasattr(_obstacle, "bounding_box"):
                    loc = _obstacle.bounding_box.location
                    _obstacle.get_transform().transform(loc)

                    extent = _obstacle.bounding_box.extent
                    _rotation_matrix = self.get_matrix(
                        carla.Transform(
                            carla.Location(0, 0, 0), _obstacle.get_transform().rotation
                        )
                    )

                    rotated_extent = np.squeeze(
                        np.array(
                            (
                                np.array([[extent.x, extent.y, extent.z, 1]])
                                @ _rotation_matrix
                            )[:3]
                        )
                    )

                    bb = np.array(
                        [
                            [loc.x, loc.y, loc.z],
                            [rotated_extent[0], rotated_extent[1], rotated_extent[2]],
                        ]
                    )

                else:
                    loc = _obstacle.get_transform().location
                    bb = np.array([[loc.x, loc.y, loc.z], [0.5, 0.5, 2]])

                obst.append(bb)

        return obst
    
    
    def _get_3d_bbs(self, max_distance=50):

        bounding_boxes = {
            "traffic_lights": [],
            "stop_signs": [],
            "vehicles": [],
            "pedestrians": [],
        }

        bounding_boxes["traffic_lights"] = self._find_obstacle_3dbb(
            "*traffic_light*", max_distance
        )
        bounding_boxes["stop_signs"] = self._find_obstacle_3dbb("*stop*", max_distance)
        bounding_boxes["vehicles"] = self._find_obstacle_3dbb("*vehicle*", max_distance)
        bounding_boxes["pedestrians"] = self._find_obstacle_3dbb(
            "*walker*", max_distance
        )

        return bounding_boxes
    

    def _translate_tl_state(self, state):

        if state == carla.TrafficLightState.Red:
            return 0
        elif state == carla.TrafficLightState.Yellow:
            return 1
        elif state == carla.TrafficLightState.Green:
            return 2
        elif state == carla.TrafficLightState.Off:
            return 3
        elif state == carla.TrafficLightState.Unknown:
            return 4
        else:
            return None

    
    def tick(self, input_data):
        self._actors = self._world.get_actors()
        self._traffic_lights = get_nearby_lights(
            self._vehicle, self._actors.filter("*traffic_light*")
        )
        self._stop_signs = get_nearby_lights(
            self._vehicle, self._actors.filter("*stop*")
        )

        # topdown = input_data["map"][1][:, :, 2]
        # topdown = draw_traffic_lights(topdown, self._vehicle, self._traffic_lights)
        # topdown = draw_stop_signs(topdown, self._vehicle, self._stop_signs)

        # result = super().tick(input_data)
        # result["topdown"] = topdown

        # return result


    def _get_affordances(self):

        # affordance tl
        affordances = {}
        affordances["traffic_light"] = None

        # update data
        self._actors = self._world.get_actors()
        self._traffic_lights = get_nearby_lights(
            self._vehicle, self._actors.filter("*traffic_light*")
        )
        self._stop_signs = get_nearby_lights(
            self._vehicle, self._actors.filter("*stop*")
        )

        affecting = self._vehicle.get_traffic_light()
        if affecting is not None:
            for light in self._traffic_lights:
                if light.id == affecting.id:
                    affordances["traffic_light"] = self._translate_tl_state(
                        self._vehicle.get_traffic_light_state()
                    )

        affordances["stop_sign"] = self._affected_by_stop

        return affordances
    
    
    def collect_actor_data(self):
        data = {}
        vehicles = self._world.get_actors().filter("*vehicle*")
        for actor in vehicles:
            loc = actor.get_location()
            if loc.distance(self._vehicle.get_location()) > 50:
                continue
            _id = actor.id
            data[_id] = {}
            data[_id]["loc"] = [loc.x, loc.y, loc.z]
            ori = actor.get_transform().rotation.get_forward_vector()
            data[_id]["ori"] = [ori.x, ori.y, ori.z]
            box = actor.bounding_box.extent
            data[_id]["box"] = [box.x, box.y]
            vel = actor.get_velocity()
            data[_id]["vel"] = [vel.x, vel.y, vel.z]
            data[_id]["tpe"] = 0

        walkers = self._world.get_actors().filter("*walker*")
        for actor in walkers:
            loc = actor.get_location()
            if loc.distance(self._vehicle.get_location()) > 50:
                continue
            _id = actor.id
            data[_id] = {}
            data[_id]["loc"] = [loc.x, loc.y, loc.z]
            ori = actor.get_transform().rotation.get_forward_vector()
            data[_id]["ori"] = [ori.x, ori.y, ori.z]
            box = actor.bounding_box.extent
            data[_id]["box"] = [box.x, box.y]
            vel = actor.get_velocity()
            data[_id]["vel"] = [vel.x, vel.y, vel.z]
            data[_id]["tpe"] = 1

        lights = self._world.get_actors().filter("*traffic_light*")
        for actor in lights:
            loc = actor.get_location()
            if loc.distance(self._vehicle.get_location()) > 70:
                continue
            _id = actor.id
            data[_id] = {}
            data[_id]["loc"] = [loc.x, loc.y, loc.z]
            ori = actor.get_transform().rotation.get_forward_vector()
            data[_id]["ori"] = [ori.x, ori.y, ori.z]
            vel = actor.get_velocity()
            data[_id]["sta"] = int(actor.state)
            data[_id]["tpe"] = 2

            trigger = actor.trigger_volume
            box = trigger.extent
            loc = trigger.location
            ori = trigger.rotation.get_forward_vector()
            data[_id]["taigger_loc"] = [loc.x, loc.y, loc.z]
            data[_id]["trigger_ori"] = [ori.x, ori.y, ori.z]
            data[_id]["trigger_box"] = [box.x, box.y]
        return data
    

def get_nearby_lights(vehicle, lights, pixels_per_meter=5.5, size=512, radius=5):
    result = list()

    transform = vehicle.get_transform()
    pos = transform.location
    theta = np.radians(90 + transform.rotation.yaw)
    R = np.array(
        [
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ]
    )

    for light in lights:
        delta = light.get_transform().location - pos

        target = R.T.dot([delta.x, delta.y])
        target *= pixels_per_meter
        target += size // 2

        if min(target) < 0 or max(target) >= size:
            continue

        trigger = light.trigger_volume
        light.get_transform().transform(trigger.location)
        dist = trigger.location.distance(vehicle.get_location())
        a = np.sqrt(
            trigger.extent.x**2 + trigger.extent.y**2 + trigger.extent.z**2
        )
        b = np.sqrt(
            vehicle.bounding_box.extent.x**2
            + vehicle.bounding_box.extent.y**2
            + vehicle.bounding_box.extent.z**2
        )

        if dist > a + b:
            continue

        result.append(light)

    return result