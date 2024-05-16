import random
import numpy as np
import carla

# from team_code.planner import RoutePlanner


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
        # self._command_planner = RoutePlanner(7.5, 25.0, 257)
        # self._waypoint_planner = RoutePlanner(4.0, 50)
        # self._plan_gps_HACK = global_plan_gps
        # self._waypoint_planner.set_route(self._plan_gps_HACK, True)
        self.weather_id = random.randint(0, 20)

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

    
    def _weather_to_dict(self, carla_weather):
            weather = {
                "cloudiness": carla_weather.cloudiness,
                "precipitation": carla_weather.precipitation,
                "precipitation_deposits": carla_weather.precipitation_deposits,
                "wind_intensity": carla_weather.wind_intensity,
                "sun_azimuth_angle": carla_weather.sun_azimuth_angle,
                "sun_altitude_angle": carla_weather.sun_altitude_angle,
                "fog_density": carla_weather.fog_density,
                "fog_distance": carla_weather.fog_distance,
                "wetness": carla_weather.wetness,
                "fog_falloff": carla_weather.fog_falloff,
            }

            return weather

    
    def tick(self, input_data):
        self._actors = self._world.get_actors()
        self._traffic_lights = get_nearby_lights(
            self._vehicle, self._actors.filter("*traffic_light*")
        )
        self._stop_signs = get_nearby_lights(
            self._vehicle, self._actors.filter("*stop*")
        )

        affordances = self._get_affordances()
        bb_3d = self._get_3d_bbs(max_distance=50)
        # gps = input_data["gps"][1][:2]
        gps_data = input_data["gps"][1]
        gps = [gps_data.latitude, gps_data.longitude]
        # speed = input_data["speed"][1]["speed"]
        speed = 0
        compass = input_data["imu"][1].compass
        weather = self._weather_to_dict(self._world.get_weather())

        return {
                "lidar": input_data["lidar"][1],
                "gps": gps,
                "speed": speed,
                "compass": compass,
                "weather": weather,
                "affordances": affordances,
                "3d_bbs": bb_3d,
            }


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
    

    def _get_position(self, tick_data):
        gps = tick_data["gps"]
        gps = (gps - self._command_planner.mean) * self._command_planner.scale

        return gps


    def _should_brake(self, command):
        actors = self._world.get_actors()

        vehicle = self._is_vehicle_hazard(actors.filter("*vehicle*"), command)
        lane_vehicle = self._is_lane_vehicle_hazard(actors.filter("*vehicle*"), command)
        junction_vehicle = self._is_junction_vehicle_hazard(
            actors.filter("*vehicle*"), command
        )
        light = self._is_light_red(actors.filter("*traffic_light*"))
        walker = self._is_walker_hazard(actors.filter("*walker*"))
        bike = self._is_bike_hazard(actors.filter("*vehicle*"))
        stop_sign = self._is_stop_sign_hazard(actors.filter("*stop*"))

        # record the reason for braking
        self.is_vehicle_present = [x.id for x in vehicle]
        self.is_lane_vehicle_present = [x.id for x in lane_vehicle]
        self.is_junction_vehicle_present = [x.id for x in junction_vehicle]
        self.is_pedestrian_present = [x.id for x in walker]
        self.is_bike_present = [x.id for x in bike]
        self.is_red_light_present = [x.id for x in light]
        self.is_stop_sign_present = [x.id for x in stop_sign]

        return any(
            len(x) > 0
            for x in [
                vehicle,
                lane_vehicle,
                junction_vehicle,
                bike,
                light,
                walker,
                stop_sign,
            ]
        )


    def get_measurement(self, input_data):

        tick_data = self.tick(input_data)
        # pos = self._get_position(tick_data)
        theta = tick_data["compass"]
        speed = tick_data["speed"]
        weather = tick_data["weather"]
        near_node, far_node = [0, 0], [0, 0]
        near_command = ''
        steer, throttle, brake, target_speed = None, None, None, None
        actors = self._world.get_actors()
        loc = self._vehicle.get_location()
        self._loc = [loc.x, loc.y]
        light = self._is_light_red(actors.filter("*traffic_light*"))
        walker = self._is_walker_hazard(actors.filter("*walker*"))
        bike = self._is_bike_hazard(actors.filter("*vehicle*"))
        stop_sign = self._is_stop_sign_hazard(actors.filter("*stop*"))

        # record the reason for braking
        self.is_pedestrian_present = [x.id for x in walker]
        self.is_bike_present = [x.id for x in bike]
        self.is_red_light_present = [x.id for x in light]
        self.is_stop_sign_present = [x.id for x in stop_sign]

        data = {
            # "gps_x": pos[0],
            # "gps_y": pos[1],
            "x": self._loc[0],
            "y": self._loc[1],
            "theta": theta,
            "speed": speed,
            # "target_speed": target_speed,
            # "x_command": far_node[0],
            # "y_command": far_node[1],
            # "command": near_command.value,
            # "gt_command": self._command_planner.route[0][1].value,
            # "steer": steer,
            # "throttle": throttle,
            # "brake": brake,
            "weather": weather,
            "weather_id": self.weather_id,
            # "near_node_x": near_node[0],
            # "near_node_y": near_node[1],
            # "far_node_x": far_node[0],
            # "far_node_y": far_node[1],
            # "is_junction": self.is_junction,
            # "is_vehicle_present": self.is_vehicle_present,
            "is_bike_present": self.is_bike_present,
            # "is_lane_vehicle_present": self.is_lane_vehicle_present,
            # "is_junction_vehicle_present": self.is_junction_vehicle_present,
            "is_pedestrian_present": self.is_pedestrian_present,
            "is_red_light_present": self.is_red_light_present,
            "is_stop_sign_present": self.is_stop_sign_present,
            # "should_slow": int(self.should_slow),
            # "should_brake": int(self.should_brake),
            # "future_waypoints": self._waypoint_planner.get_future_waypoints(50),
            # "affected_light_id": self.affected_light_id,
        }

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