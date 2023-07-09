import glob
import os
import sys
import time
import numpy as np
import threading
from mayavi import mlab

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list = []

#write a function for lidar data which return lidar data which is visualisable

#Define a function for initializing lidar sensor



def carlaThreadingLoop(world):
    frame = 0
    while True:
        time.sleep(0.005)
        world.tick()
        frame += 1


try:
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    get_blueprint_of_world = world.get_blueprint_library()
    car_model = get_blueprint_of_world.filter('model3')[0]
    spawn_point = (world.get_map().get_spawn_points()[1])
    dropped_vehicle = world.spawn_actor(car_model, spawn_point)
    dropped_vehicle.set_autopilot()

    simulator_camera_location_rotation = carla.Transform(spawn_point.location, spawn_point.rotation)
    simulator_camera_location_rotation.location += spawn_point.get_forward_vector() * 30
    simulator_camera_location_rotation.rotation.yaw += 180
    simulator_camera_view = world.get_spectator()
    simulator_camera_view.set_transform(simulator_camera_location_rotation)
    actor_list.append(dropped_vehicle)

    lidar_sensor = generate_lidar_blueprint(get_blueprint_of_world)
    sensor_lidar_spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.8))
    sensor = world.spawn_actor(lidar_sensor, sensor_lidar_spawn_point, attach_to=dropped_vehicle)

    lidar_figure = mlab.figure(size=(960, 540), bgcolor=(0.05, 0.05, 0.05))
    visualise_lidar_using_mayavi = mlab.points3d(0, 0, 0, 0, mode='point', figure=lidar_figure)
    mlab.view(distance=25)
    lidar_point_cloud_buffer = {'pts': np.zeros((1, 3)), 'intensity': np.zeros(1)}


    def anim():
        i = 0
        while True:
            visualise_lidar_using_mayavi.mlab_source.reset(x=lidar_point_cloud_buffer['pts'][:, 0],
                                                           y=lidar_point_cloud_buffer['pts'][:, 1],
                                                           z=lidar_point_cloud_buffer['pts'][:, 2],
                                                           scalars=lidar_point_cloud_buffer['intensity'])
            mlab.savefig(f'output/{i}.png', figure=lidar_figure)
            time.sleep(0.1)
            i += 1


    #write a sensor.listen() method to return lidar data to lidar_data function

    loopThread = threading.Thread(target=carlaThreadingLoop, args=[world], daemon=True).start()
    anim()

    actor_list.append(sensor)

    time.sleep(1000)
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
