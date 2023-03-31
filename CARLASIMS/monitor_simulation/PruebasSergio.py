import carla
import random
import logging
def carla_fun():
    client = carla.Client("192.168.1.12", 20000)
    client.set_timeout(10.0)
    client.load_world("Arriba2")
    print(client.get_required_files())
    world = client.get_world()
    world.get_spectator().set_transform(carla.Transform(carla.Location(0, 0, 382)))


    return [client, world]
walkers_list = []
all_id = []
car_sim = carla_fun()
world = car_sim[1]
client = car_sim[0]
blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
walker_bp = random.choice(blueprintsWalkers)
spawn_points = [carla.Transform(carla.Location(23.66, 141.74, 380)), carla.Transform(carla.Location(30.51, 122.86, 380)), carla.Transform(carla.Location(39.47, 94.57, 380)), carla.Transform(carla.Location(25.3, 92.29, 380)), carla.Transform(carla.Location(14.48, 129.72, 380)), carla.Transform(carla.Location(15.52, 141.78, 380))]
# for i in range(50):
#     spawn_point = carla.Transform()
#     spawn_point.location = world.get_random_location_from_navigation()
#     if (spawn_point.location != None):
#         spawn_points.append(spawn_point)

batch = []
for spawn_point in spawn_points:
    #print(spawn_point.location)
    #spawn_point.location = carla.Location(x= spawn_point.location.x, y= spawn_point.location.y, z= spawn_point.location.z+10)
    #print(spawn_point.location)
    walker_bp = random.choice(blueprintsWalkers)
    batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

# apply the batch
results = client.apply_batch_sync(batch, True)
print(results)
for i in range(len(results)):
    if results[i].error:
        logging.error(results[i].error)
    else:
        walkers_list.append({"id": results[i].actor_id})

batch = []
walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
for i in range(len(walkers_list)):
    batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))

# apply the batch
results = client.apply_batch_sync(batch, True)
for i in range(len(results)):
    if results[i].error:
        logging.error(results[i].error)
    else:
        walkers_list[i]["con"] = results[i].actor_id

for i in range(len(walkers_list)):
    all_id.append(walkers_list[i]["con"])
    all_id.append(walkers_list[i]["id"])
all_actors = world.get_actors(all_id)

world.wait_for_tick()
print(walkers_list)
j = True
for i in range(0, len(all_actors), 2):
        random_index = random.randrange(len(spawn_points))
    # start walker
        all_actors[i].start()
    # set walk to random point
    #print(all_actors[i].get_control())
        all_actors[i].go_to_location(spawn_points[random_index].location)
    # random max speed
        all_actors[i].set_max_speed(1 + random.random())
        actor_vel = world.get_actor(all_actors[i]).get_velocity()



# stop pedestrians (list is [controller, actor, controller, actor ...])
for i in range(0, len(all_id), 2):
    all_actors[i].stop()

# destroy pedestrian (actor and controller)
client.apply_batch([carla.command.DestroyActor(x) for x in all_id])