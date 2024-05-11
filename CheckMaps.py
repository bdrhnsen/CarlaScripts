import carla
client = carla.Client('localhost', 2000)

world = client.get_world()

spawn_points = world.get_map().get_spawn_points()
print(spawn_points)
print(client.get_available_maps())