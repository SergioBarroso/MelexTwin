import carla
import time
import sys

client = carla.Client("127.0.0.1", p)
client.set_timeout(10.0)
client.load_world("Arriba2")