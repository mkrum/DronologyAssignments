import dronekit_sitl
import dronekit
import json
import argparse
import os
import threading
import time
import signal
import util
import logging

from magnets import force, distance, distance_v2

_LOG = logging.getLogger(__name__)
_LOG.setLevel(logging.INFO)

fh = logging.FileHandler('main.log', mode='w')
fh.setLevel(logging.INFO)
formatter = logging.Formatter('| %(levelname)6s | %(funcName)8s:%(lineno)2d | %(message)s |')
fh.setFormatter(formatter)
_LOG.addHandler(fh)


DO_CONT = False

# make sure you change this so that it's correct for your system 
ARDUPATH = os.path.join('/', 'home', 'mkrum', 'git', 'ardupilot')


def load_json(path2file):
    d = None
    try:
        with open(path2file) as f:
            d = json.load(f)
    except Exception as e:
        exit('Invalid path or malformed json file! ({})'.format(e))

    return d


def connect_vehicle(instance, home):
    home_ = tuple(home) + (0,)
    home_ = ','.join(map(str, home_))
    sitl_defaults = os.path.join(ARDUPATH, 'Tools', 'autotest', 'default_params', 'copter.parm')
    sitl_args = ['-I{}'.format(instance), '--home', home_, '--model', '+', '--defaults', sitl_defaults]
    sitl = dronekit_sitl.SITL(path=os.path.join(ARDUPATH, 'build', 'sitl', 'bin', 'arducopter'))
    sitl.launch(sitl_args, await_ready=True)

    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + instance * 10)
    conn_string = ':'.join([tcp, ip, port])

    vehicle = dronekit.connect(conn_string)
    vehicle.wait_ready(timeout=120)

    return vehicle, sitl

def get_vehicle_id(i):
    return 'drone{}'.format(i)

def state_out_work(dronology, vehicles):
    while DO_CONT:
        for i, v in enumerate(vehicles):
            state = util.StateMessage.from_vehicle(v, get_vehicle_id(i))
            state_str = str(state)
            _LOG.info(state_str)
            dronology.send(state_str)

        time.sleep(1.0)


class Waypoint(object):
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

def vehicle_navigation(*args):
    v_id, vehicles, waypoints = args
    vehicle = vehicles[v_id]
    other_vehicles = [ vehic for ind, vehic in enumerate(vehicles) if ind != v_id ]
    waypoints = [ Waypoint(*w) for w in waypoints ]
    util.arm_and_takeoff(waypoints[0].alt, vehicle)

    while waypoints:
        location = vehicle.location.global_relative_frame
	check_collisions(vehicles, 1)
        
        new_x, new_y, new_z = force(vehicle, waypoints[0], other_vehicles)
        vehicle.simple_goto(dronekit.LocationGlobalRelative(new_x, new_y, new_z))
        time.sleep(1.0)
        location = vehicle.location.global_relative_frame
        
	if distance_v2(location, waypoints[0]) <= 5:
            print('Drone {} arrived at waypoint'.format(v_id))
            waypoints.pop(0)

    print("Drone" + str(v_id) + " Reached Final Waypoint")

    #stop
    vehicle.simple_goto(dronekit.LocationGlobalRelative(location.lat, 
                                                        location.lon, 
                                                        location.alt),
                        groundspeed=20)
    exit()

def check_collisions(vehicles, dist=1):
    col = 0
    for ind, vehic in enumerate(vehicles):
	for other_ind, other_vehic in enumerate(vehicles):

	    if ind != other_ind:
		col_dist = distance_v2(vehic.location.global_relative_frame, other_vehic.location.global_relative_frame)
	        if( col_dist <= dist ):
		    print("Collision between " + str(ind) + " and " + str(other_ind) + " at "  + str(col_dist) + "m")
		    col+=1

    return col/2


def main(path_to_config, ardupath=None):
    if ardupath is not None:
        global ARDUPATH
        ARDUPATH = ardupath
    
    global DO_CONT
    DO_CONT = True

    config = load_json(path_to_config)
    dronology = util.Connection()
    dronology.start()

    # A list of sitl instances.
    sitls = []
    # A list of drones. (dronekit.Vehicle)
    vehicles = []
    # A list of lists of lists (i.e., [ [ [lat0, lon0, alt0], ...] ...]
    # These are the waypoints each drone must go to!
    routes = []

    # Example:
    # vehicle0 = vehicles[0]
    # waypoints_for_vehicle0 = routes[0]
    # for waypoint in waypoints_for_vehicle0:
    #    lat, lon, alt = waypoint
    #    vehicle0.simple_goto(lat, lon, alt)

    # The above example obviously won't work... you'll need to write some code to figure out when the current waypoint
    # has been reached and it's time to go to the next waypoint.

    # Define the shutdown behavior
    def stop(*args):
        global DO_CONT
        DO_CONT = False

        for v in v_threads:
            v.join()

        w0.join()

        for v, sitl in zip(vehicles, sitls):
            v.close()
            sitl.stop()

        dronology.stop()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    
    # Start up all the drones specified in the json configuration file
    for i, v_config in enumerate(config):
        home = v_config['start']
        vehicle, sitl = connect_vehicle(i, home)

        handshake = util.DroneHandshakeMessage.from_vehicle(vehicle, get_vehicle_id(i))
        dronology.send(str(handshake))

        sitls.append(sitl)
        vehicles.append(vehicle)
        routes.append(v_config['waypoints'])
        
    # Create a thread for sending the state of drones back to Dronology
    w0 = threading.Thread(target=state_out_work, args=(dronology, vehicles))
    # Start the thread.
    w0.start()

    # At this point, all of the "behind the scenes stuff" has been set up.
    # It's time to write some code that:
    #   1. Starts up the drones (set the mode to guided, arm, takeoff)
    #   2. Sends the drones to their waypoints
    #   3. Hopefully avoids collisions!

    
    # You're encouraged to restructure this code as necessary to fit your own design.
    # Hopefully it's flexible enough to support whatever ideas you have in mind.
   

    v_threads = []
    for v in range(len(vehicles)):
        v_threads.append(threading.Thread(target=vehicle_navigation, args=(v, vehicles, routes[v])))
        v_threads[-1].start()


    # wait until ctrl c to exit
    while DO_CONT:
        time.sleep(5.0)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('path_to_config', type=str, help='the path to the drone configuration file.')
    ap.add_argument('--ardupath', type=str, default=ARDUPATH)
    args = ap.parse_args()
    main(args.path_to_config, ardupath=args.ardupath)
