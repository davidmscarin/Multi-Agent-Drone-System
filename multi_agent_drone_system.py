import getpass
import asyncio
import spade
from spade import wait_until_finished
from spade.agent import Agent
#from spade.template import Templateimport datetime
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour, OneShotBehaviour
from spade.message import Message
from spade.template import Template
import numpy as np
import random
import time
import itertools
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import warnings
warnings.filterwarnings("ignore")
import os
import Interface
import wireframe
import pygame

MAX_X = 10
MAX_Y = 10
MAX_Z = 10
MAX_WEIGHT = 20
MAX_DRONES = 15
PACKAGES_TO_GENERATE = 10

#class to represent a package
class Package():
    def __init__(self, id, hub_id, x, y, z, weight) -> None:
        self.id = id
        self.hub_id = hub_id
        self.x = x
        self.y = y
        self.z = z
        self.weight = weight
        self.position = [0,0,0]
        self.pos = np.array([x, y, z])
        self.start_time = time.time()
        self.start_help = 0

#implementing the environment
class Environment():
    def __init__(self) -> None:
        #information recorded in the environment, variable names are self-explanatory
        self.package_count = 1
        self.drone_positions = {}
        self.drone_delivery = {}
        self.drone_cost = {}
        self.drone_help_cost = {}
        self.drone_next_pos = {}
        self.hub_positions = {}
        self.battery_status = {}
        self.carry_capacity = {}
        self.drone_speed = {}
        self.hub_packages = {}
        self.drone_destination = {}
        self.drone_list = ['drone1@localhost', 'drone2@localhost', 'drone3@localhost']
        self.hub_list = []
        self.drone_weight = {}
        self.drone_predicted_distance = {}
        self.drone_to_collect = {}
        self.drone_state = {} #0 = waiting for instructions, 1 = already coming to hub,  2 = delivering packages
        self.dropped_packages = []
        self.time_to_deliver = 0
        self.help = [0,0]
        self.drone_package_count = {'drone1@localhost' : [0,0], 'drone2@localhost' : [0,0], 'drone3@localhost' : [0,0]}
        self.drone_distance = {'drone1@localhost' : 0, 'drone2@localhost' : 0, 'drone3@localhost' : 0}
        #matrices for matplotlib 3d plotting
        self.x_matrix_drone = []
        self.y_matrix_drone = []
        self.z_matrix_drone = []
        self.x_matrix_hub = []
        self.y_matrix_hub = []
        self.z_matrix_hub = []
        self.x_matrix_dropped_package = []
        self.y_matrix_dropped_package = []
        self.z_matrix_dropped_package = []
        self.delivery_count = 0
        self.pylist = []


    #methods the agents can call to alter the environment information
    def update_drone_weight(self,drone_id,  weight):
        self.drone_weight[drone_id] += weight
        print(f"updated {drone_id} weight to {self.drone_weight[drone_id]}")

    def update_drone_position(self, drone_id, position):
        self.drone_positions[drone_id] = position
        print(f"updated {drone_id} position to {self.drone_positions[drone_id]}")
    
    def update_drone_destination(self, drone_id, position):
        self.drone_destination[drone_id] = position
        print(f"Updated {drone_id} destination to {position}")

    def update_battery(self, drone_id, new_battery):
        self.battery_status[drone_id] = min(100,self.battery_status[drone_id]+new_battery)
        print(f"Updated {drone_id} battery status to {self.battery_status[drone_id]}")
    
    def update_carry_capacity(self, drone_id, carry_capacity):
        self.carry_capacity[drone_id] = carry_capacity
        print(f"Updated {drone_id} carry_capacity to {self.carry_capacity[drone_id]}")

    def update_drone_delivery(self, drone_id, package):
        self.drone_delivery[drone_id].append(package)
        print(f"Drone {drone_id} is carrying deliveries:")
        for package in self.drone_delivery[drone_id]:
            print(package.id)

    def get_drone_position(self, drone_id):
        return self.drone_positions[drone_id]

    def set_hub_position(self, hub_id, pos):
        self.hub_positions[hub_id] = pos

    def get_hub_positions(self):
        return self.hub_positions

    def get_packages(self, hub_id):
        return self.hub_packages[hub_id]

    def spawn_package(self, hub_id):
        new_package = Package(self.package_count, hub_id, random.randint(0, MAX_X), random.randint(0, MAX_Y), random.randint(0, MAX_Z), random.randint(5,MAX_WEIGHT))
        self.hub_packages[hub_id].append(new_package)                
        print(f"New package at Hub {hub_id}. Deliver at {new_package.pos} with id {new_package.id}")
        self.package_count+=1  
        return new_package

    #display 3D plot using matplotlib
    def interface_display(self):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(projection='3d')

        x_drone = np.array(self.x_matrix_drone)
        y_drone = np.array(self.y_matrix_drone)
        z_drone = np.array(self.z_matrix_drone)

        x_hub = np.array(self.x_matrix_hub)
        y_hub = np.array(self.y_matrix_hub)
        z_hub = np.array(self.z_matrix_hub)

        points, = ax.plot([], [], [], '1')
        points2, = ax.plot([], [], [], 'v')

        for i in range(3):
            ax.plot(x_hub[i], y_hub[i], z_hub[i], 'o')

        def update_points(n):
            points.set_data(np.array([x_drone[n, :], y_drone[n, :]]))
            points.set_3d_properties(z_drone[n, :], 'z')
            return points,

        ax.set_xlim([-2, 12])
        ax.set_ylim([-2, 12])
        ax.set_zlim([0, 12])
        ani=animation.FuncAnimation(fig, update_points, len(x_drone), interval=60, blit=True, repeat=True)
        plt.show()

    
#initializing the Drone Agent
class DroneAgent(Agent):
    def __init__(self, jid: str, password: str, environment, x, y, z, speed, carry_capacity, verify_security: bool = False):
        #add to the environment all the information about this drone
        super().__init__(jid, password, verify_security)
        self.environment = environment
        self.environment.drone_speed[str(self.jid)] = speed
        self.environment.carry_capacity[str(self.jid)] = carry_capacity
        self.environment.drone_delivery[str(self.jid)] = []
        self.environment.drone_positions[str(self.jid)] = np.array([x, y, z])
        self.environment.battery_status [str(self.jid)] = 100
        self.environment.drone_weight[str(self.jid)] = 0
        self.environment.drone_predicted_distance[str(self.jid)] = 0
        self.environment.drone_to_collect[str(self.jid)] = []
        self.environment.drone_state[str(self.jid)] = 0

    #behaviour to listen to messages sent by either the hub or other drones
    class ListenToMessages(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=90)
            if msg:
                print(f"{str(self.agent.jid)} drone: message received from {str(msg.sender)} with content: {format(msg.body)}")
                if msg.body[:3] == 'New':
                    receiver = 'hub'
                    package_id = int(msg.body.split(' ')[-1])
                    receiver_id = str(msg.sender)
                    await self.send_response(receiver, receiver_id, package_id)

                elif msg.body[:3]=='Ass':
                    package_id = int(msg.body.split(' ')[-1])
                    self.agent.environment.drone_package_count[str(self.agent.jid)][0] += 1
                    for package in self.agent.environment.hub_packages[str(msg.sender)]:
                        if package.id == package_id:
                            self.agent.environment.drone_to_collect[str(self.agent.jid)].append(package)
                            self.agent.environment.drone_weight[str(self.agent.jid)] += package.weight
                            self.agent.environment.drone_predicted_distance[str(self.agent.jid)] += 2 * sum(abs(self.agent.environment.hub_positions[str(msg.sender)]-package.pos))
                            if self.agent.environment.drone_state[str(self.agent.jid)] == 0:
                                self.agent.environment.drone_state[str(self.agent.jid)] = 1
                                await self.agent.move_drone(self.agent.environment.hub_positions[str(msg.sender)],str(msg.sender))
                            break
                
                #conditions for deciding if the message was sent by another drone or by the hub and understanding what the message contains
                elif 'dropped' in msg.body:
                    receiver = 'drone'
                    package_id = int(msg.body.split(' ')[-1])
                    receiver_id = str(msg.sender)
                    await self.send_response(receiver, receiver_id, package_id, 0)

                elif 'stopped' in msg.body:
                    receiver = 'drone'
                    receiver_id = str(msg.sender)
                    await self.send_response(receiver, receiver_id, 1, 1)
                
                elif 'cost' in msg.body:
                    help_type = int(msg.body.split(' ')[1])
                    cost = int(msg.body.split(' ')[-1])
                    receiver_id = str(msg.sender)
                    self.agent.environment.drone_help_cost[receiver_id] = cost
                    if len(self.agent.environment.drone_help_cost)==len(self.agent.environment.drone_list)-1:
                        await self.assign_help(help_type)
                        self.agent.environment.drone_help_cost = {}
                
                elif 'helping' in msg.body:
                    help_type = int(msg.body.split(' ')[-1])
                    package = self.agent.environment.dropped_packages.pop(-1)
                    package.start_help = time.time()
                    await self.help_behaviour(help_type, package)

            else:
                print("No message received")
                self.kill()

        async def on_end(self):
            await self.agent.stop()
        
        #send response after having received and obtained information from message
        async def send_response(self, receiver, receiver_id, package_id, help_type = 0):
            print(f"{str(self.agent.jid)} sending response")
            #different response types depending on whether the receiver is a hub or another drones
            if receiver=='hub':
                msg = Message(to = str(receiver_id),sender=str(self.agent.jid))
                msg.set_metadata("performative", "inform") 
                cost = await self.calculate_move_cost(receiver_id, package_id)
                msg.body = f"{str(self.agent.jid)} drone cost is {cost}"
                print(f"successfully sent message to {receiver_id}")
                await self.send(msg)

            elif receiver=='drone':
                msg = Message(to = str(receiver_id),sender=str(self.agent.jid))
                msg.set_metadata("performative", "inform")
                if help_type == 0: 
                    cost = await self.calculate_move_cost(receiver_id, package_id, 0, True)
                    msg.body = f"Help {help_type} {str(self.agent.jid)} drone cost is {cost}"
                    print(f"successfully sent message to {receiver_id}")
                    await self.send(msg)
                
                if help_type == 1:
                    cost = await self.calculate_move_cost(receiver_id, package_id, 1, True) 
                    msg.body = f"Help {help_type} {str(self.agent.jid)} drone cost is {cost}"
                    print(f"successfully sent message to {receiver_id}")
                    await self.send(msg)
        
        #tell another drone to help it
        async def assign_help(self, help_type):
            drone_id = min(self.agent.environment.drone_help_cost, key = self.agent.environment.drone_help_cost.get)
            min_cost = self.agent.environment.drone_help_cost[drone_id]
            if min_cost >= 1000:
                n = len(self.agent.environment.drone_list)+1
                if n <= MAX_DRONES:
                    jid = "drone" + str(n) + "@localhost"
                    self.agent.environment.drone_list.append(jid)
                    self.agent.environment.drone_package_count[jid] = [0,0]
                    self.agent.environment.drone_distance[jid] = 0
                    drone_id = jid
            msg = Message(to = str(drone_id), sender= str(self.agent.jid))
            msg.set_metadata("performative", "inform")
            msg.body = f"{drone_id} is helping {str(self.agent.jid)} with problem {help_type}"
            print(f"drone {drone_id} is helping")
            await self.send(msg)
        

        async def help_behaviour(self, help_type, package = 0):
            if help_type==0:
                self.agent.environment.drone_state[str(self.agent.jid)] = 2
                await self.agent.move_drone(package.position, ["help",package])
            if help_type==1:
                #mover drone ate ao sitio e sleep um bocado e continuar o que estava a fazer antes
                pass
        
        #cost function used to estimate the cost of help or picking up a package
        async def calculate_move_cost(self, receiver_id, package_id, help_type = 0, help = False):
            #different cost for help (aka dropped a package) or picking up a package from a hub
            if help==False:
                for package in self.agent.environment.hub_packages[receiver_id]:
                    if package.id == package_id:
                        delivery_position = package.pos
                        delivery_weight = package.weight
                dist_hub_drone = sum(abs(self.agent.environment.drone_positions[str(self.agent.jid)]-self.agent.environment.hub_positions[receiver_id]))
                dist_hub_delivery = sum(abs(self.agent.environment.hub_positions[receiver_id] - delivery_position))
                cost = (dist_hub_drone+dist_hub_delivery)//self.agent.environment.drone_speed[str(self.agent.jid)]
                battery_needed = dist_hub_delivery*2
                current_weight = self.agent.environment.drone_weight[str(self.agent.jid)]
                #cost is 1000 if its "impossible" for the drone to do the proposed job
                #checks if the drones has enough battery to do the trip and enough carry capacity to pick up the next delivery
                if self.agent.environment.drone_state[str(self.agent.jid)] == 2:
                    cost += 500
                for pack in self.agent.environment.drone_to_collect[str(self.agent.jid)]:
                    if pack.hub_id != receiver_id:
                        cost = 1000
                if current_weight+delivery_weight > self.agent.environment.carry_capacity[str(self.agent.jid)]:
                    cost = 1000
                if battery_needed + self.agent.environment.drone_predicted_distance[str(self.agent.jid)] > 100:
                    cost = 1000
                return cost

            elif help==True:

                if help_type == 0:
                    for pack in self.agent.environment.dropped_packages:
                        if pack.id == package_id:
                            package = pack
                    dist_drone_to_drone = sum(abs(self.agent.environment.drone_positions[str(self.agent.jid)] - package.position))
                    dist_drone_to_package = sum(abs(package.pos - package.position))
                    max_pack_hub = 0
                    for hub in list(self.agent.environment.hub_positions.keys()):
                        dist_pack_hub = sum(abs(package.position-self.agent.environment.hub_positions[hub]))
                        if dist_pack_hub>max_pack_hub:
                            max_pack_hub = dist_pack_hub
                    dist_package_hub = max_pack_hub
                    cost = (dist_drone_to_drone+dist_drone_to_package)//self.agent.environment.drone_speed[receiver_id]
                    battery_needed = dist_drone_to_drone+dist_drone_to_package+dist_package_hub
                    if self.agent.environment.drone_state[str(self.agent.jid)] != 0:
                        cost = 1000
                    if package.weight > self.agent.environment.carry_capacity[str(self.agent.jid)]:
                        cost = 1000
                    if battery_needed > self.agent.environment.battery_status[str(self.agent.jid)]:
                        cost = 1000
                    return cost


                elif help_type==1:
                    dist = sum(abs(self.agent.environment.drone_positions[str(self.agent.jid)] - self.agent.environment.drone_positions[receiver_id]))
                    dist_hub_drone = sum(abs(self.agent.environment.drone_positions[str(self.agent.jid)]-self.agent.environment.hub_positions[receiver_id]))
                    cost = dist+dist_hub_drone//self.agent.environment.drone_speed[str(self.agent.jid)]
                    battery_needed = dist + dist_hub_drone
                    if self.agent.environment.drone_state[str(self.agent.jid)] == 2:
                            cost += 500
                    if current_weight+delivery_weight > self.agent.environment.carry_capacity[str(self.agent.jid)]:
                        cost = 1000
                    if battery_needed + self.agent.environment.drone_predicted_distance[str(self.agent.jid)] > 100:
                        cost = 1000
                    return cost

    #behaviour to simulate random events (dropping a package)
    class RandomEvent(OneShotBehaviour):
        async def run(self):
            random_event = 0
            await self.send_distress_signal(random_event)
            if random_event == 0 and len(self.agent.environment.drone_delivery[str(self.agent.jid)])>1:
                self.agent.environment.drone_delivery[str(self.agent.jid)].pop(-1)
        
        async def send_distress_signal(self, random_event):
            if random_event==0 and len(self.agent.environment.drone_delivery[str(self.agent.jid)])>1:
                dropped_package_id = self.agent.environment.drone_delivery[str(self.agent.jid)][-1].id
                dropped_package_pos = self.agent.environment.drone_positions[str(self.agent.jid)]
                package = self.agent.environment.drone_delivery[str(self.agent.jid)][-1]
                position = self.agent.environment.drone_positions[str(self.agent.jid)].copy()
                position[2] = 0
                package.position = position
                self.agent.environment.dropped_packages.append(package)
                for drone_id in self.agent.environment.drone_list:
                    if drone_id != str(self.agent.jid):
                        msg = Message(to = str(drone_id),sender=str(self.agent.jid))
                        msg.set_metadata("performative", "inform")
                        msg.body = f"Drone {str(self.agent.jid)} dropped package at {dropped_package_pos} and it fell to {position} with id {dropped_package_id}"
                        print(f"successfully sent distress signal to {drone_id}")
                        await self.send(msg)
                

    #calculate the best route for movement
    def best_route(self, hub):
        perm = list(itertools.permutations(self.environment.drone_delivery[str(self.jid)]))
        min_dist = 10000
        #using the distance between the current point and the destination
        for route in perm:
            dist = self.soma_dist(route) + sum(abs(route[0].pos-self.environment.hub_positions[hub]))
            if dist<min_dist:
                min_dist = dist
                best_route = route
        return list(best_route)
    
    def soma_dist(self,route):
        dist = 0
        for dest in range(len(route)-1):
            dist += sum(abs(route[dest].pos-route[dest+1].pos))
        return dist


    async def setup(self):
        print("Drone Agent Started")
        self.add_behaviour(self.ListenToMessages())

    
    #trying possibilities for the next possible move
    def next_attempt(self,arround,attempt, move):
        for i in range(-1,2,2):
            if arround[attempt[0],attempt[1],i] == 0 and move[2]+i >=0:
                return [attempt[0],attempt[1],i]
        for i in range(-1,2,2):
            if arround[attempt[0],i,attempt[2]] == 0:
                return [attempt[0],i,attempt[2]]
        for i in range(-1,2,2):
            if arround[i,attempt[1],attempt[2]] == 0:
                return [i,attempt[1],attempt[2]]
        for i in range(-1,2,2):
            for j in range(-1,2,2):
                if arround[attempt[0],i,j] and move[2]+j >=0:
                    return [attempt[0],i,j]
        for i in range(-1,2,2):
            for j in range(-1,2,2):
                if arround[i,attempt[1],j] and move[2]+j >=0:
                    return [i,attempt[1],j]
        for i in range(-1,2,2):
            for j in range(-1,2,2):
                if arround[i,j,attempt[2]]:
                    return [i,j,attempt[2]]
        for i in range(-1,2,2):
            for j in range(-1,2,2):
                for k in range(-1,2,2):
                    if arround[i,j,k] == 0 and move[2]+k >=0:
                        return [i,j,k]
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if arround[i,j,k] == 0 and move[2]+k >=0:
                        return [i,j,k]

    #function to move the drone
    async def move_drone(self, destination, hub=[0]):
        class MoveDrone(PeriodicBehaviour):
            #the drones periodically moves towards its assigned destination acourding to the best route computed
            async def run(self):
                actual = self.agent.environment.drone_positions[str(self.agent.jid)]
                if np.array_equal(actual,destination):
                    print(f"{str(self.agent.jid)} has arrived at {destination}")
                    self.kill()
                else:
                    rand = random.randint(1,100)
                    if rand > 80:
                        if len(self.agent.environment.drone_delivery[str(self.agent.jid)]) > 1:
                            self.agent.add_behaviour(self.agent.RandomEvent())
                    to_move = [0,0,0]
                    if actual[0]!=destination[0]:
                        if destination[0]-actual[0]>0:
                            to_move[0]=1
                        else:
                            to_move[0]=-1
                    if actual[1]!=destination[1]:
                        if destination[1]-actual[1]>0:
                            to_move[1]=1
                        else:
                            to_move[1]=-1
                    if actual[2]!=destination[2]:
                        if destination[2]-actual[2]>0:
                            to_move[2]=1
                        else:
                            to_move[2]=-1
                    #computing the drones position in the next iteration
                    new_pos = actual+to_move
                    self.agent.environment.drone_next_pos[str(self.agent.jid)] = new_pos
                    attempt = to_move
                    arround = np.zeros((3,3,3), dtype=int)
                    can_crash = False
                    for hub_id in self.agent.environment.hub_list:
                        if np.array_equal(self.agent.environment.hub_positions[hub_id],new_pos):
                            can_crash = True
                    #logic to avoid drone crashes
                    for drone_id in self.agent.environment.drone_list:
                        if drone_id != str(self.agent.jid):
                            if not(can_crash):
                                if np.array_equal(self.agent.environment.drone_positions[drone_id],new_pos):
                                    print(f"{str(self.agent.jid)} was goint to crash into {drone_id} by moving to {new_pos}")
                                if np.array_equal(self.agent.environment.drone_positions[drone_id],actual+attempt):
                                    arround[attempt[0],attempt[1],attempt[2]] = 1
                                    attempt = self.agent.next_attempt(arround,to_move, actual)
                    self.agent.environment.update_drone_position(str(self.agent.jid), actual+attempt)
                    waste = abs(attempt[0]) + abs(attempt[1]) + abs(attempt[2])
                    self.agent.environment.drone_distance[str(self.agent.jid)] += waste
                    self.agent.environment.update_battery(str(self.agent.jid), -waste)
            
            async def on_end(self):
                if(self.agent.environment.drone_state[str(self.agent.jid)]==1):
                    self.agent.charge(hub)
                elif (self.agent.environment.drone_state[str(self.agent.jid)]==2):
                    if hub[0] == "help":
                        package = hub[1]
                        self.agent.environment.help[0] += round(time.time()-package.start_help,2)
                        self.agent.environment.help[1] += 1    
                        self.agent.environment.drone_delivery[str(self.agent.jid)].append(package)
                        print(f"{str(self.agent.jid)} has picked up the dropped package")
                        await self.agent.move_drone(self.agent.environment.drone_delivery[str(self.agent.jid)][0].pos)
                    else:
                        if self.agent.environment.drone_delivery[str(self.agent.jid)][0].start_help != 0:
                            self.agent.environment.help[0] += round(time.time()-self.agent.environment.drone_delivery[str(self.agent.jid)][0].start_help,2)
                            self.agent.environment.help[1] += 1
                        time_taken = round(time.time()-self.agent.environment.drone_delivery[str(self.agent.jid)][0].start_time,2)
                        self.agent.environment.delivery_count += 1
                        self.agent.environment.time_to_deliver += time_taken
                        print(f"{str(self.agent.jid)} delivering package with id {self.agent.environment.drone_delivery[str(self.agent.jid)][0].id} at {self.agent.environment.drone_delivery[str(self.agent.jid)][0].pos} and took {time_taken} seconds")
                        self.agent.environment.drone_package_count[str(self.agent.jid)][1] += 1
                        self.agent.environment.drone_delivery[str(self.agent.jid)].pop(0)
                        if len(self.agent.environment.drone_delivery[str(self.agent.jid)]) == 0:
                            print(f"{str(self.agent.jid)} has nothing else to deliver")
                            if len(self.agent.environment.drone_to_collect[str(self.agent.jid)])==0:
                                self.agent.environment.drone_state[str(self.agent.jid)] = 0
                            else:
                                self.agent.environment.drone_state[str(self.agent.jid)] = 1
                                hub_id = self.agent.environment.drone_to_collect[str(self.agent.jid)][0].hub_id
                                await self.agent.move_drone(self.agent.environment.hub_positions[hub_id],hub_id)
                                print(f"{str(self.agent.jid)} heading back to {self.agent.environment.drone_to_collect[str(self.agent.jid)][0].hub_id}")
                        
                        else:
                            await self.agent.move_drone(self.agent.environment.drone_delivery[str(self.agent.jid)][0].pos)
                
        #the time the drone takes to move is computed according to its velocity
        self.add_behaviour(MoveDrone(period=1/(self.environment.drone_speed[str(self.jid)])))
    

    #function to recover packages
    def get_packages(self, hub):
                temp_pack = []
                #finding its assigned package since the hub can have multiple packages available from pickup
                for package in self.environment.hub_packages[hub]:
                    found = False
                    for package_pick in self.environment.drone_to_collect[str(self.jid)]:
                        if package.id == package_pick.id:
                            self.environment.drone_delivery[str(self.jid)].append(package)
                            found = True
                            break
                    if not(found):
                        temp_pack.append(package)
                self.environment.hub_packages[hub] = temp_pack
                self.environment.drone_to_collect[str(self.jid)] = []
                #alter the state of the drone to represent it has a delivery to make
                self.environment.drone_state[str(self.jid)] = 2
                print(f"{str(self.jid)} has picked up {len(self.environment.drone_delivery[str(self.jid)])} packages")
    


    #function to charge the drone
    def charge(self, hub):
        class ChargeDrone(PeriodicBehaviour):
            async def run(self):
                #drones charge at hubs when picking up a delivery until their battery is fully charged
                if self.agent.environment.battery_status[str(self.agent.jid)] >= 100:
                    self.agent.get_packages(hub)
                    best = self.agent.best_route(hub)
                    self.agent.environment.drone_delivery[str(self.agent.jid)] = best
                    self.agent.environment.drone_weight[str(self.agent.jid)] = 0
                    self.agent.environment.drone_predicted_distance[str(self.agent.jid)] = 0
                    await self.agent.move_drone(self.agent.environment.drone_delivery[str(self.agent.jid)][0].pos)
                    self.kill()
                else:
                    for i in self.agent.environment.hub_positions.keys():
                        if np.array_equal(self.agent.environment.drone_positions[str(self.agent.jid)],self.agent.environment.hub_positions[i]):
                            self.agent.environment.update_battery(str(self.agent.jid), 5)
        
        self.add_behaviour(ChargeDrone(period=0.2))
            
        
    
#initializing Hub Agent
class HubAgent(Agent):
    def __init__(self, jid: str, password: str, environment, x, y, z, verify_security: bool = False):
        super().__init__(jid, password, verify_security)
        #on setup adds to environment information about the new hub
        self.environment = environment
        self.environment.hub_list.append(str(self.jid))
        self.environment.hub_positions[str(self.jid)] = np.array([x, y ,z])
        self.environment.drone_cost[str(self.jid)] = {}
    
    async def setup(self):
        print("Hub Agent Started")
        self.environment.hub_packages[str(self.jid)] = []
        self.add_behaviour(self.ListenToDrone())
    

    #behaviour to listen for messages
    class ListenToDrone(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=90)
            if msg:
                print(f"{str(self.agent.jid)} hub: message received with content: {format(msg.body)}")
                cost = int(msg.body.split(' ')[-1])
                self.agent.environment.drone_cost[str(self.agent.jid)][str(msg.sender)] = cost 
                if len(self.agent.environment.drone_cost[str(self.agent.jid)])==len(self.agent.environment.drone_list):
                    await self.assign_package()
                    self.agent.environment.drone_cost[str(self.agent.jid)] = {}
            else:
                print("No message received")
                self.kill()
        
        async def on_end(self):
            await self.agent.stop()
        
        #having received the cost estimation from each drone uses this information to call for the drone with the lowest cost
        async def assign_package(self):
            drone_id = min(self.agent.environment.drone_cost[str(self.agent.jid)], key = self.agent.environment.drone_cost[str(self.agent.jid)].get)
            min_cost = self.agent.environment.drone_cost[str(self.agent.jid)][drone_id]
            if min_cost >= 1000:
                n = len(self.agent.environment.drone_list)+1
                if n <= MAX_DRONES:
                    jid = "drone" + str(n) + "@localhost"
                    self.agent.environment.drone_list.append(jid)
                    self.agent.environment.drone_package_count[jid] = [0,0]
                    self.agent.environment.drone_distance[jid] = 0
                    drone_id = jid
            #sending message to the selected drone informing them they can come pick up the package
            msg = Message(to = str(drone_id), sender= str(self.agent.jid))
            msg.set_metadata("performative", "inform")
            msg.body = f"Assigned package to {drone_id} with package id {self.agent.environment.hub_packages[str(self.agent.jid)][-1].id}"
            print(f"successfully sent message to {drone_id}")
            await self.send(msg)
    
    #periodically simulating the arrival of a new package with a random destination and weigth
    async def spawn_package(self, wait_time = 0, periodic = 5):
        class SpawnRandomPackage(PeriodicBehaviour):
            async def run(self):
                print("Attempting spawn package behaviour")
                new_package = self.spawn_package()
                print(f"Packages in {str(self.agent.jid)}:")
                print(self.agent.environment.hub_packages[str(self.agent.jid)])
                #send message to drones informing them theres a package available for pickup
                await self.send_instruction_new_package(new_package)
                self.counter += 1
                if self.counter == PACKAGES_TO_GENERATE:
                    self.kill()
            
            async def on_start(self):
                self.counter = 0

            def spawn_package(self):
                return self.agent.environment.spawn_package(str(self.agent.jid))

            async def send_instruction_new_package(self,new_package):
                #sends package ID which drones uses to access package information throught the environment
                for drone_id in self.agent.environment.drone_list:
                    msg = Message(to = str(drone_id), sender= str(self.agent.jid))
                    msg.set_metadata("performative", "inform")
                    msg.body = f"New package with id {new_package.id}"
                    print(f"successfully sent message to {drone_id}")
                    await self.send(msg)
        
        class Wait_Before_Calling(OneShotBehaviour):
            async def run(self):
                await asyncio.sleep(wait_time)
                self.kill()
            
            async def on_end(self):
                self.agent.add_behaviour(SpawnRandomPackage(period=periodic))

        self.add_behaviour(Wait_Before_Calling())

    
    async def send_message_to_drone(self, message, drone_id):
        class SendMessage(OneShotBehaviour):
            async def run(self):
                print("Attempting to send message")
                await self.send_message(message, drone_id)
            
            async def send_message(self, message, drone_id):
                msg = Message(to = drone_id)
                msg.set_metadata("performative", "inform")
                msg.body = message
                print(f"successfully sent message to {drone_id}")
                await self.send(msg)
        
        self.add_behaviour(SendMessage())
    
    #peridiocally records positions for visual representation
    class RecordPosition(PeriodicBehaviour):
        #recording data for plotting 3D matplotlib graph and pygame visuals
        #records data for hub position, dropped packages and drone position
        async def run(self):
            atual = []
            atualx_drone = []
            atualy_drone = []
            atualz_drone = []
            atualx_hub = []
            atualy_hub = []
            atualz_hub = []
            for i in range(1,MAX_DRONES+1):
                drone = "drone" + str(i) + "@localhost"
                if drone in self.agent.environment.drone_list:
                    atualx_drone.append(self.agent.environment.drone_positions[drone][0])
                    atualy_drone.append(self.agent.environment.drone_positions[drone][1])
                    atualz_drone.append(self.agent.environment.drone_positions[drone][2])
                else:
                    atualx_drone.append(-50)
                    atualy_drone.append(-50)
                    atualz_drone.append(-50)
            for hub in self.agent.environment.hub_list:
                atualx_hub.append(self.agent.environment.hub_positions[hub][0])
                atualy_hub.append(self.agent.environment.hub_positions[hub][1])
                atualz_hub.append(self.agent.environment.hub_positions[hub][2])
                copy = self.agent.environment.hub_positions[hub].copy()
                copy = np.append(copy,[0])
                atual.append(copy)

            for hub in self.agent.environment.hub_list:
                if len(self.agent.environment.hub_packages[hub])>0:
                    copy = self.agent.environment.hub_positions[hub].copy()
                    copy = np.append(copy,[2])
                    atual.append(copy)
            
            for drone in self.agent.environment.drone_list:
                copy = self.agent.environment.drone_positions[drone].copy()
                copy = np.append(copy,[1])
                atual.append(copy)
                if len(self.agent.environment.drone_delivery[drone])>0:
                    copy = self.agent.environment.drone_positions[drone].copy()
                    copy = np.append(copy,[2])
                    atual.append(copy)
                for pack in self.agent.environment.drone_delivery[drone]:
                    copy = pack.pos.copy()
                    copy = np.append(copy,[3])
                    atual.append(copy)

            atual_x_dropped = np.arange(len(self.agent.environment.hub_list)*PACKAGES_TO_GENERATE)
            atual_x_dropped.fill(-50)

            atual_y_dropped = np.arange(len(self.agent.environment.hub_list)*PACKAGES_TO_GENERATE)
            atual_y_dropped.fill(-50)

            atual_z_dropped = np.arange(len(self.agent.environment.hub_list)*PACKAGES_TO_GENERATE)
            atual_z_dropped.fill(-50)
            for package in self.agent.environment.dropped_packages:
                
                copy = package.position.copy()
                x, y, z = package.position
                atual_x_dropped[package.id-1] = x
                atual_y_dropped[package.id-1] = y
                atual_z_dropped[package.id-1] = z
                copy = np.append(copy,[2])
                atual.append(copy)
                copy = pack.pos.copy()
                copy = np.append(copy,[3])
                atual.append(copy)
            
            atualx_drone = np.append(atualx_drone, atual_x_dropped)
            atualy_drone = np.append(atualy_drone, atual_y_dropped)
            atualz_drone = np.append(atualz_drone, atual_z_dropped)
            
            #data must be in the correct matrix format
            self.agent.environment.x_matrix_drone.append(atualx_drone)
            self.agent.environment.y_matrix_drone.append(atualy_drone)
            self.agent.environment.z_matrix_drone.append(atualz_drone)
            self.agent.environment.x_matrix_hub.append(atualx_hub)
            self.agent.environment.y_matrix_hub.append(atualy_hub)
            self.agent.environment.z_matrix_hub.append(atualz_hub)
            self.agent.environment.x_matrix_dropped_package.append(atual_x_dropped)
            self.agent.environment.y_matrix_dropped_package.append(atual_y_dropped)
            self.agent.environment.z_matrix_dropped_package.append(atual_z_dropped)
            self.agent.environment.pylist.append(atual)

            if self.agent.environment.delivery_count >= len(self.agent.environment.hub_list) * PACKAGES_TO_GENERATE:
                self.kill()



#main loop
async def main():
    #initializing the environment
    env1 = Environment()

    #initializing all the agents
    drone1 = DroneAgent("drone1@localhost", "password", env1, 1, 4, 3, 2, 20) #"normal" drone
    drone2 = DroneAgent("drone2@localhost", "password", env1, 5, 10, 10, 1, 40) #"heavy" drone more carry capacity less speed
    drone3 = DroneAgent("drone3@localhost", "password", env1, 3, 5, 6, 4, 10) #"light" drone more speed less carry capacity 
    drone4 = DroneAgent("drone4@localhost", "password", env1, 5, 5, 10, 2, 20)
    drone5 = DroneAgent("drone5@localhost", "password", env1, 5, 7, 2, 1, 40)
    drone6 = DroneAgent("drone6@localhost", "password", env1, 8, 1, 3, 4, 10)
    drone7 = DroneAgent("drone7@localhost", "password", env1, 2, 5, 10, 2, 20)
    drone8 = DroneAgent("drone8@localhost", "password", env1, 8, 7, 2, 1, 40)
    drone9 = DroneAgent("drone9@localhost", "password", env1, 3, 9, 3, 4, 10)
    drone10 = DroneAgent("drone10@localhost", "password", env1, 6, 2, 1, 2, 20)
    drone11 = DroneAgent("drone11@localhost", "password", env1, 10, 4, 2, 1, 40)
    drone12 = DroneAgent("drone12@localhost", "password", env1, 0, 1, 3, 4, 10)
    drone13 = DroneAgent("drone13@localhost", "password", env1, 7, 0, 3, 2, 20)
    drone14 = DroneAgent("drone14@localhost", "password", env1, 4, 3, 2, 1, 40)
    drone15 = DroneAgent("drone15@localhost", "password", env1, 6, 2, 3, 4, 10)
    hub1 = HubAgent("hub1@localhost", "password", env1, 0, 0, 0)
    hub2 = HubAgent("hub2@localhost", "password", env1, 10, 10 ,0)
    hub3 = HubAgent("hub3@localhost", "password", env1, 0, 10 ,0)

    await drone1.start(auto_register=True)
    await drone2.start(auto_register=True)
    await drone3.start(auto_register=True)
    await drone4.start(auto_register=True)
    await drone5.start(auto_register=True)
    await drone6.start(auto_register=True)
    await drone7.start(auto_register=True)
    await drone8.start(auto_register=True)
    await drone9.start(auto_register=True)
    await drone10.start(auto_register=True)
    await drone11.start(auto_register=True)
    await drone12.start(auto_register=True)
    await drone13.start(auto_register=True)
    await drone14.start(auto_register=True)
    await drone15.start(auto_register=True)
    await hub1.start(auto_register=True)
    await hub2.start(auto_register=True)
    await hub3.start(auto_register=True)

    #start recording the environment for further display
    hub1.add_behaviour(HubAgent.RecordPosition(period=0.2))
    await hub1.spawn_package(0,3)
    await hub2.spawn_package(0.5,3)
    await hub3.spawn_package(1,3)
    await spade.wait_until_finished(drone1)
    await drone1.stop()
    print("drone1 agent has stoped")
    await spade.wait_until_finished(drone2)
    await drone2.stop()
    print("drone2 agent has stoped")
    await spade.wait_until_finished(drone3)
    await drone3.stop()
    print("drone3 agent has stoped")
    await spade.wait_until_finished(drone4)
    await drone4.stop()
    print("drone4 agent has stoped")
    await spade.wait_until_finished(drone5)
    await drone5.stop()
    print("drone5 agent has stoped")
    await spade.wait_until_finished(drone6)
    await drone6.stop()
    print("drone6 agent has stoped")
    await spade.wait_until_finished(drone7)
    await drone7.stop()
    print("drone7 agent has stoped")
    await spade.wait_until_finished(drone8)
    await drone8.stop()
    print("drone8 agent has stoped")
    await spade.wait_until_finished(drone9)
    await drone9.stop()
    print("drone9 agent has stoped")
    await spade.wait_until_finished(drone10)
    await drone10.stop()
    print("drone10 agent has stoped")
    await spade.wait_until_finished(drone11)
    await drone11.stop()
    print("drone11 agent has stoped")
    await spade.wait_until_finished(drone12)
    await drone12.stop()
    print("drone12 agent has stoped")
    await spade.wait_until_finished(drone13)
    await drone13.stop()
    print("drone13 agent has stoped")
    await spade.wait_until_finished(drone14)
    await drone14.stop()
    print("drone14 agent has stoped")
    await spade.wait_until_finished(drone15)
    await drone15.stop()
    print("drone15 agent has stoped")
    await spade.wait_until_finished(hub1)
    await hub1.stop()
    print("hub1 agent has stoped")
    await spade.wait_until_finished(hub2)
    await hub2.stop()
    print("hub2 agent has stoped")
    await spade.wait_until_finished(hub3)
    await hub3.stop()
    print("hub3 agent has stoped")
    print("Program finished")
    

     #metrics for evaluating delivery and help time
    avg_delivery_time = round(env1.time_to_deliver/(PACKAGES_TO_GENERATE*len(env1.hub_list)),2)
    avg_help_time = round(env1.help[0]/env1.help[1],2)
    print(f"The average delivery time was {round(env1.time_to_deliver/(PACKAGES_TO_GENERATE*len(env1.hub_list)),2)} seconds")
    if env1.help[1] > 0:
        print(f"The average help time was {round(env1.help[0]/env1.help[1],2)} seconds")

    #plotting the metrics

    metrics = {"Average Delivery Time": avg_delivery_time, 'Average Help Time': avg_help_time}
    metric_names = list(metrics.keys())
    metric_values = list(metrics.values())
    fig = plt.figure(figsize=(10, 8))
    plt.bar(metric_names, metric_values, color = 'maroon', width = 0.4)
    plt.ylabel('Seconds', fontweight ='bold', fontsize = 15) 
    plt.show()

    #plotting the metrics

    assigned = []
    actual = []
    distance = []
    for id in env1.drone_list:
        assigned.append(env1.drone_package_count[id][0])
        actual.append(env1.drone_package_count[id][1])
        distance.append(env1.drone_distance[id])
    barWidth = 0.25
    fig, ax = plt.subplots(figsize =(9, 5)) 
    br1 = np.arange(len(assigned)) 
    br2 = [x + barWidth for x in br1] 
    plt.bar(br1, assigned, color ='r', width = barWidth, edgecolor ='grey', label ='Assigned Packages') 
    plt.bar(br2, actual, color ='g', width = barWidth, edgecolor ='grey', label ='Delivered Packages')
    ax.set_xlabel('Drones', fontweight ='bold', fontsize = 15) 
    ax.set_ylabel('Number of Packages', fontweight ='bold', fontsize = 15) 
    plt.xticks([r + barWidth for r in range(len(assigned))], ["drone"+str(i) for i in range(1,len(assigned)+1)])
    axes2 = ax.twinx()
    axes2.plot(["drone"+str(i) for i in range(1,len(assigned)+1)],distance, color='b', label='Distance Covered')
    axes2.set_ylabel('Distance', fontweight ='bold', fontsize = 15)
    plt.legend()
    plt.show()
    env1.interface_display()
    
    #displaying what happened on the environment
    #os.chdir(os.getcwd()+'\\ISIA\\Trabalho1\\Drones')
    pv = Interface.EnvironmentViewer(600,600)
    env = wireframe.Wireframe()
    pv.addWireframe('Environment',env)
    running = True
    i=0
    while running:
        env.addNodes(env1.pylist[i])
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pv.display()
        env.remNodes()
        i+=1
        if i==len(env1.pylist)-1:
            running=False
            pygame.QUIT


if __name__ == "__main__":
    spade.run(main())
