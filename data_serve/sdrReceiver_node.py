import pyModeS as pms
from pyModeS.decoder import adsb 
from adsbData import adsbStateVector

import socket 
from time import sleep, time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from threading import Thread

TCP_IP = "127.0.0.1"
TCP_PORT = 30002
BUFFER_SIZE = 1024

class sdrNode(Node):
    def __init__(self):
        super().__init__('sdr_node')

        self.adsb_publisher_ = rclpy.create_publisher(String, 'adsb_feed', 10)

        self.sdr_thread = Thread(target=self.sdr_interface)
        self.sdr_thread.start()


    def sdr_interface(self):

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((TCP_IP, TCP_PORT))
            
            #Set up the dictionary that stores the states to be built
            self.icao_dictionary = {}

            self.running = True
            self.updated = 0
            while self.running:
                data = self.socket.recv(BUFFER_SIZE)
                data = data.decode('utf-8')
                final = data.index(";")
                hex_message = data[1:final]

                self.updated = 0

                binary_message = bin(int(hex_message,16))[2:]

                icao = pms.icao(hex_message)
                if icao not in self.icao_dictionary.keys():
                    self.icao_dictionary[icao] = adsbStateVector(icao)
                else:
                    pass

                #find Type Code and use to determine message type
                update_state_vector = self.icao_dictionary.get(icao)
                if adsb.typecode(hex_message) in range(1,4):
                    category = adsb.category(hex_message)
                    callsign = adsb.callsign(hex_message)
                    if category is not None and callsign is not None:
                        update_state_vector.updateCategory(category)
                        update_state_vector.updateCallsign(callsign)
                    self.updated = 1


                elif adsb.typecode(hex_message) in range(9,18):
                    if int(binary_message[53]) == 0:
                        even_message = hex_message
                        even_ts = time()
                        update_state_vector.updateEven(even_message, even_ts)
                        odd_message, odd_ts = update_state_vector.getOdd()
                    else:
                        odd_message = hex_message
                        odd_ts = time()
                        update_state_vector.updateOdd(odd_message, odd_ts)
                        even_message, even_ts = update_state_vector.getEven()

                    if None not in [odd_message, even_message, even_ts, odd_ts]:
                        latitude, longitude = adsb.airborne_position(even_message, odd_message, even_ts, odd_ts)
                        altitudeBaro = adsb.altitude(hex_message)
                       
                        if altitudeBaro is not None and latitude is not None and longitude is not None:
                            update_state_vector.updatePositionBaro(altitudeBaro, latitude, longitude)
                        update_state_vector.clearMsgs()
                        update_state_vector.checkInPolygon()
                        self.updated = 1

                elif adsb.typecode(hex_message) in range(20,22):
                    if int(binary_message[53]) == 0:
                        even_message = hex_message
                        even_ts = time()
                        update_state_vector.updateEven(even_message, even_ts)
                        odd_message, odd_ts = update_state_vector.getOdd()
                    else:
                        odd_message = hex_message
                        odd_ts = time()
                        update_state_vector.updateOdd(odd_message, odd_ts)
                        even_message, even_ts = update_state_vector.getEven()
                    if None not in [odd_message, even_message, even_ts, odd_ts]:
                        latitude, longitude = adsb.airborne_position(even_message, odd_message, even_ts, odd_ts)
                        altitudeGNSS = adsb.altitude(hex_message)
                        if altitudeGNSS is not None and latitude is not None and longitude is not None:
                            update_state_vector.updatePositionGNSS(altitudeGNSS, latitude, longitude)
                        update_state_vector.clearMsgs()
                        update_state_vector.checkInPolygon()
                        self.updated = 1

                elif adsb.typecode(hex_message) == 19:
                    speed, angle, vert_rate, speed_type = adsb.velocity(hex_message)
                    if speed_type == 'GS' and speed is not None and angle is not None and vert_rate is not None:
                        update_state_vector.updateVelocity(speed, angle, vert_rate)
                    self.updated = 1
                else:
                    pass


                states_to_remove = []

                state_list = []
                for i in self.icao_dictionary.keys():
                    state_vector = self.icao_dictionary[i]
                    if(state_vector.getInPolygon() and self.updated):
                        state_list.append(state_vector.getStateVector())
                    if state_vector.timePassed() is not None:
                        if state_vector.timePassed()>30:
                            states_to_remove.append(i)

                if state_list is not None:
                    self.adsb_publisher_.publish(state_list)

                for key in states_to_remove:
                    self.icao_dictionary.pop(key)

def main(args=None):
    rclpy.init(args=args)

    sdr_node = sdrNode()
    rclpy.spin(sdr_node)

    sdr_node.destroy_node()
    rclpy.shutdown()
