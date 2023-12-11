# ===========================================================================
#
# Holonite Waxing Station (AWS)
#
# image process & path planning application
#
# main, ros2 and mqtt communication
#
# ===========================================================================

import random
import time
import json
import pickle

# ROS2
import rclpy
from rclpy.node import Node
import std_msgs
from std_msgs.msg import String

# MQTT
from paho.mqtt import client as paho_mqtt
# import paho.mqtt

class PathError( ValueError ): pass


# ===========================================================================


# stub
def plan_path( 
    cycle,
    coordinates, 
    images, 
    profiles 
):
    print( "PATH PLANNING" )
    print( f"    cycle : {cycle}" )
    print( f"    coordinates : {coordinates}" )
    print( f"    profiles : {profiles}" )


    return {  
            "cycle": cycle,
            "steps": [  
                {
                    "destination": {  
                        "x": 9,  
                        "y": 9, 
                        "z": 9  
                    }, 
                    "maximum_speed": {  
                        "x": 9,  
                        "y": 9, 
                        "z": 9  
                    }, 
                    "spraying": "true" 
                }
            ] 
        } 


# ===========================================================================
#
# interface details
#
# ===========================================================================

class wouter_camera:
    topic = "camera_data"
    
class carlos_mqtt:
    broker = "192.168.0.114"
    port = 1883
    username = "test"
    password = "test"


# ===========================================================================
#
# MQTT connection
#
# ===========================================================================

class mqtt_client:

    def __init__( self, settings ): 
        client_id = f'publish-{random.randint(0, 1000)}'
        self._client = paho_mqtt.Client( client_id )
        
        self._client.on_connect = \
            lambda a, b, c, d: self.on_connect( a, b, c, d )
       
        
        self._client.username_pw_set( 
            settings.username, 
            settings.password 
        )
        self._client.connect( 
            settings.broker, 
            settings.port 
        )       

    def on_connect( self, client, userdata, flags, rc ) :
        if rc == 0:
            print( "Connected to MQTT Broker!")
        else:
            print( f"Failed to connect, return code {rc}\n")        
            
    def spin_once( self ):
        self._client.loop(timeout=0)    
        
    def subscribe( self, topic_name, callback ):  
        self._client.subscribe( topic_name )            
        self._client.message_callback_add(
            topic_name,
            lambda client, userdata, message: 
                callback( json.loads( message.payload.decode()) )
        )
        
    def publish( self, topic, data ):
        self._client.publish( 
            topic, 
            payload = json.dumps( data ),
            qos = 0, 
            retain = False
        ) 
            

# ===========================================================================
#
# ROS2 connection
#
# ===========================================================================

class ros_node( Node ):

    def __init__( self ):
        rclpy.init()    
        super().__init__( "aws" )

    def spin_once( self ):
        rclpy.spin_once( self, timeout_sec = 0 )    
        
    def subscribe( self, topic_name, callback ):
        self.create_subscription(
            std_msgs.msg.UInt8MultiArray,
            topic_name,
            callback,
            10
        )    


# ===========================================================================
#
# machine control
#
# ===========================================================================

class holonite_automatic_waxing_machine:

    def __init__( self, ros_config, mqtt_config ):
        self._log( "create ROS2 node" )
        self._ros = ros_node()
        self._log( "connect MQTT" )
        self._mqtt = mqtt_client( mqtt_config )
        self._ros.subscribe( 
            ros_config.topic, 
            lambda data: self._receive_image( data )
        )
        self._mqtt.subscribe( 
            "profiles", 
            lambda data: self._receive_profiles( data )
        )
        self._mqtt.subscribe( 
            "image_coordinates", 
            lambda data: self._receive_image_coordinates( data )
        )
        self._mqtt.subscribe( 
            "path_request", 
            lambda data: self._receive_path_request( data )
        )
        self._reset()
        self._log( "start" )
        
    def spin_once( self ):
        self._ros.spin_once()
        self._mqtt.spin_once()      
        
    def spin( self, sleep_time = 0.001 ):
        while True:
            self.spin_once()
            time.sleep( sleep_time )
            
    def _log( self, message ):
        print( message )
                    
            
    # =======================================================================        
    #
    # state machine
    #
    # =======================================================================       
    
    def _reset( self ):
        self._expected_number_of_images = 15
        self._coordinates = []
        self._images = []
        self._cycle = None  
        self._profiles = None      
        
    def _update_cycle( self, data ):
        cycle = data[ "cycle" ]
        if cycle != self._cycle:
            self._log( f"    new cycle {cycle}" )
            self._reset()    
            self._cycle = cycle        
            
    def _receive_image( self, data ):
        self._log( "image received" )
        self._images.append( data )
        
    def _receive_profiles( self, data ):
        self._log( "profiles received " )
        print( f"{data}" )
        self._update_cycle( data )
        self._profiles = data[ "heights" ]
        with open( f"profiles-{self._cycle}.pickle", "wb" ) as file:
            pickle.dump( self._profiles, file)

    def _receive_image_coordinates( self, data ):
        self._log( "image coordinates received" )
        print( f"{data}" )
        self._update_cycle( data )
        coordinate = data[ "xCoordinate" ]
        self._coordinates.append( coordinate )      

    def _receive_path_request( self, data ):
        self._log( "path request received" )
        self._update_cycle( data )
        
        n = len( self._coordinates )
        if n != self._expected_number_of_images:
            self._send_path_error( f"invalid number of coordinates: {n}" )
            return
            
        n = len( self._images )
        if n != self._expected_number_of_images:
            self._send_path_error( f"invalid number of images {n}" )
            return
        
        if self._profiles is None:
            self._send_path_error( f"no profiles" )
            return
            
        try:    
            path = plan_path( 
                self._cycle,
                self._coordinates, 
                self._images, 
                self._profiles 
            )
        except PathError as e:
            self._send_path_error( str( e ) )
            return
            
        self._send_path_response( path )    
        
    def _send_path_response( self, path ):
        self._log( "path response sent" )
        self._mqtt.publish( "path_response", path )
    
    def _send_path_error( self, error ):
        self._log( f"path error sent [{error}]" )    
        self._mqtt.publish( "error", error )
        
    # =======================================================================        


# ===========================================================================


     
# MQTT
def publish( client):
    
    msg_json = {  
            "cycle": 9,
            "steps": [  
                {
                    "destination": {  
                        "x": 9,  
                        "y": 9, 
                        "z": 9  
                    }, 
                    "maximum_speed": {  
                        "x": 9,  
                        "y": 9, 
                        "z": 9  
                    }, 
                    "spraying": "true" 
                }
            ] 
        }   
    
    # while True:
    time.sleep(1)
    msg = f"messages: {msg_json}"
    result = client.publish(topic_path_response, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic_path_response}`")
    else:
        print(f"Failed to send message to topic {topic_path_response}")


# ===========================================================================

    
def main(args=None):
    m = holonite_automatic_waxing_machine( 
        wouter_camera, 
        carlos_mqtt 
    )
    m.spin()

if __name__ == '__main__':
    m = holonite_automatic_waxing_machine( 
        ros_config = wouter_camera, 
        mqtt_config = carlos_mqtt 
    )
    m.spin()

    
