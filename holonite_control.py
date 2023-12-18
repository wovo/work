# ===========================================================================
#
# Holonite Waxing Station (AWS)
#
# image process & path planning application
#
# main, ros2 and mqtt communication
#
# ToDo
# - header, copyright, etc.
# - python conventions
# - test???
# - profile problems
# - dump a mould that gives an error? pickle?
# - save data, for later playback
# - playback tool for kamal
#
# ===========================================================================

import random
import time
import json
import pickle
import datetime
import os
import sys
import pathlib

# MQTT
from paho.mqtt import client as paho_mqtt

# camera
import ifm3dpy

import patherror
sys.path.append('../../')  # Navigate two directories up
import vision.main_spray_planner as spray_planner


# ===========================================================================

# stub
def plan_path( 
    cycle,
    coordinates, 
    images, 
    profiles 
):
    spray_plan = spray_planner.spray_planner(
        cycle = cycle, 
        depths = images, 
        profiles = profiles
    )
    print("--returned above spray_planner--")
    print(spray_plan)
    return spray_plan


# ===========================================================================
#
# configuration details
#
# ===========================================================================

class wouter_camera:
    address = "192.168.0.69"
    reconnect_timout = 10
    
class carlos_mqtt:
    broker = "192.168.0.113"
    port = 1883
    username = "test"
    password = "test"
    reconnect_timout = 10    

    
# ===========================================================================

def now_string( filename = False ):
    return datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S:%f")
    
def now_filename():
    return now_string().replace( "/", "-" ).replace( ":", "-" )
    
class path_data:
    def __init__( 
        self, 
        cycle, 
        coordinates, 
        images, 
        profiles
    ):
        self.cycle = cycle
        self.coordinates = coordinates
        self.images = images
        self.profiles = profiles
        self.path = None
        self.exception = None
        
    def plan_path( self ):    
        try:    
            self.path = plan_path(
                self.cycle,
                self.coordinates,
                self.images,
                self.profiles
            )            

        except patherror.PathError as e:
            self.error = str( e )
        
    def dump( 
        self, 
        directory = "dumps/" 
    ):
        pathlib.Path( directory ).mkdir( parents = True, exist_ok = True )    
        with open( f"{directory}/{now_filename()}.pickle", "wb" ) as file:
            pickle.dump( self, file )
            
    def dump_images( self ):
        for n, buffer in enumerate( self.images ):
            with open( f"image-{n}.ppm", "w" ) as f:
                m = max( [ max( line ) for line in buffer ] )
                f.write( f"P3 {len( buffer[ 0 ] )} {len( buffer )} {m}\n" )
                for line in buffer:
                    for pixel in line:
                        f.write( f"{pixel} {pixel} {pixel} " )
                    f.write( "\n" ) 
                    
    def set_error( 
        self, 
        error: str
    ) -> None:
        self.error = error

    def set_path( 
        self, 
        path 
    ) -> None:
        self.path = path
        
            
def path_data_from_file( file_name ):
    if file_name.find( "." ) < 0:
        file_name += ".pickle"
    with open( file_name, "rb" ) as file:
        return pickle.load( file )   


# ===========================================================================
#
# MQTT connection
#
# ===========================================================================

class mqtt_client:

    # =======================================================================        
    
    def __init__( 
        self, 
        config, 
        log 
    ) -> None:
        client_id = f'publish-{random.randint(0, 1000)}'
        self._client = paho_mqtt.Client( client_id )
        self._log = log
        self.connected = False
        self._config = config
        self._last_attempt = 0
        self._subscriptions = []
        
        self._client.on_connect = \
            lambda a, b, c, d: self.on_connect( a, b, c, d )
       
        self._client.username_pw_set( 
            self._config.username, 
            self._config.password 
        )
        
    # =======================================================================        

    def _connect( self ) -> None:
        if self.connected:
            return
            
        if time.time() - self._last_attempt < self._config.reconnect_timout:   
            return
            
        try:
            self._client.connect( 
                self._config.broker, 
                self._config.port 
            )       
            for topic_name, callback in self._subscriptions:
                self._subscribe( topic_name, callback )
            self.connected = True
            
        except TimeoutError as e:    
            self._log( f"MQTT not connected:\n{e}" )            

    # =======================================================================        

    def on_connect( 
        self, 
        client, 
        userdata, 
        flags, 
        rc 
    ) -> None:
        if rc == 0:
            self._log( "MQTT broker connected")
        else:
            self._log( f"Failed to connect, return code {rc}\n")     
            self.connected = True            
            
    # =======================================================================        

    def spin_once( self ) -> None:
        self._connect()
        self._client.loop( timeout = 0 )   

    # =======================================================================        

    def _subscribe( 
        self, 
        topic_name, 
        callback 
    ) -> None:
        self._client.subscribe( topic_name )            
        self._client.message_callback_add(
            topic_name,
            lambda client, userdata, message: 
                callback( json.loads( message.payload.decode()) )
        )   
        
    # =======================================================================        

    def subscribe( 
        self, 
        topic_name: str, 
        callback 
    ) -> None:
        self._subscriptions.append( [ topic_name, callback ] )
        if self.connected:
            self._subscribe( topic_name, callback )
        
    # =======================================================================    
    # publish data to the PLC via the MQTT server
    # =======================================================================    

    def publish( 
        self, 
        topic: str, 
        data: dict[ str, any ]
    ) -> None:
        if self.connected:
            self._client.publish( 
                topic, 
                payload = json.dumps( data ),
                qos = 0, 
                retain = False
            ) 

    # =======================================================================        
            

# ===========================================================================
#
# O3D camera interface
#
# ===========================================================================

class camera:

    # =======================================================================    
    # 
    # =======================================================================    

    def __init__( 
        self, 
        config, 
        callback, 
        log 
    ) -> None:
        self._callback = callback
        self._config = config
        self.connected = False
        self._log = log
        self._last_attempt = 0
        
    # =======================================================================    
    # 
    # =======================================================================    

    def _connect( self ) -> None:
        if self.connected:
            return
            
        if  time.time() - self._last_attempt< self._config.reconnect_timout:   
            return
            
        try:
            self._frame = None        
            self._camera = ifm3dpy.device.Device( ip = self._config.address )
            self._grabber = ifm3dpy.framegrabber.FrameGrabber( self._camera )
            self._grabber.start()       
            
            self._last_attempt = time.time() 
            self.connected = True
            self._log( "camera connected" )
            
        except Exception as e:
            self._log( f"camera not connected:\n{e}" )

    # =======================================================================    
    # 
    # =======================================================================    

    def spin_once( self ) -> None:
        self._connect()
        
        if not self.connected:
            return
    
        try:            
            if self._frame is None:
                self._frame = self._grabber.wait_for_frame()
            
            ready, frame = self._frame.wait_for( 0 )   
            if ready:
                buffer = frame.get_buffer( frame.get_buffers()[ 4 ] )
                self._frame = self._grabber.wait_for_frame()
                self._callback( buffer )            
                
        except Exception as e:
            self.connected = False
            self._log( f"camera disconnected:\n{e }" )                

    # =======================================================================    

# ===========================================================================
#
# machine control
#
# ===========================================================================

class holonite_automatic_waxing_machine_controller:

    # =======================================================================    
    # 
    # =======================================================================    

    def __init__( 
        self, 
        camera_config, 
        mqtt_config,
        log_file_name = None,
        dump_data = False,
        dump_path_errors = False
    ):
    
        self._log_file_name = log_file_name
        self._dump_data = dump_data
        self._dump_path_errors = dump_path_errors
        self._log( "===== startup" )
        self._log( "connect to the camera" )
        self._camera = camera( 
            camera_config, 
            lambda data: self._receive_image( data ),
            lambda s: self._log( s )
        )
        
        self._log( "connect MQTT" )
        self._mqtt = mqtt_client( 
            mqtt_config,
            lambda s: self._log( s )        
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
        self._log( "start the engine" )
        
    # =======================================================================    
    # 
    # =======================================================================    

    def spin_once( self ):
        self._camera.spin_once()
        self._mqtt.spin_once()      
        
    # =======================================================================    
    # 
    # =======================================================================    

    def spin( self, sleep_time = 0.001 ):
        while True:
            self.spin_once()
            time.sleep( sleep_time )
            
    # =======================================================================    
    # 
    # =======================================================================    

    def _log( self, message ):
        s = f"{now_string()} : {message}"
        print( s )
        if self._log_file_name is not None:
            with open( self._log_file_name, "a+" ) as log_file:
                log_file.write( s + "\n" )
                       
    # =======================================================================        
    #
    # processing "state machine"
    #
    # =======================================================================       
    
    def _reset( self ):
        self._expected_number_of_images = 15
        self._coordinates = []
        self._images = []
        self._cycle = None  
        self._profiles = None      
        
    # =======================================================================    
    # 
    # =======================================================================    

    def _update_cycle( self, data, forced = False ):
        cycle = data[ "cycle" ]
        if ( cycle != self._cycle ) or forced:
            self._log( f"==== new cycle {cycle}" )
            self._reset()    
            self._cycle = cycle        
            
    # =======================================================================    
    # 
    # =======================================================================    

    def _receive_image( self, data ):
        self._log( f"image received #{1+len(self._images)}" )
        self._images.append( data )
        
    # =======================================================================    
    # 
    # =======================================================================    

    def _receive_profiles( self, data ):
        self._log( "profiles received " )
        self._update_cycle( data, forced = True )
        self._profiles = data[ "heights" ]
        #with open( f"profiles-{self._cycle}.pickle", "wb" ) as file:
        #    pickle.dump( self._profiles, file)

    # =======================================================================    
    # 
    # =======================================================================    

    def _receive_image_coordinates( self, data ):
        self._log( "image coordinates received" )
        #print( f"{data}" )
        self._update_cycle( data )
        coordinate = data[ "xCoordinate" ]
        self._log( f"coordinates received #{1+len(self._coordinates)}" )        
        self._coordinates.append( coordinate )      

    # =======================================================================    
    # 
    # =======================================================================    

    def _receive_path_request( self, data ):
        self._log( "path request received" )
        self._update_cycle( data )
        
        n = len( self._coordinates )
        if n != self._expected_number_of_images:
            self._send_path_error( f"invalid number of coordinates: {n}" )
            return
            
        if not self._camera.connected:
            self._send_path_error( "camera not connected" )
            return        
            
        n = len( self._images )
        if n != self._expected_number_of_images:
            self._send_path_error( f"invalid number of images: {n}" )
            return
        
        if self._profiles is None:
            self._send_path_error( f"no profiles" )
            return
            
        data = path_data(             
            self._cycle,
            self._coordinates, 
            self._images, 
            self._profiles 
        )  
        
        data.plan_path()
        if data.path is None:
            self._send_path_error( data.error )
        else:            
            self._send_path_response( data.path )
            
        if self._dump_data:
            data.dump()            
            
        
    # =======================================================================    
    # 
    # =======================================================================    

    def _send_path_response( self, path ):
        self._log( "path response sent" )
        self._mqtt.publish( 
            "path_response", 
            path 
        )
    
    # =======================================================================    
    # 
    # =======================================================================    

    def _send_path_error( self, error ):
        self._log( f"path error sent [{error}]" )    
        self._mqtt.publish( 
            "path_error", 
            { 
                "cycle": self._cycle, 
                "error": error 
            }
        )    
        
    # =======================================================================        


# ===========================================================================      
#
# ===========================================================================      

if __name__ == '__main__':
    if len( sys.argv ) > 1:
    
        if sys.argv[ 1 ] == "read":
        
            if len( sys.argv ) < 3:
                print( "specify a file name" )
            
            else:
                data = path_data_from_file( sys.argv[ 2 ] )
                data.plan_path()        
                print( f"path = {data.path}" )
                print( f"error = {data.error}" )
            
        elif sys.argv[ 1 ] == "dump":
        
            if len( sys.argv ) < 3:
                print( "specify a file name" )
            
            else:
                data = path_data_from_file( sys.argv[ 2 ] )
                data.dump_images()
            
        else:
            print( f"invalid command {sys.argv[ 1 ]}" )        
    
    else:
        m = holonite_automatic_waxing_machine_controller( 
            camera_config = wouter_camera, 
            mqtt_config = carlos_mqtt,
            log_file_name = "logging.txt",
            dump_data = True,
            dump_path_errors = True
        )
        m.spin()

    
# ===========================================================================

