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
# 
#
# ===========================================================================

import random
import time
import json
import pickle
import datetime


# MQTT
from paho.mqtt import client as paho_mqtt

# camera
import ifm3dpy

# stubbed path
import carlos_path


# ===========================================================================

# stub
class PathError( ValueError ): pass
def plan_path( 
    cycle,
    coordinates, 
    images, 
    profiles 
):
    return carlos_path.carlos_path( cycle )


# ===========================================================================
#
# interface details
#
# ===========================================================================

class wouter_camera:
    address = "192.168.0.69"
    
class carlos_mqtt:
    broker = "192.168.0.110"
    port = 1883
    username = "test"
    password = "test"

    
# ===========================================================================

def now_string():
    return datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")


# ===========================================================================
#
# MQTT connection
#
# ===========================================================================

class mqtt_client:

    def __init__( self, settings, log ): 
        client_id = f'publish-{random.randint(0, 1000)}'
        self._client = paho_mqtt.Client( client_id )
        self._log = log
        
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
            self._log( "MQTT broker connected")
        else:
            self._log( f"Failed to connect, return code {rc}\n")        
            
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
# O3D camera interface
#
# ===========================================================================

class camera:

    def __init__( self, config, callback, log ):
        self._callback = callback
        self._frame = None
        self._config = config
        self._connected = False
        self._log = log
        
    def _connect( self ):
        if self._connected:
            return
            
        try:
        
            self._camera = ifm3dpy.device.Device( ip = self._config.address )
            self._grabber = ifm3dpy.framegrabber.FrameGrabber( self._camera )
            self._grabber.start()  
            self._connected = True
            self._log( "camera connected" )
            
        except Exception as e:
            self._log( f"camera not connected:\n{e }" )

    def spin_once( self ):
        self._connect()
        
        if not self._connected:
            return
    
        try:
            if self._frame is None:
                self._frame = self._grabber.wait_for_frame()
            
            ready, frame = self._frame.wait_for( 0 )   
            if ready:
                buffer = frame.get_buffer( frame.get_buffers()[ 0 ] )
                self._frame = self._grabber.wait_for_frame()
                self._callback( buffer )            
                
        except Exception as e:
            self._connected = False
            self._log( f"camera disconnected:\n{e }" )                


# ===========================================================================
#
# machine control
#
# ===========================================================================

class holonite_automatic_waxing_machine_controller:

    def __init__( 
        self, 
        camera_config, 
        mqtt_config,
        log_file_name = None,
        dump_profiles = False,
        dump_path_errors = False
    ):
    
        self._log_file_name = log_file_name
        self._dump_profiles = dump_profiles
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
        
    def spin_once( self ):
        self._camera.spin_once()
        self._mqtt.spin_once()      
        
    def spin( self, sleep_time = 0.001 ):
        while True:
            self.spin_once()
            time.sleep( sleep_time )
            
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
        
    def _update_cycle( self, data, forced = False ):
        cycle = data[ "cycle" ]
        if ( cycle != self._cycle ) or forced:
            self._log( f"==== new cycle {cycle}" )
            self._reset()    
            self._cycle = cycle        
            
    def _receive_image( self, data ):
        self._log( f"image received #{1+len(self._images)}" )
        self._images.append( data )
        
    def _receive_profiles( self, data ):
        self._log( "profiles received " )
        if self._dump_profiles:
            with open( f"profile-{self._cycle}.json", "w" ) as dump_file:
                dump_file.write( f"{data}" )
        self._update_cycle( data, forced = True )
        self._profiles = data[ "heights" ]
        #with open( f"profiles-{self._cycle}.pickle", "wb" ) as file:
        #    pickle.dump( self._profiles, file)

    def _receive_image_coordinates( self, data ):
        self._log( "image coordinates received" )
        #print( f"{data}" )
        self._update_cycle( data )
        coordinate = data[ "xCoordinate" ]
        self._log( f"coordinates received #{1+len(self._coordinates)}" )        
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
            self._send_path_error( f"invalid number of images: {n}" )
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
            
            if self._dump_path_errors:
                path = f"path-error-{now_string()}"
                os.makedirs( path, exist_ok = True )
                with open( f"{path}/error.txt", "w" ) as file:
                    file.write( f"{e}" )                  
                with open( f"{path}/profile.pickle", "w" ) as file:
                    pickle.dump( self._profile, file )       
                for n, image in enumerate( self._images ):
                    with open( f"{path}/image_{n}.pickle", "w" ) as file:
                        pickle.dump( image, file)      
                
            
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

if __name__ == '__main__':
    m = holonite_automatic_waxing_machine_controller( 
        camera_config = wouter_camera, 
        mqtt_config = carlos_mqtt,
        log_file_name = "logging.txt",
        dump_profiles = True,
        dump_path_errors = True
    )
    m.spin()

    
# ===========================================================================
