import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
# MQTT
import random
import time
from paho.mqtt import client as mqtt_client

# MQTT
broker = "172.17.0.1"
port = 1883
topic_path_response = "path_response"
topic_path_request = "path_request"
# Generate a Client ID with the publish prefix.
client_id = f'publish-{random.randint(0, 1000)}'


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '/path_response/toros', 10)
        timer_period = 0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0


    def timer_callback(self):
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

        # ROS publishing
        msg_string = json.dumps(msg_json)
        msg = String()
        msg.data = msg_string
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
     
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


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client
    
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    mqtt_client = connect_mqtt()
    
    while True:
        rclpy.spin_once(minimal_publisher, timeout_sec = 0)
        mqtt_client.loop(timeout=0)
        publish(mqtt_client)

    # rclpy.shutdown()


if __name__ == '__main__':
    main()

    
