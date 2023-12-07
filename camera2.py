import time
import ifm3dpy

def waiting( x ):
   for n in range( 100000 ):
       print( f"{n} waiting" )
       ready, frame = x.wait_for( 0 )
       if ready:
            return frame
       time.sleep( 0.01 )

def main():
    camera = ifm3dpy.device.Device( ip = "192.168.0.69" )
    grabber = ifm3dpy.framegrabber.FrameGrabber( camera )
    grabber.start()
    while True:
        frame = grabber.wait_for_frame()
        print( "wait" )
        f = waiting( frame )
        print("got it" )
        print( f.get_buffers() )
    
main()

