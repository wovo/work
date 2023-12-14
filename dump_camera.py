import time
import ifm3dpy

def waiting( x ):
   for n in range( 100000 ):
       print( f"{n} waiting" )
       ready, frame = x.wait_for( 0 )
       if ready:
            return frame
       time.sleep( 0.5 )
       
def save( frame ):   
    buffer = frame.get_buffer( frame.get_buffers()[ 0 ] )
    print( buffer )
    with open( "x.ppm", "w" ) as f:
        m = max( [ max( line ) for line in buffer ] )
        f.write( f"P3 {len( buffer[ 0 ] )} {len( buffer )} {m}\n" )
        for line in buffer:
            for pixel in line:
                f.write( f"{pixel} {pixel} {pixel} " )
            f.write( "\n" )    
    
        
"""        
	ifm3d::SimpleImageBuffer::Img distance = img->DistanceImage();
       	else if(distance.format == ifm3d::pixel_format::FORMAT_16U)
	{	  //if data format is 16U then distances are in millimeters, Hence max distance is multiplied by 1000. 
		scaleImageToRGB<unsigned short>(distance, confidence, distance_scaled, min_distance, max_distance * 1000);
		findMinAndMax<unsigned short>(amplitude, confidence, min, max);
		scaleImageToRGB<unsigned short>(amplitude, confidence, amplitude_scaled, min, max);
	}
    writePPMFile(distance_scaled, "/dev/shm/distanceImage.ppm"))
"""


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
        save( f )
main()

