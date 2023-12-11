import ifm3dpy
import asyncio

async def main():
    camera = ifm3dpy.device.Device( ip = "192.168.0.69" )
    grabber = ifm3dpy.framegrabber.FrameGrabber( camera )
    frame = grabber.wait_for_frame()
    print( "waiting" )
    await frame
    print( "got it" )
    
asyncio.run( main() )

