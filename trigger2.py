import ifm3dpy


def main():
    camera = ifm3dpy.device.Device( ip = "192.168.0.69" )
    grabber = ifm3dpy.framegrabber.FrameGrabber( camera )
    trigger = grabber.sw_trigger()
    #trigger.wait()

main()
