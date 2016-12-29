#!/usr/bin/env python3
import sys
import time
import argparse

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install Pillow numpy` to run this example')
    
import cozmo
from cozmo import robot
from cozmo import util
from cozmo import base
from cozmo import behavior
from cozmo import world

robot = None

# Define an annotator using the annotator decorator
@cozmo.annotate.annotator
def clock(image, scale, annotator=None, world=None, **kw):
    d = ImageDraw.Draw(image)
    bounds = (0, 0, image.width, image.height)
    text = cozmo.annotate.ImageText(time.strftime("%H:%m:%S"),
            position=cozmo.annotate.TOP_LEFT)
    text.render(d, bounds)

# Define another decorator as a subclass of Annotator
class Battery(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)
        
def run(sdk_conn):
    # The method runs once Cozmo is connected.  
    global robot
    robot = sdk_conn.wait_for_robot()

    robot.world.image_annotator.add_static_text('text', 'Coz-Cam', position=cozmo.annotate.TOP_RIGHT)
    robot.world.image_annotator.add_annotator('clock', clock)
    robot.world.image_annotator.add_annotator('battery', Battery)

    import IPython
    IPython.embed()
    
def get_robot():
    if robot == None:
        print("ERROR: YOU HAVE NOT CONNECTED TO THE ROBOT")
    return robot
    
if __name__ == '__main__':
    cozmo.setup_basic_logging()
    try:
        cozmo.connect_with_tkviewer(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
    
