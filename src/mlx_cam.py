## @file mlx_cam.py
# 
#  RAW VERSION
#  This version uses a stripped down MLX90640 driver which produces only raw
#  data, not calibrated data, in order to save memory.
# 
#  This file contains a wrapper that facilitates the use of a Melexis MLX90640
#  thermal infrared camera for general use. The wrapper contains a class MLX_Cam
#  whose use is greatly simplified in comparison to that of the base class,
#  @c class @c MLX90640, by mwerezak, who has a cool fox avatar, at
#  @c https://github.com/mwerezak/micropython-mlx90640
# 
#  To use this code, upload the directory @c mlx90640 from mwerezak with all
#  its contents to the root directory of your MicroPython device, then copy
#  this file to the root directory of the MicroPython device.
# 
#  There's some test code at the bottom of this file which serves as a
#  beginning example.
# 
#  @author mwerezak Original files, Summer 2022
#  @author JR Ridgely Added simplified wrapper class @c MLX_Cam, January 2023
#  @copyright (c) 2022-2023 by the authors and released under the GNU Public
#      License, version 3.

import utime as time
from machine import Pin, I2C
from mlx90640 import MLX90640
from mlx90640.calibration import NUM_ROWS, NUM_COLS, IMAGE_SIZE, TEMP_K
from mlx90640.image import ChessPattern, InterleavedPattern


## @brief   Class which wraps an MLX90640 thermal infrared camera driver to
#           make it easier to grab and use an image. 
#  @details This image is in "raw" mode, meaning it has not been calibrated
#           (which takes lots of time and memory) and only gives relative IR
#           emission seen by pixels, not estimates of the temperatures.
class MLX_Cam:

    ## @brief   Set up an MLX90640 camera.
    #  @param   i2c An I2C bus which has been set up to talk to the camera;
    #           this must be a bus object which has already been set up
    #  @param   address The address of the camera on the I2C bus (default 0x33)
    #  @param   pattern The way frames are interleaved, as we read only half
    #           the pixels at a time (default ChessPattern)
    #  @param   width The width of the image in pixels; leave it at default
    #  @param   height The height of the image in pixels; leave it at default
    def __init__(self, i2c = I2C(1, freq = 1000000), address=0x33, pattern=ChessPattern,
                 width=NUM_COLS, height=NUM_ROWS):

        ## The I2C bus to which the camera is attached
        self._i2c = i2c
        ## The address of the camera on the I2C bus
        self._addr = address
        ## The pattern for reading the camera, usually ChessPattern
        self._pattern = pattern
        ## The width of the image in pixels, which should be 32
        self._width = width
        ## The height of the image in pixels, which should be 24
        self._height = height
        ## Tracks whether an image is currently being retrieved
        self._getting_image = False
        ## Which subpage (checkerboard half) of the image is being retrieved
        self._subpage = 0

        # The MLX90640 object that does the work
        self._camera = MLX90640(i2c, address)
        self._camera.set_pattern(pattern)
        self._camera.setup()

        ## A local reference to the image object within the camera driver
        self._image = self._camera.raw
    def get_csv(self, array, limits=None):

        if limits and len(limits) == 2:
            scale = (limits[1] - limits[0]) / (max(array) - min(array))
            offset = limits[0] - min(array)
        else:
            offset = 0.0
            scale = 1.0
        for row in range(self._height):
            line = ""
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          + offset) * scale)
                if col:
                    line += ","
                line += f"{pix}"
            yield line
        return



    def get_image_nonblocking(self):

        # If this is the first recent call, begin the process
        if not self._getting_image:
            self._subpage = 0
            self._getting_image = True
        
        # Read whichever subpage needs to be read, or wait until data is ready
        if not self._camera.has_data:
            return None
        
        image = self._camera.read_image(self._subpage)
        
        # If we just got subpage zero, we need to come back and get subpage 1;
        # if we just got subpage 1, we're done
        if self._subpage == 0:
            self._subpage = 1
            return None
        else:
            self._getting_image = False
            return image
    def run(self, threshold, array):
        limits = [0,99]
        """!
        @brief   Generate a string containing image data in CSV format.
        @details This function generates a set of lines, each having one row of
                 image data in Comma Separated Variable format. The lines can
                 be printed or saved to a file using a @c for loop.
        @param   array The array of data to be presented
        @param   limits A 2-iterable containing the maximum and minimum values
                 to which the data should be scaled, or @c None for no scaling
        """
        val = [0]*32
        if limits and len(limits) == 2:
            scale = (limits[1] - limits[0]) / (max(array) - min(array))
            offset = limits[0] - min(array)
        else:
            offset = 0.0
            scale = 1.0
        for row in range(self._height):
            line = ""
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          * scale) + offset)
                if pix >= threshold:
                    pix = 99
                elif pix < threshold:
                    pix = 0
                if col:
                    pass
                val[col] += int(pix)
                
            #for column,value in enumerate(line.split(',')):
                #val[column] += int(value)
        print('this is the value',val,'this is the max',max(val))
        sums = 0
        for x in range(32):
            sums += (x+1)*val[x]
            
        centroid = sums / sum(val)
            
        return centroid
    


