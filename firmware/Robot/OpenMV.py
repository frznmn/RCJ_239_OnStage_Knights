# Aprilfamid Example
#
# This example shows the power of the OpenMV Cam to detect April famid
# on the OpenMV Cam M7. The M4 versions cannot detect April famid.

import sensor, image, time, math
from pyb import UART

uart = UART(1, 9600)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()
tagsize = 0.43

# Note! Unlike find_qrcodes the find_aprilfamid method does not need lens correction on the image to work.
# The apriltag code supports up to 6 tag families which can be processed at the same time.
# Returned tag objects will have their tag family and id within the tag family.

sensor.set_auto_exposure(False)
current_exposure_time_in_microseconds=  sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds* 1.4))


tag_families = 0
tag_families |= image.TAG16H5 # comment out to disable this family
minToTag = 110
#tag_families |= image.TAG25H7 # comment out to disable this family
#tag_families |= image.TAG25H9 # comment out to disable this family
#tag_families |= image.TAG36H10 # comment out to disable this family
#tag_families |= image.TAG36H11 # comment out to disable this family (default family)
#tag_families |= image.ARTOOLKIT # comment out to disable this family

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other famid families just use TAG36H11 which is the default family.

def sgn(a):
    if a > 0:
        return 1
    if a < 0:
        return -1
    return 0

def family_name(tag):
    if(tag.family() == image.TAG16H5):
        return "TAG16H5"
    if(tag.family() == image.TAG25H7):
        return "TAG25H7"
    if(tag.family() == image.TAG25H9):
        return "TAG25H9"
    if(tag.family() == image.TAG36H10):
        return "TAG36H10"
    if(tag.family() == image.TAG36H11):
        return "TAG36H11"
    if(tag.family() == image.ARTOOLKIT):
        return "ARTOOLKIT"

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.lens_corr(2)
    toUart = 0
    tags = [0, 0]
    for tag in img.find_apriltags(families=tag_families): # defaults to TAG36H11 without "families".
        img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        print_args = (family_name(tag), tag.id(), (180 * tag.rotation()) / math.pi)
        a = str(family_name(tag)) + " " + str(tag.id())
        #img.draw_string(tag.cx() - tag.w() // 2, tag.cy() + tag.h() // 2, a)
        if tag.family() == 1 and tag.id() == 2:
            tags[0] = tag
        if tag.family() == 1 and tag.id() == 3:
            tags[1] = tag
        if tag.family() == 1 and tag.id() == 4  :
            toUart = 1
    if tags[0] != 0 and tags[1] != 0:
        errorTag = tags[0].cx() + tags[1].cx() - img.width()
        errorTag = min(errorTag * sgn(errorTag), 249) * sgn(errorTag)
        print(errorTag)
        errorTag += 250;
        toUart += errorTag * 4
    if tags != [0, 0]:
        z_t0 = 0
        if tags[0] != 0:
            z_t0 = tags[0].z_translation()
        else:
            z_t0 = tags[1].z_translation()
        z_t1 = 0
        if tags[1] != 0:
            z_t1 = tags[1].z_translation()
        else:
            z_t1 = tags[0].z_translation()
        toTag = -int((z_t0 + z_t1) * 5)
        #print(toTag)
        toTag -= minToTag
        toTag = max(0, toTag)
        toUart += toTag * 2000
        toUart += 2
    print("toUart =", toUart)
    uart.write(str(int(toUart)))
    uart.write("_")
    time.sleep_ms(10)
