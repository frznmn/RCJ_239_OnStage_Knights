import sensor, image, time
from pyb import UART


uart = UART(1, 9600)

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
#sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

#current_exposure_time_in_microseconds=  sensor.get_exposure_us()
#sensor.set_auto_exposure(False, \
    #exposure_us = 100000)
sensor.set_contrast(3)
sensor.set_saturation(3)
sensor.set_brightness(3)
sensor.set_quality(100)
sensor.set_auto_exposure(False, exposure_us=10000)
#print(current_exposure_time_in_microseconds)

keye = 0.25

gals = (56, 100, 18, 106, 54, 123)

ruka = (25, 93, -42, -17, -4, 112)

def sgn(a):
    if(a > 0):
        return 1
    if(a < 0):
        return -1
    return 0

while(True):
    #clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()
    img.lens_corr(1.3)
    gals_pix = [0, 0, 0, 0]
    ruka_pix = [0, 0, 0, 0]
    for g in img.find_blobs([gals], pixels_threshold=100, roi=(0, img.height() // 2, img.width(), img.height() // 2)):
        if g.pixels() > ruka_pix[0]:
            gals_pix[0] = g.pixels()
            gals_pix[1] = g.x() + g.w() // 2 - img.width() // 2
            gals_pix[2] = g.w()
            gals_pix[3] = g.rect()
    for r in img.find_blobs([ruka], pixels_threshold=100, roi=(0, 0, img.width(), img.height() * 3 // 4)):
        if r.pixels() > ruka_pix[0]:
            ruka_pix[0] = r.pixels()
            ruka_pix[1] = r.x() + r.w() // 2 - img.width() // 2
            ruka_pix[2] = r.h() / r.w()
            ruka_pix[3] = r.rect()
            if gals_pix[1] != 0 and gals_pix[2] != 0 and ruka_pix[1] != 0:
                ruka_pix[1] = abs(ruka_pix[1] - gals_pix[1]) / gals_pix[2]
            else:
                ruka_pix[1] = 0
    istag = 0
    tag_rect = 0
    for tag in img.find_apriltags(families=1):
        if tag.id() == 3:
            istag = 1
            tag_rect = tag.rect()
    #if gals_pix[3] != 0:
        #img.draw_rectangle(gals_pix[3])
    if ruka_pix[3] != 0:
        img.draw_rectangle(ruka_pix[3])
    if tag_rect != 0:
        img.draw_rectangle(tag_rect)
    udar = 0
    if ruka_pix[1] != 0 and ruka_pix[2] != 0:
        if ruka_pix[2] > 1.5:
            udar = 2
        else:
            if ruka_pix[1] > 4.5:
                udar = 1
            else:
                udar = 3
    #print(ruka_pix[1], ruka_pix[2])
    print('udar =', udar)
    print('w =', gals_pix[2])
    toUart = istag
    toUart += min(max(gals_pix[1] + 100, 0), 199) * 2
    toUart += min(gals_pix[2], 99) * 400
    toUart += udar * 40000
    print('toUart =', toUart)
    uart.write(str(int(toUart)))
    uart.write("_")
    time.sleep_ms(10)
    #print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
                                    # to the IDE. The FPS should increase once disconnected.
