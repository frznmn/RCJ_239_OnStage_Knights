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
sensor.set_auto_exposure(False)
current_exposure_time_in_microseconds=  sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds* 0.7))
#print(current_exposure_time_in_microseconds)

gals = (41, 97, 28, 123, 21, 123)
ruka = (17, 96, -118, -37, -5, 104)

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
    gals_pix = [0, 0, 0]
    ruka_pix = [0, 0, 0, 0]
    for g in img.find_blobs([gals], pixels_threshold=30, roi=(0, img.height() // 2, img.width(), img.height() // 2), merge=False):
        if abs(g.x() + g.w() // 2 - img.width() // 2) < abs(gals_pix[0]) or gals_pix[0] == 0:
            gals_pix[0] = g.x() + g.w() // 2 - img.width() // 2
            gals_pix[1] = g.w()
            gals_pix[2] = g.rect()
    for r in img.find_blobs([ruka], pixels_threshold=30, roi=(0, 0, img.width() // 2, img.height() // 2), merge=False):
        if r.pixels() > ruka_pix[0]:
            ruka_pix[0] = r.pixels()
            ruka_pix[1] = r.cx() - img.width() // 2
            ruka_pix[2] = r.h() / r.w()
            ruka_pix[3] = r.rect()
            if gals_pix[0] != 0 and gals_pix[1] != 0 and ruka_pix[1] != 0:
                ruka_pix[1] = abs(ruka_pix[1] - gals_pix[0]) / gals_pix[1]
            else:
                ruka_pix[1] = 0
    istag = 0
    tag_rect = 0
    for tag in img.find_apriltags(families=17):
        if tag.family() == 16 and tag.id() == 6:
            istag = 1
            tag_rect = tag.rect()
        if tag.family() == 1 and tag.id() == 1:
            gals_pix[0] = tag.cx() - img.width() // 2
            gals_pix[1] = tag.w()
            gals_pix[2] = tag.rect()
    if gals_pix[2] != 0:
        img.draw_rectangle(gals_pix[2], color = (255, 0, 0))
    if ruka_pix[3] != 0:
        img.draw_rectangle(ruka_pix[3], color = (255, 0, 0))
    if tag_rect != 0:
        img.draw_rectangle(tag_rect, color = (255, 0, 0))
    udar = 0
    if ruka_pix[1] != 0 and ruka_pix[2] != 0:
        if ruka_pix[2] > 1.6:
            udar = 2
        else:
            if ruka_pix[2] < 1:
                udar = 3
            else:
                udar = 1
    print(ruka_pix[1], ruka_pix[2])
    print('udar =', udar)
    print('w =', gals_pix[1])
    toUart = istag
    toUart += min(max(gals_pix[0] + 100, 0), 199) * 2
    toUart += min(gals_pix[1], 99) * 400
    toUart += udar * 40000
    print('toUart =', toUart)
    uart.write(str(int(toUart)))
    uart.write("_")
    time.sleep_ms(10)
    #print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
                                    # to the IDE. The FPS should increase once disconnected.
