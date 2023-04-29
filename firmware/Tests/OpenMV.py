import sensor, image, time
from pyb import UART

uart = UART(1, 9600)

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

sensor.set_auto_exposure(False)
current_exposure_time_in_microseconds=  sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds* 1.4))

keye = 0.25

gals = (0, 100, -16, 15, 37, 68)

ruka = (10, 43, -15, 16, -128, -16)

while(True):
    #clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()
    gals_pix = [0, 0, 0, 0]
    ruka_pix = [0, 0, 0, 0]
    for g in img.find_blobs([gals], pixels_threshold=100):
        if g.pixels() > ruka_pix[0]:
            gals_pix[0] = g.pixels()
            gals_pix[1] = g.x() + g.w() // 2 - img.width() // 2
            gals_pix[2] = g.w()
            gals_pix[3] = g.rect()
    for r in img.find_blobs([ruka], pixels_threshold=100):
        if r.pixels() > ruka_pix[0]:
            ruka_pix[0] = r.pixels()
            ruka_pix[1] = r.x() + r.w() // 2 - img.width() // 2
            ruka_pix[2] = r.w()
            ruka_pix[3] = r.rect()
    tag0 = 0
    tag_rect = 0
    for tag in img.find_apriltags(families=1):
        if tag.id() == 0:
            tag0 = 1
            tag_rect = tag.rect()
    if gals_pix[3] != 0:
        img.draw_rectangle(gals_pix[3])
    if ruka_pix[3] != 0:
        img.draw_rectangle(ruka_pix[3])
    if tag_rect != 0:
        img.draw_rectangle(tag_rect)
    if gals_pix[1] != 0 and gals_pix[2] != 0 and ruka_pix[1] != 0:
        delta_gr = abs(ruka_pix[1] - gals_pix[1]) / gals_pix[2]
    else:
        delta_gr = 0
    toUart = tag0
    toUart += min(gals_pix[1], 99) * 2
    toUart += min(gals_pix[2], 99) * 200
    toUart += min(ruka_pix[2], 99) * 20000
    toUart += int(delta_gr * 100) * 2000000
    print('toUart', toUart)
    uart.write(str(int(toUart)))
    uart.write("_")
    #print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
                                    # to the IDE. The FPS should increase once disconnected.
