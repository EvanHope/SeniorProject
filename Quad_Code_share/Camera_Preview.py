from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.rotation = 180
camera.start_preview()
sleep(200)
#camera.capture('/home/pi/Navio2/SeniorProject/Quad_Code_share/Pictures/image.jpg')
camera.stop_preview()