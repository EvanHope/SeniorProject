from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview()
sleep(5)
camera.capture('/Navio2/SeniorProject/Quad_Code_share/Pictures/image.jpg')
camera.stop_preview()