import GPS

GPS = GPS.U_blox()
for ind in range(0, 10):
	GPS.enable_posllh()


buffer = GPS.bus.xfer2([100])
for byt in buffer:
	GPS.scan_ubx(byt)
	if(GPS.mess_queue.empty() != True):
		GPS_data = GPS.parse_ubx()



if GPS_data is not None:
	print ("Location:", "{:+3.6f}".format(GPS_data.lat/10000000.0), "{:+3.6f}".format(GPS_data.lon/10000000.0), "{:+4.1f}".format(GPS_data.heightSea/1000.0))
	print ("Loc Accuracy:", "{:+3.3f}".format(GPS_data.horAcc/1000.0), "{:+3.3f}".format(GPS_data.verAcc/1000.0))