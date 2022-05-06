import  cv2
import  math 
import  numpy as np  


# From calibration matrix
matrix     = np.loadtxt(open("matrix.txt","rb"))
distortion = np.loadtxt(open("distortion.txt","rb"))


#arucoDict 	= cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoDict 	= cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

arucoParams = cv2.aruco.DetectorParameters_create()


cam = cv2.VideoCapture(0)
while(cam.isOpened()):

	ret, image = cam.read()

	if ret == True:

		#image   = cv2.resize(frame, None, fx=1/2, fy=1/2, interpolation=cv2.INTER_AREA)

		# Detect Aruco markers
		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
		cv2.aruco.drawDetectedMarkers(image, corners, borderColor=(0, 0, 255))

		markerSizeInCM = 0.025 
		rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, matrix, distortion)

		
		#if len(corners) > 0:  # meaning that there is at least one markers

			#for rv, tv  in zip (rvec,tvec):
				#cv2.aruco.drawAxis(image, matrix, distortion, rv, tv, 5)   


		if len(corners) > 1:  # meaning that there are at least two markers
  
			mark_1 = tvec[0]
			mark_2 = tvec[1]

			dist = np.linalg.norm(mark_1 - mark_2)
			print("Distance between markers:", dist, "CM")

		else:
			print("Need more markers")


	cv2.imshow('frame', image)
	

	if cv2.waitKey(1) == ord('q'):
		break


cam.release()
cv2.destroyAllWindows()
