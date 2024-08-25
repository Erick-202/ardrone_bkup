#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
import time


# -------------------CENRARSE en la carretera usando LINEAS   ->  ya

#----->     ESte programa esta basado en el de moment_align pero queremos que use la idea del andres de los 2 rectangulos

# Calibrar area y thresh antes de usar

kernel = np.ones((3, 3), np.uint8)

cam_w = 640
cam_h = 480

global ang_anterior, horiz_anterior
ang_anterior = None
horiz_anterior = None

estado = 0

threshValue = 0

areaMin =10000
areaMax = 100000

global hz
hz = 400

#Estas son las dimensiones de la ROI 1
x1, y1 = cam_w // 2 - 100 , 0
x2, y2 = cam_w // 2 + 100, 3* cam_h // 6

#Estas son las dimensiones de la ROI 2
i1, j1 = cam_w // 2 - 100 , 3 * cam_h // 6
i2, j2 = cam_w // 2 + 100, cam_h



#EStas compensaciones son para enviar el centro real del frame al master
comp_x = (cam_w - (x2-x1))//2
comp_y = 1 * cam_h // 6

global cX_master, cY_master
cX_master = cam_w//2    #El dato que se va a enviar al master
cY_master = cam_h//2

#EStas compensaciones son para enviar el centro real del frame al master
comp_i = (cam_w - (i2-i1))//2


comp_j = 3* cam_h // 6
global cI_master, cJ_master
cI_master = cam_w//2    #El dato que se va a enviar al master
cJ_master = cam_h//2


def image_callback(data):
    global cam_w, cam_h, ang_anterior, estado, cX_master , cY_master, cI_master, cJ_master, horiz_anterior
    global frame, thresh
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # Process image
    frame = current_frame

    # Output debugging information to the terminal
    # rospy.loginfo("receiving video frame")

    cX = cam_h // 2
    cY = cam_w // 2

    # Bordes 4-------------------------------------------------------------------------
    # Convertir a HSV
    # cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    """
    x1, y1 = 0, cam_h // 3
    x2, y2 = cam_w,  2 * cam_h // 3

    """

    if estado == 0:
        #print("Estado 0")
        gray0 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, thresh0 = cv2.threshold(gray0, threshValue, 255, cv2.THRESH_BINARY_INV)
        eroded_maskBlack0 = cv2.erode(thresh0, kernel, iterations=2)
        _, contornos0, _ = cv2.findContours(eroded_maskBlack0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contornos0 != None:

            for c in contornos0:
                # print (c.dtype)
                area = cv2.contourArea(c)
                # print  area
                if area in range(areaMin, areaMax):
                    M = cv2.moments(c)
                    if (M["m00"] == 0): M["m00"] = 1
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M['m01'] / M['m00'])

                    #Centro dinamico
                    cv2.circle(frame, (cX, cY), 7, (0, 255, 0), 5)

                    #Centro estatico
                    cv2.circle(frame, (cam_w//2, cam_h//2), 7, (0, 0, 255), 5)

                    #Diferencia
                    cv2.line(frame, (cX, cY), (cam_w//2, cam_h//2), (0, 255, 255), 3)

                    #Rango aceptable
                    cv2.line(frame, (cam_w//2 - 100, 0), (cam_w//2 - 100, cam_h), (255, 255, 0), 3)
                    cv2.line(frame, (cam_w // 2 + 100, 0), (cam_w // 2 + 100, cam_h), (255, 255, 0), 3)


                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(frame, '{},{}'.format(cX, cY), (cX + 10, cY), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(frame, str(area), (cX, cY + 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

                    nuevoContorno = cv2.convexHull(c)
                    cv2.drawContours(frame, [nuevoContorno], 0, (255, 0, 0), 3)

                    #NO enviar datos repetidos
                    if horiz_anterior is None or abs(cX - horiz_anterior) >= 1:
                        #print (str(cX))
                        # enviando a nodo maestro
                        enviar_dist_horiz(cX)

                    if cX in range(cam_w//2 - 100, cam_w//2 + 100):
                        estado = 1



    # ----------------------------------------------------------------------------------------------------------------

    elif estado == 1:

        roi1 = frame[y1:y2, x1:x2]

        roi2 = frame[j1:j2, i1:i2]

        gray1 = cv2.cvtColor(roi1, cv2.COLOR_BGR2GRAY)
        ret, thresh1 = cv2.threshold(gray1, threshValue, 255, cv2.THRESH_BINARY_INV)  # 120 en el gymnasio, 50 en el cap

        gray2 = cv2.cvtColor(roi2, cv2.COLOR_BGR2GRAY)
        ret, thresh2 = cv2.threshold(gray2, threshValue, 255, cv2.THRESH_BINARY_INV)  # 120 en el gymnasio, 50 en el cap

        eroded_maskBlack1 = cv2.erode(thresh1, kernel, iterations=2)

        blur1 = cv2.medianBlur(eroded_maskBlack1, 9)

        eroded_maskBlack2 = cv2.erode(thresh2, kernel, iterations=2)

        blur2 = cv2.medianBlur(eroded_maskBlack2, 9)

        cv2.rectangle(roi1, (0, 0), (x2 - x1, y2 - y1), (255, 255, 0), 3)

        #para que el angulo me quede 1/1 y no 0/0, lo inicializo asi
        cX = cam_w//2
        cI = cam_w//2
        cY = cam_h//2
        cJ = cam_h//2

        _, contornos1, _ = cv2.findContours(eroded_maskBlack1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contornos1 == None:
            cX_master = cam_w // 2
            cY_master = 1 * cam_w // 6

        else:

            for c in contornos1:
                # print (c.dtype)
                area1 = cv2.contourArea(c)
                #print  area
                if area1 > areaMin and area1 < areaMax:
                    M1 = cv2.moments(c)
                    if (M1["m00"] == 0): M1["m00"] = 1
                    cX = int(M1["m10"] / M1["m00"])
                    cY = int(M1['m01'] / M1['m00'])

                    cv2.circle(roi1, (cX, cY), 7, (0, 255, 0), 3)
                    #cv2.line(roi1 (0, cY), (cam_w, cY), (0, 255, 255), 3)

                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(roi1, '{},{}'.format(cX, cY), (cX + 10, cY), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(roi1, str(area1), (cX, cY + 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

                    nuevoContorno1 = cv2.convexHull(c)
                    cv2.drawContours(roi1, [nuevoContorno1], 0, (255, 0, 0), 3)


                    if cX == 0:
                        cX_master = cam_w//2
                        cY_master = cam_h//6

                    else:
                        cX_master = cX + comp_x
                        cY_master = cY + comp_y


        _, contornos2, _ = cv2.findContours(eroded_maskBlack2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contornos2 == None:
            cI_master = cam_w//2
            cJ_master = 5*cam_w//6

        else:

            for c2 in contornos2:
                # print (c.dtype)
                area2 = cv2.contourArea(c2)
                #print  area
                if area2 > areaMin and area2 < areaMax:
                    M2 = cv2.moments(c2)
                    if (M2["m00"] == 0): M2["m00"] = 1
                    cI = int(M2["m10"] / M2["m00"])
                    cJ = int(M2['m01'] / M2['m00'])

                    cv2.circle(roi2, (cI, cJ), 7, (0, 255, 0), 3)
                    #cv2.line(roi1 (0, cY), (cam_w, cY), (0, 255, 255), 3)

                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(roi2, '{},{}'.format(cI, cJ), (cI + 10, cJ), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(roi2, str(area2), (cI, cJ + 30), font, 0.75, (0, 0, 255), 1, cv2.LINE_AA)

                    nuevoContorno2 = cv2.convexHull(c2)
                    cv2.drawContours(roi2, [nuevoContorno2], 0, (255, 0, 0), 3)


                    if cI == 0:
                        cI_master = cam_w//2
                        cJ_master = 5* cam_h//6

                    else:
                        cI_master = cI + comp_i
                        cJ_master = cJ + comp_j


        #angulo = math.atan() * 180/ math.pi



        try:
            angulo = math.atan((cJ_master-cY_master)/(cI_master-cX_master))

        except:
            angulo = math.pi / 2
        angulo = angulo * (180 / math.pi) * -1
        #print('ANGULOS : ', angulo)


        cv2.line(frame,(cX_master,cY_master), (cI_master,cJ_master),(255,255,0),thickness=3)
        cv2.putText(frame,str(angulo), (50 , 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA )

        #print (str(angulo))
        # enviando a nodo maestro
        if angulo != -90 and angulo != 90 and angulo != 45 and angulo != -45 and angulo != 0 and angulo != -0:
            #Solo enviar angulos diferentes
            if ang_anterior is None or abs(angulo - ang_anterior) >= 0.5:
                enviar_angulo(angulo)
                #ang_anterior = angulo
                #print(ang_anterior)

        if cX not in range(cam_w // 2 - 101, cam_w // 2 + 101):
            estado = 0

    #----------------------------------------------------------------------------------------------------------------



    #cv2.imshow("Region Of Interest 1 ", roi1)
    #cv2.imshow("ROI 2", roi2)
    #cv2.imshow("Centroid Mask" , eroded_maskBlack)
    #cv2.imshow("THresh", thresh)
    #cv2.imshow("Street Centering", frame)
    cv2.imshow("Angle Align" , frame)
    #cv2.moveWindow("Angle Align", 600, 600)


    cv2.waitKey(1)

def enviar_dist_horiz(dist_horiz):
    global horiz_anterior, hz
    pub_horiz = rospy.Publisher('align_horiz', Float64, queue_size=10)  # 2 para enviar datos mas reales
    rate = rospy.Rate(hz)  # 10hz
    # rospy.loginfo(ids)
    pub_horiz.publish(dist_horiz)
    rate.sleep()
    print("Enviado HOriz: ", str(dist_horiz))
    horiz_anterior = dist_horiz

def enviar_angulo(angulo):
    global ang_anterior, hz
    pub = rospy.Publisher('angulo', Float64, queue_size=2)
    rate = rospy.Rate(hz)
    pub.publish(angulo)
    rate.sleep()
    print('Enviado: : ', str(angulo))
    ang_anterior = angulo

def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    print ('receive msg')
    rospy.init_node('street_sub_py', anonymous=True)

    # -----  Estas publiaciones se hacen por que el master nunca lee el primer dato por alguna razon
    # enviando a nodo maestro
    pub = rospy.Publisher('angulo', Float64, queue_size=2)
    pub_horiz = rospy.Publisher('align_horiz', Float64, queue_size=2)
    rate = rospy.Rate(400)  # 10hz
    # rospy.loginfo(ids)
    pub.publish("Iniciando Comunicacion con mi Maestro ANGLE")
    pub_horiz.publish("Iniciando Comunicacion con mi Maestro HORIZONTAL")
    rate.sleep()



    # -------------------------------------------------------------

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/ardrone/bottom/image_raw', Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    receive_message()