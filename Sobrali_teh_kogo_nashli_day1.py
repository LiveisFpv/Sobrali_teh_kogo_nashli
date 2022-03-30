import rospy
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from pyzbar import pyzbar
flag = False

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
#Ожидание прилета в точку 
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.3, frame_id='', auto_arm=False, tolerance=0.1):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
#Функция посадки
def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
#Для подсчета корректировки
def PIDresult(Pk,Ik,Dk,P,I,D,err,preverr,tim):
    P = err#Считаем P составляющую
    if tim<2:
        I = I+err*tim#Считаем I составляющую
    if preverr!=1000000000 and tim<2:
        D = (err-preverr)/tim#Считаем D составляющую
    out=P*Pk+I*Ik+D*Dk#Складываем состовляющие, умножая их на коэффициенты
    return out

xprob=-1
yprob=-1

endprint=""

oldtime=0
realtime=0
k=10
#Y PID
preverr=1000000000
PY=0
IY=0
DY=0
#Yaw PID
Pyaw=0
Iyaw=0
Dyaw=0
preverryaw=1000000000

#Коэффициенты PID по Y
Pk=0.001*k
Ik=0.002*k
Dk=0.00016*k

#Коэффициенты PID по Yaw
Pkyaw=0.0016*k
Ikyaw=0.002*k
Dkyaw=0.00016*k

navigate_wait(x=0, y=0, z=0.5, speed=0.6, yaw=float('nan'), frame_id='body', auto_arm=True)

rospy.init_node('computer_vision_sample')
#Инициализируем топики
#image_pub = rospy.Publisher('/debugline', Image,queue_size=1)
defect = rospy.Publisher('/defect_detect', Image,queue_size=1)
detect =rospy.Publisher('/oil_detect', Image,queue_size=1)

bridge = CvBridge()
xc=0
yc=0
number=1

linecheck=True
#Считываем QR
while not flag:
    #Получаем кадры из топика
    cv_image = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    #Декодируем его
    barcodes = pyzbar.decode(cv_image)
    if not flag:
        for barcode in barcodes:
            b_data = barcode.data.decode("utf-8")
            b_type = barcode.type
            (x, y, w, h) = barcode.rect
            xc = x + w/2
            yc = y + h/2
            #print("Found {} with data {} with center at x={}, y={}".format(b_type, b_data, xc, yc))
	    #print(b_data)
            if type(b_data) != type(None):
                flag = b_data

#Получам данные начала трубопровода              
data = flag.split()
xa, ya = data[0], data[1]
#Запоминаем начальные координаты
telem = get_telemetry(frame_id='aruco_map')
xold=telem.x
yold=telem.y
zold=telem.z

print(xa,ya)

endprint+="Navigation area x="+xa+', y='+ya+"\n"+"\n"
#Летим к началу трубопровода
navigate_wait(x=float(xa),y=float(ya),z=0.8,frame_id='aruco_map')
#Полет по линии с детектом пробоин и пятен
while True:
    #Получаем данные с камеры
    cv_image = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    #Запоминаем время для пид регулятора
    realtime=time.time()
    #Ищем трубопровод
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)# Change BGRtoHSV
    yellow = cv2.inRange(hsv, (29,50,130), (61,255,255))#цвет нефтепровода
    
    #image_pub.publish(bridge.cv2_to_imgmsg(yellow, 'mono8'))

    contours_blk, _ = cv2.findContours(yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#Search moments
    contours_blk.sort(key=cv2.minAreaRect)#Sort moments
    #Если трубопровод есть летим по нему
    if len(contours_blk) > 0:
        cnt = contours_blk[0]
        #проверка правильности полета по трубопроводу
        if cv2.contourArea(cnt) > 100:
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle
            #Определяем прямую в пространстве
            if linecheck==True:
                angle = int(math.atan2((h_min),(w_min))*180)
                linecheck=False
            #Ищем центр изображения
            xcenter = cv_image.shape[1] / 2
            ycenter = cv_image.shape[0] / 2
                
            #Ищем ошибку и считаем PID по формуле
            angle=angle*(-1)
            err=(x_min - xcenter)*(-1)
            Yout=PIDresult(Pk,Ik,Dk,PY,IY,DY,err,preverr,realtime-oldtime)
            yawout=PIDresult(Pkyaw,Ikyaw,Dkyaw,Pyaw,Iyaw,Dyaw,angle,preverryaw,realtime-oldtime)
            #Запоминаем прошлую ошибку для пид регулятора
            preverr=err
            preverryaw=angle
                
            #print("line", round(angle, 2), Yout, yawout)
                
            #детект дыр
            timemaster=time.time()
            yellow1=cv2.bitwise_not(yellow)
            contours0, _ = cv2.findContours(yellow1.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # перебираем все найденные контуры в цикле
            for cnt in contours0:
                rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
                (x_minkvad, y_minkvad), (w_minkvad, h_minkvad), angle = rect
                #print(cv2.contourArea(cnt),abs(y_minkvad-ycenter),abs(x_minkvad-xcenter))
                if cv2.contourArea(cnt)<900 and abs(y_minkvad-ycenter)<20 and abs(x_minkvad-xcenter)<20:
                    telem = get_telemetry(frame_id='aruco_map')
                    #Проверяем что не та же самая пробоина
                    if xprob==-1:
                        pass
                    elif abs(xprob-telem.x)+abs(yprob-telem.y)<0.6:
                        break
                    else:
                        pass
                    #Запоминаем ее координаты
                    xprob=telem.x
                    yprob=telem.y
                    box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
                    box = np.int0(box) # округление координат
                    cv2.drawContours(cv_image,[box],0,(180, 105, 255),3) # рисуем прямоугольник
                    defect.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
                    #ищем пятно
                    red = cv2.inRange(hsv, (0, 90, 90), (23, 255, 255))#цвет пятна
                    contours_red, _ = cv2.findContours(red.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#Search moments
                    contours_red.sort(key=cv2.minAreaRect)#Sort moments
                    S=0
                    #Отправляем дыру в топик
                    detect.publish(bridge.cv2_to_imgmsg(red, 'mono8'))
                    cv_image1=cv_image
                    if len(contours_red)>0:
                        cnt = contours_red[0]
                        if cv2.contourArea(cnt)>10:
                            #Ищем площадь пятна
                            kof=0.1/w_min
                            S=cv2.contourArea(cnt)*kof*kof
                            rect = cv2.minAreaRect(cnt)
                            box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
                            box = np.int0(box) # округление координат
                            cv2.drawContours(cv_image1,[box],0,(255, 0, 0),3) # рисуем прямоугольник
                            detect.publish(bridge.cv2_to_imgmsg(cv_image1, 'bgr8'))
                    print("defect:",str(telem.x),str(telem.y))
                    if S!=0:
                        print("oil area:",S)
                    endprint+=str(number)+". "+"x="+str(telem.x)+", y="+str(telem.y)+", S="+str(S)+"\n"
                    number+=1
            timemaster2=time.time()
            realtime+=timemaster2-timemaster
                
            #Полет по нефтепроводу

            navigate(x=0.3, y=Yout, z=0, speed=max(abs(Yout),0.5), yaw=float('nan'), yaw_rate=yawout, frame_id='body', auto_arm=False)
    else:
        break
    #Запоминаем время для пид регулятора    
    oldtime=time.time()
#print("Return to start")
print(endprint)
#Летим на место взлета и сажаем коптер
navigate_wait(x=xold,y=yold,z=zold,frame_id="aruco_map",tolerance=0.05)
land_wait()