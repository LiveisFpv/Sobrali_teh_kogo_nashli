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
from math import hypot,acos
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

def areacheck():
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
        return(cv2.contourArea(cnt))
    return 0


xprob=-1
yprob=-1


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


rospy.init_node('computer_vision_sample')
#Инициализируем топики
image_pub = rospy.Publisher('/debugline', Image,queue_size=1)
defect = rospy.Publisher('/defect_detect', Image,queue_size=1)
detect =rospy.Publisher('/oil_detect', Image,queue_size=1)

bridge = CvBridge()

damage = []
minangle=100

yellow_mn = [25,48,125]
yellow_mx = [65, 255, 255]
red_mn = [0, 90, 90]
red_mx = [23, 255, 255]
def image_callback(cv_image):
    xprob, yprob = -1, -1
    S = 0
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # Change BGRtoHSV
    yellow = cv2.inRange(hsv, np.array(yellow_mn), np.array(yellow_mx))  # цвет нефтепровода
    contours_yel, _ = cv2.findContours(yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Search moments
    contours_yel.sort(key=cv2.minAreaRect)  # Sort moments
    xcenter = cv_image.shape[1] / 2
    ycenter = cv_image.shape[0] / 2

    if len(contours_yel) > 0:
        cnt = contours_yel[0]
        if cv2.contourArea(cnt) > 100:
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect
            # Ищем центр изображения
            xcenter = cv_image.shape[1] / 2
            ycenter = cv_image.shape[0] / 2
            # детект дыр
            yellow1 = cv2.bitwise_not(yellow)
            contours0, _ = cv2.findContours(yellow1.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours0:
                rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
                (x_minkvad, y_minkvad), (w_minkvad, h_minkvad), angle = rect
                if 30 < cv2.contourArea(cnt) < 900:
                    telem_im = get_telemetry(frame_id='aruco_map')
                    #box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
                    #box = np.int0(box) # округление координат
                    cv2.drawContours(cv_image,contours0,0,(180, 105, 255),3) # рисуем прямоугольник
                    defect.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
                    if abs(y_minkvad - ycenter) < 20 and abs(x_minkvad - xcenter) < 20:
                        # Проверяем что не та же самая пробоина
                        xprob = telem_im.x
                        yprob = telem_im.y
                        if len(damage)>0:
                            for dam in damage:
                                x_dam, y_dam = dam[1], dam[2]
                                if math.sqrt((x_dam - xprob)**2 + (y_dam - yprob)**2) < 0.3:
                                    xprob = -1
                                    yprob = -1
                                    break
                        else:
                            pass
    #ищем пятно
    red = cv2.inRange(hsv, np.array(red_mn), np.array(red_mx)) #цвет пятна
    contours_red, _ = cv2.findContours(red.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#Search moments
    contours_red.sort(key=cv2.minAreaRect)#Sort moments
    cv_image1=cv_image
    if len(contours_red)>0:
        cnt = contours_red[0]
        if cv2.contourArea(cnt)>10:
            rect = cv2.minAreaRect(cnt)
            #box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
            #box = np.int0(box) # округление координат
            cv2.drawContours(cv_image1,contours_red,0,(255, 0, 0),3) # рисуем прямоугольник
            detect.publish(bridge.cv2_to_imgmsg(cv_image1, 'bgr8'))
            if xprob != -1 and yprob != -1:
                kof=0.1/w_min
                S=cv2.contourArea(cnt)*kof*kof
    if xprob != -1 and yprob != -1:
        count = len(damage) + 1
        damage.append((count, xprob, yprob, S))
        print("defect: {} {}".format(xprob, yprob))
        print("oil area: {}".format(S))


xc=0
yc=0
number=1
navigate_wait(x=0, y=0, z=0.7, speed=0.6, yaw=float('nan'), frame_id='body', auto_arm=True)
linecheck=True
#Считываем QR
while not flag:
    #Получаем кадры из топика
    cv_image=bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    #image_callback(cv_image)
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

#Летим к началу трубопровода
navigate_wait(x=float(xa),y=float(ya),z=0.8,frame_id='aruco_map')
#Полет по линии с детектом пробоин и пятен
oldy_min=0
oldh_min=0
razvetvlen=[]
while True:
    telem=get_telemetry(frame_id="aruco_map")
    set_position(x=telem.x,y=telem.y,z=0.8,frame_id='aruco_map',yaw=float("nan"))

    image=cv_image
    image_callback(cv_image)
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    # Находим границы
    edges = cv2.Canny(gray,100,150,apertureSize=3)
    lines=[]
    lines = cv2.HoughLinesP(
                    edges, # Input edge image
                    1, # Distance resolution in pixels
                    np.pi/180, # Angle resolution in radians
                    threshold=100, # Min number of votes for valid line
                    minLineLength=50, # Min allowed length of line
                    maxLineGap=10 # Max allowed gap between line for joining them
                    )
    lines_list=[]
    #print(lines)
    # Iterate over points
    x1=0
    x2=0
    y1=0
    y2=0
    center = cv_image.shape[1] / 2
    try:
        for points in lines:
            # Достаем координаты
            x1,y1,x2,y2=points[0]
            
    except:
        pass
    if not(x1==0 and x2==0 and y1==0 and y2==0) and linecheck==False:
        #Находим все прямые и заносим в массив
        minangle=100
        for points in lines:
            realtime=time.time()
            # Достаем координаты
            x1,y1,x2,y2=points[0]
            #Ищем углы в радианах и приводим их в нужный вид
            c=hypot(abs(x2-x1),abs(y2-y1))
            angle=acos((x2-x1)/c)
            angle=angle/3.1415*180
            w_min=abs(x2-x1)
            h_min=abs(y2-y1)
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle
            if y1>y2:
                angle=angle*(-1)
            angle=angle*(-1)
            #print(angle)
            #print(x1,x2,y1,y2,angle)
        if len(lines)>=3 and len(razvetvlen)==0:
            for points in lines:
                x1,y1,x2,y2=points[0]
                #Ищем углы в радианах и приводим их в нужный вид
                c=hypot(abs(x2-x1),abs(y2-y1))
                angle=acos((x2-x1)/c)
                angle=angle/3.1415*180
                w_min=abs(x2-x1)
                h_min=abs(y2-y1)
                if angle < -45:
                    angle = 90 + angle
                if w_min < h_min and angle > 0:
                    angle = (90 - angle) * -1
                if w_min > h_min and angle < 0:
                    angle = 90 + angle
                if y1>y2:
                    angle=angle*(-1)
                angle=angle*(-1)
                minangle=min(abs(angle),minangle)
                if 10<abs(angle)<70:
                    razvetvlen.append([x1,x2,y1,y2,angle,telem.x,telem.y,telem.yaw])
            if len(razvetvlen)>1 and minangle<10:
                x1,x2,y1,y2,angle,telemx,telemy,telemyaw=razvetvlen[-1]
                set_position(x=telemx,y=telemy,z=0.8,frame_id='aruco_map',yaw=telemyaw)
                rospy.sleep(2)
                set_position(yaw=angle/180*np.pi,yaw_rate=0.75,frame_id="body")
                rospy.sleep(2)
                navigate_wait(x=0.5,frame_id="body")
                razvetvlen.pop()
            else:
                razvetvlen=[]


    
    #Получаем данные с камеры
    cv_image=bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    #Запоминаем время для пид регулятора
    realtime=time.time()
    #Ищем трубопровод
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)# Change BGRtoHSV
    yellow = cv2.inRange(hsv, np.array(yellow_mn), np.array(yellow_mx))#цвет нефтепровода
    
    image_pub.publish(bridge.cv2_to_imgmsg(yellow, 'mono8'))

    contours_blk, _ = cv2.findContours(yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#Search moments
    contours_blk.sort(key=cv2.minAreaRect)#Sort moments
    #Если трубопровод есть летим по нему
    if len(contours_blk) > 0:
        cnt = contours_blk[0]
        #проверка правильности полета по трубопроводу
        if cv2.contourArea(cnt) > 80:
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle
            #Проверка провильности полета по нефтепроводу
            #print(h_min,oldh_min,angle)
            if abs(angle)<2 and linecheck==True and oldh_min!=0:
                if h_min-oldh_min<0:
                    set_position(yaw=3.14,frame_id="body",yaw_rate=0.75)
                    rospy.sleep(5)
                    linecheck=False
            #Определяем прямую в пространстве
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

            oldh_min=h_min
            navigate(x=0.6, y=Yout, z=0, speed=min(2,max(abs(Yout),0.6)), yaw=float('nan'), yaw_rate=min(yawout,1), frame_id='body', auto_arm=False)
            rospy.sleep(0.1)
            #Полет по нефтепроводу

            
    else:
        if len(razvetvlen)==0:
            break
        else:
            #Берем координаты
            x1,x2,y1,y2,angle,telemx,telemy,telemyaw=razvetvlen[-1]
            razvetvlen.pop()
            #Летим в начало разветвления
            navigate_wait(x=telemx,y=telemy,z=0.8,frame_id="aruco_map")
            set_position(x=telemx,y=telemy,z=0.8,frame_id='aruco_map',yaw=telemyaw)
            rospy.sleep(2)
            set_position(yaw=angle/180*np.pi,yaw_rate=0.75,frame_id="body")
            rospy.sleep(2)
            navigate_wait(x=0.5,frame_id="body")
    #Запоминаем время для пид регулятора    
    oldtime=time.time()
#print("Return to start")
print("Navigation area x={}, y={}".format(xa, ya), end="")
print()
print()
damage.sort(key=lambda x: x[0])
for dam in damage:
    count, x, y, area = dam[0], dam[1], dam[2], dam[3]
    print("{}. x={}, y={}, S={}".format(count, x, y, area), end="")
    print()
navigate_wait(z=0.4,frame_id="body")
#Летим на место взлета и сажаем коптер
navigate_wait(x=xold,y=yold,z=zold,frame_id="aruco_map",tolerance=0.05)
set_position(frame_id="body")
land_wait()