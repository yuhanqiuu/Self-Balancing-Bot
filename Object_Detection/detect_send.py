import cv2 #opencv
import urllib.request #to open and read URL
import numpy as np
from send_email_utils import send_email_with_frame

#OBJECT CLASSIFICATION PROGRAM FOR VIDEO IN IP ADDRESS
sender_email="qiuyuhan66@gmail.com"
receiver_email="qiuyuhan66@gmail.com",
app_password="zklynsxlbrsvoyrv"

url = 'http://192.168.137.134/jpeg'
#url = 'http://192.168.1.6/'
winName = 'ESP32 CAMERA'
cv2.namedWindow(winName,cv2.WINDOW_AUTOSIZE)
#scale_percent = 80 # percent of original size    #for image processing

classNames = []
classFile = 'coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,240)
#net.setInputSize(480,480)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

while(1):
    imgResponse = urllib.request.urlopen (url) # here open the URL
    imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
    img = cv2.imdecode (imgNp,-1) #decodificamos

    unchanged_img = img.copy()
    
    #img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) # vertical
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #black and white
    
    frame_count = 0
    detect_every = 20  # Perform detection every 3 frames

    if (frame_count % detect_every)==0:

        classIds, confs, bbox = net.detect(img,confThreshold=0.5)
        print(classIds,bbox)

        if len(classIds) != 0:
            for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
                cv2.rectangle(img,box,color=(0,255,0),thickness = 3) #mostramos en rectangulo lo que se encuentra
                cv2.putText(img, classNames[classId-1], (box[0]+10,box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0),2)


    cv2.imshow(winName,img) #  show the picture

    #wait for ESC to be pressed to end the program
    tecla = cv2.waitKey(5) & 0xFF
    if tecla == ord('1'):
        print("Sending email...")
        send_email_with_frame(sender_email, receiver_email, app_password, image=img)
    
    if tecla == ord('2'):
        print("Sending email...")
        send_email_with_frame(sender_email, receiver_email, app_password, image=unchanged_img)

    elif tecla == 27:
        break
cv2.destroyAllWindows()

