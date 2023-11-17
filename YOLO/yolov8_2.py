from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator  # ultralytics.yolo.utils.plotting is deprecated
import pandas as pd
import time

model = YOLO('Cubes_best.pt')
camera = cv2.VideoCapture(0)
for i in range(1):
    return_value, image = camera.read()
    time.sleep(5)
    cv2.imwrite('project/captured/'+str(i)+'.png', image)
del(camera)
img=cv2.imread('project/captured/0.png')

def POINTS(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE :  
        colorsBGR = [x, y]
        print(colorsBGR)

roi=img[76:203,42:143]
results = model(source=roi,save=True,project='project',name='predict/',exist_ok=True,conf=0.8)# saves the results and saves the labels in runs/detect/predict_..
#results=model(roi)
#print(results)
save_dir='project/predict/0.png'


cap = cv2.VideoCapture(2)
#cap.set(3, 640)
#cap.set(4, 480)

while True:
    _, img = cap.read()
    frame=cv2.resize(img,(1020,500))
    roi=img[76:203,42:143] #Format[y:h,x:w]
    img = cv2.rectangle(img, (42,76), (143,203), (255, 0, 0), 2)
    # BGR to RGB conversion is performed under the hood
    # see: https://github.com/ultralytics/ultralytics/issues/2575
    results = model.predict(roi)
    a=results[0].boxes.data
    px=pd.DataFrame(a).astype("float")
    list=[]

    for index,row in px.iterrows():
        x1=int(row[0])
        y1=int(row[1])
        x2=int(row[2])
        y2=int(row[3])
        d=int(row[5])
        print(d)
        cv2.rectangle(roi,(x1,y1),(x2,y2),(0,255,0))


    for re in results:
        annotator = Annotator(img)
        boxes = re.boxes.cpu().numpy()
        ab=[]                         # get boxes on cpu in numpy
        for box in boxes:                                          # iterate boxes
            r = box.xyxy[0].astype(int)                            # get corner points as int
            Klass = re.names[int(box.cls[0])]                  # Get Class names
            print(r,Klass)
            d=r.tolist()
            ab.append(d)
            b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
            c = box.cls
            annotator.box_label(b, model.names[int(c)])
            print(d)
        boxes = re.boxes.cpu().numpy() #Get bounding boxes as images saved
        for i, box in enumerate(boxes):
            r = box.xyxy[0].astype(int)
            crop = img[r[1]:r[3], r[0]:r[2]]
        #cv2.imwrite('project/cropped/'+str(i) + ".png", crop)
        print("__________")
        """boxes = re.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
            c = box.cls
            annotator.box_label(b, model.names[int(c)])"""
          
    img = annotator.result()  
    cv2.imshow('YOLO V8 Detection', img)     
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()