###############################  YOLO.py ##########################################

from ultralytics import YOLO
import numpy as np
import cv2
import pandas as pd
model = YOLO('Cubes_best.pt')  # load a custom model
camera = cv2.VideoCapture(0)
for i in range(1):
    return_value, image = camera.read()
    cv2.imwrite('project/captured/'+str(i)+'.png', image)
del(camera)


# Predict with the model on only 1 image
img=cv2.imread('project/captured/0.png')

def POINTS(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE :  
        colorsBGR = [x, y]
        print(colorsBGR)
        

frame=cv2.resize(img,(1020,500))
roi=frame[64:273,297:690]
results = model.predict(source=roi,save=True, save_txt=True,project='project',name='predict/',exist_ok=True)# saves the results and saves the labels in runs/detect/predict_..
#results=model(roi)
#print(results)

save_dir='project/predict/0.png'
#img=cv2.imread('test_images/test1.jpg')



for result in results:
                                          # iterate results
    boxes = result.boxes.cpu().numpy()
    ab=[]                         # get boxes on cpu in numpy
    for box in boxes:                                          # iterate boxes
        r = box.xyxy[0].astype(int)                            # get corner points as int
        Klass = result.names[int(box.cls[0])]                  # Get Class names
        print(r,Klass)
        d=r.tolist()
        ab.append(d)
        print(d)
    boxes = result.boxes.cpu().numpy() #Get bounding boxes as images saved
    for i, box in enumerate(boxes):
        r = box.xyxy[0].astype(int)
        crop = img[r[1]:r[3], r[0]:r[2]]
        cv2.imwrite('project/cropped/'+str(i) + ".png", crop)
    print("__________")
    #print(ab)  # Converts Numpy array to list.
    ##___ab are the coordinates of bbox and can be used for further calculations 
arr=np.array(ab)
#print(arr[0,0])
print("____________________________________________________________")
 

"""
#Ray_Tracking_Method
from time import time
import numpy as np
import matplotlib.path as mpltPath

# regular polygon for testing
lenpoly = 100
polygon = [[np.sin(x)+0.5,np.cos(x)+0.5] for x in np.linspace(0,2*np.pi,lenpoly)[:-1]]

# random points set of points to test 
N = 10000
points = np.random.rand(N,2)


# Ray tracing
def ray_tracing_method(x,y,poly):

    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

start_time = time()
inside1 = [ray_tracing_method(point[0], point[1], polygon) for point in points]
print("Ray Tracing Elapsed time: " + str(time()-start_time))

# Matplotlib mplPath
start_time = time()
path = mpltPath.Path(polygon)
inside2 = path.contains_points(points)
print("Matplotlib contains_points Elapsed time: " + str(time()-start_time))"""
