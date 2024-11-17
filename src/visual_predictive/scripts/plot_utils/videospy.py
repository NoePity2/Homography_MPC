import cv2
import numpy as np
import glob

def writeVideo(route,numImages):
 
    img_array = []
    for i in range(1,numImages+1):
        filename = route + str(i) + ".jpg"
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)
     
    print(len(img_array))
    out = cv2.VideoWriter('/home/noe/Desktop/plot_utils/Experimento_2_pioneer/matches_2_pioneer.mp4',cv2.VideoWriter_fourcc(*'DIVX') , 30, size)
     
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()



if __name__ == '__main__':
    imagesRoute = '/home/noe/Desktop/plot_utils/Experimento_2_pioneer/'
    writeVideo(imagesRoute,329)
