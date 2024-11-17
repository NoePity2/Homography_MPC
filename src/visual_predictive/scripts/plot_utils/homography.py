import cv2
import numpy as np

from matplotlib import pyplot, cm, colors, lines, animation
import matplotlib.pyplot as plt
import scipy.misc, scipy.io, scipy.optimize, scipy.cluster.vq, scipy.spatial

def drawMatches(img1, kp1, img2, kp2, matches,mask):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.8
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated 
    keypoints, as well as a list of DMatch data structure (matches) 
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    # Create the output image
    # The rows of the output are the largest between the two images
    # and the columns are simply the sum of the two together
    # The intent is to make this a colour image, so make this 3 channels
    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1[:,:,0], img1[:,:,1], img1[:,:,2]])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2[:,:,0], img2[:,:,1], img2[:,:,2]])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat, trueMatch in zip(matches,mask):
        if(trueMatch):
            # Get the matching keypoints for each of the images
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx

            # x - columns
            # y - rows
            (x1,y1) = kp1[img1_idx].pt
            (x2,y2) = kp2[img2_idx].pt

            # Draw a small circle at both co-ordinates
            # radius 4
            # colour blue
            # thickness = 1
            cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
            cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

            # Draw a line in between the two points
            # thickness = 1
            # colour blue
            cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255,0,0), 1)


    # Show the image
    cv2.imshow('Matched Features', out)
    cv2.waitKey(0)
    cv2.destroyWindow('Matched Features')

    # Also return the image if you'd like a copy
    return out

def GetHomographyORB(img1,img2,nMinPointsToEstimateConstraint=10,MAX_FEATURES = 1000,GOOD_MATCH_PERCENT = 0.15,ransacReprojThreshold=3.0):

    # Detect ORB features and compute descriptors.
    orb = cv2.ORB(MAX_FEATURES,WTA_K=2)
    kp1,descriptors1 = orb.detectAndCompute(img1,None)
    kp2,descriptors2 = orb.detectAndCompute(img2,None)

    #Match features.
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = matcher.match(descriptors1,descriptors2)
    
    # Sort matches by score
    matches.sort(key=lambda x: x.distance, reverse=False)

    # Remove not so good matches
    numGoodMatches = int(len(matches)*GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]
    
    print "num of matches = {}".format(len(matches))
    index_match_qpoints = []
    if (len(matches)>nMinPointsToEstimateConstraint):

        # Extract location of good matches
        points1 = np.zeros((len(matches),2),dtype=np.float32)
        points2 = np.zeros((len(matches),2),dtype=np.float32)

        for i, match in enumerate(matches):
            index_match_qpoints.append(match.trainIdx)
            points1[i,:] = kp1[match.queryIdx].pt
            points2[i,:] = kp2[match.trainIdx].pt
        
        # Find homography
        h, mask = cv2.findHomography(points1,points2,cv2.RANSAC,ransacReprojThreshold)

        return h, True, mask, points1, points2, kp1, kp2, index_match_qpoints
    else:
        return np.eye(3), False, 0, points1, points2, kp1, kp2, index_match_qpoints


import cv2
import numpy as np
import glob

def writeVideo(route,numImages):
 
    img_array = []
    for i in range(2,numImages):
        filename = route + str(i) + ".jpg"
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)
     
    out = cv2.VideoWriter('/home/user/pymanoid/examples/videos/rectified/rectified.mp4',cv2.cv.CV_FOURCC(*'DIVX'), 30, size)
     
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()


def main_write_videos():
    imagesRoute = '/home/user/pymanoid/examples/videos/rectified/image_robot_draw_rectified_'
    writeVideo(imagesRoute,353)

def main_wraping_images_homography():
    #Read reference image.
    refFilename = "images/image_robot_293.jpg"
    imReference = cv2.imread(refFilename,cv2.IMREAD_COLOR)
    cv2.imshow("imReference",imReference)

    # # Read current image.
    currFilename = "videos/occlusion/image_robot_draw_real_occlusion_2.jpg"
    imCurrent    = cv2.imread(currFilename,cv2.IMREAD_COLOR)
    cv2.imshow("imCurrent",imCurrent)
    h, flag, mask, points1, points2  = GetHomographyORB(imReference,imCurrent,False)
    cv2.waitKey(0)
    height, width, channels = imReference.shape

    imCurrReg = cv2.warpPerspective(imReference,h,(width,height))
    cv2.imshow("Aligned",imCurrReg)
    cv2.imwrite("Aligned.jpg",imCurrReg)

    
    pts  = np.float32([ [0,0],[0,height-1],[width-1,height-1],[width-1,0] ]).reshape(-1,1,2)
    dst  = cv2.perspectiveTransform(pts,np.matrix(h))
    cv2.polylines(imCurrent,[np.int32(dst)], True, (0, 255, 0), thickness=4)
    cv2.imshow("Aligned_2",imCurrent)
    cv2.imwrite("Aligned_2.jpg",imCurrent)
    cv2.waitKey(0)


def projectPointsWithHomography(points,Hc_to_r):
    pc  = np.zeros(points.shape)
    prh = cv2.convertPointsToHomogeneous(np.float32(points))
    prh = np.reshape( prh,(prh.shape[0],3) )
    Hr_to_c = np.linalg.inv(Hc_to_r)
    for i in range(0,points.shape[0]):
        prhT    = np.transpose(prh[i])
        pch     = np.dot(Hr_to_c,prhT)
        pc[i,0] = pch[0]/pch[2]
        pc[i,1] = pch[1]/pch[2]
   
    return pc

import matplotlib.pyplot as plt

def  drawPointsTwoImagesInOne(img1, kp1, img2, kp2):


    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    # Create the output image
    # The rows of the output are the largest between the two images
    # and the columns are simply the sum of the two together
    # The intent is to make this a colour image, so make this 3 channels
    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1[:,:,0], img1[:,:,1], img1[:,:,2]])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2[:,:,0], img2[:,:,1], img2[:,:,2]])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for i in range(0,len(kp1)):
        # x - columns
        # y - rows
        (x1,y1) = kp1[i,:]
        (x2,y2) = kp2[i,:]

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        #cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255,0,0), 1)

    # Also return the image if you'd like a copy
    return out

def drawPointsInImage(img1, kp1, color=(255,0,0)):
    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]

    # Create the output image
    # The rows of the output are the largest between the two images
    # and the columns are simply the sum of the two together
    # The intent is to make this a colour image, so make this 3 channels
    out = np.zeros((rows1,cols1,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1[:,:,0], img1[:,:,1], img1[:,:,2]])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for i in range(0,len(kp1)):
        # x - columns
        # y - rows
        (x1,y1) = kp1[i,:]

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, color, 1)   

    # Also return the image if you'd like a copy
    return out

def getNotMatchingAndMatchingPoints(points,index_match_points):
    no_matches = []
    matches    = []
    for i in range(points.shape[0]): 
        if not(i in index_match_points):
            no_matches.append(points[i,:])
        else:
            matches.append(points[i,:])
   
    return  np.array(no_matches), np.array(matches)

def gaussianDistance( X, sigma = 1.0 ):
    """
    Create an adjacency using gaussian distance as the neigborhood similarity measure
    """
    m           = np.shape( X )[0]
    adjacency   = np.zeros((m, m)) # adjacency matrix
    
    for i in range( 0, m ):
        for j in range( 0, m ):
            if i >= j: # since it's symmetric, just assign the upper half the same time we assign the lower half
                continue 
            adjacency[j, i] = adjacency[i, j] = sum( (X[i] - X[j])**2 )

    adjacency = np.exp( -adjacency / (2 * sigma ** 2) ) - np.identity( m )
    
    return adjacency
    
def degreeMatrix( adjacency ):
    """
    Create a degree matrix which is actually sum of each columns of the adjacency matrix
    Later on, this 'vector' can be converted to diagonal matrix using numpy.diag() function
    """
    return adjacency.sum( axis=1 )


from sklearn.cluster import SpectralClustering
from matplotlib import cm
from matplotlib.colors import ListedColormap, LinearSegmentedColormap



def spectralClusteringManually(X,K=1):
    # from dataset, create adjacency, degree, and laplacian matrix
    adjacency = gaussianDistance( X, sigma=0.1 )
    degree    = degreeMatrix( adjacency )
    L         = np.diag(degree) - adjacency
    
    # perform whitening on the Laplacian matrix
    #deg_05    = diag( degree  ** -1)
    #L         = deg_05.dot( L ).dot( deg_05 )
    
    
    # use eig to obtain eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eig( L )
    eigenvalues = np.real(eigenvalues)
    
    # Sort the eigenvalues ascending, the first K zero eigenvalues represent the connected components
    idx          = eigenvalues.real.argsort()
    eigenvalues.sort()
    evecs        = eigenvectors[:, idx]
    eigenvectors = evecs[:, 0:K]
    print ("evecs = {}".format(evecs.shape))
    print ("eigenvectors = {}".format(eigenvectors.shape))
    
    color_array = ['b', 'r', 'g', 'y']

    fig = pyplot.figure( figsize=(15, 5) )
    fig.canvas.set_window_title( 'Spectral Clustering' )
    
    # First perform the normal K-means on the original dataset and plot it out
    centroids, labels = scipy.cluster.vq.kmeans2( X, K )
    data = np.c_[X, labels]
    ax = fig.add_subplot( 121 )
    ax.set_title('K means clustering')
    for k in range( 0, K ):
        ax.scatter( data[data[:, 2]==k, 0], data[data[:, 2]==k, 1], c=color_array[k], marker='o')

    # Then we perform spectral clustering, i.e. K-means on eigenvectors
    centroids, labels = scipy.cluster.vq.kmeans2( eigenvectors.astype(float), K )
    data = np.c_[X, labels]
    ax = fig.add_subplot( 122 )
    ax.set_title('Spectral clustering')
    for k in range( 0, K ):
        ax.scatter( data[data[:, 2]==k, 0], data[data[:, 2]==k, 1], c=color_array[k], marker='o')

    pyplot.show()

def sklearnSpectralClustering(data,K_=1):
        # number of clusters we expect
    K = K_
    X = data
    #model  = SpectralClustering(n_clusters=K,affinity="nearest_neighbors")
    model  = SpectralClustering(n_clusters=4,affinity="rbf",gamma=0.001)
    labels = model.fit_predict(X)
    #plt.figure(2)
    #plt.scatter(X[:, 0], X[:, 1], c=labels, s=100)
    #plt.show()
    return min(X[labels==0][:,0]), max(X[labels==0][:,0]), X[labels==0][:,0],min(X[labels==0][:,1]), max(X[labels==0][:,1])


def getSideOfTarget(umin=0,umax=0,umatched=None):
    N  = len(umatched)
    d1 = (sum(umatched-umin))/N
    d2 = (sum(umatched-umax))/N

    if(abs(d1)<abs(d2)):
        return 'left'
    else:
        return 'right'


if __name__ == "__main__":
    #Read reference image.
    refFilename  = "image_robot_293.jpg"
    imReference  = cv2.imread(refFilename,cv2.IMREAD_COLOR)
    #cv2.imshow("imReference",imReference)

    # # Read current image.
    currFilename = "image_robot_66.jpg"
    imCurrent    = cv2.imread(currFilename,cv2.IMREAD_COLOR)
    #cv2.imshow("imCurrent",imCurrent)
    imClose      = np.zeros((imCurrent.shape[0],imCurrent.shape[1],1), np.uint8)
    Hc_to_r, flag, mask, pc, pr, kpc, kpr, non_match_index  = GetHomographyORB(imCurrent,imReference)
    #print ("pc = {}".format(pc.shape))


    pr_detected               = np.asarray([[p.pt[0], p.pt[1]] for p in kpr])
    pr_non_matched,pr_matched = getNotMatchingAndMatchingPoints(pr_detected,non_match_index)
    pr_projected_non_matched  = projectPointsWithHomography(pr_non_matched,Hc_to_r)
    pr_projected_matched      = projectPointsWithHomography(pr_matched,Hc_to_r)
    pr_projected_detected     = projectPointsWithHomography(pr_detected,Hc_to_r)
    for i in range(pr_projected_matched.shape[0]):
        p = pr_projected_matched[i,:]
        cv2.circle(imClose,(int(p[0]),int(p[1])),50,255,-1)
    count = 0
    for i in range(pr_projected_non_matched.shape[0]):
        p = pr_projected_non_matched[i,:]
        u = int(p[0])
        v = int(p[1])
        if u>0 and v>0 and u<imCurrent.shape[0] and v<imCurrent.shape[1] and imClose[u][v]==0:
            count=count+1
    pr_projected_non_matched_isolated = np.zeros((count,2))
    count = 0
    for i in range(pr_projected_non_matched.shape[0]):
        p = pr_projected_non_matched[i,:]
        u = int(p[0])
        v = int(p[1])
        if u>0 and v>0 and u<imCurrent.shape[0] and v<imCurrent.shape[1] and imClose[u][v]==0:
            pr_projected_non_matched_isolated[count,:] = pr_projected_non_matched[i,:]
            count=count+1

    #print("pc_projected_non_matches = ",pr_projected_non_matched.shape)
    #print("pr_projected_matched     = ",pr_projected_matched.shape)
    #print("pr_projected_detected    = ",pr_projected_detected.shape)

    plt.plot(pr_projected_non_matched_isolated[:,0],pr_projected_non_matched_isolated[:,1],'b+')
    plt.plot(pr_projected_matched[:,0],pr_projected_matched[:,1],'r+')
    
    plt.show()
    #img_reference = drawPointsInImage(imReference,pr_detected,(255,0,0))
    #img_reference = drawPointsInImage(img_reference,pr,(255,0,255))
    #cv2.imwrite("Reference_image.jpg",img_reference)

    # img_current   = drawPointsInImage(imCurrent,pr_projected_non_matched,(255,0,0))
    # img_current   = drawPointsInImage(img_current,pr_projected_matched,(255, 0, 0))
    # img_current   = drawPointsInImage(img_current,pc,(255, 0, 255))
    # cv2.imwrite("current_image.jpg",img_current)

    u_1, u_2, cluster1, v_1, v_2 = sklearnSpectralClustering(pr_projected_non_matched_isolated[0:500],2)
    print "(",u_1,",",u_2,")", "(",v_1,",",v_2,")"
    
    side = getSideOfTarget(u_1,u_2,pr_projected_matched[:,0])
    print("The target is on {}".format(side))
    
    

