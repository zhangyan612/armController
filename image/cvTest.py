# import computer vision library(cv2) in this code
import cv2
 
# main code
if __name__ == "__main__" :
 
    # mentioning absolute path of the image
    img_path = "C:\\Users\\JohnSong\\Desktop\\image.jpg"
 
    # using imread() method of cv2
    image = cv2.imread(img_path)
 
    # show the image on the newly created image window
    cv2.imshow('image window',image)
