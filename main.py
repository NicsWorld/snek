import math
import random
import cvzone
import cv2
import numpy as np
from playsound import playsound
from cvzone.HandTrackingModule import HandDetector

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

detector = HandDetector(detectionCon=0.8, maxHands=1)

class SnakeGameClass:
  def __init__(self, pathFood, pathSnakeHead):
    self.points = [] # points in the snake
    self.lengths = [] # distance between each point
    self.currentLength = 0 # total length of the snake
    self.allowedLength = 150 # total allowed length of snek
    self.previousHead = 0,0 # previous coords of head point

    self.imgFood = cv2.imread(pathFood, cv2.IMREAD_UNCHANGED)
    self.snakeHead = cv2.imread(pathSnakeHead, cv2.IMREAD_UNCHANGED)
    self.hFood, self.wFood,_ = self.imgFood.shape
    self.foodPoint = 0,0
    self.score = 0
    self.gameOver = False

    self.randomFoodLocation()


  def randomFoodLocation(self):
    self.foodPoint = random.randint(100, 1000),random.randint(100, 600)   

  def update(self, imgMain, currentHead):
    if self.gameOver:
      cvzone.putTextRect(imgMain, "", [300,400], scale = 7, colorR = (170, 255, 0), colorT = (2, 0, 0), thickness=5, offset=20)
      # cvzone.putTextRect(imgMain, f'Score: {self.score}', [300,550], scale = 7, thickness=5, colorR = (170, 255, 0),colorT = (2, 0, 0), offset=20)
    else:  
      px, py = self.previousHead
      cx, cy = currentHead

      self.points.append([cx, cy])
      distance = math.hypot(cx-px, cy-py)
      self.lengths.append(distance)
      self.currentLength += distance
      self.previousHead = cx,cy

      #length reduction
      if self.currentLength > self.allowedLength:
        for i, length in enumerate(self.lengths):
          self.currentLength -= length
          self.lengths.pop(i)
          self.points.pop(i)
          if self.currentLength < self.allowedLength:
            break


      # check if snake ate the food
      rx, ry = self.foodPoint
      if rx - self.wFood // 2 < cx < rx + self.wFood//2 and ry - self.hFood // 2 < cy < ry + self.hFood // 2:
        self.randomFoodLocation()
        self.allowedLength += 50
        self.score += 1
        print(self.score)
        playsound("eat2.wav")

      # draw the snake
      if self.points:
        for i, point in enumerate(self.points):
          if i !=0:
            cv2.line(imgMain, self.points[i-1], self.points[i], (173,255,47), 20)
        # draw circle around index finger tip
        cv2.circle(imgMain, self.points[-1], 20, (173,255,47), cv2.FILLED)
        # cant quite position this snek correctly
        # imgMain = cvzone.overlayPNG(imgMain, self.snakeHead,self.points[-1])

      # draw food
      rx, ry = self.foodPoint
      imgMain = cvzone.overlayPNG(imgMain, self.imgFood,(rx-self.wFood//2, ry-self.hFood//2))

      cvzone.putTextRect(imgMain, f'Score: {self.score}', [50,675], colorR = (170, 255, 0), colorT = (2, 0, 0), scale = 3, thickness=3, offset=10)
      # draw title
      cvzone.putTextRect(imgMain, f'Snek', [640,80], colorR = (170, 255, 0), colorT = (2, 0, 0), scale = 3, thickness=3, offset=10)

      # check for collision
      pts = np.array(self.points[:-2], np.int32)
      pts = pts.reshape((-1, 1, 2))
      cv2.polylines(imgMain, [pts], False, (2,0,0), 3)
      minDist = cv2.pointPolygonTest(pts, (cx, cy), True)

      if  -1 <= minDist <=1:
        print("Hit")
        self.gameOver = True
        self.points = [] # points in the snake
        self.lengths = [] # distance between each point
        self.currentLength = 0 # total length of the snake
        self.allowedLength = 150 # total allowed length of snek
        self.previousHead = 0,0 # previous coords of head point
        self.randomFoodLocation()


    return imgMain

game = SnakeGameClass("cookieFin.png", "snek.png")

while True:
  # success, img = cap.read()
  # hands, img = detector.findHands(img)
  success, img = cap.read()
  img = cv2.flip(img, 1)
  hands, img = detector.findHands(img, flipType=False)
  if hands:
    # dictionary of fingers
    lmList = hands[0]['lmList']
    # find index finger
    pointIndex = lmList[8][0:2]
    img = game.update(img, pointIndex)
  cv2.imshow("Image", img)
  key = cv2.waitKey(1)
  if key == ord('r'):
    game.gameOver = False