class Obstacle:
    def __int__(self, cv):
        self.cv = cv

    def detect(self):
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return self.cv.Canny(imgGray)

        return
