import cv2 as cv
import numpy as np
from .common import *
from .filter import lowPassFilter

calib_path = "aruco/calib_data/matrix.npy"
#calib_path = "aruco/calib_data/MultiMatrix.npz"

FRAME_DELAY = 0

class video:
    def __init__(self) -> None:
        self.center = (960, 540)
        self.dim_rescaled = (960, 540)
        self.dim = (1920, 1080)
        self.video = cv.VideoCapture(0, cv.CAP_DSHOW)
        _status = self.video.isOpened()
        if not _status:
            raise Exception("Error: Camera Not Connected, Try Again")
        else:
            print("Camera Connected")
        self.video.set(cv.CAP_PROP_FPS, 40)
        #self.video.set(cv.CAP_PROP_AUTOFOCUS, 0)
        self.video.set(cv.CAP_PROP_FRAME_WIDTH, self.dim[0])
        self.video.set(cv.CAP_PROP_FRAME_HEIGHT, self.dim[1])
        self.video.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        _tr = (self.video.get(cv.CAP_PROP_EXPOSURE))
        _tr = (self.video.get(cv.CAP_PROP_ISO_SPEED))
        self.video.set(cv.CAP_PROP_EXPOSURE, -8)
        self.video.set(cv.CAP_PROP_ISO_SPEED, -8)
        _tr = (self.video.get(cv.CAP_PROP_EXPOSURE))
        _tr = (self.video.get(cv.CAP_PROP_ISO_SPEED))
        self.video.set(cv.CAP_PROP_FOCUS, 255)
        #self.video.set(cv.CAP_PROP_BUFFERSIZE, 1)
        #self.video.set(cv.CAP_PROP_FRAME_COUNT, 1)
        #self.video.set(cv.CAP_PROP_ISO_SPEED, 1/10000)
        #print(self.video.set(cv.CAP_PROP_CONTRAST, 240.0))

    def read(self):
        ret, frame = self.video.read()
        if not ret:
            raise Exception("Error: Unable to Read Frame")
        return frame

class arucoGPS:
    def __init__(self, state : arucoState, target_ID : int, targetAngle : lowPassFilter) -> None:
        self.debug = False

        calib = np.load(calib_path, allow_pickle=True)
        self.video = video()

        self.target_id = target_ID

        self.cam_mat = calib[0]
        self.dist_coef = calib[1]

        self.MARKER_SIZE_1 = 3.3 # cm

        self.marker_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = cv.aruco.DetectorParameters_create()

        # Dictionary to store coords of all detected markers
        self.coord_data = {}
        self.rvec = {}

        self.dronePos = state
        self.targetAngle = targetAngle

    def loop(self, target : XYZ):
        try:
            frame = self.video.read()
        except:
            return self.stop()
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Optional Threshold Filter, default thresh = 80
        #ret, gray_frame = cv.threshold(gray_frame, 100, 255, cv.THRESH_BINARY)

        # Detecting Markers
        marker_corners, marker_IDs, reject = cv.aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )

        if marker_corners:
            total_markers = range(0, len(marker_IDs))
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                rVec, tVec, _ = cv.aruco.estimatePoseSingleMarkers(marker_corners, self.MARKER_SIZE_1, self.cam_mat, self.dist_coef)
                (rVec - tVec).any()

                # Drawing Markers
                cv.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_left = corners[0].ravel()
                top_right = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left =  corners[3].ravel()

                distance = np.sqrt(tVec[i][0][0]**2 + tVec[i][0][1]**2 + tVec[i][0][2]**2)

                _t_X = round(tVec[i][0][0],1)
                _t_Y = round(tVec[i][0][1],1)
                _t_Z = round(tVec[i][0][2],1)
                self.coord_data[ids[0]] = [_t_X, _t_Y, _t_Z]
                self.rvec[ids[0]] = rVec[i][0]

                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                cv.putText(
                    frame,
                    f"x:{_t_X} y: {_t_Y} ",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
            
            if (self.target_id in self.coord_data):# and (self.ground_id in self.coord_data):
                t_id = self.target_id
                self.dronePos.update(self.coord_data[self.target_id])

                if self.debug : print("Coordinates", self.coord_data[self.target_id])

                tar_rot_mtx = cv.Rodrigues(self.rvec[t_id])[0].reshape((3,3))
                rets = cv.RQDecomp3x3(tar_rot_mtx)
                self.targetAngle.update(rets[0][2] + 180) 

        cv.imshow("frame", cv.resize(frame, self.video.dim_rescaled, cv.INTER_LINEAR))
        #cv.imshow("gray", cv.resize(gray_frame, self.video.dim_rescaled, cv.INTER_LINEAR))
        key = cv.waitKey(2)
        if key == ord("q"):
            return self.stop()
        return 0

    def stop(self):
        self.video.video.release()
        cv.destroyAllWindows()
        return 1


if __name__ == "__main__":
    state = arucoState()
    aruco = arucoGPS(state)
    while True:
        _err = aruco.loop()