import cv2
import matplotlib.pyplot as plt
import pandas as pd

center = (960, 540)
dim_rescaled = (960, 540)
dim = (1920, 1080)
video = cv2.VideoCapture(0, cv2.CAP_DSHOW)
print(video.isOpened())
if not video.isOpened():
    raise ValueError
video.set(cv2.CAP_PROP_FPS, 30)
video.set(cv2.CAP_PROP_AUTOFOCUS, 0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, dim[0])
video.set(cv2.CAP_PROP_FRAME_HEIGHT, dim[1])
video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
print(video.get(cv2.CAP_PROP_EXPOSURE))
print(video.get(cv2.CAP_PROP_ISO_SPEED))
video.set(cv2.CAP_PROP_EXPOSURE, -8)
video.set(cv2.CAP_PROP_ISO_SPEED, -8)
print(video.get(cv2.CAP_PROP_EXPOSURE))
print(video.get(cv2.CAP_PROP_ISO_SPEED))
video.set(cv2.CAP_PROP_FOCUS, 0)

cap = video

FPS = 30

PADDING_FRAME_CNT = 50

values = []
times = []

for i in range(PADDING_FRAME_CNT):
    ret, frame = cap.read()

count = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print('error at %d' % count)
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mean_val = cv2.mean(v)
    values.append(mean_val[0])
    times.append(count / FPS)
    count = count + 1
    cv2.imshow("frame", cv2.resize(frame, dim_rescaled, cv2.INTER_LINEAR))
    key = cv2.waitKey(2)
    if key == ord("q"):
        video.release()
        cv2.destroyAllWindows()

#cap.release()

times = times[:-PADDING_FRAME_CNT]
values = values[:-PADDING_FRAME_CNT]
count -= PADDING_FRAME_CNT


meanv = sum(values) / len(values)
ma = max(values)
mi = min(values)
hi_cnt = 0
lo_cnt = 0
now = None
for v in values:
    if v > meanv:
        if now != 'hi':
            hi_cnt += 1
        now = 'hi'
    else:
        if now != 'lo':
            lo_cnt += 1
        now = 'lo'


real_duration = count / FPS
print('duration: %.4fs' % real_duration)
freq = ((lo_cnt + hi_cnt) / real_duration / 2)
per_diff = (ma-mi)*2/(ma+mi)*100
print('estimated frequency: %.4fhz' % freq)
print('percentage diff: %.4f%%' % per_diff)


df = pd.DataFrame({'x': times, 'y': values})

plt.figure(figsize=(20, 5))
#plt.title(VIDEO_NAME)
plt.plot('x', 'y', data=df, linestyle='-', marker='o')
plt.show()