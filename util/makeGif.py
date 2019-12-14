import cv2
import os


files = os.listdir('makeMovie')
files.sort()

sorted(files)
sortedList = 2744 *[None]
img_array = []
for f in files:
    figure, num = f.split("_")
    num,_ = num.split('.')
    num = int(num)
    f = 'makeMovie' + '/' + f
    sortedList[num-1] = f

print(len(sortedList))
del sortedList[::2]
print(len(sortedList))
sortedList = sortedList[::3]

for filename in sortedList:
    print(filename)
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)

out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 5, size)

for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
