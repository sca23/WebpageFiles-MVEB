import shutil

for i in range(100):
    shutil.copy2('D:/Img/bottle.xml', 'D:/copies/bottle{}.xml'.format(i))
    shutil.copy2('D:/Img/bottle.jpg', 'D:/copies/bottle{}.jpg'.format(i))
