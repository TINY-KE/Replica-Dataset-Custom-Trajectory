+ 生成数据集
./build/ReplicaSDK/ReplicaRenderer \
/media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/mesh.ply \
/media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/textures \
/media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/glass.sur

+ 可视化
./build/bin/ReplicaViewer mesh.ply /path/to/atlases [mirrorFile]
./build/ReplicaSDK/ReplicaViewer /media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/mesh.ply /media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/textures 
./build/ReplicaSDK/ReplicaViewer /media/robotlab/WD_BLACK1/Replica-Dataset/download/hotel_0/mesh.ply /media/robotlab/WD_BLACK1/Replica-Dataset/download/hotel_0/textures 

+ 记录轨迹
    + DEMO：
    ./ReplicaCapturer mesh.ply textures [glass.sur] outFolder
    + 实例： room_1
    ./build/ReplicaSDK/write_campose \
    /media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/mesh.ply \
    /media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/textures \
    /home/robotlab/dataset/debug
    + 实例： hotel_0
    ./build/ReplicaSDK/write_campose \
    /media/robotlab/WD_BLACK1/Replica-Dataset/download/hotel_0/mesh.ply \
    /media/robotlab/WD_BLACK1/Replica-Dataset/download/hotel_0/textures \
    /home/robotlab/dataset/debug

+ 读取轨迹并保存图片：
    + 实例： room_1
    ./build/ReplicaSDK/render_from_campose \
        /media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/mesh.ply \
        /media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/textures \
        /media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/glass.sur
    + 实例： hotel_0
    ./build/ReplicaSDK/render_from_campose \
        /media/robotlab/WD_BLACK1/Replica-Dataset/download/hotel_0/mesh.ply \
        /media/robotlab/WD_BLACK1/Replica-Dataset/download/hotel_0/textures \
        /media/robotlab/WD_BLACK1/Replica-Dataset/download/hotel_0/glass.sur