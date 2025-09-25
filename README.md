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
./ReplicaCapturer mesh.ply textures [glass.sur] outFolder
./build/ReplicaSDK/ReplicaCapturer \
/media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/mesh.ply \
/media/robotlab/WD_BLACK1/Replica-Dataset/download/room_1/textures \
/home/robotlab/dataset/debug