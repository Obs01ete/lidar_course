# Instructions to configure hardware and software for the LiDAR Perception course
By Dmitrii Khizbullin

## 1. Obtain a machine

### (A) Local machine
Get Ubuntu Desktop host OS version 18.04+. NVidia or AMD GPU with OpenGL support is highly recommended.

### (B) Create an AWS EC2 instance

Select "Ubuntu Server 18.04 LTS (HVM), SSD Volume Type" image. Choose "g4dn.2xlarge" machine. Increase system disk size to 300 GB. Expose TCP port 6080 in the security group.
| Instance type| RAM | CPUs | storage | GPU | cost |
|---|---|---|---|---|---|
| g4dn.2xlarge | 8 CPU | 32 GiB | NVMe SSD | NVIDIA T4 16GB GPU | $0.838 per Hour |

## 2. Download and unpack Kitti tracking data

106 GB free space required. Download:
```
cd /home/ubuntu
mkdir git
cd git
git clone https://github.com/Obs01ete/lidar_course_setup.git
```
```
cd /home/ubuntu
mkdir kitti
cd kitti
# optionally run a TMUX session.
bash /home/ubuntu/git/lidar_course_setup/download_data.sh
```
Download may take 40 minutes. Meanwhile the docker image can be built.

Unpack downloaded zip archives:
```
unzip 'data_tracking_*.zip' -d tracking/
```

## 3. Clone sources of this course
```
cd /home/ubuntu
cd git
# The repo below will be available soon
git clone https://github.com/Obs01ete/lidar_course.git
```

## 4. Create docker image

### Install docker if not installed
https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04

### (A) Download the image from Dockerhub

Pre-built container (1.3GB):
```
docker pull dmitriikhizbullin/lidar_course:latest
```

### (B) Build the image from scratch

```
docker build --tag dmitriikhizbullin/lidar_course:local /home/ubuntu/git/lidar_course_setup
```
The process may take up to 30 minutes to build and takes up to 32 GB RAM. If the build fails due to out-of-memory while compiling PCL, reduce `-j8` to `-j4` in Dockerfile.

Once done, launch the container.
```
docker run -p 6080:80 -v /dev/shm:/dev/shm -v /home/ubuntu/git:/ws -v /home/ubuntu/kitti/:/kitti -e RESOLUTION=1600x900 dmitriikhizbullin/lidar_course:local
```
Open noVNC session in the browser `http://<your_aws_machine_dns>:6080`. Examples:
1. http://localhost:6080/
2. http://ec2-18-222-312-123.us-east-2.compute.amazonaws.com:6080/

## 5. Build and run the code for point cloud processing

In noVNC in the browser:
```
cd /ws/lidar_course/
mkdir build
cd build
cmake ..
make -j8
```

Run the application:
```
./process-sequence /kitti/tracking/training/ 0000
```

You should see an Ubuntu Desktop GUI window in noVNC browser tab. There should be an animated colored point cloud with buildings, trees, cars, cyclists and pedestrians.
