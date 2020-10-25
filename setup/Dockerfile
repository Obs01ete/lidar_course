FROM dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt

RUN apt-get update && \
    apt-get install -y apt-utils cmake git build-essential \
        gcc-7 g++-7 htop wget vim

RUN apt-get install -y libflann-dev && \
    apt-get install -y libpng16-16 libpng-dev && \
    apt-get install -y libgl1-mesa-dev && \
    apt-get install -y libeigen3-dev

RUN mkdir /build
WORKDIR /build

RUN apt-get install -y libxt-dev
RUN wget https://www.vtk.org/files/release/8.1/VTK-8.1.0.tar.gz && \
    tar xzf VTK-8.1.0.tar.gz && \
    ( \
        cd VTK-8.1.0 && \
        mkdir build && \
        cd build && \
        cmake .. && \
        make -j8 && \
        make install \
    ) && \
    rm VTK-8.1.0.tar.gz && \
    rm -rf VTK-8.1.0

RUN apt-get install -y libgl1-mesa-dev && \
    apt-get install -y libboost-all-dev && \
    apt-get install -y freeglut3-dev pkg-config && \
    apt-get install -y libxmu-dev libxi-dev && \
    apt-get install -y mono-complete && \
    apt-get install -y openjdk-8-jdk openjdk-8-jre && \
    apt-get install -y libglew-dev && \
    apt-get install -y libqhull7 libqhull-dev

RUN git clone https://github.com/PointCloudLibrary/pcl.git && \
    ( \
        cd pcl && \
        git checkout pcl-1.11.1 && \
        mkdir build && \
        cd build && \
        cmake .. && \
        make -j8 && \
        make install \
    ) && \
    rm -rf pcl

CMD ["/bin/bash"]
