# 1. ros:melodic을 베이스 이미지로 설정
FROM ros:melodic


# 필요한 패키지 설치 (wget, git)
RUN apt update && apt install -y wget git

# 2. cmake 버전 업그레이드
RUN mkdir -p /home/lidartag_lib && cd /home/lidartag_lib && \
    wget https://github.com/Kitware/CMake/releases/download/v3.18.2/cmake-3.18.2.tar.gz && \
    tar -xvf cmake-3.18.2.tar.gz && \
    cd cmake-3.18.2 && \
    ./bootstrap && \
    make && \
    make install

# 3. nlopt 라이브러리 설치를 위한 기본 세팅
RUN apt update && \
    apt install -y python3 python3-pip && \
    pip3 install numpy && \
    pip3 install swig

# 4. nlopt 라이브러리 설치
RUN cd /home/lidartag_lib && \
    git clone https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    mkdir build && \
    cd build && \
    cmake .. -DNLOPT_OCTAVE=Off -DNLOPT_MATLAB=Off -DNLOPT_GUILE=Off && \
    make && \
    make install

# 5. tbb 라이브러리 설치
RUN cd /home && \
    git clone https://github.com/wjakob/tbb && \
    cd tbb && \
    cd build && \
    cmake .. && \
    cmake --build . --config Release -- -j 6 && \
    make install

# 6. 연관된 ros 패키지 설치
RUN apt update && \
    apt install -y ros-melodic-tf && \
    apt install -y ros-melodic-velodyne-pointcloud

# 시스템 재부팅은 Dockerfile에서 할 수 없으므로 생략합니다.

# 완료된 이미지 설정
WORKDIR /home/lidartag_lib

