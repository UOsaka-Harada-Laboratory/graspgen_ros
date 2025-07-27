FROM nvcr.io/nvidia/pytorch:23.07-py3 AS graspgen_ros_base

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ENV TORCH_CUDA_ARCH_LIST="8.6"
ENV LANG=en_US.UTF-8
ENV TZ=Asia/Tokyo

# ---- Locale & System Dependencies ----
RUN apt update && apt install -y --no-install-recommends \
    locales \
    tzdata \
    software-properties-common \
    lsb-release \
    build-essential \
    cmake \
    ninja-build \
    libgl1-mesa-glx \
    libglu1-mesa-dev \
    libglfw3 \
    libxext6 \
    libsm6 \
    libglx-mesa0 \
    x11-utils \
    mesa-utils \
    libopenblas-dev \
    libomp-dev \
    libssl-dev \
    libffi-dev \
    libglib2.0-dev \
    libbz2-dev \
    liblzma-dev \
    libncurses5-dev \
    libncursesw5-dev \
    zlib1g-dev \
    libreadline-dev \
    libsqlite3-dev \
    libgdbm-dev \
    libnss3-dev \
    libc6-dev \
    libfcl-dev \
    libboost-python-dev \
    libosmesa6-dev \
    curl \
    gnupg2 \
    wget \
    git \
    vim \
    byobu \
    git-lfs \
    python3-pip \
    python3-tk \
    && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS2 Humble Install ----
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS2 bash settings ----
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-futa" >> ~/.bashrc

# ---- Python Packages ----
RUN pip install --upgrade pip && \
    pip install --upgrade setuptools==68.2.2 setuptools-scm==8.0.4 wheel ninja numpy==1.23.5

# ---- PyTorch Geometric ----
RUN pip install \
    torch==2.1.0 \
    torchvision==0.16.0 \
    torch-cluster -f https://data.pyg.org/whl/torch-2.1.0+cu121.html && \
    pip install \
    torch-scatter -f https://data.pyg.org/whl/torch-2.1.2+cu121.html

# ---- GraspGen dependencies ----
RUN pip install \
    diffusers==0.11.1 \
    timm==1.0.15 \
    huggingface-hub==0.25.2 \
    pyrender==0.1.45 \
    PyOpenGL==3.1.0 \
    pyglet==1.5.11 \
    imageio \
    pickle5 \
    opencv-python \
    python-fcl \
    qpsolvers[clarabel] \
    yourdfpy==0.0.56 \
    trimesh==4.5.3 \
    webdataset \
    objaverse==0.1.7 \
    addict \
    yapf==0.40.1 \
    tensorboardx \
    sharedarray \
    torch-geometric \
    h5py \
    hydra-core \
    matplotlib \
    meshcat \
    scikit-learn \
    scipy \
    tensorboard \
    scene-synthesizer[recommend]

# ---- Build Manifold ----
RUN mkdir -p /install/Manifold && \
    cd /install && git clone --recursive -j8 https://github.com/hjwdzh/Manifold.git && \
    mkdir -p Manifold/build && cd Manifold/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
ENV PATH="${PATH}:/install/Manifold/build/"

# ---- GraspGen Clone ----
WORKDIR /ros2_ws/src
RUN git clone https://github.com/NVlabs/GraspGen.git --recursive
RUN cd /ros2_ws/src/GraspGen/ && touch COLCON_IGNORE
RUN git clone https://huggingface.co/adithyamurali/GraspGenModels --recursive && \
    mkdir GraspGen/models && mv GraspGenModels/* GraspGen/models/
RUN cd /ros2_ws/src/GraspGenModels/ && touch COLCON_IGNORE 

# ---- pointnet2_ops build ----
WORKDIR /ros2_ws/src/GraspGen/pointnet2_ops
RUN pip install .

# ---- GraspGen install ----
WORKDIR /ros2_ws/src/GraspGen
RUN pip install -e .

# ---- Python dependencies ----
RUN pip install spconv-cu120
RUN pip uninstall -y numpy scikit-learn
RUN pip install numpy==1.24.4
RUN pip install --no-cache-dir scikit-learn

# ---- Developer convenience ----
WORKDIR /ros2_ws/

RUN echo "cd /ros2_ws/src/graspgen_tutorials/script && python obj2stl.py /ros2_ws/src/GraspGen/assets/.../XXX.obj /share/tool_mesh/XXX.stl" >> ~/.bash_history
RUN echo "cd /ros2_ws/src/graspgen_tutorials/script && python stl2pcjson.py /share/obj_mesh/XXX.stl /share/pcjson/XXX.json" >> ~/.bash_history

RUN echo "byobu" >> ~/.bash_history
RUN echo "colcon build --symlink-install --executor sequential" >> ~/.bash_history
RUN echo "meshcat-server" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials mesh_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_mesh_antipodal_generator.yaml" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials mesh_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_mesh_suction_generator.yaml" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials pointcloud_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_pointcloud_antipodal_generator.yaml" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials pointcloud_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_pointcloud_antipodal_generator.yaml" >> ~/.bash_history
RUN echo "ros2 service call /generate_grasp std_srvs/srv/Empty" >> ~/.bash_history
