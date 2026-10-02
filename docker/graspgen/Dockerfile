FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ENV TORCH_CUDA_ARCH_LIST="12.0"

# Use the CUDA version shipped with the PyTorch cu128 wheel so local CUDA
# extensions, including pointnet2_ops, can be compiled for RTX 50 (sm_120).
# Remove forward-compatibility libraries; the host driver is newer than CUDA 12.8.
RUN rm -rf /usr/local/cuda/compat
ENV LANG=en_US.UTF-8
ENV TZ=Asia/Tokyo
ENV PIP_BREAK_SYSTEM_PACKAGES=1
# Note: do NOT set PIP_IGNORE_INSTALLED globally. That causes pip to re-resolve
# every transitive dep (e.g. torch) from the latest upstream, breaking carefully
# pinned installs. Add `--ignore-installed` per-line where apt-managed packages
# (zipp, six, numpy, ...) without RECORD files cause uninstall errors.

# ---- Locale & System Dependencies ----
RUN apt update && apt install -y --no-install-recommends \
    locales \
    tzdata \
    software-properties-common \
    lsb-release \
    build-essential \
    cmake \
    ninja-build \
    libgl1 \
    libglx-mesa0 \
    libxrender1 \
    libxext6 \
    libsm6 \
    mesa-utils \
    x11-utils \
    libopenblas-dev \
    libomp-dev \
    libssl-dev \
    libffi-dev \
    libglib2.0-dev \
    libfcl-dev \
    libboost-python-dev \
    libosmesa6-dev \
    ca-certificates \
    curl \
    gnupg2 \
    wget \
    git \
    vim \
    byobu \
    git-lfs \
    python3-pip \
    python3-tk \
    python3-dev \
    && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS2 Jazzy Install ----
# ---- ROS 2 apt source and signing key ----
RUN curl -fsSL \
    https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.noble_all.deb \
    -o /tmp/ros2-apt-source.deb && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb

RUN apt update && apt install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS2 bash settings ----
RUN { \
    echo "source /opt/ros/jazzy/setup.bash"; \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh"; \
    echo "export _colcon_cd_root=/opt/ros/jazzy/"; \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"; \
    echo "source /ros2_ws/install/setup.bash"; \
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-root"; \
    } >> ~/.bashrc

# ---- Python toolchain ----
# --ignore-installed: the base image's pip/setuptools are apt-managed (no RECORD
# file) so pip refuses to uninstall them. Skip the uninstall and install fresh.
# setuptools<80: colcon --symlink-install uses `setup.py develop --editable`
# which was removed in setuptools 80.
RUN pip install --ignore-installed "setuptools<80" wheel ninja

# ---- PyTorch (upstream, CUDA 12.8) ----
RUN pip install \
    torch==2.7.1 \
    torchvision==0.22.1 \
    --index-url https://download.pytorch.org/whl/cu128

# ---- PyTorch Geometric extensions (pre-built wheels matching upstream torch) ----
RUN pip install \
    torch-cluster torch-scatter \
    -f https://data.pyg.org/whl/torch-2.7.0+cu128.html

# ---- GraspGen dependencies ----
# Constraints pin a Blackwell-compatible PyTorch build. RTX 50-series GPUs
# require sm_120 kernels, which are not present in the older cu121 wheel.
RUN echo 'torch==2.7.1+cu128' > /tmp/constraints.txt && \
    echo 'torchvision==0.22.1+cu128' >> /tmp/constraints.txt && \
    echo 'setuptools<80' >> /tmp/constraints.txt && \
    pip install --ignore-installed -c /tmp/constraints.txt \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    diffusers==0.11.1 \
    timm==1.0.15 \
    huggingface-hub==0.25.2 \
    pyrender==0.1.45 \
    PyOpenGL==3.1.0 \
    pyglet==1.5.11 \
    imageio \
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
    scene-synthesizer[recommend] && \
    rm /tmp/constraints.txt

# ---- Build Manifold ----
RUN mkdir -p /install/Manifold && \
    cd /install && git clone --depth 1 --recursive -j8 https://github.com/hjwdzh/Manifold.git && \
    mkdir -p Manifold/build && cd Manifold/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
ENV PATH="${PATH}:/install/Manifold/build/"

# ---- GraspGen Clone ----
WORKDIR /ros2_ws/src
RUN git clone --depth 1 --recursive https://github.com/NVlabs/GraspGen.git
RUN cd /ros2_ws/src/GraspGen/ && touch COLCON_IGNORE
RUN git clone https://huggingface.co/adithyamurali/GraspGenModels --recursive && \
    mkdir GraspGen/models && mv GraspGenModels/* GraspGen/models/
RUN cd /ros2_ws/src/GraspGenModels/ && touch COLCON_IGNORE

# ---- pointnet2_ops build ----
WORKDIR /ros2_ws/src/GraspGen/pointnet2_ops
RUN pip install --no-build-isolation -v .

# ---- GraspGen install ----
# --no-deps: GraspGen pins torch==2.1.0 which conflicts with the installed torch 2.7
WORKDIR /ros2_ws/src/GraspGen
RUN pip install --no-deps -e .

# ---- Python dependencies ----
# Keep the Blackwell-compatible PyTorch build while installing spconv.
RUN echo 'torch==2.7.1+cu128' > /tmp/constraints.txt && \
    echo 'torchvision==0.22.1+cu128' >> /tmp/constraints.txt && \
    echo 'setuptools<80' >> /tmp/constraints.txt && \
    pip install --ignore-installed -c /tmp/constraints.txt \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    spconv-cu124 && \
    rm /tmp/constraints.txt
# Ubuntu's NumPy is installed by apt and has no pip RECORD file. Remove only
# previous pip wheels from /usr/local, then install a mutually compatible set;
# leaving the NumPy 2.x binary modules in place causes ABI import failures.
RUN rm -rf \
        /usr/local/lib/python3.12/dist-packages/numpy \
        /usr/local/lib/python3.12/dist-packages/numpy-*.dist-info \
        /usr/local/lib/python3.12/dist-packages/h5py \
        /usr/local/lib/python3.12/dist-packages/h5py-*.dist-info \
        /usr/local/lib/python3.12/dist-packages/scipy \
        /usr/local/lib/python3.12/dist-packages/scipy-*.dist-info \
        /usr/local/lib/python3.12/dist-packages/sklearn \
        /usr/local/lib/python3.12/dist-packages/scikit_learn-*.dist-info && \
    pip install --no-cache-dir --ignore-installed \
    numpy==1.26.4 \
    h5py==3.10.0 \
    scipy==1.13.1 \
    scikit-learn==1.5.1

# ---- Re-pin setuptools<80 as the final state ----
# Earlier pip installs pulled setuptools 80+ as a transitive dep of upstream
# torch, which removed the `setup.py develop --editable` command that colcon
# --symlink-install relies on.
RUN pip install --ignore-installed "setuptools<80"

# ---- Remove setuptools-scm so it does not auto-activate during colcon build ----
# The setup() finalize-hook scans for git files and logs ERROR when the bind-mounted
# source dir has no .git, even though graspgen_tutorials never uses setuptools-scm.
RUN pip uninstall -y setuptools-scm

# ---- Developer convenience ----
WORKDIR /ros2_ws/

RUN echo "ros2 run graspgen_tutorials obj2stl.py /ros2_ws/src/GraspGen/assets/.../XXX.obj /share/tool_mesh/XXX.stl" >> ~/.bash_history
RUN echo "ros2 run graspgen_tutorials stl2pcjson.py /share/obj_mesh/XXX.stl /share/pcjson/XXX.json" >> ~/.bash_history

RUN echo "byobu" >> ~/.bash_history
RUN echo "colcon build --symlink-install --executor sequential" >> ~/.bash_history
RUN echo "meshcat-server" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials mesh_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_mesh_antipodal_generator.yaml" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials mesh_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_mesh_suction_generator.yaml" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials pointcloud_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_pointcloud_antipodal_generator.yaml" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials pointcloud_collisionfree_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_pointcloud_collisionfree_antipodal_generator.yaml" >> ~/.bash_history
RUN echo "ros2 launch graspgen_tutorials pointcloud_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_pointcloud_suction_generator.yaml" >> ~/.bash_history
RUN echo "ros2 service call /generate_grasp std_srvs/srv/Empty" >> ~/.bash_history
