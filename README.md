# graspgen_ros

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![ROS 2: Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![License: BSD](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/UOsaka-Harada-Laboratory/graspgen_ros)

ROS 2 Jazzy example nodes and Docker environment for [GraspGen](https://github.com/NVlabs/GraspGen)

## Dependencies

### Inside the Docker container

- Ubuntu 24.04 (base image: `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04`)
- ROS 2 Jazzy
- Python 3.12
- PyTorch 2.4.0 + cu121 (wheel-bundled CUDA runtime)

### Host machine (tested)

- [Ubuntu 24.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=24.04+LTS)
  - Docker 27.4.1
  - Docker Compose 2.32.1
- NVIDIA GeForce RTX 3070 Laptop GPU (Ampere, sm_86)
  - NVIDIA Driver 535.288.01
  - CUDA v12.2

> **Note on GPU/driver compatibility:** PyTorch is installed with the `cu121`
> wheel so the bundled CUDA 12.1 runtime works with the host driver 535
> (native CUDA 12.2) via Minor Version Compatibility. Newer drivers (>= 550)
> should also work, but the CUDA 12.6 toolkit in the container is only used
> at build time; the container does **not** require driver >= 560 because
> `/usr/local/cuda/compat` is removed during build and `NVIDIA_DISABLE_REQUIRE`
> is set in [docker-compose.yml](docker-compose.yml).

## Installation

1. Install dependencies and clone
    ```bash
    sudo apt install byobu
    git clone git@github.com:UOsaka-Harada-Laboratory/graspgen_ros.git --recursive --depth 1
    ```
2. Build the Docker image
    ```bash
    cd graspgen_ros
    COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build
    ```
    Initial build downloads a large base image (~5 GB) and GraspGen model
    weights via Git LFS (~5 GB), and compiles `pointnet2_ops` from source.
    Expect 25–40 minutes on a typical laptop.

## Usage with docker

1. Start the container in the background
    ```bash
    docker compose up -d
    docker ps    # confirm graspgen_container is "Up"
    ```
    Running `colcon build` is triggered at container start; wait ~5 seconds
    until `Summary: 1 package finished` appears in `docker logs graspgen_container`.

2. Execute a demo script

### Antipodal grasp generation for object mesh
  ```bash  
  ./utils/mesh_antipodal_demo.sh
  ```  

<img src=image/mesh_antipodal_demo.png width=500>  

### Suction grasp generation for object mesh
  ```bash  
  ./utils/mesh_suction_demo.sh  
  ```  

<img src=image/mesh_suction_demo.png width=500>  

### Antipodal grasp generation for object point cloud
  ```bash  
  ./utils/pointcloud_antipodal_demo.sh  
  ```

<img src=image/pointcloud_antipodal_demo.png width=500>  

### Collision-free antipodal grasp generation for object point cloud
  ```bash  
  ./utils/pointcloud_collisionfree_antipodal_demo.sh  
  ```

<img src=image/pointcloud_collisionfree_antipodal_demo.png width=500>  

### Suction grasp generation for object point cloud
  ```bash  
  ./utils/pointcloud_suction_demo.sh  
  ```  

<img src=image/pointcloud_suction_demo.png width=500>  

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)  

We always welcome collaborators!

## License

This software is released under the BSD 3-Clause License, see [LICENSE](./LICENSE).
