# graspgen_ros

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License: BSD](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/UOsaka-Harada-Laboratory/graspgen_ros)

ROS2 example nodes and Docker environment for [GraspGen](https://github.com/NVlabs/GraspGen)

## Dependencies

### Docker build environments (host machine tested)

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - Docker 27.4.1
  - Docker Compose 2.32.1
- NVIDIA GeForce RTX 3090
  - NVIDIA Driver 550.144.03
  - CUDA v12.4 

## Installation

1. Install dependency and software
    ```bash
    sudo apt install byobu && git clone git@github.com:UOsaka-Harada-Laboratory/graspgen_ros.git --recursive --depth 1
    ```
2. Construct docker image from Dockerfile
    ```bash
    cd graspgen_ros && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
    ```

## Usage with docker

1. Construct docker container from the image built
    ```bash  
    docker compose up  
    ```  

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

### Suction grasp generation for object point cloud
  ```bash  
  ./utils/pointcloud_suction_demo.sh  
  ```  

<img src=image/pointcloud_suction_demo.png width=500>  

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)  

We always welcome collaborators!

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
