### 环境准备
### Ubuntu 20.04
1. Install other dependencies:
    ```bash
    sudo apt install -y \
        git \
        g++ \
        cmake \
        can-utils \
        libopencv-dev \
        libfmt-dev \
        libeigen3-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        libusb-1.0-0-dev \
        nlohmann-json3-dev \
        screen
    ```
2. Install [OpenVINO](https://docs.openvino.ai/2025/get-started/install-openvino.html?PACKAGE=OPENVINO_BASE&VERSION=v_2025_1_0&OP_SYSTEM=LINUX&DISTRIBUTION=APT)
3. Build:
    ```bash
    cmake -B build
    make -C build/ -j`nproc`
    ```
4. Verify:
    ```bash
    ./build/yolo_test
