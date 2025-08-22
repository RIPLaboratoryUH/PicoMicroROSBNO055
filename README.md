# Setup Instructions

> **Note:** It's recommended to run each command one line at a time to avoid errors and make debugging easier.

---

## 1. Install Prerequisites

Run the following commands to install required packages:

```bash
sudo apt install build-essential
sudo apt install cmake
sudo apt install gcc-arm-none-eabi
sudo apt install libnewlib-arm-none-eabi
sudo apt install doxygen
sudo apt install git
sudo apt install python3
```

---

## 2. Install the Pico Toolchain

1. Visit the [micro-ROS Pico GitHub](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) page.
2. Scroll to the **Dependencies** section and find the **GCC ARM Embedded Toolchain** (version 9.3.1 or similar).
3. Download the archive (e.g., `.bz2` file) by right-clicking and saving it.
4. Extract the archive.
5. Move the extracted folder to your home directory:

    ```bash
    mv gcc-arm-none-eabi-9-2020-q2-update ~/
    ```

6. Add the toolchain path to your `~/.bashrc`:

    ```bash
    export PICO_TOOLCHAIN_PATH=$HOME/gcc-arm-none-eabi-9-2020-q2-update/
    ```

7. Apply the changes:

    ```bash
    source ~/.bashrc
    ```

---

## 3. Download and Configure the Pico SDK

1. Download the SDK:

    ```bash
    cd ~
    git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
    ```

2. Add the SDK path to your `~/.bashrc`:

    ```bash
    export PICO_SDK_PATH=$HOME/pico-sdk
    ```

3. Apply the changes:

    ```bash
    source ~/.bashrc
    ```

---

## 4. Download and Install `picotool`

1. Run the following commands:

    ```bash
    cd ~
    git clone https://github.com/raspberrypi/picotool.git
    cd picotool
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

2. Verify the installation:

    ```bash
    picotool
    ```

    If installed correctly, this command will display usage information.

---
## 5. Clone the Project Repository

Create a new directory and clone the project repository:

```bash
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src
git clone https://github.com/RIPLaboratoryUH/PicoMicroROSBNO055.git
```

This will create a workspace directory and download the project files you'll use to build and flash micro-ROS on the Raspberry Pi Pico.

---

## 6. Build the micro-ROS Project

Now navigate to your micro-ROS folder, create a build directory, and compile the project.

> **Important:** Make any necessary changes to this setup **before** running the build commands in this step.
> 
> The three main files you may want to edit are:
>
> - `CMakeLists.txt`  
> - `pico_micro_ros_example.c`
> - `picow_udp_transports.h`
>
> For example, if you're using a **Raspberry Pi Pico 2W** instead of a **Pico W**, you need to modify line 2 of `CMakeLists.txt`.
> 
> You also need to update your SSID and WiFi password in `pico_micro_ros_example.c`. This is also where you include how your sensor operates.
> 
> Also you need to update your IP address in `picow_udp_transports.txt`.
>
Run the following commands to build your project:

```bash
cd ~/micro_ros_ws/src/PicoMicroRos/
mkdir build
cd build
cmake ..
make
```
## 7. Flash the Raspberry Pi Pico

To flash the compiled program to your Pico:

1. Hold down the **BOOTSEL** button on the Pico as you plug it into your computer via USB.  
   The Pico should mount as a USB mass storage device.

2. Run the appropriate command based on your device:

### For Raspberry Pi Pico W:

```bash
cd ~/micro_ros_ws/src/PicoMicroRos/build/
cp ./pico_micro_ros_example.uf2 /media/$USER/RPI-RP2
```

### For Raspberry Pi Pico 2W:

```bash
cd ~/micro_ros_ws/src/PicoMicroRos/build/
cp ./pico_micro_ros_example.uf2 /media/$USER/RP2350
```

This will copy the compiled `.uf2` binary to your Pico, flashing the firmware.

## 8. Install Docker and micro-ROS Docker Plugin

To build static libraries for micro-ROS, you'll need Docker.

1. Follow the official Docker installation guide for Ubuntu:  
   👉 [Install Docker on Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

2. Once Docker is successfully installed, pull the micro-ROS static library builder image by running:

```bash
sudo docker pull microros/micro_ros_static_library_builder:jazzy
```
## 9. Run the micro-ROS Agent

You’re now ready to run the micro-ROS agent and communicate with your Pico.

1. Disconnect the Pico from USB and power it externally.
2. Then run the following command to launch the micro-ROS agent on your host machine:

```bash
sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy udp4 --dev -p 8891
```

> ⏱️ **Note:** The Pico is programmed to search for the host agent for **1 minute** after startup. You can change this duration in the source code if needed.

3. If you have ROS 2 installed, verify communication by listing active topics:

```bash
ros2 topic list
```

You should now see your micro-ROS topic in the list.

