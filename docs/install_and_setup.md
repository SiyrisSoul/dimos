# How can I setup DimOS?

<!-- Our auto-installer *is in beta* if you'd like to try the automated/interactive version of this guide (`sh <(curl -fsSL "https://raw.githubusercontent.com/dimensionalOS/dimos/main/bin/install")`) -->

## Prerequisites
- Linux or MacOS
- A standard C compiler (gcc, clang, etc.)
- Python 3.10+
- [portaudio](https://www.portaudio.com/) (needed by `pyaudio`)
- [ffmpeg](https://ffmpeg.org/)
- [git lfs](https://git-lfs.github.com/)
- [uv](https://docs.astral.sh/uv/getting-started/installation/) For fast Python package management
- You probably won't need to manually install these, but they are technically needed:
    - [lcm](https://lcm-proj.github.io/lcm/content/build-instructions.html#installing-the-python-module-on-unix-based-systems)
    - gnu grep, gnu sed, iproute2, pkg-config
    - on MacOS, you'll need to have xcode command line tools (or xcode the full app)
    - [cmake](https://cmake.org/download/)
    - [gstreamer](https://gstreamer.freedesktop.org/)
    - [libjpeg-turbo](https://github.com/libjpeg-turbo/libjpeg-turbo)
    - [opencv](https://opencv.org/releases.html)
    - [libuv](https://github.com/libuv/libuv)
    - [sdl2](https://www.libsdl.org/download-2.0.php)
    - [sdl2-image](https://www.libsdl.org/projects/SDL_image/)

<!-- NOTE: these lists potentially need to be edited every time the pyproject.toml is updated! -->
You can install those:
- on MacOS with
```sh
brew install cmake curl grep gnu-sed portaudio ffmpeg git-lfs jpeg-turbo lcm libpng openssl pkg-config gstreamer cairo python@3.13 opencv
```

- on Ubuntu/Debian with 
```sh
sudo apt-get install -y build-essential ca-certificates cmake curl pkg-config net-tools openssl python3 python3-pip python3-opencv python3-pyaudio iputils-ping ffmpeg g++ git git-lfs iproute2 libjpeg-dev libjpeg-turbo8-dev liblcm-dev liblcms2-dev libopencv-dev libpng-dev libportaudio2 gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav python3-gi python3-gi-cairo
```

- Using environment isolation tools: Dev Containers, Dockerfile, or Nix Flake: See our [template repo](https://github.com/dimensionalOS/dimos-template) for the details


## Setup 

After prerequisites, all that needs to be run is:

```sh
uv pip install dimos
```


## Optional Features

Dimos can be installed with optional features to enable additional functionality.

<!-- TODO: mention Isaac Sim, Cuda extra steps, ROS, and talk about ENV vars for each option (like mujoco and rerun) -->

- ### Visualization:
    - Add feature with: `uv pip install dimos[visualization]`
    - prerequisites:
        - common names:
            - opencv, vulkan, gtk3
        - install commands:
            - for Ubuntu/Debian with
                ```sh
                sudo apt-get install -y libgtk-3-dev libvulkan1 libxkbcommon-x11-0 mesa-vulkan-drivers vulkan-tools
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install gtk+3 molten-vk vulkan-loader
                ```
    

- ### Sim:
    - Add feature with: `uv pip install dimos[sim]`
    - prerequisites:
        - common names:
            - glfw, libxslt
        - install commands:
            - for Ubuntu/Debian with 
                ```sh
                sudo apt-get install -y libglfw3 libxslt1-dev patchelf
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install libxslt
                ```
    
    <!-- - Streaming setup (Ubuntu 22.04 recommended):
        - NVIDIA driver 535+ installed (`nvidia-smi` to verify)
        - CUDA toolkit: `sudo apt install -y nvidia-cuda-toolkit`
        - NVIDIA Container Toolkit (runtime config + `docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi` validation)
        - Pull images as needed, e.g. `sudo docker pull nvcr.io/nvidia/isaac-sim:4.2.0`
        - To run streaming examples:
            - `cd docker/simulation`
            - Isaac: `docker compose -f isaac/docker-compose.yml build && docker compose -f isaac/docker-compose.yml up`
            - Genesis: `docker compose -f genesis/docker-compose.yml build && docker compose -f genesis/docker-compose.yml up`
        - RTSP stream available at `rtsp://localhost:8554/stream` (view in VLC or RTSP-capable player) -->

- ### Agents:
    - Add feature with: `uv pip install dimos[agents]`
    - prerequisites:
        - common names:
            - portaudio, rust, libffi
        - install commands:
            - for Ubuntu/Debian with 
                ```sh
                sudo apt-get install -y cargo libasound2-dev libffi-dev libstdc++6 libswresample-dev libyaml-dev make portaudio19-dev rustc
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install libffi libyaml rust
                ```
    

- ### Web:
    - Add feature with: `uv pip install dimos[web]`
    - prerequisites:
        - common names:
            - ffmpeg, libuv, libsndfile
        - install commands:
            - for Ubuntu/Debian with 
                ```sh
                sudo apt-get install -y libavdevice-dev libavfilter-dev libsndfile1 libsndfile1-dev libswresample-dev libuv1-dev
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install libsndfile libuv
                ```
    

- ### Perception:
    - Add feature with: `uv pip install dimos[perception]`
    - prerequisites:
        - common names:
            - pillow/imagequant, harfbuzz, freetype
        - install commands:
            - for Ubuntu/Debian with 
                ```sh
                sudo apt-get install -y cython3 libffi-dev libfreetype6-dev libfribidi-dev libharfbuzz-dev libimagequant-dev libopenjp2-7-dev libwebp-dev libxcb1-dev tcl-dev tk-dev wget
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install freetype fribidi harfbuzz libffi libimagequant little-cms2 openjpeg openssl@3 tcl-tk webp wget
                ```
    

- ### Unitree:
    - Add feature with: `uv pip install dimos[unitree]`
    - prerequisites:
        - common names:
            - ffmpeg stack, portaudio, vulkan/gtk3
        - install commands:
            - for Ubuntu/Debian with 
                ```sh
                sudo apt-get install -y cargo cython3 libasound2-dev libavdevice-dev libavfilter-dev libffi-dev libfreetype6-dev libfribidi-dev libgtk-3-dev libharfbuzz-dev libimagequant-dev libopenjp2-7-dev libopus-dev libsndfile1 libsndfile1-dev libsrtp2-dev libstdc++6 libswresample-dev libuv1-dev libvpx-dev libvulkan1 libwebp-dev libxcb1-dev libxkbcommon-x11-0 libyaml-dev make mesa-vulkan-drivers portaudio19-dev rustc tcl-dev tk-dev vulkan-tools wget
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install freetype fribidi gtk+3 harfbuzz libffi libimagequant libsndfile libuv libvpx libyaml little-cms2 molten-vk openjpeg openssl@3 opus rust srtp tcl-tk vulkan-loader webp wget
                ```
    

- ### Manipulation:
    - Add feature with: `uv pip install dimos[manipulation]`
    - prerequisites:
        - common names:
            - octomap/fcl, spatialindex, glfw/gtk3
        - install commands:
            - for Ubuntu/Debian with 
                ```sh
                sudo apt-get install -y can-utils ethtool freeglut3-dev gir1.2-gtk-3.0 libasound2 libassimp-dev libatk-bridge2.0-0 libatk1.0-0 libatspi2.0-0 libcairo2 libcairo2-dev libccd-dev libcups2 libdbus-1-3 libdrm2 libeigen3-dev libfcl-dev libfontconfig1-dev libfreetype6 libfreetype6-dev libgbm1 libgeos++-dev libgeos-dev libgirepository1.0-dev libglfw3 libgmp-dev libgnutls30 libgtk-3-dev libhdf5-dev libhdf5-serial-dev libidn2-0 libmpc-dev libmpfr-dev libnspr4 libnss3 liboctomap-dev libopus-dev libpango-1.0-0 libqhull-dev libsodium-dev libspatialindex-dev libspatialindex6 libtasn1-6 libunistring2 libuuid1 libvpx-dev libwebp-dev libxcb-render0-dev libxcb-shm0-dev libxcomposite1 libxdamage1 libxext-dev libxfixes3 libxft-dev libxkbcommon0 libxrandr2 libyaml-dev tcl-dev tk-dev xvfb
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install assimp eigen fcl fontconfig freeglut freetype geos gmp gnutls gobject-introspection gtk+3 hdf5 libccd libidn2 libmpc libsodium libtasn1 libunistring libvpx libxft libyaml mpfr nettle octomap opus ossp-uuid pygobject3 qhull spatialindex tcl-tk webp
                ```
    

- ### Cuda:
    - Add feature with: `uv pip install dimos[cuda]`
    - prerequisites:
        - common names:
            - cuda toolkit, gtk3, hdf5
        - install commands:
            - for Ubuntu/Debian with 
                ```sh
                sudo apt-get install -y cython3 gir1.2-gtk-3.0 libavcodec58 libavformat58 libc6 libcairo2-dev libffi-dev libfontconfig1-dev libfreetype6-dev libfribidi-dev libgirepository1.0-dev libgtk-3-dev libharfbuzz-dev libhdf5-dev libjpeg8-dev libopenjp2-7-dev libqhull-dev libstdc++6 libswscale5 libv4l-dev libwebp-dev libx11-6 libxcb-render0-dev libxcb-shm0-dev libxcb1-dev libxext-dev libxfixes3 libxft-dev libxinerama1 libxrandr2 libyaml-dev make ninja-build nvidia-cuda-toolkit tcl-dev tk-dev wget
                ```

            - for MacOS with <detail><summary>(click to expand)</summary>
                ```sh
                brew install fontconfig freetype fribidi gobject-introspection gtk+3 harfbuzz hdf5 libffi libxft libyaml little-cms2 ninja openjpeg openssl@3 qhull tcl-tk webp wget
                ```
    
