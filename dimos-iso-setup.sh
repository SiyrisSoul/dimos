#!/bin/bash
set -e

echo "================================================"
echo "DimOS Navigation ISO Setup Script"
echo "================================================"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}[1/8] Setting up SSH keys for GitHub...${NC}"

# Create SSH directory
mkdir -p /root/.ssh
chmod 700 /root/.ssh

# Generate SSH keys for each repository
echo "Generating SSH deploy keys..."
echo "You'll need to add these to GitHub as deploy keys"
echo ""

# Generate key for dimos repo
ssh-keygen -t ed25519 -C "dimos-deploy-key" -f /root/.ssh/id_ed25519 -N ""

# Generate key for ros-navigation-autonomy-stack repo
ssh-keygen -t ed25519 -C "ros-navigation-deploy-key" -f /root/.ssh/id_ed25519_nav -N ""

# Add GitHub to known hosts
ssh-keyscan -t ed25519 github.com >> /root/.ssh/known_hosts

# Create SSH config for multiple repos
cat > /root/.ssh/config << 'EOF'
# Dimensional OS repo
Host github.com-dimos
    HostName github.com
    User git
    IdentityFile /root/.ssh/id_ed25519
    IdentitiesOnly yes

# Navigation / ROS stack repo
Host github.com-nav
    HostName github.com
    User git
    IdentityFile /root/.ssh/id_ed25519_nav
    IdentitiesOnly yes
EOF

# Display the public keys
echo -e "${YELLOW}=====================================${NC}"
echo -e "${YELLOW}IMPORTANT: Add these deploy keys to GitHub${NC}"
echo -e "${YELLOW}=====================================${NC}"
echo ""
echo -e "${GREEN}1. For dimensionalOS/dimos repo:${NC}"
cat /root/.ssh/id_ed25519.pub
echo ""
echo -e "${GREEN}2. For dimensionalOS/ros-navigation-autonomy-stack repo:${NC}"
cat /root/.ssh/id_ed25519_nav.pub
echo ""
echo -e "${YELLOW}Add these as deploy keys in each repo's settings${NC}"
echo -e "${YELLOW}Press Enter to continue after adding the keys...${NC}"
read -r

echo -e "${GREEN}[2/8] Installing base dependencies...${NC}"

# Update packages
apt update

# Install essentials
apt install -y \
    curl \
    wget \
    git \
    git-lfs \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    build-essential \
    python3-pip \
    python3-venv \
    net-tools \
    iputils-ping \
    nano \
    vim

echo -e "${GREEN}[3/8] Installing Docker...${NC}"

# Install Docker
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" > /etc/apt/sources.list.d/docker.list
apt update
apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Enable Docker service (using symlinks for Cubic chroot)
echo "Enabling Docker service for boot..."
ln -sf /lib/systemd/system/docker.service /etc/systemd/system/multi-user.target.wants/docker.service
ln -sf /lib/systemd/system/docker.socket /etc/systemd/system/sockets.target.wants/docker.socket

echo -e "${GREEN}[4/8] Checking for GPU support...${NC}"

# Check if NVIDIA GPU support is needed (check in actual system, not chroot)
if lspci 2>/dev/null | grep -i nvidia > /dev/null || [ -f /proc/driver/nvidia/version ]; then
    echo "NVIDIA GPU detected or forced install, installing container toolkit..."
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add - 2>/dev/null || true
    curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list
    apt update
    apt install -y nvidia-container-toolkit || echo "NVIDIA toolkit will be configured on first boot"

    # Note: nvidia-ctk configure will be run on first boot, not in chroot
    echo "NVIDIA_RUNTIME=true" >> /etc/dimos-build-vars
else
    echo "No NVIDIA GPU detected, configuring for CPU-only mode..."
    echo "NVIDIA_RUNTIME=false" >> /etc/dimos-build-vars
fi

echo -e "${GREEN}[5/8] Cloning DimOS and building Docker image...${NC}"

# Clone the repository using the custom host
cd /opt
if [ ! -d "dimos" ]; then
    git clone git@github.com-dimos:dimensionalOS/dimos.git
fi

# Clone the ROS navigation stack using the custom host
cd /opt/dimos/docker/navigation
if [ ! -d "ros-navigation-autonomy-stack" ]; then
    git clone -b jazzy git@github.com-nav:dimensionalOS/ros-navigation-autonomy-stack.git
fi

# Check for Unity models in LFS
cd /opt/dimos
if [ -f "data/.lfs/office_building_1.tar.gz" ]; then
    echo "Extracting Unity models..."
    git lfs pull
    cd docker/navigation
    tar -xf ../../data/.lfs/office_building_1.tar.gz
    mv office_building_1 unity_models
else
    echo -e "${YELLOW}Warning: Unity models not found in LFS${NC}"
    mkdir -p /opt/dimos/docker/navigation/unity_models
fi

# Create default .env file
if [ ! -f "/opt/dimos/docker/navigation/.env" ]; then
    cp /opt/dimos/docker/navigation/.env.hardware /opt/dimos/docker/navigation/.env
fi

# Note: Docker build will happen on first boot, not in chroot
echo -e "${YELLOW}Docker image will be built on first boot (15-20 minutes)${NC}"
echo "DOCKER_IMAGE_BUILT=false" >> /etc/dimos-build-vars

echo -e "${GREEN}[6/8] Creating dimensional user with sudo privileges...${NC}"

# Create default user with auto-login
useradd -m -s /bin/bash dimensional
echo "dimensional:d1mensional!" | chpasswd

# Add dimensional to sudo and docker groups with NOPASSWD
usermod -aG sudo,docker dimensional
echo "dimensional ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dimensional
chmod 440 /etc/sudoers.d/dimensional

# Fix ownership of /opt/dimos
chown -R dimensional:dimensional /opt/dimos
chown -R dimensional:dimensional /home/dimensional

# Copy SSH keys to dimensional user
mkdir -p /home/dimensional/.ssh
cp -r /root/.ssh/* /home/dimensional/.ssh/
chown -R dimensional:dimensional /home/dimensional/.ssh
chmod 700 /home/dimensional/.ssh
chmod 600 /home/dimensional/.ssh/*

# Also save to /etc/skel for any new users created later
mkdir -p /etc/skel/.ssh
cp -r /root/.ssh/* /etc/skel/.ssh/
chmod 700 /etc/skel/.ssh
chmod 600 /etc/skel/.ssh/*

echo -e "${GREEN}[7/8] Creating auto-start systemd service...${NC}"

# Create systemd service for auto-start
cat > /etc/systemd/system/dimos-navigation.service << 'EOF'
[Unit]
Description=DimOS Navigation Stack
After=docker.service network-online.target
Requires=docker.service
Wants=network-online.target

[Service]
Type=forking
RemainAfterExit=yes
WorkingDirectory=/opt/dimos/docker/navigation
Environment="DISPLAY=:0"
ExecStart=/bin/bash -c 'cd /opt/dimos/docker/navigation && ./start.sh --hardware'
ExecStop=/usr/bin/docker compose -f /opt/dimos/docker/navigation/docker-compose.yml down
Restart=on-failure
RestartSec=10
User=root

[Install]
WantedBy=multi-user.target
EOF

# Enable service using symlinks for Cubic chroot
ln -sf /etc/systemd/system/dimos-navigation.service /etc/systemd/system/multi-user.target.wants/dimos-navigation.service

echo -e "${GREEN}[7/8] Creating configuration and helper scripts...${NC}"

# Create first-boot configuration script
cat > /usr/local/bin/dimos-first-boot.sh << 'EOF'
#!/bin/bash

# Check if first boot
if [ ! -f /etc/dimos-configured ]; then
    # Switch to tty1 and show progress
    exec < /dev/tty1 > /dev/tty1 2>&1
    chvt 1 2>/dev/null || true
    clear

    echo "======================================="
    echo "   DimOS First Boot Configuration"
    echo "======================================="
    echo ""

    echo "[1/5] Starting Docker service..."
    systemctl start docker

    # Configure NVIDIA if available
    if [ -f /etc/dimos-build-vars ] && grep -q "NVIDIA_RUNTIME=true" /etc/dimos-build-vars; then
        echo "[2/5] Configuring NVIDIA container runtime..."
        nvidia-ctk runtime configure --runtime=docker
        systemctl restart docker
    else
        echo "[2/5] Running in CPU-only mode..."
    fi

    # Allow X11 connections
    echo "[3/5] Configuring display..."
    xhost +local:docker 2>/dev/null || true

    # Set up hardware-specific .env if needed
    if [ ! -f /opt/dimos/docker/navigation/.env ]; then
        cp /opt/dimos/docker/navigation/.env.hardware /opt/dimos/docker/navigation/.env
    fi

    # Check if Docker image needs loading
    if [ ! -f /opt/dimos-docker-built ]; then
        echo "[4/5] Loading Docker image..."

        # Check for pre-built tar.gz
        if [ -f /opt/dimos-nav-image.tar.gz ]; then
            echo "Found pre-built Docker image (compressed), loading..."
            echo "This may take a few minutes..."
            gunzip -c /opt/dimos-nav-image.tar.gz | docker load
            if [ ${PIPESTATUS[1]} -eq 0 ]; then
                touch /opt/dimos-docker-built
                echo "✓ Docker image loaded successfully!"
            else
                echo "✗ Failed to load Docker image tar.gz!"
                echo "Press Enter to continue..."
                read
            fi
        elif [ -f /opt/dimos-nav-image.tar ]; then
            echo "Found pre-built Docker image (uncompressed), loading..."
            docker load < /opt/dimos-nav-image.tar
            if [ $? -eq 0 ]; then
                touch /opt/dimos-docker-built
                echo "✓ Docker image loaded successfully!"
            else
                echo "✗ Failed to load Docker image tar!"
                echo "Press Enter to continue..."
                read
            fi
        else
            echo "✗ No pre-built Docker image found!"
            echo "  Expected /opt/dimos-nav-image.tar.gz or /opt/dimos-nav-image.tar"
            echo "  You need to build the image manually:"
            echo "  1. Build on host: docker compose -f docker/navigation/docker-compose.yml build"
            echo "  2. Save compressed: docker save dimos_autonomy_stack:jazzy | gzip > dimos-nav-image.tar.gz"
            echo "  3. Copy to /opt/dimos-nav-image.tar.gz in the ISO"
            echo "Press Enter to continue..."
            read
        fi
    else
        echo "[4/5] Docker image already loaded, skipping..."
    fi

    echo "[5/5] Finalizing setup..."
    touch /etc/dimos-configured

    echo ""
    echo "======================================="
    echo "   Setup Complete! Starting menu..."
    echo "======================================="
    sleep 3
fi
EOF

chmod +x /usr/local/bin/dimos-first-boot.sh

# Create service for first boot
cat > /etc/systemd/system/dimos-first-boot.service << 'EOF'
[Unit]
Description=DimOS First Boot Configuration
After=network-online.target docker.service
Wants=network-online.target
Requires=docker.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/dimos-first-boot.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# Enable service using symlinks for Cubic chroot
ln -sf /etc/systemd/system/dimos-first-boot.service /etc/systemd/system/multi-user.target.wants/dimos-first-boot.service

# Create hardware configuration helper
cat > /usr/local/bin/configure-dimos-hardware << 'EOF'
#!/bin/bash

echo "================================================"
echo "DimOS Hardware Configuration Tool"
echo "================================================"
echo ""

# Function to get network interfaces
get_interfaces() {
    ip -o link show | awk -F': ' '{print $2}' | grep -v lo
}

echo "Available network interfaces:"
get_interfaces
echo ""

read -p "Enter Robot IP (e.g., 10.0.0.191, or press Enter to skip): " ROBOT_IP
read -p "Enter LiDAR IP (e.g., 192.168.1.137): " LIDAR_IP
read -p "Enter Network Interface for LiDAR (e.g., enp100s0): " LIDAR_INTERFACE
read -p "Enter LiDAR Computer IP (default: 192.168.1.5): " LIDAR_COMPUTER_IP
LIDAR_COMPUTER_IP=${LIDAR_COMPUTER_IP:-192.168.1.5}

# Update .env file
ENV_FILE="/opt/dimos/docker/navigation/.env"
if [ -f "$ENV_FILE" ]; then
    sed -i "s/ROBOT_IP=.*/ROBOT_IP=$ROBOT_IP/" $ENV_FILE
    sed -i "s/LIDAR_IP=.*/LIDAR_IP=$LIDAR_IP/" $ENV_FILE
    sed -i "s/LIDAR_INTERFACE=.*/LIDAR_INTERFACE=$LIDAR_INTERFACE/" $ENV_FILE
    sed -i "s/LIDAR_COMPUTER_IP=.*/LIDAR_COMPUTER_IP=$LIDAR_COMPUTER_IP/" $ENV_FILE

    echo ""
    echo "Configuration updated!"
    echo "Restart the navigation stack? (y/n)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "Restarting navigation stack..."
        systemctl restart dimos-navigation
        echo "Navigation stack restarted."
    fi
else
    echo "Error: Environment file not found at $ENV_FILE"
fi
EOF

chmod +x /usr/local/bin/configure-dimos-hardware

# Create status check script
cat > /usr/local/bin/dimos-status << 'EOF'
#!/bin/bash

echo "DimOS Navigation Stack Status"
echo "=============================="
echo ""

# Check Docker service
if systemctl is-active --quiet docker; then
    echo "✓ Docker is running"
else
    echo "✗ Docker is not running"
fi

# Check DimOS navigation service
if systemctl is-active --quiet dimos-navigation; then
    echo "✓ DimOS Navigation is running"
    echo ""
    echo "Active containers:"
    docker ps --filter "name=dimos" --format "table {{.Names}}\t{{.Status}}"
else
    echo "✗ DimOS Navigation is not running"
fi

echo ""
echo "To view logs: journalctl -u dimos-navigation -f"
echo "To configure hardware: configure-dimos-hardware"
echo "To restart: systemctl restart dimos-navigation"
EOF

chmod +x /usr/local/bin/dimos-status

echo -e "${GREEN}[8/8] Creating default user, auto-login, and menu system...${NC}"

# Create default user
useradd -m -s /bin/bash -G sudo,docker dimensional 2>/dev/null || true
echo "dimensional:d1mensional!" | chpasswd

# Set up auto-login for dimensional user on tty1
mkdir -p /etc/systemd/system/getty@tty1.service.d
cat > /etc/systemd/system/getty@tty1.service.d/autologin.conf << 'EOF'
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin dimensional --noclear %I $TERM
EOF

# Create status menu script
cat > /usr/local/bin/dimos-status-menu << 'EOF'
#!/bin/bash

# Check if first boot is still running
if pgrep -f "dimos-first-boot" > /dev/null; then
    echo "First boot configuration in progress..."
    echo "Monitoring build output:"
    tail -f /var/log/dimos-build.log
    exit 0
fi

while true; do
    clear
    echo "===================================="
    echo "   Dimensional Terminal Interface"
    echo "===================================="
    echo ""

    # Check Docker image status
    if [ -f /opt/dimos-docker-built ]; then
        echo "✓ Docker Image: Ready"
    else
        echo "⚠ Docker Image: Not built"
        echo ""
        echo "Press 'b' to build Docker image now"
    fi

    echo ""
    # Show system status
    echo -n "Docker Service: "
    systemctl is-active docker >/dev/null 2>&1 && echo "✓ Running" || echo "✗ Stopped"

    echo -n "Navigation Stack: "
    systemctl is-active dimos-navigation >/dev/null 2>&1 && echo "✓ Running" || echo "✗ Stopped"

    echo ""
    echo "Options:"
    echo "1) Start Navigation Stack"
    echo "2) Stop Navigation Stack"
    echo "3) Configure Hardware"
    echo "4) View Logs"
    echo "5) Shell Access"
    echo "6) Reboot"
    echo "7) Shutdown"

    if [ ! -f /opt/dimos-docker-built ]; then
        echo "b) Build Docker Image"
    fi

    echo "q) Quit Menu (to shell)"
    echo ""
    read -n 1 -p "Select option: " choice
    echo ""

    case $choice in
        1)
            if [ ! -f /opt/dimos-docker-built ]; then
                echo "Docker image not built yet! Build first (option 'b')"
                sleep 3
            else
                echo "Starting navigation stack..."
                systemctl start dimos-navigation
                sleep 2
            fi
            ;;
        2)
            echo "Stopping navigation stack..."
            systemctl stop dimos-navigation
            sleep 2
            ;;
        3)
            configure-dimos-hardware
            ;;
        4)
            echo "Press Ctrl+C to return to menu"
            sleep 2
            journalctl -u dimos-navigation -f
            ;;
        5)
            echo "Entering shell. Type 'exit' to return to menu."
            bash
            ;;
        6)
            echo "Rebooting..."
            reboot
            ;;
        7)
            echo "Shutting down..."
            shutdown -h now
            ;;
        b|B)
            if [ ! -f /opt/dimos-docker-built ]; then
                echo "Starting Docker build (15-20 minutes)..."
                cd /opt/dimos
                docker compose -f docker/navigation/docker-compose.yml build --progress=plain 2>&1 | tee /var/log/dimos-build.log
                if [ ${PIPESTATUS[0]} -eq 0 ]; then
                    touch /opt/dimos-docker-built
                    echo "Build complete! Press Enter..."
                else
                    echo "Build failed! Check /var/log/dimos-build.log. Press Enter..."
                fi
                read
            fi
            ;;
        q|Q)
            echo "Exiting to shell. Run 'dimos-status-menu' to return."
            exit 0
            ;;
    esac
done
EOF

chmod +x /usr/local/bin/dimos-status-menu

# Add auto-start menu to dimensional user's bashrc
cat >> /home/dimensional/.bashrc << 'EOF'

# Auto-start DimOS menu on tty1
if [[ -z $DISPLAY ]] && [[ $(tty) = /dev/tty1 ]]; then
    /usr/local/bin/dimos-status-menu
fi
EOF

# Copy SSH keys to user
mkdir -p /home/dimensional/.ssh
cp /root/.ssh/id_ed25519* /home/dimensional/.ssh/
cp /root/.ssh/config /home/dimensional/.ssh/
cp /root/.ssh/known_hosts /home/dimensional/.ssh/
chown -R dimensional:dimensional /home/dimensional/.ssh
chmod 700 /home/dimensional/.ssh
chmod 600 /home/dimensional/.ssh/id_ed25519
chmod 600 /home/dimensional/.ssh/id_ed25519_nav

# Clean apt cache to reduce ISO size
apt clean
rm -rf /var/lib/apt/lists/*
rm -rf /tmp/*

echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "The ISO is now configured with:"
echo "  - Docker and NVIDIA container support"
echo "  - DimOS repository cloned and Docker image built"
echo "  - Auto-start systemd service"
echo "  - SSH keys for GitHub access"
echo "  - Default user: dimensional (password: dimensional)"
echo ""
echo "After booting from this ISO:"
echo "  1. The navigation stack will auto-start"
echo "  2. Run 'configure-dimos-hardware' to set hardware IPs"
echo "  3. Run 'dimos-status' to check system status"
echo ""
echo "You can now continue with Cubic to generate the ISO."