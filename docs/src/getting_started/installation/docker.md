# Installation (Docker)

This section provides instructions for running the simulation within a Docker container. Before proceeding, ensure that your system meets the [system requirements](../requirements.md). If you are using a different operating system, you may need to adjust the following steps accordingly or refer to the official documentation for each step.

## 1. Install [Docker Engine](https://docs.docker.com/engine)

First, install Docker Engine by following the [official installation instructions](https://docs.docker.com/engine/install). For example:

```bash
curl -fsSL https://get.docker.com | sh
sudo systemctl enable --now docker

sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## 2. Install [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit)

Next, install the NVIDIA Container Toolkit, which is required to enable GPU support for Docker containers. Follow the [official installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) or use the following commands:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 3. Gain Access to the [Isaac Sim Docker Image](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)

To run the simulation, you need access to the Isaac Sim Docker image, which requires registering an account and generating an API key from NVIDIA GPU Cloud (NGC).

### 3.1 Register and Log In to [NVIDIA GPU Cloud (NGC)](https://www.nvidia.com/en-us/gpu-cloud)

Visit the [NGC portal](https://ngc.nvidia.com/signin) and register or log in to your account.

### 3.2 Generate Your [NGC API Key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/README.md#ngc-api-keys)

Follow the [official guide](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/README.md#generating-personal-api-key) to generate your personal NGC API key.

### 3.3 Log In to NGC via Docker

Once you have your API key, log in to NGC through Docker:

```bash
docker login nvcr.io
```

When prompted for a username, enter `$oauthtoken` (exactly as shown):

```bash
Username: $oauthtoken
```

When prompted for a password, use the API key you just generated:

```bash
Password: <NGC API Key>
```

## 4. Clone the Repository

Next, clone the `space_robotics_bench` repository locally. Make sure to include the `--recurse-submodules` flag to clone also the submodule containing simulation assets.

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
```

## 5. Build the Docker Image

Now, you can build the Docker image for `space_robotics_bench` by running the provided [`.docker/build.bash`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/build.bash) script. Note that the first build process may take up to 30 minutes (depending on your network speed and system configuration).

```bash
space_robotics_bench/.docker/build.bash
```

## 6. Verify the Image Build

To ensure that the image was built successfully, run the following command. You should see the `space_robotics_bench` image listed among recently created Docker images.

```bash
docker images
```
