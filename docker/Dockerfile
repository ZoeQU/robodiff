FROM nvidia/cuda:11.6.1-cudnn8-devel-ubuntu20.04

# # set cuda 
# RUN ln -sf /usr/local/cuda-11.6 /usr/local/cuda \
#     && echo "export CUDA_HOME=/usr/local/cuda" >> /root/.bashrc \
#     && echo "export PATH=${PATH}:${CUDA_HOME}/bin" >> /root/.bashrc \
#     && echo "export LD_LIBRARY_PATH=${CUDA_HOME}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> /root/.bashrc

# install necessary packages 
RUN apt-get update \
    && apt-get install -y wget unzip libosmesa6-dev libgl1-mesa-glx libglfw3 v4l-utils apt-transport-https\
    patchelf git vim sudo libspnav-dev libusb-1.0.0 spacenavd curl lsb-release usbutils  iputils-ping\
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    
# install miniconda
RUN apt-get update && apt-get install -y wget libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf git \
    && mkdir -p /opt/conda \
    && wget https://repo.anaconda.com/miniconda/Miniconda3-py39_24.1.2-0-Linux-x86_64.sh -O /opt/conda/miniconda.sh \ 
    && bash /opt/conda/miniconda.sh -b -p /opt/miniconda \
    && echo ". /opt/miniconda/bin/activate" >> /root/.bashrc
    
ENV PATH="/opt/miniconda/bin:${PATH}"

# deploy diffusion_policy
RUN cd /root \
    && git clone https://github.com/ZoeQU/robodiff.git \
    && cd /root/robodiff \
    # && /bin/bash -c "source /opt/miniconda/bin/activate && /opt/miniconda/bin/conda env create -f conda_environment.yaml"
    && conda env create -f conda_environment_real.yaml 
 
SHELL ["conda", "run", "-n", "robodiff", "/bin/bash", "-c"]
# set work directory
WORKDIR /root/robodiff

COPY test_print.py /root/robodiff/

COPY setup.sh /root/robodiff/

# set default command
# CMD ["tail", "-f", "/dev/null"]

# COPY setup.sh /root/setup.sh
# RUN chmod +x /root/setup.sh
# CMD ["/root/setup.sh"]
# CMD ["conda", "run", "-n", "robodiff", "python", "test_print.py"]
