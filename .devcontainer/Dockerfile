# Get environment variables, default is the author's username

# Pull debian
FROM debian:stable

# Install dependencies update apt
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -qy build-essential \
    gcc g++ \
    libgsl0-dev \
    liblapack-dev \
    libopenblas-dev \
    libeigen3-dev \
    python3-tk \
    automake \
    libpcre3-dev \
    git \
    cmake \
    python3-dev \
    pkg-config \ 
    wget
RUN apt-get install -y byacc 
RUN apt-get install -y  python3-scipy python3-numpy python3-matplotlib
RUN rm -rf /var/lib/apt/lists/*

# Define working directory
# WORKDIR /home/reswarm

# RUN git clone https://github.com/acados/acados.git -b master acados
# RUN cd /home/reswarm/acados && git submodule update --recursive --init

# Test
RUN apt update && apt install -y lsb-release
RUN lsb_release -a

# Set Python to Python3
RUN cd /usr/bin && ln -s python3 python
RUN apt install -y python3-pip

# Install CasADi
RUN pip3 install casadi
RUN python3 -c "import casadi"

# Update acados source code, already cloned when compiling swig
WORKDIR /home/reswarm
RUN git clone https://github.com/acados/acados.git -b master acados
RUN cd /home/reswarm/acados && git submodule update --recursive --init

# Compile acados with Python interface
# Note that supporting packages were installed when compiling swig
RUN cd /home/reswarm/acados && \
    rm -rf build && mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DUNIT_TESTS=ON .. && \
    make install

# Set environment variables for acados lib, note that user is root
ENV PYTHONPATH="${HOME}/local/lib:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/usr/local/lib:$HOME/local/lib:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/reswarm/acados/lib"
ENV ACADOS_SOURCE_DIR="/home/reswarm/acados"

# Test
RUN pip3 install /home/reswarm/acados/interfaces/acados_template
RUN cd /home/reswarm/acados/examples/acados_python/getting_started && \
    python3 -c "import acados_template"

### Implementation-specific 
# Install depends
RUN apt install -y python3-yaml
RUN pip3 install -U PyYAML
RUN pip3 install glog 

# Environment installs
RUN pip3 install -U pycodestyle --user
ENV PYTHONPATH=/workspaces/reswarm_dmpc/src:$PYTHONPATH

# Make port 22 available to the world outside this container
EXPOSE 22
