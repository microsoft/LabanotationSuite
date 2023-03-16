FROM python:2.7.10-slim

SHELL ["bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
    software-properties-common \
    git \
    wget \
    emacs

RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    sh Miniconda3-latest-Linux-x86_64.sh -b -p /opt/miniconda3 && \
    rm -r Miniconda3-latest-Linux-x86_64.sh
ENV PATH /opt/miniconda3/bin:$PATH

RUN conda update -n base -c defaults conda \
    && conda create -n labansuite python=2.7 -y
RUN echo "source activate labansuite" >> ~/.bashrc

RUN source activate labansuite \
    && pip install matplotlib==2.2.4 \
    && pip install opencv-python==4.2.0.32

RUN git clone --recursive https://github.com/microsoft/LabanotationSuite.git
WORKDIR /LabanotationSuite/GestureAuthoringTools/LabanEditor/src
 
# COPY ./main.py /LabanotationSuite/GestureAuthoringTools/LabanEditor/src/main.py
# COPY ./graphLaban.py /LabanotationSuite/GestureAuthoringTools/LabanEditor/src/graphLaban/graphLaban.py
CMD ["bash"]
