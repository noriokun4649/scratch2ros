FROM node:lts-alpine

WORKDIR /var/docker/scratch

RUN apk update && \
    apk upgrade && \
    apk --no-cache --virtual add git python3 make g++ && \
    git clone --depth 1 https://github.com/LLK/scratch-gui.git . && \
    npm install && \
    git clone https://github.com/noriokun4649/scratch2ros.git && \
    git clone https://github.com/champierre/ml2scratch.git && \
    git clone https://github.com/champierre/posenet2scratch.git && \
    git clone https://github.com/champierre/tm2scratch.git && \
    git clone https://github.com/champierre/scratch2maqueen.git && \
    git clone https://github.com/champierre/facemesh2scratch.git && \
    git clone https://github.com/champierre/handpose2scratch.git && \
    git clone https://github.com/champierre/speech2scratch.git && \
    git clone https://github.com/sugiura-lab/scratch3-qrcode.git && \
    git clone https://github.com/champierre/ic2scratch.git && \
    sh scratch3-qrcode/install.sh && \
    sh speech2scratch/install.sh && \
    sh handpose2scratch/install.sh && \
    sh facemesh2scratch/install.sh && \
    sh scratch2maqueen/install.sh && \
    sh tm2scratch/install.sh && \
    sh posenet2scratch/install.sh && \
    sh ml2scratch/install.sh && \
    sh scratch2ros/install.sh && \
    sh ic2scratch/install.sh && \
    apk del

VOLUME /var/docker/scratch

ENTRYPOINT [ "npm", "start" ]