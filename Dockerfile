FROM python:2
# WORKDIR /canfestival

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=canfestival
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN apt-get update && apt-get install --no-install-recommends -y \
    sudo 

ENV USER=${USER_NAME}

RUN echo "canfestival ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

# RUN chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

USER ${USER_NAME}

CMD ["bash"]
