FROM osrf/ros:jazzy-desktop-full
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    vim && \
		rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
ARG USERNAME=developer

RUN userdel -r ubuntu && \
		groupadd -g ${GID} ${USERNAME} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} && \
		echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
		chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/projects/reluctant

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo 'export PS1="\[\e[32m\]\u@mirs-sim\[\e[m\]:\[\e[34m\]\w\[\e[m\]\\$ "' >> /home/${USERNAME}/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /home/${USERNAME}/.bashrc && \
    echo "alias ru='rosdep update'" >> /home/${USERNAME}/.bashrc && \
    echo "alias ri='rosdep install --from-path src --ignore-src -r -y'" >> /home/${USERNAME}/.bashrc && \
    echo "alias si='source install/setup.bash'" >> /home/${USERNAME}/.bashrc

CMD ["bash"]
