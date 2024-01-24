FROM intel4coro/base-notebook:20.04-noetic-full-xpra

ENV PATH=$PATH:/home/user/.local/bin
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/
COPY --chown=${NB_USER}:users . pycram/
RUN vcs import --input pycram/binder/pycram-http.rosinstall --recursive

COPY --chown=${NB_USER}:users binder/noetic.rosinstall /home/${NB_USER}/noetic.rosinstall
RUN vcs import --input /home/${NB_USER}/noetic.rosinstall --recursive

# === Following step should be replace to ===
USER ${NB_USER}
RUN cd pycram \
  && git submodule update --init \
  && git clone https://github.com/Tigul/neem_interface_python.git src/neem_interface_python \
  && cd src/neem_interface_python \
  && git clone https://github.com/benjaminalt/neem-interface.git src/neem-interface
# === To ===
# RUN cd pycram
  # && git submodule update --init --recursive
# === When all .gitmodules use https urls ===

RUN pip install --requirement ${PYCRAM_WS}/src/pycram/requirements.txt --user 
RUN pip install --requirement ${PYCRAM_WS}/src/pycram/src/neem_interface_python/requirements.txt --user \
  && pip cache purge

#############################################
#RUN source /opt/ros/noetic/setup.bash           # source ROS
#RUN mkdir -p ${PYCRAM_WS}/src/giskard_ws/src                 # create directory for workspace
#RUN cd ${PYCRAM_WS}/src/giskard_ws                         # go to workspace directory
#RUN catkin init                                 # init workspace, you might have to pip install catkin-tools
#RUN cd ${PYCRAM_WS}/src/giskard_ws/src                                      # go to source directory of workspace
#RUN wstool merge https://raw.githubusercontent.com/SemRoCo/giskardpy/master/rosinstall/noetic.rosinstall
                                           # update rosinstall file
#RUN wstool update       # pull source repositories
#USER root
#RUN apt-get update
#RUN rosdep install -y --ignore-src --from-paths .  # install dependencies available through apt
#RUN pip3 install -r giskardpy/requirements.txt  # install python deps
#RUN cd ..                                       # go to workspace directory
#RUN catkin build                                # build packages
#############################################

RUN pip uninstall numpy -y
RUN pip install --upgrade pip

COPY --chown=${NB_USER}:users binder/me ${PYCRAM_WS}/src/me

USER root
RUN apt-get update

# Build pycram workspace
WORKDIR  ${PYCRAM_WS}
USER root
RUN rosdep update \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
USER ${NB_USER}
RUN catkin build

WORKDIR ${PYCRAM_WS}/src/pycram
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter", "lab", "--allow-root", "--NotebookApp.token=''", "--no-browser", "--ip=0.0.0.0"]
RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz
