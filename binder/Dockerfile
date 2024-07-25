FROM intel4coro/base-notebook:20.04-noetic-full-xpra

# Set environment
ENV PATH=$PATH:/home/user/.local/bin
ENV PYCRAM_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${PYCRAM_WS}/src/
#COPY --chown=${NB_USER}:users . pycram/
RUN vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/pycram-https.rosinstall --recursive

# === Following step should be replace to ===
USER ${NB_USER}
RUN cd pycram
RUN git remote remove origin
RUN git remote add origin https://github.com/K3cks/pycram
RUN git pull
RUN git checkout binder-multirobot
# === To ===
# RUN cd pycram
  # && git submodule update --init --recursive
# === When all .gitmodules use https urls ===

RUN pip install --requirement ${PYCRAM_WS}/src/pycram/requirements.txt --user 
RUN pip install --requirement ${PYCRAM_WS}/src/pycram/src/neem_interface_python/requirements.txt --user \
  && pip cache purge

# Remove double numpy version 
RUN pip uninstall numpy -y
RUN pip install --upgrade pip

COPY --chown=${NB_USER}:users binder/me ${PYCRAM_WS}/src/me

# Install tiago
WORKDIR ${PYCRAM_WS}/src/
RUN git clone https://github.com/code-iai/iai_tiago.git
WORKDIR ${PYCRAM_WS}/src/iai_tiago/
RUN git submodule update --init 

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

RUN pip install service_identity

# Jupyter Config
COPY --chown=${NB_USER}:users binder/jupyter-config.json /opt/conda/share/jupyter/lab/settings/overrides.json

WORKDIR ${PYCRAM_WS}/src/pycram
RUN git config --global --add safe.directory ${PWD}
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["jupyter", "lab", "--allow-root", "--NotebookApp.token=''", "--no-browser", "--ip=0.0.0.0"]
RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-rviz/master/dist/jupyterlab_rviz-0.3.1.tar.gz
