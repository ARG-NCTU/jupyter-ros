# arg-jupyter-ros
arg-jupyter-ros is based on https://github.com/RoboStack/jupyter-ros

## Install and Run jupyter-ros via Docker

### CPU:

#### Build docker 

```bash
    $ cd docker/cpu
    $ docker build -t argnctu/jupyter-ros .      # It takes about 40-60 mins
``` 

#### Pull docker image from Dockerhub 
```bash
    $ docker pull argnctu/jupyter-ros    # It takes about 20 mins (depend on you network bandwith)
``` 

#### Run
The docker container can be run on your local machine or on a remote workstation.

Linux
```bash
    $ docker run --rm -it -p 8888:8888/tcp -v /home/[username]:/hosthome argnctu/jupyter-ros
```

Mac
```bash
    $ docker run --rm -it -p 8888:8888/tcp -v /Users/[username]:/hosthome argnctu/jupyter-ros
```

### GPU

#### Build docker 

```bash
    $ cd docker/gpu
    $ docker build -t argnctu/jupyter-ros:gpu .      # It takes about 40-60 mins
``` 

#### Pull docker image from Dockerhub 

If your nvidia-driver [version >= 410.48](https://docs.nvidia.com/deploy/cuda-compatibility/index.html#binary-compatibility__table-toolkit-driver):
```bash
    $ docker pull argnctu/jupyter-ros:gpu
```

#### Run Docker with CUDA support (assume you have installed [nvidia-docker](https://github.com/NVIDIA/nvidia-docker))

Linux
```bash
    $ nvidia-docker run --rm -it -p 8888:8888/tcp -v /home/[username]:/hosthome argnctu/jupyter-ros
    
    # or use new nvidia-docker command:
    # docker run --gpus all --rm -it -p 8888:8888/tcp -v /home/[username]:/hosthome argnctu/jupyter-ros
```

---
## Both GPU and CPU
In the container:

```bash
    # cd /
    $ jupyter notebook --ip=0.0.0.0 --port=8888 --allow-root
```
On your local machine:

    Open web browser and input the token to: 
    127.0.0.1:8888

## Basic Usage CPU

You could directly run the exampe in:
* notebooks/ROS 3D Grid.ipynb 

### Use jupyter-ros with a ROS bag

By default we have add the volumn linked:
* ~/ locally (at workstation)
* /hosthome in docker container

#### Get some open ROS bags

We suggest the MIT Stata Dataset (https://projects.csail.mit.edu/stata/downloads.php).

```bash
    $ wget http://infinity.csail.mit.edu/data/2011/2011-01-25-06-29-26.bag
```

Or the SubT STIX ROS bags
```bash
    $ wget https://subt-data.s3.amazonaws.com/smoke_test/subt_edgar_hires_2019-04-12-15-52-44.bag
```

#### Play a ROS bag and publish topics


#### Run notebooks/ROS Laser Scan.ipynb


## Basic Usage GPU

Verify your cuda-support function: type python to enter python-shell, then type:
```python
import torch
print(torch.cuda.is_available())  # It will return True if installation is successful
exit()
```

Enjoy!

## Troubleshooting

#### Failed to docker run  (case 1)
![docker run failed](figures/docker_run_failed_cuda.png)
Please re-checked your nvidia driver by typing "nvidia-smi", then choose correct tag of the docker image.


#### Failed to display Jupyter
If you got the following error when you run a cell ```failed to display Jupyter Widget of type VBox```, 
you can solve it by running the following command ```jupyter nbextension enable --py --sys-prefix widgetsnbextension```
