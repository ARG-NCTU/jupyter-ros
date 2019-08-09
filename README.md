# arg-jupyter-ros
arg-jupyter-ros is based on https://github.com/RoboStack/jupyter-ros

## Install jupyter-ros via Docker

### Build docker 

```
    $ docker build -t argnctu/jupyter-ros .
``` 

## Run jupyter notebook

The docker container can be run on your local machine or on a remote workstation.

### Run Docker
```
    $ docker run --rm -it -p 8888:8888/tcp argnctu/jupyter-ros
```

In the container:

```
    # cd /
    $ jupyter notebook --ip=0.0.0.0 --port=8888 --allow-root
```

On your local machine:

    Open web browser and input the token to: 
    127.0.0.1:8888

## Basic Usage

You could directly run the exampe in:
* notebooks/ROS 3D Grid.ipynb 


## Advanced (with a ROS bag)

### Get some open ROS bags

We suggest the MIT Stata Dataset (https://projects.csail.mit.edu/stata/downloads.php).

```
    $ wget http://infinity.csail.mit.edu/data/2011/2011-01-25-06-29-26.bag
```

Or the SubT STIX ROS bags
```
    $ wget https://subt-data.s3.amazonaws.com/smoke_test/subt_edgar_hires_2019-04-12-15-52-44.bag
```

### Play a ROS bag and publish topics


### Run notebooks/ROS Laser Scan.ipynb



Enjoy!

## Troubleshooting

If you got the following error when you run a cell ```failed to display Jupyter Widget of type VBox```, 
you can solve it by running the following command ```jupyter nbextension enable --py --sys-prefix widgetsnbextension```
