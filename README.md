# arg-jupyter-ros
arg-jupyter-ros is based on https://github.com/RoboStack/jupyter-ros

## How to use jupyter-ros

### Build docker 

    $ docker build -t jupyter-ros:jack .
 
### Run Docker

    $ docker run --rm -it -p 8888:8888/tcp jupyter-ros:jack

### Run jupyter notebook

In the docker:

    $ cd /
    $ jupyter notebook --ip=0.0.0.0 --port=8888 --allow-root

On your host:

    open web browser
    type 127.0.0.1:8888

Enjoy!

## Troubleshooting

If you got the following error when you run a cell ```failed to display Jupyter Widget of type VBox```, 
you can solve it by running the following command ```jupyter nbextension enable --py --sys-prefix widgetsnbextension```
