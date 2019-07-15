#                                            Kidnapped-Vehicle

Project Description: The robot-car has been "kidnapped" and transported to a new location. However, it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

As a solution I have implemented a 2 dimensional particle filter in C++. 
The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). 
At each time step the filter also gets observation and control data.


### What are Particle filters?

Particle filters are methods used to solve filtering problems arising in signal processing and Bayesian statistical inference. The filtering problem consists of estimating the internal states in dynamical systems when partial observations are made.

For example:

Consider the following scenario:

![](http://i.imgur.com/R4XQ73H.png)

This is a floor plan of an environment where a robot is located in the middle and has to perform global localization.
Which means it needs to know where it is based on sensor measuraments. The blue stripes are its sensors range. The sensors indicate near obstacles. 
Each of the red dots (particles) is a discrete guess to where it is. It has position (x,y) and a heading direction (angle).
All of the particles combine the Particle filter.
At the beginning the particles are uniformly spread:

![](http://i.imgur.com/R4XQ73H.png)

But after applying the Particle filter only the correct set of particles survive:

![](http://i.imgur.com/sb5HdwB.png)


![](http://i.imgur.com/Jyc0LEy.png)


![](http://i.imgur.com/p5D3Nea.png)


### The Project

In the project I performed observation measurement transformations, along with identifying measurement landmark associations in order to correctly calculate each particle's weight.
The goal here is to find a weight parameter for each particle that represents how well that particle fits to being in the same location as the actual car:


![](http://i.imgur.com/BizHlq2.jpg)


##### Ouput video:

[![Kidnapped Car](http://i.imgur.com/tjwXD8F.png)](https://vimeo.com/229889875 "Kidnapped Car")
