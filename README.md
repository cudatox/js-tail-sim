# js-tail-sim
A pure javascript/html tail simulator with minimal dependencies that can run inside a browser.


## About
JSTailSim is an animatronic tail simulator that uses [verlet integration](https://en.m.wikipedia.org/wiki/Verlet_integration) and a simplified [FABRIK constraint solver](http://www.andreasaristidou.com/FABRIK.html). This simulator was originally created to aid in component selection for an animation tail.



For the purposes of this simulation, the tail is a chain of point masses connected by rigid bodies and constrained by angle of rotation. The forces exerted on the simulated tail are exerted in the same manner that a cable running either side of the length of the tail would exert them. In other words, the effect of the cable tension is simulated.



## Caveats
This simulation, like many simulations should not be regarded as completely physically accurate. Rather, it aims to be a good physical approximation that may help in selecting initial component parameters.

Simulation of long chains of rigid bodies is difficult as these systems already tend to exhibit a great deal of [chaos](https://en.wikipedia.org/wiki/Chaos_theory). In addition to this, as this simulator uses an integration based approach, integration errors will accumulate over time. 

Both of these factors can contribute to simulation instability. This simulator employs dampening techniques to help mitigate these effects, however they will certainly have an impact on the simulation accuracy. For that reason, the dampening factors can be adjusted by the user. 


## Usage
Simply clone the repo and run open index.html in your browser


## Acknowledgements
JSTailSim uses the code from the following projects:

* [SkeletonCSS](https://github.com/dhg/Skeleton)
* [GitHub's excellent gitignore repository](https://github.com/github/gitignore)

Additionally, the constraint solver technique used in JSTailSim was made possible by the research of [Dr. Andreas Aristidou](http://www.andreasaristidou.com/FABRIK.html). 