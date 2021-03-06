# Vehicle dynamics using Matlab: Stat-space models
This project has the following structure:
- +mt
	- +ss
	- +tools
- sym-modeling
- documentation
- examples
## +mt
This is the main folder library. In order to use this the user should include this line in his matlab script:

`import mt.*`

### +statespace

All state-space models will be added here:

- **carNonlinear :** Nonlinear state-space equation of car dynamics
- **carNonlinearVxConstant :** Nonlinear state-space equation with constant longitudinal velocity of car dynamics
- **carLinear :** Linear state-space equation of car dynamics. Linearize around (0,0)
- **carLinearVxConstant :** Linear state-space equation of car with constant longitudinal velocity.
- **semitrailerNonlinear :** Nonlinear state-space equation of semi-trailer dynamics.
- **semitrailerNonlinearVxConstant :** Nonlinear state-space equation of semi-trailer with constant longitudinal velocity. Linearize around (X0,U0) = (0,0)
- **semitrailerLinear :** Linear state-space equation of semi-trailer dynamics. Linearize around (X0,U0) = (0,0)
- **semitrailerLinearVxConstant :** Linear state-space equation of semi-trailer with constant longitudinal velocity. Linearize around (0,0)

### +tools 
Scripts use for modeling. The idea is to use this function for modeling using Euler-Lagrange Approach.

**transM :** Calculation of transformation matrix of a define Frame {V} with respect to a reference Frame {E}.

**Rz :** Calculation of rotation matrix around axis Z.

**linearizeStateSpace :** Linearize nonlinear state-space model around an stationary-point.

**dtGen :** Generation of symbolic derivative of vector. 

**dfdt :** Estimation of df/dt while 'f' depends on generalized coordinates 'q'. Chain rule is applied

**calQ :** Calculation of generalized force of a system as a function of the generalized coordinates.

**solveddq :** Estimate the expression of the second derivative of the generalized coordinates (q) based on q, first derivative of q, Kinetic and Potential energy and generalized force of a system. 

**rk4 :** Next discrete step calculation using Runge-Kutta 4 for state-stace equations.

**wagon :** Calculation of wheels and borders position with respect of Reference or Earth Frame. Inputs: vehicle position, inclination and dimensions of vehicle.

**plotCar :** Plotting minimalistic car shape.

## sym-modeling

**car-modelling.m :** Car model using single axle bicycle model using Euler-Lagrange equation. Documentation: documentation/car-modelling/car-modelling.pdf

**semitrailer-modelling.m :** Semitrailer model using single axle bicycle model using Euler-Lagrange equation. Documentation: documentation/semitrailer-modelling/semitrailer-modelling.pdf

## Examples

**plot_car_trajectory :** Example of plottting car using **carNonlinearVxConstant** state-space,**rk4**, **wagon** and **plotCar**. Steering input is constant.
**plot_car_trajectory_2 :** Example of plottting car using **carNonlinearVxConstant** state-space, **rk4**,  **wagon** and **plotCar**. Steering input is sinusoidal and then constant.
**plot_car_trajectory :** Example of plottting car using **semitrailerNonlinearVxConstant** state-space, **rk4**, **wagon** and **plotCar**. Steering is maneuver for lane change.

