# Reflections


## Description of the Model

The model consists of a state vector that consists of 6 elements, these are: 

* x (vehicle x position)
* y (vehicle y position)
* psi (vehicle heading)
* v (vehicle velocity)
* cte (cross track error)
* epsi (heading error)

We have control over this system with two inputs :

* delta (steering)
* a (throttle)


The motion model of the system is described with the following equations:

    x   [t + 1] = x[t] + v[t] * cos(psi[t]) * dt;
    y   [t + 1] = y[t] + v[t] * sin(psi[t]) * dt;
    psi [t + 1] = psi[t] + v[t]/LF*psi[t] * delta[t] *dt;
    v   [t + 1] = v[t] + a[t] * dt;
    cte [t + 1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt;  
    epsi[t + 1] = psi[t] - psi_dest[t] + v[t]/Lf *delta[t] *dt; 

Where f(x) is the polynomial fitted to the waypoints evaluated at 'x'.


Tho adjust the system I tuned the cost function and added weights. I also modified the cross track error component so as to increase weight for the future errors, because I thought would give a better result.

In addition, I saw large throttle and steering values decreased the stability. So I heavily penalized the use of large steering values and changes in both steering and throttle values.

Cost function formulation is as follows.

	  // Initial element is the cost function
	  fg[0] = 0;
	  
	  
	  for (int t = 0; t<N; t++)
	  {
		  // Penalize Cross track error and psi error
		  // penalize future cte more than the current.
		  fg[0] += 10*t*CppAD::pow(vars[cte_start + t],2);
		  fg[0] += 1000*CppAD::pow(vars[epsi_start + t],2);
		  // Penalize speed error
		  fg[0] += 10*CppAD::pow(vars[v_start + t]-ref_v,2);
	  }
	  
	  for (int t = 0; t<N-1; t++)
	  {
		  // Penalize over use of throttle and steering
		  fg[0] += 60000*CppAD::pow(vars[delta_start + t],2);
		  fg[0] += 20*CppAD::pow(vars[a_start + t],2);
	  }
	  
	  for (int t = 0; t<N-2; t++)
	  {
		  // Penalize sharp movements
		  fg[0] += 100 * CppAD::pow(vars[delta_start + t] - vars[delta_start + t + 1],2);
		  fg[0] += 50*CppAD::pow(vars[a_start + t] - vars[a_start + t + 1],2);
	  }


## Timestamp Length and Duration

For timestamp length I chose 10 elements, separated with 0.5 seconds. This forsees far enough into future to be useful but not too far to slow down the computations.


## Polynomial Fitting and Waypoint Preprocessing

Before polynomial fitting I translated the waypoints to vehicle coordinates. Which will be useful for projecting into future as will be described in the next heading.  

After that, I fit a polynomial of 2nd order to the converted waypoints using `folyfit()` function.

## Dealing with Latency

In order to deal with latency I projected the vehicle x, y coordinates 100 miliseconds into the future and and then converted the waypoints to the future coordinates where the actuation will begin. 

I projected only x, and y points into the future. I also experimented with projecting the psi value into the future but I did not get better results, so I decided not to project this value. I never projected velocity at the moment. However projecting the velocity might increase the performance. 


## Simulation - Result
Here is a result of the MPC simulation doing two complete laps around the track that I recorded on my system:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/IOvOYyKB6b4/0.jpg)](https://youtu.be/IOvOYyKB6b4)
(Clicking on the image will take you to the youtube video.)

Please note that, I optimized the system for my own cpu. If you have a more recent cpu you might get different results due to cpu time constraint of the optimizer. You might need to reduce the cpu time of the optimizer to get a similar result.


