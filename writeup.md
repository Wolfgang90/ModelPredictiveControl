# Writeup Udacity Model Predictive Control (MPC) Project

## 1 Model description

The implemententaton of the MPC is located in the second part of section `2.0 Process input values from simulator` in the file `main.cpp` as well as in the file `MPC.cpp` which is called from `main.cpp`.

In `main.cpp` `2.4 Calculate current state taking into account simulator latency` the current state in which the simulator will be in, after accounting for the actuator latency, is calculated. The state includes the coordinates `x` and `y`, the orientation `psi`, the speed `v`, the cross track error `cte` and the orientation error `epsi`. `2.5 Make prediction for the upcoming states` calls `mpc.Solve()` to predict the upcoming states. Based on the output the actuator output is calculated.
```
165           msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);
166           msgJson["throttle"] = vars[1];
```
The core part of the model in `MPC.cpp` consists of the class `FG_eval` and the class `MPC` and its `Solve()` function. 

The class `FG_eval` updates the vector `fg` with the costs for the current state and sets the constraints.

`MPC::Solve()` sets upper and lower bounds for the non-actuator state variables, for the actuators, for acceleration and deceleration and for the constraints. Subsequently, `FG_eval` is employed to calculate costs and to set constraints. Finally `CppAD::ipopt` is utilized to perform the nonlinear optimization for the optimal solution path.

## 2 Timestep Length and Elapsed Duration (N & dt)

I ended up with a timestep length `N` of 10 and an elapsed duration between timesteps `dt` of 0.1. A longer length `N` requires more capacity without adding significant improvement. In connection with `N = 10` a duration `dt` of 0.1 displayed good results. One could also reduce `dt` to 0.05 with good results. However, one also would have to increase `N` which would require more calculation capacity and does not improve the result.

## 3 Polynomial Fitting and MPC Preprocessing

The section `2.0 Process input values from simulator` in the file `main.cpp` contains polynomial fitting and MPC preprocessing. In `2.1 Operationalize values received from simulator` the variables obtained from the simulator as a JSON object are stored individually in variables. The global map coordinates are subsequently transformed into the local car coordinate system in `2.2 Transform desired track coordinates (ptsx and ptsy) from global map coordinate system to local car coordinates`. In `2.3 Fit desired track coordinates in local car coordinates` the function `polifit()` is used to find the coefficients for a 3rd-order polynomial best fitting the determined local car coordinates.

## 4 Model Predictive Control with Latency
In `main.cpp` `2.4 Calculate current state taking into account simulator latency` the actuator latency is addressed. Based on the latency value of 100 milliseconds and the current actuator status x, y, psi and v are projected 100 milliseconds into the future.

```
133           double x_projected = v * latency;
134           double y_projected = 0;
135           double psi_projected = -v * steer_value / Lf * latency;
136           double v_projected = v + throttle_value * latency;
```

These values are subsequently used to describe the state of the system used as input for `mpc.Solve()` determining the required adjustments to the actuators.
