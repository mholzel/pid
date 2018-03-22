# PID Car Control
In this project, we will develop a PID controller to steer a car around a track. The difficulty imposed by this project is in the fact that we don't have a baseline model for the system. Instead, we will be using [this simulator](https://github.com/udacity/self-driving-car-sim/releases), which accepts a steering angle and throttle for the car, and returns the speed and cross-tracking error. Our goal is to have the car stay on the track for sufficiently high speeds, so we want to reduce this cross-tracking error. 

## Handling the lack of model

Why is designing a PID controller without a model hard? 

The reason that it is hard to develop a controller without a model is that it is hard to test different PID gains. For instance, suppose we start with an initial guess for the PID gains of `[0, 0, 0]`. To test these gains, we would have to start the executable, then click through a bunch of buttons to simulate the car with these parameters, then compute whatever cost function we want to optimize.  In optimization problems, we typically would run through 1000's of guesses for these parameters. If we don't have a model for the system, and we want to test 1000's of parameters, we would have to go through this process 1000's of times. What a drag...

Therefore, if we don't want to have to open the test program 1000's of times, we have two options:

1. Either we optimize the PID controller online (while the simulation is running). 
2. We try to generate a model of the system offline, and tune the PID controller for that model.

## Online optimization

First, we are going to try to optimize the PID controller online. Even if you want to use approach 2 outlined above, you will need to accomplish this step just so that you can get data to perform the system identification. Hence doing some type of online optimization is probably necessary. Furthermore, if we can meet our target using just online optimization, we don't need to generate an offline model. So we will start here. 

The first consideration when doing any type of optimization is in determining what cost function we want to optimize. For example, letting $e(t_k)$ denote the tracking error at time $t_k$, we might try to minimize the sum of the squares of the tracking errors over some interval $t_1,\ldots,t_N$:
$$
\dfrac{1}{N}\sum_{k=1}^{N} e^2(t_k)
$$
or the sum of the absolute values 
$$
\dfrac{1}{N}\sum_{k=1}^{N} \Big| e(t_k) \Big|
$$
or just the maximum error over that interval.  

For simplicity, let's say that we want to minimize the sum of squares. Great... Which time interval should we optimize? 

Surprisingly, there are a lot of choices here. For example, let's say we are at some arbitrary time in the simulation. We could try to design a controller that will optimize the controller to minimize the cost function

1. over the previous $N$ time steps. 
2. over the next $N$ time steps. 
3. some combination of the two. 

Let's go over why you would choose each of these... 

1. It might seem strange to try to optimize the a controller to fit the previous $N$ time steps, but there is one very good reason to do it... we have measurements of the inputs and outputs at those times. Therefore we can update our controller to *the one that we **should** have used in the previous $N$ time steps*. If the dynamics don't change significantly, then presumably this controller will also be good in the near future. 
2. Obviously, we want to minimize the cost over the next $N$ time steps. We can't change the errors that we saw in the past, but we CAN change what happens in the future... right? So this interval may seem like the logical choice. 
3. Best of both worlds I guess... 

### Optimizing the future

There are essentially two ways of optimizing over the next $N$ time steps: 

1. Either we can estimate a model for the system in the background, and then update the controller to fit that model as the model changes. 
2. We can just propose a new controller, see how it does over the next few time steps, and then either keep or revert the controller, depending on how it does. 

For simplicity, we will choose the second approach, and use *twiddle* to control how we change the controller parameters. 

### Twiddling in *the Matrix*

We are going to try to use *twiddle*, which essentially just iterates over the list of parameters, perturbing each one. Instead of describing how it works, just review this descriptive pseudocode:

```
def twiddle( cost_function, initial_guess, perturbations, tol = 0.2, max_iterations = 1e3 ): 
    
    # Make sure that the perturbations are positive
    perturbations = [ abs(p) for p in perturbations ]
    
    # Our initial best guess is just the initial guess
    parameters    = initial_guess
    cost          = cost_function( parameters )
    
    # Stop when the perturbations are small in every dimension
    # Note that the list of perturbations will remain positive
    iteration     = 0
    while iteration < max_iterations and sum(perturbations) > tol:
    
        iteration += 1
        
        # Perturb each axis of the parameter vector, one at a time. 
        for i in range(len(parameters)):
        
			parameters[i] += perturbations[i]
			tmp_cost       = cost_function( parameters )

			# If the perturbation was good, keep it, and increase the perturbation next time 
			if tmp_cost < cost:
				cost              = tmp_cost
				perturbations[i] *= 1.1
			else:
				# Otherwise, try perturbing in the opposite direction
				parameters[i] -= 2 * perturbations[i]
				tmp_cost       = cost_function( parameters )

				# If that worked, keep it, and increase the perturbation next time 
				if tmp_cost < cost:
					cost              = tmp_cost
					perturbations[i] *= 1.1
				else:
					# Otherwise, reset the value, and decrease the perturbation next time
					parameters[i]    += perturbations[i]
					perturbations[i] *= 0.9
	return parameters
```

Although this pseudocode describes how one would optimize parameters in general, it doesn't really work all that well for our specific scenario. To understand why, you need to understand how our code will interact with the simulator... 

The simulator interacts with our code using `WebSockets`. Specifically, it sends us packets of telemetry data at semi-regular intervals containing:

1. crosstrack error
2. speed
3. current steering angle
4. current throttle
5. some images from the car (I think)

Similarly, it accepts steering angle and throttle commands over `WebSockets`.  In principle, you can think of our code as interacting with the simulator like this 

```
int main(){
    
    /* Set up the connection */
    WebSocket socket = ... 
    
    /* Execute when each new piece of telemetry arrives */
    socket.onNewTelemetry(...
        
        /* Read the telemetry data */
        Telemetry telemetry = ...
        double crosstrack_error = telemetry.crosstrack_error;
    
    	/* Compute the steering angle and throttle that we want to use */
    	double angle = ... 
    	double throttle = ... 
    
        /* Send the desired commands */
        send( angle, throttle );
    )    
}
```

### A Thread-based Twiddler 

The problem is that it would be difficult to design a `cost_function` that we could pass to the above `twiddle` algorithm that would work *as is* because such a function would have to pause execution of `twiddle ` until a new piece of telemetry comes in. After that, we could pass the error to the cost function and see if it the cost function has finished. Specifically, in very rough pseudocode, the `cost_function` would almost certainly have to be embedded in a class (to retain state) and would have to look something like this 

```
class Cost {
    
    int iteration = 0;
    double cost = 0;
    int N = 100;
    PID parameters;
    
    void onNewErrorMeasurement( error ){
        ++iteration;
        cost += error^2;
        if (iteration > N ){
            wake( twiddle_thread ); // resume twiddling
        }
    }
    
    double compute_cost( parameters ){
        this.parameters = parameters;
        sleep( twiddle_thread ); // until enough measurements
        tmpCost = cost;
        cost = 0
        iteration = 0;
        return tmpCost;
    }    
}
```

Similarly, the `main` code could be modified like this    

    socket.onNewTelemetry(...
        
        /* Read the telemetry data */
        Telemetry telemetry = ...
        double crosstrack_error = telemetry.crosstrack_error;
        
        /* Inform the cost function of the new error */
        cost.onNewMeasurement( crosstrack_error )
        
        /* Get the PID controller from the cost function */
        PID pid = cost.parameters;
        
    	/* Compute the steering angle and throttle that we want to use */
    	double angle = pid....
    	double throttle = pid...
    
        /* Send the desired commands */
        send( angle, throttle );
    )    
 I am outlining the approach in case you want to try it. However, I did not want to deal with threading, so instead I designed an alternative `Twiddle` class which works with the event-based model we are using. 

### An Event-based Twiddler

Instead of messing with the threading model, we can also change the structure of twiddle to an event-based paradigm. Specifically, in this paradigm, twiddle acknowledges that the cost function must be computed after a series of events are received (in our case, telemetry data). So we design a `Twiddle` class that accepts a cost function object which has an `update` method that updates the cost function in response to the new measurement, returning the cost if it has finished computing for the specified set of parameters, and returns a negative number if it hasn't (we assume that the cost is, by definition, nonnegative). For instance, the cost class might look like this

```
class Cost {

    const int N = 100
    double cost = 0
    int iteration = 0
    
	double update(error) {
	
	   ++iteration
	   cost += error
	   
	   if iteration < N
	       return -1
	   else 
	       tmpCost = cost
	       cost = 0
	       iteration = 0
	       return tmpCost
	}
}
```

the `Twiddle` class like 

```
class Twiddle {
    
    double lowest_cost
    Parameters best_parameters;
    Parameters parameters;

    Parameters &update(const Event &event) {

        CostValue cost = cost_function.update(event);
        if (cost >= 0)
            /* The cost function has finished computing the cost for the current set of 
               parameters. See how good we did and modify the parameters */
            if (cost < lowest_cost)
                best_parameters = parameters
                lowest_cost = cost
            
            /* Next, let's twiddle to try a new set of parameters */
            parameters = ...
        
        /* Return the parameters that the simulator should use for the next few iterations */
        return parameters
    }
}
```

and the `main` class like this 

```
socket.onNewTelemetry(...
    
    /* Read the telemetry data */
    Telemetry telemetry = ...
    double crosstrack_error = telemetry.crosstrack_error;
    
    /* Tell twiddle that we have a new event and get the pid controller that it 
       wants us to use in the next iteration */
    PID & pid = twiddle.update( crosstrack_error )

    /* Compute the steering angle and throttle that we want to use */
	double angle = pid....
	double throttle = pid...

    /* Send the desired commands */
    send( angle, throttle );
)    
```

There are a lot more details, but this is the approach we take. The primary advantage of this structure is that it avoids touching the threading model, so it is overall less intrusive to how the simulator was designed to be used. 

### Twiddling considerations

#### Early termination

We want to the car to be able to go around the entire track while staying close to the center of the lane. So your natural inclination might be to say that we want the time interval to be *"However many samples it takes to go around the track"*. However, we expect that the twiddle might produce some PID gains that are bad, and so the cost function might be able to exit early, thereby allowing the twiddler to try a new set of gains. 

This turns out to be quite easy to do. Specifically, since our cost is the average squared error, we can check at each iteration whether the cost will definitely be above the best cost found so far. For instance, if our cost function is averaging the errors over $N=100$ iterations, but we find that after 5 iterations
$$
\dfrac{1}{N}\sum_{k=1}^{5} e^2(t_k) > \texttt{lowest cost found so far }
$$
then we know that we can exit early because the square of the error is always nonnegative, so the cost can not decrease as we observe more samples. In the best case, the rest of the errors will be 0, and the cost will remain the same. So in this example, after 5 time steps, we can already say that the current PID gains are bad, and that we should try something else.

#### Ensuring fair simulation conditions

Suppose that PID controller 1 does bad. At the end of $N$ iterations, it has actually increased the error relative to when it started. 

Next, twiddle give us PID controller 2, and it actually does pretty good. The errors are large at the beginning because controller 1 started us in such a bad position, but at the end of $N$ iterations, controller 2 brings us back to a crosstrack error of approximately 0. Great. 

Next, twiddle produces PID controller 3. And what do you know... PID controller 3 leave us on the center line, right where controller 3 left off. 

What is the problem with this scenario? Well the cost function for controller 3 will always be better than controller 2 because controller 2 had to correct for its really bad starting position, and there were a lot of large initial errors. 

Therefore, to try to make the simulations as fair as possible for each hypothetical controller, we take the following two measures:

1. The cost function has a parameter `skip_iterations` which specifies how many of the first few errors to exclude from the cost. This should be a small number, representative of the number of iterations you would expect a good controller to take to bring itself back to the centerline. For instance, if this is 5, you are effectively saying that all good controllers should be able to correct for a bad initial position in 5 iterations, so we are only going to start computing its cost after those iterations. Bad controllers will still be bad, but good controllers won't take a hit if they were preceded by a bad controller.

2. We make sure that all controllers start with a reasonably bad initial state. Specifically, in the code, you will see that we do not start optimizing a controller until 

   ```
   speed > 15 && abs(crosstrack_error) > 0.5
   ```

   This ensures that no controller will start from the centerline. They won't all be given the exact same initial condition, but nobody will be significantly better than the other.

#### Modifying the cost to include speed

Originally, our cost function just used the RMSE of the crosstrack error
$$
\dfrac{1}{N}\sum_{k=1}^{N} e^2(t_k)
$$
This turns out to be a bad idea because this will reward cars that get to the centerline and just stop moving. In some simulations, we saw the optimizer converge to a PID controller that just oscillates the wheel from *hard-right* to *hard-left* at a very high frequency. The result of doing this is that no matter what throttle you specify, the car sits still. So if it managed to get back to the centerline, then it will just sit at the centerline, and return a cost function of effectively 0. Therefore we modified the cost function to reward going fast by simply dividing by the velocity, $v$, that is,
$$
\dfrac{1}{N}\sum_{k=1}^{N} \dfrac{ e^2(t_k) }{v(t_k)}
$$

## Enough Talk

Let's try running this thing. To train the PID controller, you should modify `CMakeLists.txt` to compile `main_train.cpp` instead of `main.cpp`. As command line arguments you can pass the following:

1. Initial proportional gain 
2. Initial integral gain 
3. Initial derivative gain 
4. The number of cost iterations to average over 
5. The number of iterations to skip when computing the cost (as described in the *fair simulation* section)
6. The initial amount by which to perturb the proportional gain
7. The initial amount by which to perturb the integral gain
8. The initial amount by which to perturb the derivative gain
9. The throttle to use

In code, that looks like this:

```
const T kp = argc > 1 ? stringTo<T>(argv[1]) : 0.6;
const T ki = argc > 2 ? stringTo<T>(argv[2]) : 0.006;
const T kd = argc > 3 ? stringTo<T>(argv[3]) : 1.5;
const int cost_iterations = argc > 4 ? stringTo<int>(argv[4]) : 1000;
const int cost_skip_iterations = argc > 5 ? stringTo<int>(argv[5]) : 3;
const T kp_perturb = argc > 6 ? stringTo<T>(argv[6]) : 0.1;
const T ki_perturb = argc > 7 ? stringTo<T>(argv[7]) : 0.001;
const T kd_perturb = argc > 8 ? stringTo<T>(argv[8]) : 0.1;
const T default_throttle = argc > 9 ? stringTo<T>(argv[9]) : 0.3;
```

So feel free to specify anywhere between 1 and 9 inputs. An example of the training process is shown in this video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=HCu9NAZYuqg
" target="_blank"><img src="http://img.youtube.com/vi/HCu9NAZYuqg/0.jpg" 
alt="training" width="100%" border="10" /></a>




## Quick start 

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
