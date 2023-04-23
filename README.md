Download Link: https://assignmentchef.com/product/solved-24677-project-3
<br>
In this project, you will complete the following goals:

<ol>

 <li>Design an lateral optimal controller</li>

</ol>

[Remember to submit the write-up, plots, and codes on Gradescope.]

<h1>1             Model</h1>

The error-based linearized state-space for the lateral dynamics is as follows.

<em>e</em><sub>1 </sub>is the distance to the center of gravity of the vehicle from the reference trajectory. <em>e</em><sub>2 </sub>is the orientation error of the vehicle with respect to the reference trajectory.

In lateral vehicle dynamics, <em>ψ</em><sup>˙</sup><em><sub>des </sub></em>is a time-varying disturbance in the state space equation. Its value is proportional to the longitudinal speed when the radius of the road is constant. When deriving the error-based state space model for controller design, <em>ψ</em><sup>˙</sup><em><sub>des </sub></em>can be safely assumed to be zero.

For the longitudinal control:

Assuming <em>ψ</em><sup>˙ </sup>= 0:

<h1>2             P3: Problems</h1>

<strong>Exercise 1. </strong>For the lateral control of the vehicle, design a discrete-time infinite horizon LQR controller.

You can reuse your longitudinal PID controller from part 1 of this project, or even improve upon it. However, it may require retuning based on observed performance.

Design the two controllers in your controller.py. You can make use of Webots’ builtin code editor, or use your own.

Check the performance of your controller by running the Webots simulation. You can press the play button in the top menu to start the simulation in real-time, the fast-forward button to run the simulation as quickly as possible, and the triple fast-forward to run the simulation without rendering (any of these options is acceptable, and the faster options may be better for quick tests). If you complete the track, the scripts will generate a performance plot via matplotlib. This plot contains a visualization of the car’s trajectory, and also shows the variation of states with respect to time.

Submit your controller.py and the final completion plot as described on the title page. Your controller is <strong>required </strong>to achieve the following performance criteria to receive full points:

<ol>

 <li>Time to complete the loop = 250 s</li>

 <li>Maximum deviation from the reference trajectory = 7.0 m</li>

 <li>Average deviation from the reference trajectory = 3.5 m</li>

</ol>

Some hints that may be useful:

<ul>

 <li>Make sure to discretize your continuous state-space system with the provided timestep (delT) prior to solving the ARE.</li>

 <li>Using LQR requires manually creating two matrices <em>Q </em>and <em>R</em>. <em>Q </em>works to penalize state variables, while <em>R </em>penalizes control input. Try to think about what form your <em>Q </em>and <em>R </em>matrices should take for good performance.

  <ul>

   <li>For <em>Q</em>, large values will heavily restrict changes to the respective states, while small values will allow the states to easily change.</li>

   <li>Similarly, in <em>R</em>, large values will heavily restrict control input, while small values will allow the control input to vary widely.</li>

   <li>One idea for tuning is to set the relevant indices of <em>Q </em>and <em>R </em>to</li>

  </ul></li>

</ul>

1

<h2>(max value of the corresponding state/input)<sup>2</sup></h2>

in order to normalize the value. Make sure to experiment outside of this guideline to determine the best performance.

<strong>[10% Bonus]: </strong>Complete the loop within 130 s. The maximum deviation and the average deviation should be within in the allowable performance criteria mentioned above.

<h1>3             Appendix</h1>

<h2>(Already covered in P1)</h2>

Figure 1: Bicycle model[2]

Figure 2: Tire slip-angle[2]

We will make use of a bicycle model for the vehicle, which is a popular model in the study of vehicle dynamics. Shown in Figure 1, the car is modeled as a two-wheel vehicle with two degrees of freedom, described separately in longitudinal and lateral dynamics. The model parameters are defined in Table 2.

<h2>3.1           Lateral dynamics</h2>

Ignoring road bank angle and applying Newton’s second law of motion along the y-axis:

<em>ma</em><em>y </em>= <em>F</em><em>yf </em>cos<em>δ</em><em>f </em>+ <em>F</em><em>yr</em>

whereis the inertial acceleration of the vehicle at the center of geometry in the direction of the y axis, <em>F<sub>yf </sub></em>and <em>F<sub>yr </sub></em>are the lateral tire forces of the front and rear wheels, respectively, and <em>δ<sub>f </sub></em>is the front wheel angle, which will be denoted as <em>δ </em>later. Two terms contribute to <em>a<sub>y</sub></em>: the acceleration ¨<em>y</em>, which is due to motion along the y-axis, and the centripetal acceleration. Hence: <em>a<sub>y </sub></em>= <em>y</em>¨+ <em>ψ</em><sup>˙</sup><em>x</em>˙

Combining the two equations, the equation for the lateral translational motion of the vehicle is obtained as:

Moment balance about the axis yields the equation for the yaw dynamics as

<em>ψI</em>¨ <em>z </em>= <em>l</em><em>fF</em><em>yf </em>− <em>l</em><em>rF</em><em>yr</em>

The next step is to model the lateral tire forces <em>F<sub>yf </sub></em>and <em>F<sub>yr</sub></em>. Experimental results show that the lateral tire force of a tire is proportional to the “slip-angle” for small slip-angles when vehicle’s speed is large enough – i.e. when ˙<em>x </em>≥ 0<em>.</em>5 m/s. The slip angle of a tire is defined as the angle between the orientation of the tire and the orientation of the velocity vector of the vehicle. The slip angle of the front and rear wheel is

<em>α</em><em>f </em>= <em>δ </em>− <em>θ</em><em>V f</em>

<em>α</em><em>r </em>= −<em>θ</em><em>V r</em>

where <em>θ<sub>V p </sub></em>is the angle between the velocity vector and the longitudinal axis of the vehicle, for <em>p </em>∈ {<em>f,r</em>}. A linear approximation of the tire forces are given by

!

where <em>C<sub>α </sub></em>is called the cornering stiffness of the tires. If ˙<em>x &lt; </em>0<em>.</em>5 m/s, we just set <em>F<sub>yf </sub></em>and <em>F<sub>yr </sub></em>both to zeros.

<h2>3.2           Longitudinal dynamics</h2>

Similarly, a force balance along the vehicle longitudinal axis yields:

<em>x</em>¨ = <em>ψ</em><sup>˙</sup><em>y</em>˙ + <em>a<sub>x</sub></em>

<em>ma<sub>x </sub></em>= <em>F </em>− <em>F<sub>f </sub>F<sub>f </sub></em>= <em>fmg</em>

where <em>F </em>is the total tire force along the x-axis, and <em>F<sub>f </sub></em>is the force due to rolling resistance at the tires, and <em>f </em>is the friction coefficient.

<h2>3.3           Global coordinates</h2>

In the global frame we have:

<em>X</em>˙ = <em>x</em>˙ cos<em>ψ </em>− <em>y</em>˙ sin<em>ψ</em>

<em>Y</em>˙ = <em>x</em>˙ sin<em>ψ </em>+ <em>y</em>˙ cos<em>ψ</em>

<h2>3.4           System equation</h2>

Gathering all of the equations, if ˙<em>x </em>≥ 0<em>.</em>5 m/s, we have:

<em>Y</em>˙ = <em>x</em>˙ sin<em>ψ </em>+ <em>y</em>˙ cos<em>ψ</em>

otherwise, since the lateral tire forces are zeros, we only consider the longitudinal model.

<h2>3.5           Measurements</h2>

The observable states are:

<em>x</em>˙ 

<em>y</em>˙ 

<em>ψ</em>˙  <em>y </em>= <sub> </sub>

<em>X</em>

 

<em>Y </em> <em>ψ</em>

<h2>3.6           Physical constraints</h2>

The system satisfies the constraints that:

<em>F </em>&gt; 0 and <em>F </em>6 15736 <em>N</em>

<em>x</em>˙ &gt; 10<sup>−5 </sup><em>m/s</em>

Table 1: Model parameters.

<table width="596">

 <tbody>

  <tr>

   <td width="68">Name</td>

   <td width="278">Description</td>

   <td width="66">Unit</td>

   <td width="183">Value</td>

  </tr>

  <tr>

   <td width="68">(<em>x,</em>˙ <em>y</em>˙)</td>

   <td width="278">Vehicle’s velocity along the direction of vehicle frame</td>

   <td width="66">m/s</td>

   <td width="183">State</td>

  </tr>

  <tr>

   <td width="68">(<em>X,Y </em>)</td>

   <td width="278">Vehicle’s       coordinates    in         the          world frame</td>

   <td width="66">m</td>

   <td width="183">State</td>

  </tr>

  <tr>

   <td width="68"><em>ψ</em>, <em>ψ</em><sup>˙</sup></td>

   <td width="278">Body yaw angle, angular speed</td>

   <td width="66">rad, rad/s</td>

   <td width="183">State</td>

  </tr>

  <tr>

   <td width="68"><em>δ </em>or <em>δ<sub>f</sub></em></td>

   <td width="278">Front wheel angle</td>

   <td width="66">rad</td>

   <td width="183">State</td>

  </tr>

  <tr>

   <td width="68"><em>F</em></td>

   <td width="278">Total input force</td>

   <td width="66">N</td>

   <td width="183">Input</td>

  </tr>

  <tr>

   <td width="68"></td>

   <td width="278">mass</td>

   <td width="66"> </td>

   <td width="183"> </td>

  </tr>

  <tr>

   <td width="68"><em>l<sub>f</sub></em></td>

   <td width="278">Length from front tire to the center of mass</td>

   <td width="66">m</td>

   <td width="183">1.55</td>

  </tr>

  <tr>

   <td width="68"><em>C<sub>α</sub></em><em>I<sub>z</sub></em></td>

   <td width="278"> </td>

   <td width="66"> </td>

   <td width="183">25854</td>

  </tr>

  <tr>

   <td width="68"><em>F<sub>pq</sub></em></td>

   <td width="278"><em>p </em>∈ {<em>x,y</em>},<em>q </em>∈ {<em>f,r</em>}</td>

   <td width="66"> </td>

   <td width="183"> </td>

  </tr>

  <tr>

   <td width="68"><em>f</em></td>

   <td width="278"> </td>

   <td width="66">sec</td>

   <td width="183"> </td>

  </tr>

 </tbody>

</table>

<h2>3.7           Simulation</h2>

Figure 3: Simulation code flow

Several files are provided to you within the controllers/main folder. The main.py script initializes and instantiates necessary objects, and also contains the controller loop. This loop runs once each simulation timestep. main.py calls your controller.py’s update method on each loop to get new control commands (the desired steering angle, <em>δ</em>, and longitudinal force, <em>F</em>). The longitudinal force is converted to a throttle input, and then both control commands are set by Webots internal functions. The additional script util.py contains functions to help you design and execute the controller. The full codeflow is pictured in Figure 3.

Please design your controller in the your controller.py file provided for the project part you’re working on. Specifically, you should be writing code in the update method. Please <strong>do not </strong>attempt to change code in other functions or files, as we will only grade the relevant your controller.py for the programming portion. However, you are free to add to the CustomController class’s init  method (which is executed once when the CustomController object is instantiated).

<h2>3.8           BaseController Background</h2>

The CustomController class within each your controller.py file derives from the BaseController class in the base controller.py file. The vehicle itself is equipped with a Webots-generated GPS, gyroscope, and compass that have no noise or error. These sensors are started in the BaseController class, and are used to derive the various states of the vehicle. An explanation on the derivation of each can be found in the table below.

Table 2: State Derivation.

<table width="347">

 <tbody>

  <tr>

   <td width="68">Name</td>

   <td width="278">Explanation</td>

  </tr>

  <tr>

   <td width="68">(<em>X,Y </em>)</td>

   <td width="278">From GPS readings</td>

  </tr>

  <tr>

   <td width="68"> </td>

   <td width="278"></td>

  </tr>

 </tbody>

</table>

<h2>3.9           Trajectory Data</h2>

The trajectory is given in buggyTrace.csv. It contains the coordinates of the trajectory as (<em>x,y</em>). The satellite map of the track is shown in Figure 4.

Figure 4: Buggy track[3]

<h1>4             Reference</h1>

<ol>

 <li>Rajamani Rajesh. Vehicle Dynamics and Control. Springer Science &amp; Business Media, 2011.</li>

 <li>Kong Jason, et al. “Kinematic and dynamic vehicle models for autonomous driving control design.” Intelligent Vehicles Symposium, 2015.</li>

 <li>org, <a href="https://cmubuggy.org/reference/File:Course_hill1.png">https://cmubuggy.org/reference/File:Course_hill1.png</a></li>

 <li>“PID Controller – Manual Tuning.” <em>Wikipedia</em>, Wikimedia Foundation, August 30th,</li>

 <li><a href="https://en.wikipedia.org/wiki/PID_controller#Manual_tuning">https://en.wikipedia.org/wiki/PID_controller#Manual_tuning</a></li>

</ol>