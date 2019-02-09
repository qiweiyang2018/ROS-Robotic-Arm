### Writeup / README
[start]: ./misc_images/start.png

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Nck3ZNhzxek
" target="_blank"><img src="http://img.youtube.com/vi/Nck3ZNhzxek/0.jpg" 
alt="Robot Arm" width="960" height="720" border="10" /></a>


#### 1. Overview

At the first glance, this project contains too much information and knowledge to be digested in a reasonable amount of time, especially for a ROS beginner like me. Honestly speaking, from how the kuka_arm 3D model is built and controlled, to how the information is transfered between the target and the arm, is unknown to me. However, thanks to Udacity, the project is well designed so that I can focus on the transforms of links only without considering other stuffs that are beyond my knowledge for now. The contents of this project are definitely worthy to explore more to gain deeper understandings in ROS from all aspects.

[//]: # (Image References)

[image1]: ./misc_images/xarco_units.png
[image2]: ./misc_images/reference_frame.png
[theta1]: ./misc_images/theta1.png
[theta23]: ./misc_images/theta2.jpg
[result]: ./misc_images/result.png
[theta2]: ./misc_images/theta2.png
[theta3]: ./misc_images/theta3.png

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

##### Solution: The table below is the summary of reference frame in URDF file, from which the DH parameter table can be created. 
![alt text][image1]

#### The DH parameter table:

* Twist angle $\alpha_{i-1}$: angle between $\hat{Z}_{i-1}$ and $\hat{Z}_{i}$ measured about $\hat{X}_{i-1}$ in a right hand sense. 
* Link length $a_{i-1}$: distance from $\hat{Z}_{i-1}$ to $\hat{Z}_{i}$ measured along $\hat{X}_{i-1}$.
* Link offset $d_{i}$: signed distance from $\hat{X}_{i-1}$ to $\hat{X}_{i}$ measured along $\hat{Z}_{i}$.
* Joint angle $\theta_i$: angle between $\hat{X}_{i-1}$ and $\hat{X}_{i}$ measured about $\hat{Z}_{i}$ in a right hand sense. 

To be clear, $T_0$ represents the joint0 between fixed base link and link-1, $T_1$ represents the joint1 between link-1 and link-2, and so on so forth. 

However, $T_1^0$ represents the transform between joint0 and joint1.  

![alt text][image2]

i | Joint | $\alpha_{i-1}$ | $a_{i-1}$ | $d_{i}$ | $\theta_i$
--- | --- | --- | --- | --- | ---
1 |$T_1^0$ | 0   | 0  | 0.75 | $\theta_1$
2 |$T_2^1$ |$-\frac{\pi}{2}$ | 0.35| 0 | $\theta_2 - \frac{\pi}{2}$
3 |$T_3^2$ | 0 | 1.25 | 0 | $\theta_3$
4 |$T_4^3$ |  $-\frac{\pi}{2}$ | -0.054 | 1.5 | $\theta_4$
5 |$T_5^4$ | $\frac{\pi}{2}$ | 0 | 0 | $\theta_5$
6 |$T_6^5$ | $-\frac{\pi}{2}$ | 0 | 0 | $\theta_6$
7 |$T_G^6$ | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

##### Solution: The general form of a homogeneous transform between two reference frames described using DH convention can be written as follows: 

$T_{i-1}^i$ = $\begin{bmatrix}
    cos(\theta_i) & -sin(\theta_i) & 0 &  \alpha_{i-1} \\
    sin(\theta_i)cos(\alpha_{i-1}) & cos(\theta_i)cos(\alpha_{i-1}) & -sin(\alpha_{i-1}) &  -sin(\alpha_{i-1})d_i \\
    sin(\theta_i)sin(\alpha_{i-1}) & cos(\theta_i)sin(\alpha_{i-1}) & cos(\alpha_{i-1}) &  cos(\alpha_{i-1})d_i \\
    0 & 0 & 0 & 1
\end{bmatrix}$

Using this matrix, $T_1^0$, $T_2^1$, $T_3^2$,$T_4^3$,$T_5^4$,$T_6^5$,$T_G^6$, can be obtained accordingly. Therefore, the total homogeneous transform between the joint0 and the gripper can be found by multiplying the individual transforms together: 

$$T_G^0 = T_1^0 * T_2^1 * T_3^2 * T_4^3 * T_5^4 * T_6^5 * T_G^6$$

Note: the orientation of gripper in the URDF (RViz) and in DH parametera is different, as a consequence, transform conversion is required so that gripper can be correctly displayed in RViz. 

$$T_{G_{correct}}^0 = T_{G}^0 * R_{z}(\pi) * R_{y} (-\frac{\pi}{2})$$

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

#####solution: 

[Reference: Wrist center and spherical wrist](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/3bc41e14-e43d-4105-887c-8268a7402750)

Physically speaking, a six degree of freedom serial manipulator with a spherical wrist would use the first three joints to control the position of the wrist center while the last three joints would orient the end effector as needed.

In our project, the axes of joints 4, 5 and 6 intersect at a single point, which is the origin of the joint5, the position and orientation of the end effector are kinematically decoupled. Joints 4, 5, and 6 form a spherical wrist that determine the orientation of the end effector, while joints 1,2 and 3 determine the location of the end effector. 

(1) To find the location of the Wrist Center relative to the base frame. 

$$^0 r_{WC/0} = ^0r_{EE/0} - d \cdot ^0_6R \begin{bmatrix} 0\\0\\1\end{bmatrix}$$

As long as the location of WC relative to base frame is known, $\theta_1$ is known immediately, which is:
$$\theta_1 = atan2(Y_{wc}, X_{wc}),$$
take a look at this picture for reference:
![alt text][theta1] 



（2） To calculate $\theta_2$,$\theta_3$, we can assume $\theta_1 = 0$ for now, and refere to the diagram below: ![alt text][theta2]

* a = link length between link2 and link3
* b = Visual straight line length between link3 and link5
* c = Visual straight line length between link2 and link5
* d = link length between link3 and link4
* e = link length between link4 adn link5
* f = distance of link2 and link5 in XY plane (or the projection of c in XY plane)
* g = the height of link5
* a3 = the vertical offset between link3 and link4
* $\beta$ = the angle between a and c
* omega = the angle between c and f
* B = the angle between d and e

Let's calcuate length b first: 

$B = \pi -  asin(\frac{a3}{d})$
$b = \sqrt{d^2+e^2-2d*e*cos(B)}$

In addition, the position of link5 is same as wrist center, which is known already, and position of link2 can be calculated using DH parameter and rotating by $\theta_1$

$c = \sqrt{(J_{5x}-J_{2x})^2+(J_{5y}-J_{2y})^2+(J_{5z}-J_{2z})^2}$
$f = \sqrt{(J_{5x}-J_{2x})^2+(J_{5y}-J_{2y})^2}$
$g = J_{5z}-J_{2z}$
using law of cosines: 
$\beta = acos(\frac{b^2-a^2-c^2}{-2ac})$
$omega = atan2(g,f)$
therefore: 

$\theta_2 = \frac{\pi}{2} - \beta - omega$

By referrring to "Diagram for theta3", 
![alt text][theta3]

a,d,e, and a3 are from DH table
b and c have been calculated previously
let's define angle Y between a and b <a,b>
$Y = acos(\frac{c^2-a^2-b^2}{-2ab})$
$sigma = asin(\frac{a3}{b})$
therefore: 
$\theta_3 + (\pi - sigma - Y) = \frac{\pi}{2}$
$\theta_3 = sigma+Y-\frac{\pi}{2}$



#### Inverse Orientation

As mentioned previously, joint4,5,6 together to determine the orientation of the end effector. The overall orientation is solved based on the roll, pitch, and yaw Euler angles that are input to the inverse kinematics server. Since the first three joint angles are known, the last three joints angles must account for the difference in orientation between the first three joints and the total orientation. 

$$R_{total} = rot_z(yaw)*rot_y(pitch)*rot_x(roll) =$$ $$\begin{bmatrix}cos(yaw) & -sin(yaw)&0 \\ sin(yaw)&cost(yaw)& 0 \\0&0&1\end{bmatrix}*\begin{bmatrix}cos(pitch) & 0&sin(pitch) \\ 0&1& 0 \\-sin(pitch)&0&cos(pitch)\end{bmatrix}* \begin{bmatrix}1 & 0&0 \\ 0&cost(roll)& -sin(roll) \\0&sin(roll)&cos(roll)\end{bmatrix}$$

$$R_{3}^0 = R_1^0(\theta_1)*R_2^1(\theta2)*R_3^2(\theta3) =$$ $$\begin{bmatrix}cos(\theta_1) & -sin(\theta_1)&0 \\ sin(\theta_1)cos(\alpha_0)&cos(\theta_1)cos(\alpha_0)& -sin(\alpha_0) \\sin(\theta_1)sin(\alpha_0)&cos(\theta_1)sin(\alpha_0)&cos(\alpha_0)\end{bmatrix}*\begin{bmatrix}cos(\theta_2) & -sin(\theta_2)&0 \\ sin(\theta_2)cos(\alpha_1)&cos(\theta_2)cos(\alpha_1)& -sin(\alpha_1) \\sin(\theta_2)sin(\alpha_1)&cos(\theta_2)sin(\alpha_1)&cos(\alpha_1)\end{bmatrix}*\begin{bmatrix}cos(\theta_3) & -sin(\theta_3)&0 \\ sin(\theta_3)cos(\alpha_2)&cos(\theta_3)cos(\alpha_2)& -sin(\alpha_2) \\sin(\theta_3)sin(\alpha_2)&cos(\theta_3)sin(\alpha_2)&cos(\alpha_2)\end{bmatrix}$$
![alt text][image2]

Then, $R_6^3 = (R_3^0)^{-1} * R_{total}$

From lesson 14 part 8, we know:
$\alpha = \theta_4 = atan2(-r_{31}, \sqrt{r_{11}*r_{11}+r_{21}*r_{21}})$
$\beta = \theta_5 = atan2(r_{32}, r_{33})$
$\gamma = \theta_6 = atan2(r_{21}, r_{11})$

using atan2 function, the type of ambiguity is avoided. 
Joint 4, 5, 6, or $\theta_4, \theta_5, \theta_6$ can be found by transforming the orientation into Euler angles. These can be calculated by **euler_from_matrix()** function from the TF package. Code snippet as following: 
"alpha, beta, gamma = tf.transformations.euler_from_matrix(R3_6.tolist(), 'ryzy')"




### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
![alt text][result] 

The code was implemented according to the procedures described above. One of the problem is that sometimes arm rotates almost 360 degrees. The better solution is just check the absolute value of$\theta$, if it needs to move $\theta$, whihc is more than 180 degree, then make it move backward for 360 - $\theta$ degree.  








