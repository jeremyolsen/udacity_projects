
# Udacity Pick n Place Project
**Jeremy Olsen - slack: jolsen-oas**

[//]: #parameterseferences
[image1]: ./img/DH_Table.jpg
[image2]: ./img/combined_transform_equation.gif
[image3]: ./img/IK_thetas_2_3_tri.png
[image4]: ./img/IK_theta_1_drawing.png
[image5]: ./img/extrinsicxyz.png
[image6]: ./img/euler_alpha.gif
[image7]: ./img/euler_beta.png
[image8]: ./img/euler_gamma.gif
[image9]: ./img/all_euler_angles.png


## Kinematic Analysis
### 1. After setting up the environment, I started the Forward Kinematics demo. Using the URDF parameters I was able to perform the kinematic analysis on the Kuka KR210 and derived the table of DH parameters.

Description of DH Parameters from Lesson 2:
_Jacques Denavit and Richard Hartenberg proposed a systematic method of attaching reference frames to the links of a manipulator that simplified the homogeneous transforms. This method only requires four parameters to describe the position and orientation of neighboring reference frames._

- _α(i−1)_ **(twist angle)** = angle between _Z(i-1)_ and _Z(i)_ measured about X (i-1) in a right-hand sense.
- _a(i-1)_ **(link length)** = distance from _Z(i-1)_ to _Z(i)_ measured along _X(i-1)_ where _X(i-1)_ is perpendicular to both _Z(i-1)_ to _Z(i)_
- _d(i)_ **(link offset)** = signed distance from _X(i-1)_ to _X(i)_ measured along _Z(i)_.  _Note: this quantity will be a variable in the case of prismatic joints._
- _θ(i)_ **(joint angle)** = angle between _X(i-1)_ to _X(i)_ measured about _Z(i)_ in a right-hand sense.  _Note: this quantity will be a variable in the case of a revolute joint._

All the values are calculated from the information obtained in the KR210.urdf.xacro file.
 
 - a(i - 1) values - _notations are using the summed i-1 value (ie cell a2 would yield a(2-1) thus a1)_ - 
    - a1 : 0.35 - obtained from the joint_2 x origin value
    - a2 : 1.25 - obtained from the joint_3 z origin value
    - a3 : -0.054 - obtained from the joint_4 z origin value
 
 - d(i) values
    - d1 : 0.42 + 0.33 = 0.75 - obtained by summing the joint_2 z origin value and the joint_1 z origin value
    - d4 : 0.54 + 0.96 = 1.5 - obtained by summing the joint_5 x origin value and the joint_4 x origin value
    - d7 : 0.11 + 0.193 = 0.303 - obtained by summing the gripper_joint x origin value and the joint_6 x origin value
 

**Sketch of Kuka KR210 in Zero Configuration**
![DH Parameters Table][image1]

**DH Parameters Table for the Kuka KR210 robot arm**

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | .35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


### 2. Discussion of FK  

Rotations + Translations = Transforms. Below is the example homogeneous transform matrix in which the DH Table values will be used to replace the variables of alpha, a, d, and theta. And the composite transform matrix which is the generalized homogeneous transform between the base link and the gripper link.  

```python
 # Code Example of a Homogeneous Transform Matrix being populated by the DH parameters for the first link
 
T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                       [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                       [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                       [0, 0, 0, 1]])         
```
**Composition Homgenous Transform Equation** <br/>
![Composition Transform Equasion][image2]

```python
# Sum of all of the transform matrices will obtain the composite homogeneous transform from the base to the gripper link

t0_ee = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_ee
```
### 3. IK implementation - decoupling Inverse Orientation and Inverse Position

The goal of the Inverse Kinematics solution is to obtain the correct joint angles from the positions that are received in the request.  This is essentially the opposite of Forward Kinematics.  In the case of the Kuka KR210, we are able to break down the problem into two separate solutions, Inverse Position and Inverse Orientation, as it is able to satisfy the conditions necessary to be able to obtain a solution in 'closed form'.  The conditions are as follows:  
   - Three neighboring joint axes intersect at a single point, or
   - Three neighboring joint axes are in parallel.

The Kuka KR210, and most 6DOF manipulators, satisfies the first condition by containing a spherical wrist which will allow us to break the IK problem into two separate solution.  The position of the first 3 joints will determine the position of the wrist center which will be explained as a part of the Inverse Position solution.  

The second half of the problem, the Inverse Orientation solution, is basically a composition of rotations to orient the end effector, which will also be described below. 

**Inverse Position**
  
Before the thetas can be calculated, the wrist center needs to be derived. This is the position that the combination of the angles of joints 1-3 are going to move the center of the spherical wrist to.
  
  - Wrist Center: `wc = ee - 0.303 * rot_ee[:, 2]`
    - The calculation can be described as the end effector position, minus offset multiplied by the extracted z-axis orthonormal vectors from the `rot_ee`. `rot_ee` is the composite rotation matrix of all 6 joints, which was obtained from the rotation matrices that were created in the  forward kinematics section. 
  - Theta 1 
    - The first angle will be calculated by looking down on the robot arm's z-axis as shown by the illustration below.  Using the wrist center xy coordinates, the angle can be determined by utilizing the arc-tangent method as is shown here: `atan2(wc[1], wc[0])`.  Note: `atan2()` will be used throughout the project to determine angles.  By using `atan2()`, sign(+/-) ambiguity can be avoided with the angles vs. using sine and cosine explicitly.
    
**Visualizing Theta 1** 
![Visualizing Theta 1][image4]    
  - Thetas 2 & 3
    - The lesson provided an illustration (shown below), that visualizes the steps necessary to calculate the joint angles theta 1 & 2. There is a triangle that occurs between the 3 joints, from the base link and the wrist center.  The length of the 2 of the sides of the triangle are obtained from the values in the DH Table/URDF file and the 3rd side is derived using the Pythagorean Theorem. Theta's 2 and 3 are then able to be derived from using the Law of Cosines (SOH CAH TOA).

**Visualizing Thetas 2 & 3** <br/>
![Theta 2 3][image3]

**Inverse Orientation**

The solution the second half of the problem begins by deriving the rotation matrix for the last three joints. By utilizing the angles obtained in the Inverse Position solution and
substituting them inside the composite matrix for joints 1-3, this will produce the rotation matrix `r0_3`. This is utilized to determine the rotation matrix for joints 4-6 (`r3_6`), which will be described below.

- `r3_6 = inv(r0_3) * rot_ee`
    - `rot_ee` is the composite rotation from the base link to the gripper link. By using the inverse rotation of joints 1-3 (`inv(r0_3)`), and pre-multiply it with the composite `rot_ee`, we obtain the rotation matrix for the remaining joints (`r3_6`). 
    
**Thetas 4, 5, & 6**

The final step to determine the last three thetas begins by finding the set of Euler Angles, which is a system to describe a sequence of rotations. The orientation of any rigid body with respect to some fixed reference frame can always be described by 3 elementary rotations by a given sequence. In the case of rotations, order matters and is subjective to the situation in which they are used.

The lessons discussed the extrinsic (fixed-axis) XYZ rotation sequence which is shown in the following equation: 

![Extrinsic Euler XYZ Angle Equasion][image5]

Angles alpha, beta, gamma, which also are the angles theta 4,5 & 6, which is what is ultimately being solved for, can now be found since the values for rij (found in the left hand matrix in the illustration above) are known.  

The resulting equation to find beta is shown here:

![Euler Beta][image7]

and alpha:

![Euler Alpha][image6]

and lastly gamma:

![Euler Gamma][image8]

By using the `r3_6` rotation matrix, the extrinsic XYZ rotation approach to derive angles theta 4,5, & 6 can be accomplished. An example of this approach, which was used in the project code, is shown below:
```python

theta_4 = atan2(r3_6[2, 2], -r3_6[0, 2])

theta_5 = atan2(sqrt(r3_6[0, 2] * r3_6[0, 2] + r3_6[2, 2] * r3_6[2, 2]), r3_6[1, 2])

theta_6 = atan2(-r3_6[1, 1], r3_6[1, 0])
``` 

The extrinsic XYZ solution is not the only solution that can work to solve for the last three angles. Included below is a list list of all the Proper Euler and Tait-Bryan angles that can be utilized:

![All Euler Angles][image9]

## Project Implementation

### Discussion of the IK code implementation and results

The first step I performed was to setup remote debugging from PyCharms on my PC and connected to the VM to enable me to set breakpoints and step through code.

I primarily followed the walk-through examples with a variety of modifications.  The walk-through code that was presented was written clean and contained all of the major pieces.  The modifications I made to speed things up were to removed the whole FK portion from the for loop and have that run once on initialization of the service.  I had to add an initialization flag to ensure that any requests that were passed to the service were indeed run after the initialization had occurred.

I've commented on all of the major lines of code to show my understanding of the theory being implemented and to how each piece is performing it's role.  I've been comparing to other code that I people have posted on slack and found that there is not many differences, other then semantics, on how these routines can be implemented.  

The major area of subjectivity is how theta's 4-6 for the end-effector are being calculated.  Per research via internet and reading the slack channels, it seems that the Kuka KR210 EE rotations should be calculated using and intrinsic XYX rotations as the wrist joints are not configured as ZYX joints, but rather XYX.  I tried making some modifications using some built in method in the TF package (transformation.euler_from_matrix()) and found that I was getting inconsistent results even when using the same extrinsic ZYX approach as we are manually using here. I decided to leave well enough alone.

If I were to spend more time on this project, I would refine the theta 4-6 calculations as this seems to be the bulk of the problem with the choppy movements of the robot.
