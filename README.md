# Robot-Arm

## [Forward Kinematics](https://www.rosroboticslearning.com/forward-kinematics)
Forward kinematics computes the position and orientation of the end effector given the joint parameters (angles or displacements). The Denavit-Hartenberg (DH) convention is often used to simplify the modeling of robotic arms.

- `Links`: Links are rigid bodies that gives structure to the robot.
- `Joints`: Joints are the movable parts(actuators) of the robots that connects the links of the robot. Joints cause relative motion between adjacent links.
- `Kinematic chains`:  Kinematic chain is the assembly of links connected by joints to produce a desired motion. Human arm is an example of a kinematic chain. Human body is a group of kinematic chains connected in series.
- `Degree of Freedom (DOF)`: Degree of freedom in robotic is simply the total number of independent joints which can change the  pose of the robot. If you take a human arm from shoulder to palm (fingers not included), arm has 7 DOF. Try to identify the joints yourself.
- `End-Effector`: End-effector is the last link of the robot arm or manipulator that interacts with the environment. The last link can be gripper tool to do pick and place actions or a welding gun or simply a magnet.
- `Workspace`:  Workspace is the space in which your robot can work i.e., all reachable points by the robot's endeffector constitutes to workspace. 
- `Joint parameters`: Joint parameters is nothing but the state value of the joint. This state value depends on the type of the Joint.


`Type of Joints`: Most commonly used joints in robotics are revolute joint, prismatic joint and continuous joint.
- `Revolute` : a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits. Units: radians or degrees.
- `Prismatic` : a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits. Units: meters
- `Continuous` : a continuous hinge joint that rotates around the axis and has no upper and lower limits. Units: radians/sec or degrees/sec.

![image](https://static.wixstatic.com/media/407007_dc4c817cf5a645cfbebd988a2cf0f30d~mv2.png/v1/fill/w_414,h_354,al_c,q_85,enc_auto/407007_dc4c817cf5a645cfbebd988a2cf0f30d~mv2.png)

Attach the frame to the links:

![image](https://static.wixstatic.com/media/407007_82df530b064a4e96bce962d13d3dfcf4~mv2.png/v1/fill/w_429,h_385,al_c,q_85,enc_auto/407007_82df530b064a4e96bce962d13d3dfcf4~mv2.png)

### Translation and Rotation Matrices
Translation Matrix: The translation matrix for moving a point in 2D space by (tx,ty) is given by:
```
        [ 1 0 tx ]
    T = [ 0 1 ty ]
        [ 0 0 1  ]
```
Rotation Matrix: The rotation matrix for rotating a point by an angle θ (in radians) around the origin is:
```
        [ cosθ -sinθ 0 ]
    R = [ sinθ cosθ  1 ]
        [ 0     0    1 ]
```
Combining Translation and Rotation
To apply both translation and rotation to a point, you can multiply the transformation matrices. If you want to rotate and then translate, you first apply the rotation and then the translation:
```
    P= T . R
```
 
### Denavit-Hartenberg Parameters
For each joint 𝑖, define the following parameters:
- 𝜃𝑖 : Joint angle (rotation about the Z-axis)
- 𝑑𝑖 : Link offset (translation along the Z-axis)
- 𝑎𝑖 : Link length (translation along the X-axis)
- 𝛼𝑖 : Link twist (rotation about the X-axis)

### DH Table
| Joint | theta  |    d   |    a   |  anpha |
| ----- | ------ | ------ | ------ | ------ |
|   1   |   θ1   |   d1   |   a1   |   α1   |
|   2   |   θ2   |   d2   |   a2   |   α2   |
|   3   |   θ3   |   d3   |   a3   |   α3   |
|   4   |   θ4   |   d4   |   a4   |   α4   |
|   5   |   θ5   |   d5   |   a5   |   α5   |
|   6   |   θ6   |   d6   |   a6   |   α6   |

`DH Parameter`s: Replace the d, a, and alpha vectors with your robot's actual parameters.

`Matrix Operations`: This implementation uses basic matrix multiplication. For better performance, especially in robotics, consider using libraries like [Eigen](https://gitlab.com/libeigen/eigen) or kdl.

## Inverse Kinematics
Inverse kinematics computes the joint parameters that achieve a desired position and orientation of the end effector.

## Example

```
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

// Function to calculate transformation matrix using DH parameters
vector<vector<double>> dh_transform(double theta, double d, double a, double alpha) {
    vector<vector<double>> T(4, vector<double>(4, 0));
    T[0][0] = cos(theta);
    T[0][1] = -sin(theta) * cos(alpha);
    T[0][2] = sin(theta) * sin(alpha);
    T[0][3] = a * cos(theta);
    
    T[1][0] = sin(theta);
    T[1][1] = cos(theta) * cos(alpha);
    T[1][2] = -cos(theta) * sin(alpha);
    T[1][3] = a * sin(theta);
    
    T[2][0] = 0;
    T[2][1] = sin(alpha);
    T[2][2] = cos(alpha);
    T[2][3] = d;
    
    T[3][3] = 1;

    return T;
}

// Function to multiply two transformation matrices
vector<vector<double>> multiply_matrices(const vector<vector<double>>& A, const vector<vector<double>>& B) {
    vector<vector<double>> C(4, vector<double>(4, 0));
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

// Main function to calculate the overall transformation matrix
vector<vector<double>> forward_kinematics(const vector<double>& thetas) {
    // Define the DH parameters for the robot (example values)
    vector<double> d = {d1, d2, d3, d4, d5, d6}; // Replace with actual values
    vector<double> a = {a1, a2, a3, a4, a5, a6}; // Replace with actual values
    vector<double> alpha = {alpha1, alpha2, alpha3, alpha4, alpha5, alpha6}; // Replace with actual values

    vector<vector<double>> T = {{1, 0, 0, 0},
                                 {0, 1, 0, 0},
                                 {0, 0, 1, 0},
                                 {0, 0, 0, 1}}; // Identity matrix

    for (int i = 0; i < 6; ++i) {
        // Calculate the transformation matrix for each joint
        vector<vector<double>> T_i = dh_transform(thetas[i], d[i], a[i], alpha[i]);
        T = multiply_matrices(T, T_i);
    }

    return T; // Final transformation matrix
}

int main() {
    // Example joint angles (in radians)
    vector<double> joint_angles = {0, M_PI/4, M_PI/4, 0, 0, 0}; // Replace with actual values
    vector<vector<double>> final_transform = forward_kinematics(joint_angles);

    // Print the final transformation matrix
    for (const auto& row : final_transform) {
        for (const auto& value : row) {
            cout << value << " ";
        }
        cout << endl;
    }

    return 0;
}

```

Using Lib

```
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainiksolver_pos_lma.hpp>
#include <kdl/chainfksolver_pos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace KDL;

int main() {
    // Define the robot's kinematic chain (6 joints)
    Chain chain;

    // Define the DH parameters for each joint (example values)
    // Joint 1
    chain.addSegment(Segment(Joint(Joint::RotZ), 
        Frame(Vector(0, 0, 0)), 
        Rot::RotZ(0)));
    
    // Joint 2
    chain.addSegment(Segment(Joint(Joint::RotZ), 
        Frame(Vector(0, 0, 1)), 
        Rot::RotZ(0)));
    
    // Joint 3
    chain.addSegment(Segment(Joint(Joint::RotZ), 
        Frame(Vector(0, 0, 1)), 
        Rot::RotZ(0)));

    // Joint 4
    chain.addSegment(Segment(Joint(Joint::RotZ), 
        Frame(Vector(0, 0, 1)), 
        Rot::RotZ(0)));

    // Joint 5
    chain.addSegment(Segment(Joint(Joint::RotZ), 
        Frame(Vector(0, 0, 1)), 
        Rot::RotZ(0)));

    // Joint 6
    chain.addSegment(Segment(Joint(Joint::RotZ), 
        Frame(Vector(0, 0, 1)), 
        Rot::RotZ(0)));

    // Initialize joint positions
    JntArray joint_positions(chain.getNrOfJoints());
    joint_positions(0) = M_PI / 4; // Joint 1
    joint_positions(1) = M_PI / 4; // Joint 2
    joint_positions(2) = 0;        // Joint 3
    joint_positions(3) = 0;        // Joint 4
    joint_positions(4) = 0;        // Joint 5
    joint_positions(5) = 0;        // Joint 6

    // Compute forward kinematics
    Frame end_effector_frame;
    ChainFkSolverPos_recursive fk_solver(chain);
    if (fk_solver.JntToCart(joint_positions, end_effector_frame) >= 0) {
        cout << "End effector position: " << end_effector_frame.p.x() << ", "
             << end_effector_frame.p.y() << ", "
             << end_effector_frame.p.z() << endl;
    } else {
        cout << "Error in forward kinematics calculation!" << endl;
    }

    return 0;
}
```
Replace the DH parameters in the Frame and Segment definitions according to your robot's specifications.

