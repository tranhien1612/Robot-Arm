# Robot-Arm

## DH Table
| Joint | theta  |    d   |    a   |  anpha |
| ----- | ------ | ------ | ------ | ------ |
|   1   |   θ1   |   d1   |   a1   |   α1   |
|   2   |   θ2   |   d2   |   a2   |   α2   |
|   3   |   θ3   |   d3   |   a3   |   α3   |
|   4   |   θ4   |   d4   |   a4   |   α4   |
|   5   |   θ5   |   d5   |   a5   |   α5   |
|   6   |   θ6   |   d6   |   a6   |   α6   |

`DH Parameter`s: Replace the d, a, and alpha vectors with your robot's actual parameters.

`Matrix Operations`: This implementation uses basic matrix multiplication. For better performance, especially in robotics, consider using libraries like Eigen or Armadillo.

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
