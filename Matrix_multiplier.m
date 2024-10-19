% Define symbolic theta
syms theta;

% Define the two matrices symbolically
A = [cos(theta), -sin(theta), 0;
     sin(theta),  cos(theta), 0;
     0,           0,          1];

B = [-1,  0,  0;
     0,  1, 0;
     0,  0,  -1];

% Perform matrix multiplication
C = A * B;

% Display the result
disp('Result of A * B:');
disp(C);
