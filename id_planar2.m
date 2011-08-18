% This is an attempt to manually write the Newton-Euler inverse dynamics algorithm
% "instantiated" for a fixed-base, planar robot with two links, with all loops
% unrolled.

function tau = id_planar2(q, qd, qdd)

gEarth = -9.81;

% Transformation matrices from link frame to the joint frame.
% They are constants, since depend only on the geometry of the robot.
% For a fixed base robot, the first joint has a transform equal to the identity,
%  by convention 

Xtree{1} =[
1     0     0     0     0     0;
0     1     0     0     0     0;
0     0     1     0     0     0;
0     0     0     1     0     0;
0     0     0     0     1     0;
0     0     0     0     0     1 ];

Xtree{2} =[
     1     0     0     0     0     0;
     0     1     0     0     0     0;
     0     0     1     0     0     0;
     0     0     0     1     0     0;
     0     0     1     0     1     0;
     0    -1     0     0     0     1
 ];% this corresponds to a translation [1 0 0]
 
% Motion subspace matrices, which depend only on the type of the various joint,
%  thus are constant.
S{1} = [0;0;1;0;0;0];
S{2} = [0;0;1;0;0;0];

% Inertia matrices for all the moving bodies.
% These values are computed under the following assumptions:
%  - each link has mass 1 Kg
%  - each link is 1 meter long
%  - each link is an hollow cylinder infinitely thin (that is, the internal and
%    external radii are the same)
%  - such radius is 1/20 meter long
I{1} = [
      0025         0         0         0         0         0;
         0    0.3346         0         0         0   -0.5000;
         0         0    0.3346         0    0.5000         0;
         0         0         0    1.0000         0         0;
         0         0    0.5000         0    1.0000         0;
         0   -0.5000         0         0         0    1.0000
];
I{2} = [
      0025         0         0         0         0         0;
         0    0.3346         0         0         0   -0.5000;
         0         0    0.3346         0    0.5000         0;
         0         0         0    1.0000         0         0;
         0         0    0.5000         0    1.0000         0;
         0   -0.5000         0         0         0    1.0000
];


%v0 = 0;
a0 = [0;0;0;0;0;-gEarth];

% FIRST LINK
%%%%%%%%%%%%
% Since we are dealing with the first joint and moving link, the parent link is
%  the fixed base (no velocity, acceleration due to gravity).

s = sin(q(1));
c = cos(q(1));

% Transform between the two frames embedded in the first rotational joint (axis z)
%  It transforms from the frame in the parent to the frame in the child.
XJ = [ c  s  0  0  0  0 ;
      -s  c  0  0  0  0 ;
       0  0  1  0  0  0 ;
       0  0  0  c  s  0 ;
       0  0  0 -s  c  0 ;
       0  0  0  0  0  1
    ];

% Transform between the parent link frame and the child link frame
cXp{1} = XJ * Xtree{1};

vJ = S{1} * qd(1);         % velocity of the joint, as 6-vector
v{1} = vJ;                 % the velocity of the link comes only from the joint

% The expression for the acceleration is slightly simpler for the first link,
%  because the velocity of the parent is zero
a{1} = (cXp{1}*a0) + (S{1}*qdd(1));


xprodForce = [
   0        -v{1}(3)    v{1}(2)     0         -v{1}(6)    v{1}(5)  ;
  v{1}(3)    0         -v{1}(1)     v{1}(6)    0         -v{1}(4) ;
 -v{1}(2)    v{1}(1)    0          -v{1}(5)    v{1}(4)    0    ;
   0         0          0           0         -v{1}(3)    v{1}(2)  ;
   0         0          0           v{1}(3)    0         -v{1}(1) ;
   0         0          0          -v{1}(2)    v{1}(1)    0
];


f{1} = I{1} * a{1} + xprodForce * I{1} * v{1};

% SECOND LINK
%%%%%%%%%%%%%

s = sin(q(2));
c = cos(q(2));

% Transform between the two frames embedded in the first rotational joint (axis z)
%  It transforms from the frame in the parent to the frame in the child.
XJ = [ c  s  0  0  0  0 ;
      -s  c  0  0  0  0 ;
       0  0  1  0  0  0 ;
       0  0  0  c  s  0 ;
       0  0  0 -s  c  0 ;
       0  0  0  0  0  1
    ];

% Transform between the parent link frame and the child link frame
cXp{2} = XJ * Xtree{2};

% The velocity due only to the joint:
vJ = S{2} * qd(2);
% The velocity of the link now depends also on the velocity of the parent (i.e.
%  the velocity of the parent is in general not null)
v{2} = (cXp{2} * v{1}) + vJ; 

vJcross = [ % the matrix for the spatial cross product of the joint velocity
    0        -v{2}(3)   v{2}(2)   0        0        0    ;
	v{2}(3)   0        -v{2}(1)   0        0        0    ;
   -v{2}(2)   v{2}(1)   0         0        0        0    ;
    0        -v{2}(6)   v{2}(5)   0       -v{2}(3)  v{2}(2) ;
    v{2}(6)   0        -v{2}(4)   v{2}(3)  0       -v{2}(1) ;
   -v{2}(5)   v{2}(4)   0        -v{2}(2)  v{2}(1)  0
];

% The expression for the acceleration now needs the cross product matrix of the
%  velocity (see last term of the addition)
a{2} = (cXp{2} * a{1}) + (S{2} * qdd(2)) + (vJcross * S{2}) * qd(2);

xprodForce = [
   0        -v{2}(3)    v{2}(2)     0         -v{2}(6)    v{2}(5)  ;
  v{2}(3)    0         -v{2}(1)     v{2}(6)    0         -v{2}(4) ;
 -v{2}(2)    v{2}(1)    0          -v{2}(5)    v{2}(4)    0    ;
   0         0          0           0         -v{2}(3)    v{2}(2)  ;
   0         0          0           v{2}(3)    0         -v{2}(1) ;
   0         0          0          -v{2}(2)    v{1}(1)    0
];

f{2} = I{2} * a{2} + xprodForce * I{2} * v{2};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau(2,1) = S{2}' * f{2};

% To update the force of the parent we need the *force* transform *from child to
%  parent*, which happens to be transpose of the spatial transform from parent
%  to child, "cXp"
f{1} = f{1} + cXp{2}' * f{2};

tau(1,1) = S{1}' * f{1};



