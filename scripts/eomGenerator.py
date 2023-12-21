import sympy
import sympy.physics.mechanics
import sympy.utilities.codegen

## System Description
# THE QUADCOPTER MODEL IS MADE OF FIVE BODIES. THE MAIN BODY IS BODY A, WHICH INCLUDES QUADCOPTER FRAME, FILGHT CONTROLLER ELECTRONICS, ESCS AND THE STATORS OF THE FOUR MOTORS. THE OTHER FOUR BODIES ARE B, C, D AND E. EACH OF THESE BODIES REPRESENT THE ROTOR OF EACH MOTOR ALONG WITH THE RIGIDLY ATTACHED PROPELLER. THE QUADCOPTER IS ASSUMED TO HAVE AN X CONFIGURATION WITH "PROP-IN" MOTOR ROTATION AS SHOWN BELOW.
# 
#        -->             <--
#       | B               E |
#           \           /
#              \     /
#                 X
#              /     \
#           /           \
#       | C               D |
#        -->             <--

## Define Symbolic Variables
# Define generalized coordinates
q1, q2, q3 = sympy.physics.mechanics.dynamicsymbols("q1:4", real=True); # 3 coordinates for the position of Body A
e0, e1, e2, e3 = sympy.physics.mechanics.dynamicsymbols("e0:4", real=True); # 4 euler parameters for the orientation of Body A
q4, q5, q6, q7 = sympy.physics.mechanics.dynamicsymbols("q4:8", real=True); # 4 coordinates for the rotation of the motors w.r.t. Body A

# Define quasi-speeds
u1, u2, u3 = sympy.physics.mechanics.dynamicsymbols("u1:4", real=True); # 3 speeds for the translational velocity of Body A
w1, w2, w3 = sympy.physics.mechanics.dynamicsymbols("w1:4", real=True); # 3 speeds for the angular velocity of Body A
u4, u5, u6, u7 = sympy.physics.mechanics.dynamicsymbols("u4:8", real=True); # 4 speeds for the angular velocity of the motors w.r.t. Body A

# Define generalized speeds
q1d, q2d, q3d = sympy.physics.mechanics.dynamicsymbols("q1:4", 1, real=True); # 3 speeds for the velocity of the main body
e0d, e1d, e2d, e3d = sympy.physics.mechanics.dynamicsymbols("e0:4", 1, real=True); # 4 euler rates for the attitude of the body
q4d, q5d, q6d, q7d = sympy.physics.mechanics.dynamicsymbols("q4:8", 1, real=True); # 4 speeds for the angular velocity of the motors w.r.t. the body

# Define generalized accelerations
u1d, u2d, u3d = sympy.physics.mechanics.dynamicsymbols("u1:4", 1, real=True); # 3 accelerations of body A
w1d, w2d, w3d = sympy.physics.mechanics.dynamicsymbols("w1:4", 1, real=True); # 3 angular accelerations of body A
u4d, u5d, u6d, u7d = sympy.physics.mechanics.dynamicsymbols("u4:8", 1, real=True); # 4 angular accelerations of motors

# Define kinematic equations
kd = [q1d - u1, q2d - u2, q3d - u3, e0d - (-e1*w1-e2*w2-e3*w3)/2, e1d - (e0*w1-e3*w2+e2*w3)/2, e2d - (e3*w1+e0*w2-e1*w3)/2, e3d - (-e2*w1+e1*w2+e0*w3)/2, q4d - u4, q5d - u5, q6d - u6, q7d - u7];

## Define other constants
# Geometric Constants
A_P_AB = sympy.MatrixSymbol("A_P_AB", 3, 1); # Constants to define position of Body B = [lB; wB; hB]
A_P_AC = sympy.MatrixSymbol("A_P_AC", 3, 1); # Constants to define position of Body C = [lC; wC; hC]
A_P_AD = sympy.MatrixSymbol("A_P_AD", 3, 1); # Constants to define position of Body D = [lD; wD; hD]
A_P_AE = sympy.MatrixSymbol("A_P_AE", 3, 1); # Constants to define position of Body E = [lE; wE; hE]
pDia = sympy.symbols("pDIa", positive=True); # Propeller diameter
g = sympy.symbols("g", positive=True); # Gravitational Constant

# Mass and Inertia Constants
mVals = sympy.MatrixSymbol("mVals",5,1); # Mass for each body, mVals = [mA; mB; mC; mD; mE]
IA = sympy.MatrixSymbol('IA',6,1); # Six values of inertia tensor, IA = [IAxx; IAyy; IAzz; IAxy; IAyz; IAzx]
IB = sympy.MatrixSymbol('IB',6,1); # Six values of inertia tensor, IB = [IBxx; IByy; IBzz; IBxy; IByz; IBzx]
IC = sympy.MatrixSymbol('IC',6,1); # Six values of inertia tensor, IC = [ICxx; ICyy; ICzz; ICxy; ICyz; ICzx]
ID = sympy.MatrixSymbol('ID',6,1); # Six values of inertia tensor, ID = [IDxx; IDyy; IDzz; IDxy; IDyz; IDzx]
IE = sympy.MatrixSymbol('IE',6,1); # Six values of inertia tensor, IE = [IExx; IEyy; IEzz; IExy; IEyz; IEzx]

## Define frames and bodies
# Inertial Frame
N = sympy.physics.mechanics.ReferenceFrame('N');
No = sympy.physics.mechanics.Point('No');
No.set_vel(N,0);

# Define all the bodies with body attached frame and point
A123 = N.orientnew('A123','Quaternion',(e0,e1,e2,e3));
Ao = No.locatenew('Ao', q1*N.x + q2*N.y + q3*N.z);
AIAA = (sympy.physics.mechanics.inertia(A123, IA[0], IA[1], IA[2], IA[3], IA[4], IA[5]),Ao);
A = sympy.physics.mechanics.RigidBody('A', Ao, A123, mVals[0], AIAA);

B123 = A123.orientnew('B123','Axis',(q4,A123.z));
Bo = Ao.locatenew('Bo', A_P_AB[0]*A123.x + A_P_AB[1]*A123.y + A_P_AB[2]*A123.z);
BIBB = (sympy.physics.mechanics.inertia(B123, IB[0], IB[1], IB[2], IB[3], IB[4], IB[5]),Bo);
B = sympy.physics.mechanics.RigidBody('B', Bo, B123, mVals[1], BIBB);

C123 = A123.orientnew('C123','Axis',(q5,A123.z));
Co = Ao.locatenew('Co', A_P_AC[0]*A123.x + A_P_AC[1]*A123.y + A_P_AC[2]*A123.z);
CICC = (sympy.physics.mechanics.inertia(C123, IC[0], IC[1], IC[2], IC[3], IC[4], IC[5]),Co);
C = sympy.physics.mechanics.RigidBody('C', Co, C123, mVals[2], CICC);

D123 = A123.orientnew('D123','Axis',(q6,A123.z));
Do = Ao.locatenew('Do', A_P_AD[0]*A123.x + A_P_AD[1]*A123.y + A_P_AD[2]*A123.z);
DIDD = (sympy.physics.mechanics.inertia(D123, ID[0], ID[1], ID[2], ID[3], ID[4], ID[5]),Do);
D = sympy.physics.mechanics.RigidBody('D', Do, D123, mVals[3], DIDD);

E123 = A123.orientnew('E123','Axis',(q7,A123.z));
Eo = Ao.locatenew('Eo', A_P_AE[0]*A123.x + A_P_AE[1]*A123.y + A_P_AE[2]*A123.z);
EIEE = (sympy.physics.mechanics.inertia(E123, IE[0], IE[1], IE[2], IE[3], IE[4], IE[5]),Eo);
E = sympy.physics.mechanics.RigidBody('E', Eo, E123, mVals[4], EIEE);

## Define velocities
A123.set_ang_vel(N,w1*A123.x + w2*A123.y + w3*A123.z);
B123.set_ang_vel(A123,u4*A123.z);
C123.set_ang_vel(A123,u5*A123.z);
D123.set_ang_vel(A123,u6*A123.z);
E123.set_ang_vel(A123,u7*A123.z);
Ao.set_vel(N, u1*N.x + u2*N.y + u3*N.z);
Bo.v2pt_theory(Ao,N,A123);
Co.v2pt_theory(Ao,N,A123);
Do.v2pt_theory(Ao,N,A123);
Eo.v2pt_theory(Ao,N,A123);

## Apply forces and moments
fVals = sympy.MatrixSymbol("fVals", 4, 1); # fVals = [fB; fC; fD; fE]
tVals = sympy.MatrixSymbol("tVals", 4, 1); # tVals = [tB; tC; tD; tE]
FList = [(Ao,-g*A.mass*N.z),(Bo,(fVals[0] - g*B.mass)*N.z),(Co,(fVals[1] - g*C.mass)*N.z),(Do,(fVals[2] - g*D.mass)*N.z),(Eo,(fVals[3] - g*E.mass)*N.z)];
TList = [(A123, tVals[0]*A123.z - tVals[1]*A123.z + tVals[2]*A123.z - tVals[3]*A123.z),\
         (B123,-tVals[0]*A123.z),\
         (C123, tVals[1]*A123.z),\
         (D123,-tVals[2]*A123.z),\
         (E123, tVals[3]*A123.z)];
FL = FList + TList;

## Define a list of bodies
BL = [A,B,C,D,E];

## Apply Kane's method
KM = sympy.physics.mechanics.KanesMethod(N, q_ind=[q1, q2, q3, e0, e1, e2, e3, q4, q5, q6, q7], u_ind=[u1, u2, u3, w1, w2, w3, u4, u5, u6, u7], kd_eqs=kd);
(fr, frs) = KM.kanes_equations(bodies=BL, loads=FL);
COEF = KM.mass_matrix;
RHS = KM.forcing_full;

## Substitute state variables
Q = sympy.MatrixSymbol('Q',21,1);
COEF = COEF.subs({q1:Q[0,0],q2:Q[1,0],q3:Q[2,0],e0:Q[3,0],e1:Q[4,0],e2:Q[5,0],e3:Q[6,0],q4:Q[7,0],q5:Q[8,0],q6:Q[9,0],q7:Q[10,0],u1:Q[11,0],u2:Q[12,0],u3:Q[13,0],w1:Q[14,0],w2:Q[15,0],w3:Q[16,0],u4:Q[17,0],u5:Q[18,0],u6:Q[19,0],u7:Q[20,0]});
RHS = RHS.subs({q1:Q[0,0],q2:Q[1,0],q3:Q[2,0],e0:Q[3,0],e1:Q[4,0],e2:Q[5,0],e3:Q[6,0],q4:Q[7,0],q5:Q[8,0],q6:Q[9,0],q7:Q[10,0],u1:Q[11,0],u2:Q[12,0],u3:Q[13,0],w1:Q[14,0],w2:Q[15,0],w3:Q[16,0],u4:Q[17,0],u5:Q[18,0],u6:Q[19,0],u7:Q[20,0]});

## Create output variables
coef = sympy.MatrixSymbol("coef",COEF.shape[0],COEF.shape[1]);
rhs = sympy.MatrixSymbol("rhs",RHS.shape[0],RHS.shape[1]);

## Generate Code
[(c_name, c_code), (h_name, c_header)] = sympy.utilities.codegen.codegen(
    name_expr=('coef',sympy.Equality(coef,COEF,evaluate=False)),
    language='C',
    project="quad_sim",
    to_files=False,
    header=True,
    argument_sequence=(Q,g,A_P_AB,A_P_AC,A_P_AD,A_P_AE,mVals,IA,IB,IC,ID,IE,fVals,tVals,coef)
);
cFile = open("./src/" + c_name,"w"); cFile.write(c_code); cFile.close();
hFile = open("./include/" + h_name,"w"); hFile.write(c_header); hFile.close();

[(c_name, c_code), (h_name, c_header)] = sympy.utilities.codegen.codegen(
    name_expr=('rhs',sympy.Equality(rhs,RHS,evaluate=False)),
    language='C',
    project="quad_sim",
    to_files=False,
    header=True,
    argument_sequence=(Q,g,A_P_AB,A_P_AC,A_P_AD,A_P_AE,mVals,IA,IB,IC,ID,IE,fVals,tVals,rhs)
);
cFile = open("./src/" + c_name,"w"); cFile.write(c_code); cFile.close();
hFile = open("./include/" + h_name,"w"); hFile.write(c_header); hFile.close();