import sympy as sp

sp.init_printing()
theta_1, theta_2, theta_3, theta_4 = sp.symbols("theta_1 theta_2 theta_3 theta_4")

def calculate_FK():

    h_0_1 = create_HTM_matrix(theta_1 + sp.pi/2, sp.pi/2, 0,   2.5)
    h_1_2 = create_HTM_matrix(theta_2 + sp.pi/2, sp.pi/2, 0,   0  )
    h_2_3 = create_HTM_matrix(theta_3,           -sp.pi/2, 3.5, 0  )
    h_3_4 = create_HTM_matrix(theta_4,           0,       3,   0  )

    h_0_4 = h_0_1 * h_1_2 * h_2_3 * h_3_4

    return h_0_4[:3, -1]

def create_HTM_matrix(theta, alpha, r, d):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),  sp.sin(theta) * sp.sin(alpha), r * sp.cos(theta)],
                      [sp.sin(theta),  sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), r * sp.sin(theta)],
                      [0,              sp.sin(alpha),                  sp.cos(alpha),                 d                ],
                      [0,              0,                              0,                             1                ]
                    ])

# Calculate forward kinematic equations for position of end effector
fk_pos = sp.expand(calculate_FK())
sp.pprint(fk_pos)

# Evaluate forward kinematics with example joint angles
sp.pprint(fk_pos.subs([(theta_1, 0), (theta_2, 1.57), (theta_3, 0.785), (theta_4, 0)]))

# Calculate jacobian matrix for velocity of end effector
variables = sp.Matrix([theta_1, theta_2, theta_3, theta_4])
jacobian = fk_pos.jacobian(variables)
sp.pprint(jacobian)
