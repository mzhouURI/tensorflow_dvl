class model_config:

    u = ['udot', 'vdot', 'wdot',
         'p', 'q', 'r',
         'q1', 'q2', 'q3', 'q4',
         'volt',
         's', 'sb', 'hs', 'hb',
         'z_dot', 'z'
        ]

    v = ['u', 'w',
         'udot', 'vdot', 'wdot',
         'p', 'q', 'r',
         'q1', 'q2', 'q3', 'q4',
         'volt',
         'sb',
         'z_dot', 'z'
        ]

    w = ['udot', 'vdot', 'wdot',
         'p', 'q', 'r',
         'q1', 'q2', 'q3', 'q4',
         'volt',
         'hs', 'hb',
         'z_dot', 'z'
        ]

    dropout = 0.1
    reg_ratio = 0.00001
    name = 'Combo1/'