"""Corner Pop-out scenario - dynamic obstacle crosses ego path diagonally (5x5m).

References: .docs/plans/実験環境.md
- 5m x 5m field
- Dynamic obstacle follows waypoints from upper-right toward ego start
"""

SCENARIO = {
    'env': {
        'x_min': 0.0, 'x_max': 5.0,
        'y_min': 0.0, 'y_max': 5.0,
    },
    'robot': {
        'start': (0.5, 1.0),
        'goal': (4.5, 1.0),
        'radius': 0.25,
        'v_max': 1.0,
        'a_max': 1.0,
        'sensing_range': 10.0,
    },
    'obstacles': [
        {
            'name': 'obs_0',
            'position': (4.0, 3.5),
            'radius': 0.15,
            'v_max': 0.3,
            'behavior': 'waypoint',
            'waypoints': [
                (4.0, 3.5),    # Start upper-right
                (2.5, 2.0),    # Diagonal descent
                (1.5, 1.2),    # Approach ego path
                (0.5, 1.0),    # Ego start position (stop)
            ],
        },
    ],
    'cbf': {
        'T_horizon': 2.0,
        'dt_backup': 0.05,
        'alpha': 1.0,
    },
    'gazebo': {
        'world_file': 'experiment_corner.world',
    },
}
