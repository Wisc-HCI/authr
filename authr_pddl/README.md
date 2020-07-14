# authr_pddl

This package allows you to translate Authr Plans into pddl specifications and execute with the Descarwin planner, included.

To do so, execute
`python pddlopt.py <path_to_plan_json>`

Note, if your specification involves series of transport loads to encode specific trajectories, you will need to update the `dest_whitelist.py` file, since PDDL will generally ignore these types of rules. This is done by adding the from/to destination pair to the whitelist. You will also need to encode the valid release positions in the whitelist.

Note, the Descarwin planner was compiled for certain platforms. If this compiled version does not work for you on your platform, you may need to compile from [the original](https://github.com/nojhan/descarwin), and configure the `pddlopt.py` file to call that executable instead.
