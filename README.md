# tm-gen

`tm-gen` is a console-based utility for generating synthetic traffic matrices. It is based on the gravity model from [Roughan's 2005 CCR paper](http://www.maths.adelaide.edu.au/matthew.roughan/papers/ccr_2005.pdf).

## Usage with Docker

A Docker image that contains `tm-gen` along with all necessary dependencies can be found on [Docker Hub](https://hub.docker.com/r/ngvozdiev/tm-gen/). To use it simply do:

`docker run ngvozdiev/tm-gen --help`

which will run `tm-gen` and list its arguments.

## Arguments

`--locality` (default: `"0.0"`) sets the locality factor for the traffic matrix. The larger this value the more gographically local traffic will be; if this value if left to its default value of 0 distances between nodes will not influence the resulting traffic matrix. Can be a comma-separated list in which case will generate traffic matrices for each of the locality values specified.

`--min_scale_factor` (default: `"1.3"`) controls how loaded traffic matrices will be. Aggregates in all generated traffic matrices will be scaleable by this much. This means that if you multiply each traffic matrix's demands by this number then the traffic matrix will be completely saturated. Can be a comma-separated list in which case will generate traffic matrices for each of the scale values specified.

`--output_pattern` (default: `"demand_matrices/scale_factor_$2/locality_$1/$0_$3.demands"`) traffic matrices will be saved to files named after this pattern, with $0 replaced by the topology name, $1 replaced by locality, $2 replaced by scale factor and $3 replaced by a unique integer identifier.

`--seed` (default: `1`) seed for generates traffic matrices.

`--threads` (default: `2`) number of threads to use. Keep in mind that a step in the generation process uses an LP solver, which may use more threads.

`--tm_count` (default: `1`) number of traffic matrices to generate. Will generate this many traffic matrices for each combination of topology, locality and scale factor.

`--topology_root` (default: `"/usr/local/tm-gen/topologies"`) where to look for topologies. The topologies should be specified in the format used by https://bitbucket.org/StevenGay/repetita/src, briefly summarized below. 

## Topology format

In brief, the first line of each topology file should be `NODES XX` followed by a comment line and XX lines for each of the nodes in the graph. Each node line is of the format `<node_name> <x> <y>` where x and y are the x,y coordinates of the node. The nodes section is followed by and empty line and `EDGES XX` on a new line. A comment line is next, followed by for each edge `<label> <src> <dst> <weight> <bw> <delay>`. The bandwidth is in Kbps and the delay in microseconds.
