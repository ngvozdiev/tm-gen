`tm-gen` and `tm-run` are console-based utilities for generating synthetic 
traffic matrices and running various traffic engineering algorithms on them. 

## tm-gen

The traffic generation algorithm is based on the gravity model from 
[Roughan's 2005 CCR paper](http://www.maths.adelaide.edu.au/matthew.roughan/papers/ccr_2005.pdf). 
More details about how `tm-gen` generates traffic matrices can be found 
[here](https://github.com/ngvozdiev/tm-gen/blob/master/methodology/methodology.pdf).

### Usage with Docker

A Docker image that contains `tm-gen` and `tm-run` along with all necessary 
dependencies can be found on [Docker Hub](https://hub.docker.com/r/ngvozdiev/tm-gen/). 

To run `tm-gen` simply do:

`docker run ngvozdiev/tm-gen --help`

which will run `tm-gen` and list its arguments. 

### Arguments

`--locality` (default: `"0.0"`) sets the locality factor for the traffic matrix. The larger this value the more geographically local traffic will be; if this value if left to its default value of 0 distances between nodes will not influence the resulting traffic matrix. Can be a comma-separated list in which case will generate traffic matrices for each of the locality values specified.

`--min_scale_factor` (default: `"1.3"`) controls how loaded traffic matrices will be. Aggregates in all generated traffic matrices will be scaleable by at least this much. This means that if you multiply each traffic matrix's demands by this number then the traffic matrix will be completely saturated. Can be a comma-separated list in which case will generate traffic matrices for each of the scale values specified.

`--output_pattern` (default: `"demand_matrices/scale_factor_$2/locality_$1/$0_$3.demands"`) traffic matrices will be saved to files named after this pattern, with `$0` replaced by the topology name, `$1` replaced by locality, `$2` replaced by scale factor and `$3` replaced by a unique integer identifier.

`--seed` (default: `1`) seed for generated traffic matrices.

`--threads` (default: `2`) number of threads to use. Keep in mind that a step in the generation process uses an LP solver, which may use more threads.

`--tm_count` (default: `1`) number of traffic matrices to generate. Will generate this many traffic matrices for each combination of topology, locality and scale factor.

`--topology_root` (default: `"/usr/local/tm-gen/topologies"`) where to look for topologies. The topologies should be specified in the format used by [Repetita](https://bitbucket.org/StevenGay/repetita), briefly summarized below. 

`--topology_delay_limit_ms` (default: `10`) topologies with diameter less than this will be skipped

`--topology_size_limit` (default: `70`) topologies with node count more than this will be skipped

### Topology format

In brief, the first line of each topology file should be `NODES XX` followed by a comment line and XX lines for each of the nodes in the graph. Each node line is of the format `<node_name> <x> <y>` where x and y are the x,y coordinates of the node. The nodes section is followed by and empty line and `EDGES XX` on a new line. A comment line is next, followed by for each edge `<label> <src> <dst> <weight> <bw> <delay>`. The bandwidth is in Kbps and the delay in microseconds.

`tm-gen` comes with all topologies from [Repetita](https://bitbucket.org/StevenGay/repetita). The topologies will be installed under `/usr/local/tm-gen/topologies` (which is also the default value for the `--topology_root` argument) when you do `make install`. If you are using the Docker image the topologies are pre-installed in the image, so you do not have to specify `--topology_root`.

### Examples

To generate traffic matrices with locality 0 and scale factor of 1.3 under `/tmp/demand_matrices`:

`docker run -v /tmp/demand_matrices:/demand_matrices ngvozdiev/tm-gen`

the `-v /tmp/demand_matrices:/demand_matrices` binds `/tmp/demand_matrices` on 
the local filesystem to `/demand_matrices` within the docker container, which is 
the default location in the `output_pattern` argument. Will only generate 
matrices for topologies with less than 70 nodes and diameter more than 10 ms.

## tm-run

To explore how traffic engineering solutions behave on the generated matrices 
you can use `tm-run`, which is installed in the same Docker image as `tm-gen`. 
For example 

`docker run ngvozdiev/tm-gen --entrypoint tm-run --help`

will run `tm-run` and list its arguments.

### Examples

To generate routing solutions for all traffic matrices under `/tmp/demand_matrices`:

`docker run -v /tmp/demand_matrices:/demand_matrices --entrypoint tm-run ngvozdiev/tm-gen`

For each traffic matrix the above command will run a number of routing algorithms 
and produce a file that will contain the paths and splits that the 
algorithm has assigned to each of the demands in the input traffic matrix. The 
files will have the following suffixes:

`_SP` for shortest path routing

`_B4` for Google's B4

`_MinMaxLD` for a variant of MinMax which in addition to minimizing the maximum load in the network, as a secondary objective minimizes delay

`_MinMaxK10` for a variant of MinMax that is restricted to only use the 10 shortest paths for each demand

`_LDR` for LDR

### A note about performance

The performance of both `tm-gen` and `tm-run` heavily depends on the linear 
optimizer library that they use. The Docker image comes with the 
open-source `glpk` optimizer, which works fine for smaller topologies and 
traffic matrices, but may be too slow for larger use cases. If you run into 
performance issues try installing locally and linking against CPLEX instead. 
