FROM ubuntu
RUN apt-get upgrade && apt-get update && apt-get install --no-install-recommends -y libpcap-dev libglpk-dev git cmake g++ ca-certificates make && \
    git clone --recursive https://github.com/ngvozdiev/ncode-common.git && \
    cd ncode-common && mkdir build && cd build && cmake -DNCODE_DISABLE_TESTS=YES -DNCODE_DISABLE_BENCHMARKS=YES .. && make && make install && \
    git clone --recursive https://github.com/ngvozdiev/tm-gen.git && \
    cd tm-gen && mkdir build && cd build && cmake -DTM_GEN_DISABLE_TESTS=YES -DTM_GEN_DISABLE_BENCHMARKS=YES .. && make && make install && cd / && \
    apt-get purge -y libpcap-dev libglpk-dev git cmake gcc openssl make && apt-get autoremove -y && rm -rf ./tm-gen && rm -rf ./ncode-common && \
    apt-get install -y libpcap0.8 libglpk40 && apt-get clean
 
