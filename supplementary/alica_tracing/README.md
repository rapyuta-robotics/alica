## 1 How to run tracing?

- To run tracing set the env variable TRACE_COLLECTOR_EP to the trace collector's endpoint & run the docker-compose.yml file in
alica/supplementary/alica_tracing/docker-compose.yaml:

  `$ TRACE_COLLECTOR_EP=<trace_collector_ep> docker-compose up`