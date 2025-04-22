# Low Latency CPP Patterns

A C++ repo for low latency CPP patterns - currently only has the single_consumer_single_producer pattern for simulating MMIO event handling from FPGAs and DMA reads.

## Setup

This project uses Bazel for building and managing dependencies. It has an included Dockerfile for a devcontainer workflow.

### Prerequisites

- Bazel
- C++ compiler
- Protocol Buffers

### Building

Currently, you do this from within the devcontainer. A task has been defined to do this also.

```bash
bazel build //src:single_producer_single_consumer
```

### Running

Just start it via the following bazel command.

```bash
bazel run //src:single_producer_single_consumer
```

### Future Enhancements

- Add more patterns, like multi-producer multi-consumer
- Add more optimisation variations to experiment
- Add profiling analysis

# License

This project is licensed under the MIT License - see the LICENSE file for detials.