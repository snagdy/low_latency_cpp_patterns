load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

cc_binary(
    name = "single_consumer_single_producer",
    srcs = ["single_consumer_single_producer.cpp"],
    deps = [
        # ":black_scholes_options_lib",
        # "//proto/finance/options:black_scholes_cc_proto",
        # "//proto/finance/options:black_scholes_cc_grpc",
        # "@grpc//:grpc++",
        # "@grpc//:grpc",
        "@boost.circular_buffer//:boost.circular_buffer",
        "@spdlog//:spdlog",
        # "@nlohmann_json//:json",
        # "@bazel_tools//tools/cpp/runfiles",
    ],
    # data = ["//conf:server_config.json"],
    linkopts = ["-pthread"],
    includes = ["."]
)