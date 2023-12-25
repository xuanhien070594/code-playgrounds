load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "linked_list",
    srcs = ["linked_list.cc"],
    hdrs = ["linked_list.h"],
)

cc_binary(
    name = "delete_nodes",
    srcs = ["delete_nodes.cc"],
    deps = [":linked_list"],
)
