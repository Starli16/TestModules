load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_solver_gflags",
    srcs = ["qp_solver_gflags.cc"],
    hdrs = ["qp_solver_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "qp_solver",
    srcs = ["qp_solver.cc"],
    hdrs = ["qp_solver.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "active_set_qp_solver",
    srcs = ["active_set_qp_solver.cc"],
    hdrs = ["active_set_qp_solver.h"],
    linkstatic = False,
    deps = [
        ":qp_solver",
        "//cyber/common:log",
        "//modules/common/math/qp_solver:qp_solver_gflags",
        "@eigen",
        "@qpOASES",
    ],
)

cc_test(
    name = "active_set_qp_solver_test",
    size = "small",
    srcs = ["active_set_qp_solver_test.cc"],
    deps = [
        ":active_set_qp_solver",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
