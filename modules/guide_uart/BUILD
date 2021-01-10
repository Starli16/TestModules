load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "guide_uart.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":guide_uartlib"],
)

cc_library(
    name = "guide_uartlib",
    srcs = ["guide_uart.cc"],
    hdrs = ["guide_uart.h"],
    deps = [
        "//cyber",
	    "//modules/common:uart",
    ],
)

cc_library(
    name = "uart_client",
    srcs = ["uart_client.cc"],
    hdrs = ["uart_client.h"],
    deps = [
        "//cyber",
	    "//modules/common:uart",
    ],
)

cpplint()
