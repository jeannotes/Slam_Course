cc_library(
    name = "gtest11",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/gtest-all.cc"]
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h"
    ]),
    copts = ["-Iexternal/gtestt/include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)