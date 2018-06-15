cc_binary(
    name = "hello-world",
    srcs = ["hello-world.cc"],
    deps = [
        "//main/s/include:hello-greet",
    ],
)

cc_test(
    name = "gtest",
    srcs = ["test.cc",
            ],
    includes = ["include"],
    #hdrs = ["include/hello-greet.h"],
    copts = ["-Iexternal/gtest/include"],
    deps = ["@gtest//:gtest_main", "//main/s/include:hello-greet"],
)