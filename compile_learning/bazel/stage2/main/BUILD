cc_library(
    name = "hello-greet",
    srcs = ["hello-greet.cc"],
    hdrs = ["include/hello-greet.h"],
    #includes=["include"],
    copts = ["-Imain/include"],
)

cc_binary(
    name = "hello-world",
    srcs = ["hello-world.cc"],
    includes=["include"],
    deps = [
        ":hello-greet",
    ],
)

cc_test(
    name = "gtest",
    srcs = [
        "test.cc"
        #"include/hello-greet.h",
    ],
    #hdrs = [""]
    #includes=["include"],
    copts = ["-Iexternal/gtest/include"],
    deps = ["@gtest//:gtest_main", ":hello-greet"],
)