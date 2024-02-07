set(base_examples
    # dce-example-mptcp-mmwave
    mc-twoenbs-ipv6
    mc-twoenbs
    mmwave-amc-test
    mmwave-amc-test2
    mmwave-ca-diff-bandwidth
    mmwave-ca-same-bandwidth
    mmwave-epc-amc-test
    mmwave-epc-tdma
    mmwave-example
    mmwave-simple-building-obstacle
    mmwave-simple-epc-ipv6
    mmwave-simple-epc
    mmwave-tcp-building-example
    mmwave-tcp-example
    mmwave-tdma
    qd-channel-full-stack-example
)

foreach(
  example
  ${base_examples}
)
  build_lib_example(
    NAME ${example}
    SOURCE_FILES ${example}.cc
    LIBRARIES_TO_LINK ${libmmwave}
  )
endforeach()

build_lib_example(
  NAME dce-example-mptcp-mmwave
  SOURCE_FILES dce-example-mptcp-mmwave.cc
  LIBRARIES_TO_LINK
    ${libnetwork}
    ${libcore}
    ${libdce}
    ${libpoint-to-point}
    ${libwifi}
    ${liblte}
    ${libbuildings}
    ${libmmwave}
    ${libmobility}
    ${libapplications}
    ${libnetanim}
    ${libconstant-position-mobility-model}
    ${libconfig-store}
)