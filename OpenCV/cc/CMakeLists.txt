add_executable(CoinsSplit CoinsSplit.cc)
target_link_libraries(
    CoinsSplit
    ${OpenCV_LIBS}
)

add_executable(CornerDetector CornerDetector.cc)
target_link_libraries(
    CornerDetector
    ${OpenCV_LIBS}
)

add_executable(ORB ORB.cc)
target_link_libraries(
    ORB
    ${OpenCV_LIBS}
)

add_executable(BFMatcher BFMatcher.cc)
target_link_libraries(
    BFMatcher
    ${OpenCV_LIBS}
)

add_executable(FLANN FLANN.cc)
target_link_libraries(
    FLANN
    ${OpenCV_LIBS}
)

add_executable(FindTarget FindTarget.cc)
target_link_libraries(
    FindTarget
    ${OpenCV_LIBS}
)

add_executable(RecoveryTarget RecoveryTarget.cc)
target_link_libraries(
    RecoveryTarget
    ${OpenCV_LIBS}
)