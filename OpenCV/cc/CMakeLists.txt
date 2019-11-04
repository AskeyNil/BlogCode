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