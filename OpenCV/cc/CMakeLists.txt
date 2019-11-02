add_executable(CoinsSplit CoinsSplit.cc)
target_link_libraries(
    CoinsSplit
    ${OpenCV_LIBS}
)