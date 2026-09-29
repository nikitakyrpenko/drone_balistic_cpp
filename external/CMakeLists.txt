# Сторонні header-only залежності. Кожна оформлена як INTERFACE-таргет, щоб include path і
# прапорці приїжджали до споживача через target_link_libraries, а не дублювалися вручну.

# MAVLink v2 C bindings. Vendored, деталі й pinned commit - у c_library_v2/VENDORED.md.
# SYSTEM гасить попередження згенерованих заголовків під -Wall -Wextra -pedantic.
add_library(mavlink INTERFACE)
target_include_directories(mavlink SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2)
