# Livox ROS Driver2: message-only ROS1 subset

The two `.msg` definitions are copied without edits from Livox ROS Driver2
commit `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`. CMake/package.xml are
local packaging wrappers, not a complete Livox driver. No SDK or hardware node is
built or started. Package/type names remain upstream names to preserve recorded
ROS connection contracts. Keep `LICENSE.txt` and upstream attribution.
