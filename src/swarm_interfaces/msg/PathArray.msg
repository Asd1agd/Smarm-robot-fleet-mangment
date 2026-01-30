# PathArray.msg
# Represents a list of movement commands for a robot.

# Format:
# - Positive integers: Move forward by that number of units (e.g., 10 = forward 10 units)
# - Negative values:
#   - -3 = Turn right
#   - -6 = Turn left

# Example:
# path = [10, -6, 5, -3, 3]
# Meaning: Forward 10 → Turn left → Forward 5 → Turn right → Forward 3

int32[] path

