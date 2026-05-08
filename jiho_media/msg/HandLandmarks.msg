std_msgs/Header header

uint8 num_hands          # number of detected hands (0–2)
string[] handedness      # per hand: "Left" or "Right"

# 21 landmarks per hand, row-major order
# hand 0: indices 0–20, hand 1: indices 21–41
Landmark[] landmarks
