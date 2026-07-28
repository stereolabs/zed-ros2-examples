# Standard Header
std_msgs/Header header

# Instant Frequency
float32 topic_freq
# Average Frequency
float32 topic_avg_freq

# Instant Bandwidth
float32 topic_bw
# Average Bandwidth
float32 topic_avg_bw

# End-to-end latency, from the publisher-side header stamp to the arrival in the
# benchmark callback [msec]. Only available on the typed subscription paths: a
# generic (runtime-typed) subscription receives a serialized buffer with no
# directly usable timestamp. It is 0.0 when no latency could be measured.
float32 topic_latency
# Windowed average of the end-to-end latency [msec]. 0.0 when unavailable.
float32 topic_avg_latency
