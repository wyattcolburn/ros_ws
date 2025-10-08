
for f in ~/ros_ws/ros_bag/new_gauss_2/*/seg_*/input_data/odom_data.csv; do
  rows=$(($(wc -l < "$f") - 1))
  seg=$(basename "$(dirname "$(dirname "$f")")")   # gets seg_X
  printf "%-10s %-20s %10d\n" "$seg" "$(basename "$f")" "$rows"
done
