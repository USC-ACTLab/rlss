a() {
   roscore
}

b() {
  sleep 2
  rosrun rviz rviz -d ./util/viz.rviz
}

a &
b &
wait # waits for all background processes to complete
