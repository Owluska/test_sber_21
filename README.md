# test_sber_21
# Для запуска ноды

 ```$roslaunch semantic_to_occupancy sem_to_ocp_grid.launch```
 
 
 Если необходимо, выбрать путь к *.bag файлу, для этого в файле
 [в файле](https://github.com/Owluska/test_sber_21/tree/master/src/semantic_to_occupancy/src/launch/sem_to_ocp_grid.launch)
 изменить аргумент ```path```

 # Для просмотра карты в rviz
Добавить карту с отслеживанием топика ```/costmap```

add->from topic->/costmap
