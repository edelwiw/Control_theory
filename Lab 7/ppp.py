s = "task2_p_controller_const_speed.txt task2_p_controller_const_target.txt task2_pi_controller_const_acceleration.txt  task2_pi_controller_const_speed.txt task2_special_controller_wave.txt"
arr = s.split()
for i in range(13):
    # print(f"scp robot@192.168.2.3:~/{i} {i}")
    print(f"scp robot@192.168.2.3:~/freq_response_{i}.txt freq_response_{i}.txt")