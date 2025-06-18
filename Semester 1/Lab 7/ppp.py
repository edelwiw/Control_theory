s = "task2_p_controller_const_speed.txt task2_p_controller_const_target.txt task2_pi_controller_const_acceleration.txt  task2_pi_controller_const_speed.txt task2_special_controller_wave.txt"
arr = s.split()

delay_arr = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
for i in range(len(delay_arr)):
    # print(f"scp robot@192.168.2.3:~/{i} {i}")
    # print(f"scp robot@192.168.2.3:~/freq_response_{i}.txt freq_response_{i}.txt")
    d = delay_arr[i]
    print(f"scp robot@192.168.2.3:~/P_controller_delayed_{d}.txt P_controller_delayed_{d}.txt")